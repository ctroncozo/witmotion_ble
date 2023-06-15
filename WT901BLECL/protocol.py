"""
WT901 BLE Packet Decoding Utilities

This module provides interfaces for communicating with the WT901 BLE device
over Bluetooth Low Energy (BLE).

It provides the following interfaces:
- Generate calibration instructions to be sent to the device.
- Generate synchronization instructions to be sent to the device.
- Decode received packets from the device.

Author: Cristian Troncoso
Email: ctroncoso.ai@gmail.com
License: MIT
"""

import logging
import struct
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import ClassVar, Generator, List, Optional

import numpy as np

from time_handler import TimeHandler

from .message import Msg, MsgType
from .registers import Register


class Gattuidd(Enum):
    """
    Bluetooth GATT (Generic Attribute Profile) UUIDs for WT901 BLE communication.

    These UUIDs are used to identify the relevant GATT services and characteristics
    when connecting to a WT901 sensor over Bluetooth Low Energy.

    Members
    -------
    WIT_GATT_PROFILE : str
        Standard GATT profile UUID.
    WIT_SERVICE_UUID : str
        Primary service UUID for the WT901 device.
    WIT_READ_CHAR_UUID : str
        Characteristic UUID for reading data from the device.
    WIT_WRITE_CHAR_UUID : str
        Characteristic UUID for writing commands to the device.
    """

    WIT_GATT_PROFILE = "00001801-0000-1000-8000-00805f9b34fb"
    WIT_SERVICE_UUID = "0000ffe5-0000-1000-8000-00805f9a34fb"
    WIT_READ_CHAR_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"
    WIT_WRITE_CHAR_UUID = "0000ffe9-0000-1000-8000-00805f9a34fb"


class WitPkt(Enum):
    """
    Constants and utilities for WT901 BLE packet structure.

    This enum defines the byte-level structure of data packets used to
    communicate with the WT901 sensor over BLE, including header bytes,
    flags, and packet lengths. It also provides utilities for formatting
    packet requests.

    Members
    -------
    DEFAULT_LENGTH : int
        Length of a default IMU data packet (11 bytes).
    SINGLE_REGISTER_LENGTH : int
        Length of a single register response packet (20 bytes).
    HEADER : int
        Common header byte used to identify the start of a packet (0x55).
    DEFAULTFLAG : int
        Identifier for default streaming packets.
    REGISTERFLAG : int
        Identifier for register-based response packets.
    """

    DEFAULT_LENGTH = 11
    SINGLE_REGISTER_LENGTH = 20
    HEADER = 0x55
    DEFAULTFLAG = 0x61
    REGISTERFLAG = 0x71


class UpdateRate(Enum):
    """
    Enumeration of supported update rates for WT901 sensor output.

    These constants represent frequency values (in Hz) used to configure
    the rate at which the sensor sends data packets.
    """

    R1HZ = 0x03
    R2HZ = 0x04
    R5HZ = 0x05
    R10HZ = 0x06
    R20HZ = 0x07
    R50HZ = 0x08
    R100HZ = 0x09
    R125HZ = 0x0A
    R200HZ = 0x0B
    RSINGLE = 0x0C
    RNO_OUTPUT = 0x0D


@dataclass
class IDecoder(ABC):
    """
    Decoder class interface for parsing WT901 BLE packets.

    This class provides methods for decoding different types of packets received
    from the WT901 BLE device, including default IMU packets, single register
    responses, and quaternion packets.
    There should be a decoder for each following register type:
    - Default
    - Quaternion
    - Temperature
    Each decoder should manage its own TimeHandler instance to compute the timestamp of each
    message type independently.
    """

    sequence: ClassVar[int] = 0

    @classmethod
    def decode(cls, timestamp: int, pkt: bytes) -> Msg:
        """
        Decode a WT901 BLE packet into a Msg object.
        Note: This method is not to be overridden by subclasses.

        - Increment the sequence number for the message.
        - Call the implementation-specific decoder.
        """
        cls.sequence += 1
        return cls._decode_impl(timestamp, pkt)

    @classmethod
    @abstractmethod
    def _decode_impl(cls, timestamp: int, pkt: bytes) -> Msg:
        """
        Decode a WT901 BLE packet into a Msg object. To be implemented by subclasses.

        Parameters
        ----------
        pkt : bytes
            Raw byte stream from the WT901 device.

        Returns
        -------
        Msg
            A decoded message object for each complete packet in the stream.
        """
        raise NotImplementedError


@dataclass
class QuaternionDecoder(IDecoder):
    """
    Decode quaternion data from a WT901 BLE quaternion register packet.
    """

    sequence: ClassVar[int] = 0

    @classmethod
    def _decode_impl(cls, timestamp: int, pkt: bytes) -> Msg:
        """
        Decode quaternion data from a WT901 BLE quaternion register packet.

        Expected packet layout:
            0x55 | 0x71 | 0x51 | 0x00 | Q0L | Q0H | Q1L | Q1H | Q2L | Q2H | Q3L | Q3H

        Each Qn component is decoded from int16 and scaled to [-1, 1] using:
            Qn = int16(QnH << 8 | QnL) / 32768.0

        Parameters
        ----------  
        timestamp : int
            Timestamp of the packet in milliseconds.
        pkt : bytes
            A 12-byte binary packet containing quaternion data.

        Returns
        -------
        Msg
            A structured message containing the normalized quaternion vector.
        """
        unpack_format = "BBBBhhhhhhhh"
        _, _, _, _, q0, q1, q2, q3 = struct.unpack(unpack_format, pkt)
        quat = np.array([q0, q1, q2, q3]) / 32768.0
        return Msg(
            timestamp_ms=timestamp,
            type=MsgType.QUATERNIONS,
            data={MsgType.QUATERNIONS.value: quat},
        )


@dataclass
class DefaultDecoder(IDecoder):
    """
    Decode the WT901 default IMU packet containing acceleration, angular velocity, and orientation.
    """

    timer: ClassVar[TimeHandler] = field(init=True)
    sequence: ClassVar[int] = 0

    @classmethod
    def _decode_impl(cls, timestamp: int, pkt: bytes) -> List[Msg]:
        """
        Decode the WT901 default IMU packet containing acceleration, angular velocity, and orientation.

        The default packet (flag 0x61) includes 3 sets of 3-axis measurements:
            - Acceleration: scaled to ±16g
            - Angular velocity: scaled to ±2000 deg/s
            - Euler angles (orientation): scaled to ±180 degrees

        Each group is encoded as signed 16-bit integers (int16), requiring normalization (scaled to [-1, 1]).

        Parameters
        ----------
        timestamp : int
            Timestamp of the packet in milliseconds.
        pkt : bytes
            The raw 11-byte binary packet from the sensor.

        Returns
        -------
        Msg
            A structured message object containing the decoded sensor values.
        """
        int16_scale = 32768.0
        acc_scale = 16.0
        gyro_scale = 2000.0
        angle_scale = 180.0
        unpack_format = "BBBBhhhhhhhhh"

        timestamp = cls.timer.stamp_from_host()
        data = struct.unpack(unpack_format, pkt)

        acc_data = np.array(data[2:5]) / int16_scale * acc_scale
        angvel_data = np.array(data[5:8]) / int16_scale * gyro_scale
        angle_data = np.array(data[8:]) / int16_scale * angle_scale

        acc_msg = Msg(timestamp_ms=timestamp, type=MsgType.ACCELERATION, data=acc_data)
        angvel_msg = Msg(
            timestamp_ms=timestamp, type=MsgType.ANGULAR_VELOCITY, data=angvel_data
        )
        angle_msg = Msg(timestamp_ms=timestamp, type=MsgType.ANGLE, data=angle_data)

        return [acc_msg, angvel_msg, angle_msg]


@dataclass
class TimestampDecoder(IDecoder):
    """
    Decode a timestamp register packet from the WT901 device.
    """

    timer: ClassVar[TimeHandler] = field(init=True)
    sequence: ClassVar[int] = 0

    @classmethod
    def _decode_impl(cls, timestamp: int, pkt: bytes) -> Optional[Msg]:
        """
        Decode a timestamp register packet from the WT901 device.

        The timestamp includes YY/MM/DD HH:MM:SS.MS encoded in 8-bit and 16-bit fields,
        and is converted into a full datetime object (UTC assumed).

        Parameters
        ----------
        timestamp : int
            Timestamp of the packet in milliseconds.
        pkt : bytes
            Raw binary packet from the WT901 timestamp register.

        Returns
        -------
        Msg
            A message containing the device timestamp as a datetime object.
        """
        unpack_format = "BBBBBBBBBBhhhhh"
        data = struct.unpack(unpack_format, pkt)
        current_date = cls.timer.get_current_date()
        year_base = current_date.year - (current_date.year % 100)
        current_year = year_base + data[4]

        device_date = datetime(
            year=current_year,
            month=data[5],
            day=data[6],
            hour=data[7],
            minute=data[8],
            second=data[9],
            microsecond=data[10] * 1000,
            tzinfo=timezone.utc,
        )

        return Msg(
            timestamp_ms=timestamp,
            type=MsgType.TIMESTAMP,
            data=device_date,
        )


@dataclass
class TemperatureDecoder(IDecoder):
    """
    Decode temperature data from a WT901 BLE temperature register packet.
    """

    sequence: ClassVar[int] = 0

    @classmethod
    def _decode_impl(cls, timestamp: int, pkt: bytes) -> Msg:
        """
        Decode temperature data from the WT901 BLE sensor.

        The temperature register returns a signed 16-bit integer representing
        temperature in hundredths of a degree Celsius. It is scaled to °C by dividing by 100.0.

        Parameters
        ----------
        timestamp : int
            Timestamp of the packet in milliseconds.
        pkt : bytes
            Raw binary packet from the temperature register (0x40).

        Returns
        -------
        Msg
            A message containing the temperature in degrees Celsius.
        """
        unpack_format = "4BHB"
        raw_temp, *_ = struct.unpack(unpack_format, pkt)
        temperature_c = raw_temp / 100.0

        return Msg(
            timestamp_ms=timestamp,
            type=MsgType.TEMPERATURE,
            data=temperature_c,
        )


@dataclass
class WitProtocol:
    """
    WT901BLECL5.0 protocol class.
    Responsibilities:
    1. Decode raw byte messages from the device.
    2. Encode calibration instructions for the device.
    3. Encode synchronization instructions for the device.

    Calibration
    -----------
    WT901 BLE Calibration Commands instructions.

    This class provides coroutine methods to send supported calibration commands
    to the device using the CALSW register (0x01).

    Methods require an async `cb` callback, which is responsible for transmitting
    the raw byte message (e.g., via BLE GATT write).

    All calibration modes documented here are sourced directly from:
    - WIT Standard Communication Protocol.pdf
    - WT901BLECL DataSheet.pdf
    """

    DEFAULT_MODE = 0x00
    ACCEL_CALIBRATION_MODE = 0x01
    HEIGHT_RESET_CALIBRATION_MODE = 0x03
    SET_HEADING_CALIBRATION_MODE = 0x04
    MAG_SPHERICAL_CALIBRATION_MODE = 0x07
    SET_ANGLE_REF_CALIBRATION_MODE = 0x08
    MAG_DUAL_PLANE_CALIBRATION_MODE = 0x09

    _logger: logging.Logger = field(default_factory=logging.getLogger)
    _default_decoder: DefaultDecoder = field(init=False)
    _timestamp_decoder: TimestampDecoder = field(init=False)
    _quaternion_decoder: QuaternionDecoder = field(init=False)
    _temperature_decoder: TemperatureDecoder = field(init=False)

    def __post_init__(self):
        self._default_decoder = DefaultDecoder()
        self._timestamp_decoder = TimestampDecoder()
        self._temperature_decoder = TemperatureDecoder()
        self._quaternion_decoder = QuaternionDecoder()
    
    def decode(self, pkg_timestamp: int, msg: bytes) -> Generator[Msg, None, None]:
        """
        Decode a byte stream from the WT901 BLE device into Msg objects.

        This function is a generator that yields each decoded Msg as soon as it's available.
        It handles both default data packets and register-specific responses.

        Parameters
        ----------
        pkg_timestamp : int
            Timestamp of the packet in milliseconds.
        msg : bytes
            Raw byte stream from the WT901 device.

        Yields
        ------
        Msg
            A decoded message object for each complete packet in the stream.
        """
        offset = 0
        while offset < len(msg):
            if msg[offset] != WitPkt.HEADER.value:
                self._logger.error(
                    "Invalid packet header at offset %d: %02X", offset, msg[offset]
                )
                offset += 1
                continue

            flag = msg[offset + 1]

            if flag == WitPkt.DEFAULTFLAG.value:
                pkt_size = WitPkt.DEFAULT_LENGTH.value
            elif flag == WitPkt.REGISTERFLAG.value:
                pkt_size = WitPkt.SINGLE_REGISTER_LENGTH.value
            else:
                self._logger.error("Unknown flag %02X at offset %d", flag, offset)
                offset += 1
                continue

            if offset + pkt_size > len(msg):
                self._logger.warning(
                    "Incomplete packet at end of buffer: need %d, have %d",
                    pkt_size,
                    len(msg) - offset,
                )
                break

            pkt = msg[offset : offset + pkt_size]
            pkt_type = pkt[2]

            try:
                if flag == WitPkt.DEFAULTFLAG.value:
                    yield self._default_decoder.decode(pkg_timestamp, pkt)
                elif flag == WitPkt.REGISTERFLAG.value:
                    if pkt_type == Register.TIMESTAMP.address:
                        yield TimestampDecoder.decode(pkg_timestamp, pkt)
                    elif pkt_type == Register.QUATERNIONS.address:
                        yield QuaternionDecoder.decode(pkg_timestamp, pkt)
                    elif pkt_type == Register.TEMP.address:
                        yield TemperatureDecoder.decode(pkg_timestamp, pkt)
                    else:
                        self._logger.warning(
                            f"Unsupported decoder {Register.get_name_from_address(pkt_type)}"
                        )
            except Exception as e:
                self._logger.error(f"Failed to decode packet at offset {offset}: {e}")

            offset += pkt_size

    @classmethod
    def start_accelerometer_calibration(cls):
        """
        Perform automatic accelerometer calibration (0x01).

        Instructs the user to place the WT901 device on all six faces (+X, -X, +Y, -Y, +Z, -Z).

        Returns
        -------
        bytes
            The raw byte message to send to the device.
        """
        return Register.set_register_msg(Register.CALSW, cls.ACCEL_CALIBRATION_MODE)

    @classmethod
    def start_magnetometer_spherical_calibration(cls):
        """
        Start spherical magnetic field calibration (0x07).

        The user must rotate the device in all directions (e.g., figure-8 motion).

        Returns
        -------
        bytes
            The raw byte message to send to the device.
        """
        return Register.set_register_msg(
            Register.CALSW, cls.MAG_SPHERICAL_CALIBRATION_MODE
        )

    @classmethod
    def start_magnetometer_dual_plane_calibration(cls):
        """
        Start dual-plane magnetic field calibration (0x09).

        The user must rotate the device in both horizontal and vertical planes.

        Returns
        -------
        bytes
            The raw byte message to send to the device.
        """
        return Register.set_register_msg(
            Register.CALSW, cls.MAG_DUAL_PLANE_CALIBRATION_MODE
        )

    @classmethod
    def reset_height(cls):
        """
        Reset the current height baseline (0x03).

        Instructs the user to keep the device stable at the reference height.
        Calibration completes immediately.

        Returns
        -------
        bytes
            The raw byte message to send to the device.
        """
        return Register.set_register_msg(
            Register.CALSW, cls.HEIGHT_RESET_CALIBRATION_MODE
        )

    @classmethod
    def set_heading_zero(cls):
        """
        Set the current heading angle to zero (0x04).

        Instructs the user to point the device forward to set the reference heading.

        Returns
        -------
        bytes
            The raw byte message to send to the device.
        """
        return Register.set_register_msg(
            Register.CALSW, cls.SET_HEADING_CALIBRATION_MODE
        )

    @classmethod
    def set_angle_reference(cls):
        """
        Set the current orientation as the angle reference (0x08).

        The current orientation is stored and used as a reference.

        Returns
        -------
        bytes
            The raw byte message to send to the device.
        """
        return Register.set_register_msg(
            Register.CALSW, cls.SET_ANGLE_REF_CALIBRATION_MODE
        )

    @classmethod
    def quit_calibration(cls):
        """
        Exit calibration mode and return to normal operation (0x00).
        """
        return Register.set_register_msg(Register.CALSW, cls.DEFAULT_MODE)

    @classmethod
    def synchronize_instruction(cls, date: datetime):
        """
        Synchronize the device time with the host time.
        """
        msg = [struct.pack(
            "<BBBBB", 0xFF, 0xAA, Register.YYMM.address, date.year % 100, date.month
        ), struct.pack(
            "<BBBBB", 0xFF, 0xAA, Register.DDHH.address, date.day, date.hour
        ), struct.pack(
            "<BBBBB", 0xFF, 0xAA, Register.MMSS.address, date.minute, date.second
        )]
        # WIT communication protocol stores the year as a single byte, which only has space for two digits (0–255).

        # Set Day and Hour

        # Set Minute and Second

        # Set Milliseconds
        ms = round(date.microsecond / 1000)
        msg.append(struct.pack("<BBBH", 0xFF, 0xAA, Register.MS.address, ms))
        return msg
