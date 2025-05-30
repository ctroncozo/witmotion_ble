"""
File: registers.py

Defines supported WIT sensor register constants and associated metadata.

This module provides a structured Enum-based interface for interacting with
WIT sensor registers, including register names, addresses, and expected binary
formats for decoding sensor data. It also offers utilities to generate
request messages for manually polling specific registers.

Author: Cristian Troncoso
Email: ctroncoso.ai@gmail.com
License: MIT
Version: 1.0.0
"""

import struct
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class RegName(Enum):
    """Enumeration of supported WIT sensor register names."""

    DEFAULT = "default"
    TEMP = "temperature"
    SAVE = "save"
    CALSW = "calibration"
    RATE = "return_rate"
    BAUD = "baud_rate"
    ACCEL_OFFSET = "accel_offset"
    ANGVEL_OFFSET = "angvel_offset"
    MAG_OFFSET = "mag_offset"
    TIMESTAMP = "timestamp"
    ACCEL = "acceleration"
    ANGVEL = "angular_velocity"
    MAGFIELD = "magnetic_field"
    ORIENTATION = "orientation"
    QUATERNIONS = "quaternions"

    def __str__(self) -> str:
        return self.value


@dataclass(frozen=True)
class RegisterMap:
    """Metadata for a WIT sensor register."""
    name: str
    address: int

    def __str__(self) -> str:
        """Return a readable string describing the register."""
        return f"Register name: {self.name}, address: {hex(self.address)}"


class Register(Enum):
    """
    Enumeration of supported WIT sensor registers.

    Each member contains:
    - `name`: A human-readable name for the register
    - `address`: The register address (as used in communication)

    Note:
    - Some format values are marked "NOTIMPLEMENTED" where decoding is not yet defined.
    - Address references follow the WIT standard communication protocol.
    """
    READ_ADDRESS_REG = RegisterMap("read_address_register", 0x27)

    # Default register data packet (acceleration, angular velocity, angle)
    DEFAULT = RegisterMap("default", 0x61)

    # Single return registers data packets
    TIMESTAMP = RegisterMap("timestamp", 0x30)
    YYMM = RegisterMap("yymm", 0x30)  # Year and month
    DDHH = RegisterMap("ddhh", 0x31)  # Day and hour
    MMSS = RegisterMap("mmss", 0x32)  # Minute and second
    MS = RegisterMap("millistamp", 0x33)  # Milliseconds
    MAGFIELD = RegisterMap("magnetic_field", 0x3A)
    QUATERNIONS = RegisterMap("quaternions", 0x51)
    TEMP = RegisterMap("temperature", 0x40)

    # Configuration registers
    SAVE = RegisterMap("save", 0x00)
    CALSW = RegisterMap("calibration_switch", 0x01)
    RATE = RegisterMap("rate", 0x03)
    BAUD = RegisterMap("baud_rate", 0x04)

    # Sensor calibration offsets
    ACCEL_OFFSET = RegisterMap("accel_offset", 0x05)
    ANGVEL_OFFSET = RegisterMap("angvel_offset", 0x08)
    MAG_OFFSET = RegisterMap("mag_offset", 0x0A)

    def __add__(self, other):
        raise AttributeError("Addition is not allowed on Register types")

    def __sub__(self, other):
        raise AttributeError("Subtraction is not allowed on Register types")

    def __mul__(self, other):
        raise AttributeError("Multiplication is not allowed on Register types")

    def __div__(self, other):
        raise AttributeError("Division is not allowed on Register types")

    def __truediv__(self, other):
        raise AttributeError("Division is not allowed on Register types")

    @classmethod
    def get_reg_name_from_address(cls, address: int) -> str:
        """Get the register name from its address."""
        for reg in cls:
            if reg.value.address == address:
                return reg.value.name
        raise ValueError(f"Invalid register address: {hex(address)}")

    @classmethod
    def set_register_msg(cls, register: RegisterMap, value: int) -> bytes:
        """
        Create a register set message to write to a specific register.

        Parameters
        ----------
        register : Map
            The address of the register to write to.
        value : int
            The value to write to the register.

        Returns
        -------
        bytes
            The serialized register set message.
        """
        header_flag_1 = 0xFF
        header_flag_2 = 0xAA
        wit_command_header_bytes = [header_flag_1, header_flag_2]
        return struct.pack("<BBBBB", *wit_command_header_bytes, register.address, value,
                           0x00)

    @classmethod
    def data_pack_request_msg(cls, register: RegisterMap) -> bytes:
        """
        Create a register request message to read from a specific register.

        This message is used to request manual readouts of sensor data not published
        by default (i.e., anything other than Register.DEFAULT). Once sent, the sensor
        responds with a 20-byte data packet containing 8 consecutive registers starting
        from the requested address. Each register is 2 bytes plus 4 bytes of header and metadata.

        Parameters
        ----------
        register : RegisterMap
            A RegisterMap object containing the register's metadata (name, address, format).

        Returns
        -------
        bytes
            The serialized request message to be sent to the sensor, or None if the
            requested register is the default broadcast register.
        """
        if register == cls.DEFAULT:
            return None

        return Register.set_register_msg(cls.READ_ADDRESS_REG.value, register.address)
