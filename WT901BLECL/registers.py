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
from enum import Enum
from types import MappingProxyType as Map
from typing import Final, Optional


class RegName():
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


class Register(Enum):
    """
    Enumeration of supported WIT sensor registers.

    Each member contains:
    - `name`: A human-readable name for the register
    - `address`: The register address (as used in communication)
    - `format`: A struct-compatible format string for parsing binary data

    Note:
    - Some format values are marked "NOTIMPLEMENTED" where decoding is not yet defined.
    - Address references follow the WIT standard communication protocol.
    """
    READ_ADDRESS_REG: Final[int] = 0x27


    # Default register data packet (acceleration, angular velocity, angle)
    DEFAULT: Final[Map[RegName, int]] = Map({"name": "default", "address": 0x61})

    # Single return registers data packets
    TIMESTAMP: Final[Map[RegName, int]] = Map({"name": RegName.TIMESTAMP, "address": 0x30})
    MAGFIELD: Final[Map[RegName, int]] = Map({"name": RegName.MAGFIELD, "address": 0x3A})
    QUATERNIONS: Final[Map[RegName, int]] = Map({"name": RegName.QUATERNIONS, "address": 0x51})
    TEMP: Final[Map[RegName, int]] = Map({"name": RegName.TEMP, "address": 0x40})

    # Configuration registers
    SAVE: Final[Map[RegName, int]] = Map({"name": RegName.SAVE, "address": 0x00})
    CALSW: Final[Map[RegName, int]] = Map({"name": RegName.CALSW, "address": 0x01})
    RATE: Final[Map[RegName, int]] = Map({"name": RegName.RATE, "address": 0x03})
    BAUD: Final[Map[RegName, int]] = Map({"name": RegName.BAUD, "address": 0x04})

    # Sensor calibration offsets
    ACCEL_OFFSET: Final[Map[RegName, int]] = Map({"name": RegName.ACCEL_OFFSET, "address": 0x05})
    ANGVEL_OFFSET: Final[Map[RegName, int]] = Map({"name": RegName.ANGVEL_OFFSET, "address": 0x08})
    MAG_OFFSET: Final[Map[RegName, int]] = Map({"name": RegName.MAG_OFFSET, "address": 0x0A})

    @property
    def reg_name(self) -> RegName:
        """Return the human-readable name of the register."""
        return self.value["name"]

    @property
    def address(self) -> int:
        """Return the address of the register."""
        return self.value["address"]

    def __add__(self, other):
        """Addition is defined only as address summation (not generally meaningful)."""
        return self.value["address"] + other.value["address"]

    def __sub__(self, other):
        raise AttributeError("Subtraction is not allowed on Register types")

    def __mul__(self, other):
        raise AttributeError("Multiplication is not allowed on Register types")

    def __div__(self, other):
        raise AttributeError("Division is not allowed on Register types")

    def __truediv__(self, other):
        raise AttributeError("Division is not allowed on Register types")

    def __str__(self) -> str:
        """Return a readable string describing the register."""
        return f"Register name: {self.name}, address: {hex(self.address)}"

    @classmethod
    def get_name_from_address(cls, address: int) -> str:
        """Get the register name from its address."""
        for reg in cls:
            if reg.address == address:
                return reg.name
        raise ValueError(f"Invalid register address: {hex(address)}")

    @classmethod
    def set_register_msg(cls, register: Map, value: int) -> bytes:
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
        wit_command_header_bytes = [0xFF, 0xAA]
        return struct.pack("<BBBBB", *wit_command_header_bytes, register.address, value, 0x00)

    @classmethod
    def data_pack_request_msg(cls, register: Map) -> Optional[bytes]:
        """
        Create a register request message to read from a specific register.

        This message is used to request manual readouts of sensor data not published
        by default (i.e., anything other than Register.DEFAULT). Once sent, the sensor
        responds with a 20-byte data packet containing 8 consecutive registers starting
        from the requested address. Each register is 2 bytes plus 4 bytes of header and metadata.

        Parameters
        ----------
        register : Map
            A Map object containing the register's metadata (name, address, format).

        Returns
        -------
        Optional[bytes]
            The serialized request message to be sent to the sensor, or None if the
            requested register is the default broadcast register.
        """
        if register.address == Register.DEFAULT.address:
            return None
        
        return Register.set_register_msg(Register.READ_ADDRESS_REG, register.address)