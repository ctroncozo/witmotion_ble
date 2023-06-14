"""
WT901 BLE Message and Protocol Utilities

Defines message containers, and protocol constants
for decoding and interacting with WT901 BLE sensor devices.

Author: Cristian Troncoso
Email: ctroncoso.ai@gmail.com
License: MIT
"""

from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from itertools import count
from typing import ClassVar, Optional, Union

import numpy as np
from attrs import field


class MsgType(Enum):
    """
    Enumeration of supported message types from the WT901 device.

    These types represent categories of decoded sensor data, such as acceleration,
    orientation, angular velocity, temperature, or time. Each message is expected
    to carry only one type.
    """

    ACCELERATION = "acceleration"
    ANGULAR_VELOCITY = "angular_velocity"
    ORIENTATION = "orientation"
    TIMESTAMP = "timestamp"
    ANGLE = "angle"
    QUATERNIONS = "quaternions"
    TEMPERATURE = "temperature"


@dataclass(frozen=True, kw_only=True)
class Msg:
    """
    Container for decoded messages from WT901 BLE devices.

    Each Msg instance represents a single decoded message, storing its unique sequence number,
    timestamp, message type, and associated data. The sequence number is auto-incremented for
    each new message instance, ensuring unique identification. The class is immutable after creation.

    Attributes
    ----------
    _seq : int
        Unique sequence number for the message, auto-incremented.
    _timestamp_ms : float
        Timestamp of the message in milliseconds.
    _type : MsgType
        Type or category of the message.
    _data : Optional[Union[float, np.ndarray, datetime]]
        Decoded data payload, where values can be floats, numpy arrays, datetimes, or None.
    """

    counter: ClassVar[count] = count(start=0, step=1)
    _seq: int = field(init=False, factory=lambda: next(Msg.counter))
    _timestamp_ms: float = field(init=True)
    _type: MsgType = field(init=True)
    _data: Optional[Union[float, np.ndarray, datetime]] = field(init=True)

    def __repr__(self) -> str:
        """
        Developer-friendly string representation of the Msg instance.
        """
        return f"Msg(seq={self.seq}, stamp={self.stamp:.3f}, type='{self.type.value}', data={self.data})"

    def __str__(self) -> str:
        """
        Readable string representation of the Msg for logging or debugging.
        """
        return f"[{self.seq}] {self.type.name} @ {self.stamp:.1f}ms Data: {self.data}"

    def to_dict(self) -> dict:
        """
        Convert the message to a dictionary format for serialization or logging.

        Returns
        -------
        dict
            A dictionary containing the sequence number, timestamp, type, and serialized data.
        """
        return {
            "seq": self.seq,
            "stamp": self.stamp,
            "type": self.type.value,
            "data": (
                self.data.tolist()
                if isinstance(self.data, np.ndarray)
                else (
                    self.data.isoformat()
                    if isinstance(self.data, datetime)
                    else (
                        self.data
                        if isinstance(self.data, (int, float, str, type(None)))
                        else str(self.data)
                    )
                )
            ),
        }

    @property
    def seq(self) -> int:
        """Return the message's unique sequence number."""
        return self._seq

    @property
    def stamp(self) -> float:
        """Return the message timestamp in milliseconds."""
        return self._timestamp_ms

    @property
    def type(self) -> MsgType:
        """Return the message's type/category."""
        return self._type

    @property
    def data(self) -> Optional[Union[float, np.ndarray, datetime]]:
        """Return the decoded data payload of the message."""
        return self._data
