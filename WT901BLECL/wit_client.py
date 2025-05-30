"""
file: wit_client.py
WT901 BLE Client

This module provides a Wit901BLEClient class to handle BLE communication with the WT901 sensor.

Author: Cristian Troncoso
Email: ctroncoso.ai@gmail.com
License: MIT
"""

import asyncio
import logging
from dataclasses import dataclass, field
from typing import Callable, Optional

from bleak import BleakClient, BleakError, BleakGATTCharacteristic

from WT901BLECL.message import Msg
from WT901BLECL.protocol import UpdateRate, GattUIDD, WitProtocol
from WT901BLECL.registers import Register
from WT901BLECL.time_handler import TimeHandler

wit_logger = logging.getLogger("Wit901BLEClient")


@dataclass(kw_only=True)
class Wit901BLEClient:
    """
    BLE client for the WIT901 sensor, handling BLE communication, data streaming, and time synchronization.

    Parameters
    ----------
    _client : BleakClient
        The BleakClient instance managing the BLE connection to the sensor.
    _timer : TimeHandler
        Utility for synchronizing and managing timing with the sensor.
    _synchronize : Optional[bool]
        Whether to synchronize the device clock with the host system time on connection (default: True).
    _stream_timestamps : bool
        Whether to stream timestamp packets from the device.
    _stream_temperature : bool
        Whether to stream temperature packets from the device.
    _stream_quaternions : bool
        Whether to stream quaternion packets from the device.
    _stream_callback : Callable[[Msg], None]
        Callback function for handling streamed Msg objects.
    _notify_char : BleakGATTCharacteristic
        BLE characteristic used for receiving notifications from the sensor.
    _write_char : BleakGATTCharacteristic
        BLE characteristic used for sending commands to the sensor.
    _running : bool
        Indicates if the client is actively running and processing data.
    _wit_protocol : WitProtocol
        Protocol handler for WIT901-specific command serialization and parsing.

    Methods
    -------
    __post_init__():
        Initializes the client, sets the update rate, prepares timer and protocol handler.
    calibrate_accelerometer():
        Initiates and manages the accelerometer calibration process.
    calibrate_magnetometer_spherical():
        Initiates and manages the spherical magnetometer calibration.
    synchronize():
        Synchronizes the sensor's internal clock with the host system time.
    get_gatts():
        Discovers and returns GATT services and characteristics from the sensor.
    send_command(cmd: bytes):
        Sends a raw command to the sensor via BLE.
    start():
        Starts the BLE client, connects to the sensor, and begins data processing.
    stop():
        Stops the BLE client and disconnects from the sensor.

    Streaming
    ---------
    Streams both host and device timestamps and their difference (ms) for time alignment and latency analysis (see stream.py for ZMQ output).

    Example
    -------
    '>>> client = Wit901BLEClient(BleakClient(address), TimeHandler(50), lambda msg: print(msg))'
    '>>> await client.connect()'
    # No need to calibrate every time.
    '>>> await client.calibrate_accelerometer()'
    '>>> await client.stream()'
    # Wait for the stop event
    '>>> await stop_event.wait()'
        _stream_timestamps : bool
        Whether to stream timestamp packets from the device.
    _stream_temperature : bool
        Whether to stream temperature packets from the device.
    _stream_quaternions : bool
        Whether to stream quaternion packets from the device.
    _stream_callback : Callable[[Msg], None]
        Callback function for handling streamed Msg objects.
    _notify_char : BleakGATTCharacteristic
        BLE characteristic used for receiving notifications from the sensor.
    _write_char : BleakGATTCharacteristic
        BLE characteristic used for sending commands to the sensor.
    _running : bool
        Indicates if the client is actively running and processing data.
    _wit_protocol : WitProtocol
        Protocol handler for WIT901-specific command serialization and parsing.

    Methods
    -------
    __post_init__():
        Initializes the client, sets the update rate, and prepares timer and protocol handler.
    calibrate_accelerometer():
        Initiates and manages the accelerometer calibration process.
    calibrate_magnetometer_spherical():
        Initiates and manages the spherical magnetometer calibration.
    synchronize():
        Synchronizes the sensor's internal clock with the host system time.
    get_gatts():
        Discovers and returns GATT services and characteristics from the sensor.
    send_command(cmd: bytes):
        Sends a raw command to the sensor via BLE.
    start():
        Starts the BLE client, connects to the sensor, and begins data processing.
    stop():
        Stops the BLE client and disconnects from the sensor.

    Example
    -------
    '>>> client = Wit901BLEClient(BleakClient(address), TimeHandler(50),lambda msg: print(msg))'
    '>>> await client.connect()'
    # No need to calibrate every time.
    '>>> await client.calibrate_accelerometer()'
    '>>> await client.stream()'
    # Wait for the stop event
    '>>> await stop_event.wait()'
    """

    _client: BleakClient = field(init=True)
    _timer: TimeHandler = field(init=True)
    _synchronize: Optional[bool] = field(init=True, default=True)
    _stream_timestamps: Optional[bool] = field(init=True, default=False)
    _stream_temperature: Optional[bool] = field(init=True, default=False)
    _stream_quaternions: Optional[bool] = field(init=True, default=False)
    _stream_callback: Callable[[Msg], None] = field(init=True, default=None)
    _notify_char: BleakGATTCharacteristic = field(init=False)
    _write_char: BleakGATTCharacteristic = field(init=False)
    _running: bool = field(init=False)
    _update_rate_value: int = field(init=False)
    _wit_protocol: WitProtocol = field(init=False, default_factory=WitProtocol)

    def __post_init__(self) -> None:
        # TODO: Need better way to verify supported rates
        update_rate = self._timer.update_rate
        if update_rate == 20:
            self._update_rate_value = UpdateRate.R20HZ.value
        elif update_rate == 50:
            self._update_rate_value = UpdateRate.R50HZ.value
        elif update_rate == 100:
            self._update_rate_value = UpdateRate.R100HZ.value
        else:
            raise ValueError("Invalid update rate")

        if self._stream_callback is None:
            raise ValueError("Stream callback must be provided")

    @staticmethod
    async def _countdown(seconds: int) -> None:
        """Print a countdown to console during calibration."""
        for remaining in range(seconds, 0, -1):
            print(f"\r {remaining}s remaining...", end="", flush=True)
            await asyncio.sleep(1)
        print("\r Calibration complete.")

    async def calibrate_accelerometer(self) -> None:
        """
        Performs automatic calibration of the accelerometer.
        Instruct the user to place the device on all six faces for 5 seconds on each face
        (+X, -X, +Y, -Y, +Z, -Z)
        """
        await self.send_command(WitProtocol.start_accelerometer_calibration(), response=True)
        await self._countdown(5 * 6)
        await self.send_command(WitProtocol.quit_calibration(), response=True)

    async def calibrate_magnetometer_spherical(self) -> None:
        """
        Performs automatic calibration of the magnetometer in spherical mode.
        Instruct the user to rotate the device in all directions (e.g., figure-8 motion).
        """
        await self.send_command(
            WitProtocol.start_magnetometer_spherical_calibration(), response=True)
        await self._countdown(30)
        await self.send_command(WitProtocol.quit_calibration(), response=True)

    async def synchronize(self) -> None:
        """
        Synchronize the device time with the host time.
        """
        sync_msgs = WitProtocol.synchronize_instruction(self._timer.synchronize())
        for msg in sync_msgs:
            await self.send_command(msg, response=False)

    async def get_gatts(self) -> None:
        """
        Extract notify and write characteristics from the Bleak client

        Returns
        -------
        Notify and write characteristics, in that order
        """
        for service in self._client.services:
            if service.uuid in (
                    GattUIDD.WIT_SERVICE_UUID.value,
                    GattUIDD.WIT_GATT_PROFILE.value,
            ):
                for characteristic in service.characteristics:
                    if characteristic.uuid == GattUIDD.WIT_READ_CHAR_UUID.value:
                        self._notify_char = characteristic
                        wit_logger.info("Notify characteristics available")
                    if characteristic.uuid == GattUIDD.WIT_WRITE_CHAR_UUID.value:
                        self._write_char = characteristic
                        wit_logger.info("Write characteristics available")

        if not self._notify_char:
            wit_logger.error("Notify characteristics not available")
            raise RuntimeError("Notify characteristics not available")
        if not self._write_char:
            wit_logger.error("Write characteristics not available")
            raise RuntimeError("Write characteristics not available")

    async def periodic_synchronize(self, interval: int = 10):
        """
        Periodically synchronize the device time with the host time every `interval` seconds.
        Only runs if self._synchronize is True.
        """
        while self._running and self._synchronize:
            await self.synchronize()
            await asyncio.sleep(interval)
            wit_logger.info("Synchronized successfully!")

    def _on_disconnect(self) -> None:
        """
        Callback function to be called when the client disconnects.
        """
        wit_logger.info("Disconnected successfully!")

    async def connect(self, timeout: int = 10) -> None:
        """
        Connect to the device.
        Parameters
        ----------
        timeout : int
            Timeout in seconds.
        Returns
        -------
        None
        """
        self._running = True
        try:
            await self._client.connect(timeout=timeout)
            wit_logger.info("Connected successfully!")
        except asyncio.TimeoutError:
            wit_logger.error("Connection attempt timed out.")
        except BleakError as e:
            wit_logger.error("Bleak error during connection: %s", e)
        except Exception as e:
            wit_logger.error("Other error: %s", e)
        await self.get_gatts()
        await self.send_command(Register.set_register_msg(
            Register.RATE.value, self._update_rate_value), response=True)
        await self.synchronize()
        # Start periodic synchronization if enabled
        if self._synchronize:
            self._sync_task = asyncio.create_task(self.periodic_synchronize())

    async def disconnect(self, timeout_secs: int = 5) -> None:
        """
        Disconnect from the device and block until fully disconnected.
        Parameters
        ----------
        timeout_secs : int
            Timeout in seconds.
        Returns
        -------
        None
        """
        wit_logger.info("Disconnecting device %s", self._client.address)
        await self._client.disconnect()
        self._running = False
        if hasattr(self, '_sync_task'):
            self._sync_task.cancel()
        disconnected = False
        for _ in range(timeout_secs):
            if not self._client.is_connected:
                disconnected = True
                break
            await asyncio.sleep(1)

        if disconnected:
            wit_logger.info("Disconnected successfully!")
        else:
            wit_logger.error(
                "TimeoutError: Unable to disconnect device %s", self._client.address
            )
            raise asyncio.TimeoutError(
                f"Unable to disconnect device {self._client.address}"
            )

    async def send_command(self, data: bytes, response: bool = False) -> None:
        """
        Send a command to the device.
        Parameters
        ----------
        data : bytes
            Command to send.
        response : bool
            Whether to wait for a response.
        Returns
        -------
        None
        """
        await self._client.write_gatt_char(self._write_char.uuid, data, response=response)

    async def _notification_handler(self, _, data: bytes) -> None:
        """
        Async handler for BLE notifications from the WT901 sensor.

        Parameters
        ----------
        _ : Any
            Sender of the notification (not used).
        data : bytes
            Raw bytes received from the BLE device.

        Returns
        -------
        None
        """
        if self._running:
            if self._stream_timestamps:
                await self.send_command(
                    Register.data_pack_request_msg(Register.TIMESTAMP.value), response=False)
            try:
                stamp = self._timer.timestamp()
                for msg in self._wit_protocol.decode(stamp, data):
                    self._stream_callback(msg)
            except Exception as e:
                wit_logger.error("Error handling notification: %s", e)
        else:
            wit_logger.warning("Streaming stopped")

    async def stream(self) -> None:
        """
        Start streaming data from the WT901 BLE device.

        This method enables BLE notifications on the device's notify characteristic and
        processes incoming data asynchronously. It uses an asyncio.TaskGroup to handle
        notification processing, allowing for concurrent tasks if needed in the future.

        The notification handler decodes each incoming BLE packet, attaches a host timestamp,
        and invokes the user-provided callback for each decoded message. If timestamp streaming
        is enabled, a request for the device timestamp is sent with each notification.

        Returns
        -------
        None
        """
        async with asyncio.TaskGroup() as tg:
            tg.create_task(
                self._client.start_notify(
                    self._notify_char.uuid, self._notification_handler
                )
            )
