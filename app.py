"""
WT901 BLE Client

This module provides a BleakClientWrapper class to handle BLE communication with the WT901 sensor.

Author: Cristian Troncoso
Email: ctroncoso.ai@gmail.com
License: MIT
"""

import asyncio
import logging
import sys
import time
from dataclasses import dataclass

from attrs import field
from bleak import BleakClient, BleakError, BleakGATTCharacteristic, BLEDevice
from pynput import keyboard

from scan import scan_for_device
from WT901BLECL.protocol import UpdateRate, Gattuidd, WitProtocol
from WT901BLECL.registers import Register
from time_handler import TimeHandler


@dataclass
class BleakClientWrapper:
    """
    Wrapper class for BleakClient to handle BLE communication with the device.
    """

    _client: BleakClient = field(init=True)
    _update_rate: UpdateRate = field(init=True)
    _time: TimeHandler = field(init=False)
    _notify_char: BleakGATTCharacteristic = field(init=False)
    _write_char: BleakGATTCharacteristic = field(init=False)
    _logger: logging.Logger = field(init=False)
    _running: bool = field(init=False)

    def __post_init__(self):
        self.logger = logging.getLogger()
        self._time = TimeHandler(self._update_rate.value)

    @staticmethod
    async def _countdown(seconds: int):
        """Print a countdown to console during calibration."""
        for remaining in range(seconds, 0, -1):
            print(f"\r {remaining}s remaining...", end="", flush=True)
            await asyncio.sleep(1)
        print("\r Calibration complete.")

    async def calibrate_accelerometer(self):
        """
        Performs automatic calibration of the accelerometer.
        Instruct the user to place the device on all six faces for 5 seconds on each face
        (+X, -X, +Y, -Y, +Z, -Z)
        """
        await self.send_command(WitProtocol.start_accelerometer_calibration())
        await self._countdown(5 * 6)
        await self.send_command(WitProtocol.quit_calibration())

    async def calibrate_magnetometer_spherical(self):
        """
        Performs automatic calibration of the magnetometer in spherical mode.
        Instruct the user to rotate the device in all directions (e.g., figure-8 motion).
        """
        await self.send_command(WitProtocol.start_magnetometer_spherical_calibration())
        await self._countdown(30)
        await self.send_command(WitProtocol.quit_calibration())

    async def synchronize(self):
        """
        Synchronize the device time with the host time.
        """
        await self.send_command(WitProtocol.synchronize_instruction(self._time.synchronize()))
    
    async def get_gatts(self):
        """
        Extract notify and write characteristics from the Bleak client

        Returns
        -------
        Notify and write characteristics, in that order
        """
        for service in self._client.services:
            if service.uuid in (
                Gattuidd.WIT_SERVICE_UUID.value,
                Gattuidd.WIT_GATT_PROFILE.value,
            ):
                for characteristic in service.characteristics:
                    if characteristic.uuid == Gattuidd.WIT_READ_CHAR_UUID.value:
                        self._notify_char = characteristic
                        self.logger.info("Notify characteristics available")
                    if characteristic.uuid == Gattuidd.WIT_WRITE_CHAR_UUID.value:
                        self._write_char = characteristic
                        self.logger.info("Write characteristics available")

        if not self._notify_char:
            self.logger.error("Notify characteristics not available")
            raise RuntimeError("Notify characteristics not available")
        if not self._write_char:
            self.logger.error("Write characteristics not available")
            raise RuntimeError("Write characteristics not available")

    def _on_disconnect(self):
        """
        Callback function to be called when the client disconnects.
        """
        self.logger.info("Disconnected successfully!")

    async def connect(self, timeout: int = 10):
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
            self.logger.info("Connected successfully!")
        except asyncio.TimeoutError:
            self.logger.error("Connection attempt timed out.")
        except BleakError as e:
            self.logger.error("Bleak error during connection: %s", e)
        except Exception as e:
            self.logger.error("Other error: %s", e)

        await self.get_gatts()

    async def disconnect(self, timeout_secs: int = 5):
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
        self.logger.info("Disconnecting device %s", self._client.address)
        await self._client.disconnect()
        self._running = False
        disconnected = False
        for _ in range(timeout_secs):
            if not self._client.is_connected:
                disconnected = True
                break
            await asyncio.sleep(1)

        if disconnected:
            self.logger.info("Disconnected successfully!")
        else:
            self.logger.error(
                "TimeoutError: Unable to disconnect device %s", self._client.address
            )
            raise asyncio.TimeoutError(
                f"Unable to disconnect device {self._client.address}"
            )

    async def send_command(self, data: bytes):
        """
        Send a command to the device.
        Parameters
        ----------
        data : bytes
            Command to send.
        Returns
        -------
        None
        """
        await self._client.write_gatt_char(self._write_char.uuid, data)

    async def _notification_handler(self, _, data: bytes):
        """
        Notification handler.
        Parameters
        ----------
        _ : Any
            Not used.
        data : bytes
            Data received from the device.
        Returns
        -------
        None
        """
        if self._running:
            self.logger.info("Notification received: %s", data)
        else:
            self.logger.warning("Streaming stopped")

    async def stream(self):
        """
        Stream data from the device.
        Parameters
        ----------
        None
        Returns
        -------
        None
        """

        await self.send_command(
            Register.set_register_msg(Register.RATE, self._update_rate.value)
        )

        async with asyncio.TaskGroup() as tg:
            tg.create_task(
                self._client.start_notify(
                    self._notify_char.uuid, self._notification_handler
                )
            )

    def stop(self):
        """
        Stop streaming data from the device.
        Parameters
        ----------
        None
        Returns
        -------
        None
        """
        raise Exception("Not implemented")

    def calibrate(self, senser: str):
        """
        Calibrate the device. It calibrates one sensor at a time.
        Parameters
        ----------
        senser : str
            Sensor to calibrate ()
        Returns
        -------
        None
        """
        raise Exception("Not implemented")


async def connect(ble_device: BLEDevice, stop_event: asyncio.Event):
    """
    Connect to the device and start streaming data.
    Parameters
    ----------
    ble_device : BLEDevice
        Device to connect to.
    stop_event : asyncio.Event
        Event to signal interruption.
    Returns
    -------
    None
    """
    bleak_client = BleakClientWrapper(BleakClient(ble_device), UpdateRate.R100HZ.value)
    try:
        await bleak_client.connect()
        await bleak_client.stream()
        # Wait for the stop event
        await stop_event.wait()
    except (KeyboardInterrupt, asyncio.CancelledError):
        bleak_client.logger.info(
            "Interruption detected: disconnecting device and cleaning up..."
        )
        stop_event.set()
    finally:
        await bleak_client.disconnect()


def main():
    """
    Main function.
    """
    logger = logging.getLogger("Main")
    logger.setLevel(logging.INFO)
    mac = "FC:7F:CD:6E:97:25"
    stop_event = asyncio.Event()

    def keyboard_thread(key):
        try:
            if key.char == "q":
                logger.info("Key interrupt 'q' pressed during scan or connection")
                stop_event.set()
                listener.stop()
        except AttributeError:
            logger.debug("Special key pressed: %s", key)

    listener = keyboard.Listener(on_press=keyboard_thread)
    listener.start()

    scan_data = asyncio.run(scan_for_device(mac, 2, stop_event))
    logger.info("Device found: %s", scan_data)
    if scan_data is None:
        logger.error("No device found or scan was interrupted. Exiting.")
        listener.stop()
        return
    try:
        asyncio.run(connect(scan_data.device, stop_event))
    except KeyboardInterrupt:
        logger.info("Key interrupt 'q' pressed - Waiting for disconnection...")
        time.sleep(5)
    listener.stop()


if __name__ == "__main__":
    main()
    sys.exit(0)
