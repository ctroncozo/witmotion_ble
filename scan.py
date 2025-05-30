"""
file: scan.py
BLE Device Scanner for WT901BLECL and Related Sensors

This module scans for nearby Bluetooth Low Energy (BLE) devices using the Bleak library.
It allows scanning for a fixed timeout and includes a convenience function to look up a device
by MAC address.

Author: Cristian Troncoso
Email: ctroncoso.ai@gmail.com
License: MIT
"""

import asyncio
import logging
import sys
import time
from dataclasses import dataclass
from typing import Optional

from attrs import field
from bleak import BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

scan_logger = logging.getLogger("ScanLogger")


@dataclass(frozen=True, kw_only=True)
class ScannData:
    """Dataclass to store scan results."""
    device: BLEDevice = field(init=True)
    rssi: int = field(init=True)
    advertisement_data: AdvertisementData = field(init=True)


async def scan(timeout: int = 5,
               stop_event: Optional[asyncio.Event] = None) -> dict[str, ScannData]:
    """
    Scan for BLE devices using Bleak, returning devices found within a time or packet limit.

    Parameters
    ----------
    timeout : int
        Maximum time (in seconds) to scan for devices.
    stop_event : Optional[asyncio.Event]
        Event to allow cooperative cancellation of scan.

    Returns
    -------
    dict[str, ScannData]
        A dictionary mapping device MAC addresses to ScannData objects.
    """

    devices_found_dic: dict[str, ScannData] = {}
    async with BleakScanner() as scanner:
        scan_logger.info("Scanning...")

        start = time.time()
        async for ble_device, data in scanner.advertisement_data():
            if stop_event and stop_event.is_set():
                scan_logger.info("Scan stopped by external event.")
                break
            scan_logger.info("%s with %s", ble_device, data)
            if time.time() - start > timeout:
                break
            if ble_device.address not in devices_found_dic:
                devices_found_dic[ble_device.address] = ScannData(
                    device=ble_device, rssi=data.rssi, advertisement_data=data
                )

    return devices_found_dic


async def scan_runner(timeout: int = 5,
                      stop_event: Optional[asyncio.Event] = None) -> dict[str, ScannData]:
    """
    Run the scan coroutine and return the scanned devices.

    Parameters
    ----------
    timeout : int
        Maximum time (in seconds) to scan for devices.
    stop_event : Optional[asyncio.Event]
        Event to allow cooperative cancellation of scan.

    Returns
    -------
    dict[str, ScannData]
        A dictionary of discovered BLE devices.
    """
    try:
        coro = [scan(timeout, stop_event)]
        values = await asyncio.gather(*coro)
        return values[0]
    except (KeyboardInterrupt, asyncio.CancelledError):
        scan_logger.info("Scan interrupted by user or cancellation.")
        return {}


async def scan_for_device(mac: str, timeout: int = 5,
        stop_event: Optional[asyncio.Event] = None) -> Optional[ScannData]:
    """
    Scan for BLE devices using Bleak, returning devices found within a time or packet limit.

    Parameters
    ----------
    mac : str
        MAC address of the device to look up.
    timeout : int
        Maximum time (in seconds) to scan for devices.
    stop_event : Optional[asyncio.Event]
        Event to allow cooperative cancellation of scan.

    Returns
    -------
    Optional[ScannData]
        The discovered BLE device with the specified MAC address, or None if not found.
    """
    logging.basicConfig(level=logging.INFO)
    try:
        devices = await scan_runner(timeout, stop_event)
        for d in devices.items():
            if d[0] == mac:
                return d[1]
        return None
    except (KeyboardInterrupt, asyncio.CancelledError):
        scan_logger.info("Scan for device interrupted by user or cancellation.")
        return None


def main():
    """
    Main function to run the scan.
    """
    logging.basicConfig(level=logging.INFO)
    device = asyncio.run(scan_for_device("FC:7F:CD:6E:97:25", 10))
    print(device)


if __name__ == "__main__":
    main()
    sys.exit(0)
