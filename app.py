
from bleak import BLEDevice, BleakClient
import asyncio
from wit_901_ble_client import Wit901BLEClient
from pynput import keyboard
from scan import scan_for_device
import time
import sys
import logging

app_logger = logging.getLogger("app")

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
    bleak_client = Wit901BLEClient(BleakClient(ble_device), 50)
    try:
        await bleak_client.connect()
        await bleak_client.stream()
        # Wait for the stop event
        await stop_event.wait()
    except (KeyboardInterrupt, asyncio.CancelledError):
        app_logger.info(
            "Interruption detected: disconnecting device and cleaning up..."
        )
        stop_event.set()
    finally:
        await bleak_client.disconnect()


def main():
    """
    Main function.
    """
    app_logger.setLevel(logging.INFO)
    mac = "FC:7F:CD:6E:97:25"
    stop_event = asyncio.Event()

    def keyboard_thread(key):
        try:
            if key.char == "q":
                app_logger.info("Key interrupt 'q' pressed during scan or connection")
                stop_event.set()
                listener.stop()
        except AttributeError:
            app_logger.debug("Special key pressed: %s", key)

    listener = keyboard.Listener(on_press=keyboard_thread)
    listener.start()

    scan_data = asyncio.run(scan_for_device(mac, 2, stop_event))
    app_logger.info("Device found: %s", scan_data)
    if scan_data is None:
        app_logger.error("No device found or scan was interrupted. Exiting.")
        listener.stop()
        return
    try:
        asyncio.run(connect(scan_data.device, stop_event))
    except KeyboardInterrupt:
        app_logger.info("Key interrupt 'q' pressed - Waiting for disconnection...")
        time.sleep(5)
    listener.stop()


if __name__ == "__main__":
    main()
    sys.exit(0)
