"""
file: stream.py
Stream data from a WT901BLECL5.0 device.

This module provides a simple interface for streaming data from a
WT901BLECL5.0 device.

Author: Cristian Troncoso
Email: ctroncoso.ai@gmail.com
License: MIT
"""
import asyncio
import json
import logging
import sys
import time
from typing import List, Union

import zmq
from bleak import BleakClient, BLEDevice
from pynput import keyboard

from scan import scan_for_device
from WT901BLECL.message import Msg, MsgType
from WT901BLECL.wit_client import Wit901BLEClient

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

    # Initialize ZMQ PUB socket inside connect
    context = zmq.Context()
    zmq_socket = context.socket(zmq.PUB)
    zmq_socket.bind("tcp://*:5556")

    def streaming_callback(msg):
        # If a single Msg, print it
        if isinstance(msg, Msg):
            print(msg)
            return
        # If a list of Msg, publish as before
        imu_data = {
            "acceleration": None, "angular_velocity": None, "angle": None
        }
        for m in msg:
            val = m.data
            if hasattr(val, "tolist"):
                val = val.tolist()
            if m.type.value == MsgType.ACCELERATION.value:
                imu_data["acceleration"] = val
            elif m.type.value == MsgType.ANGULAR_VELOCITY.value:
                imu_data["angular_velocity"] = val
            elif m.type.value == MsgType.ANGLE.value:
                imu_data["angle"] = val
        output = {"stamp": msg[0].stamp, "IMU": imu_data}
        zmq_socket.send_string(json.dumps(output))

    bleak_client = Wit901BLEClient(
        _client=BleakClient(ble_device),
        _update_rate_hz=50,
        _stream_timestamps=True,
        _stream_callback=streaming_callback
    )
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
                app_logger.info(
                    "Key interrupt 'q' pressed during scan or connection"
                    )
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
        app_logger.info(
            "Key interrupt 'q' pressed - Waiting for disconnection..."
            )
        time.sleep(5)
    listener.stop()


if __name__ == "__main__":
    main()
    sys.exit(0)
