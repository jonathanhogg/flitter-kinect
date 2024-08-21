"""
Kinect scene node
"""

import asyncio
import threading

import os; os.environ['LIBFREENECT2_LOGGER_LEVEL'] = "error"  # noqa

from freenect2 import Device, FrameType
from loguru import logger
import usb.core


class KinectDevice:
    USB_VENDOR = 0x045e
    USB_PRODUCT = 0x02c4
    SharedInstance = None

    @classmethod
    def shared(cls):
        if cls.SharedInstance is None:
            cls.SharedInstance = cls()
        return cls.SharedInstance

    def __init__(self):
        self._device = None
        self._color_frame = None
        self._depth_frame = None
        self._registered_color_frame = None
        self._undistorted_depth_frame = None
        self._points = None
        self._frame_lock = threading.Lock()
        asyncio.create_task(self.run_task())

    async def run_task(self):
        logger.debug("Searching for a connected Kinect (v2) device")
        while True:
            while usb.core.find(idVendor=self.USB_VENDOR, idProduct=self.USB_PRODUCT) is None:
                await asyncio.sleep(0.1)
            try:
                logger.debug("Kinect device found")
                self._device = Device()
                self._device.start(self._process_frame)
            except Exception:
                logger.exception("Error trying to start Kinect device")
                self._device = None
                await asyncio.sleep(1)
                continue
            logger.success("Kinect device started")
            while usb.core.find(idVendor=self.USB_VENDOR, idProduct=self.USB_PRODUCT) is not None:
                await asyncio.sleep(1)
            logger.error("Kinect device disconnected")
            try:
                self._device.stop()
                self._device.close()
            except Exception:
                logger.exception("Failed to correctly close Kinect device")
            self._device = None
            self._color_frame = None
            self._depth_frame = None
            self._registered_color_frame = None
            self._undistorted_depth_frame = None
            self._points = None

    def _process_frame(self, frame_type, frame):
        with self._frame_lock:
            if frame_type == FrameType.Color:
                if self._color_frame is None:
                    logger.debug("Receiving color frames")
                self._color_frame = frame
                self._registered_color_frame = None
            elif frame_type == FrameType.Depth:
                if self._depth_frame is None:
                    logger.debug("Receiving depth frames")
                self._depth_frame = frame
                self._undistorted_depth_frame = None
                self._points = None

    def __enter__(self):
        self._frame_lock.acquire()

    def __exit__(self, *_):
        self._frame_lock.release()

    def get_color_frame(self):
        return self._color_frame

    def get_depth_frame(self):
        return self._depth_frame

    def get_undistorted_depth_frame(self):
        if self._undistorted_depth_frame is None and self._color_frame is not None and self._depth_frame is not None:
            self._undistorted_depth_frame, self._registered_color_frame = self._device.registration.apply(self._color_frame, self._depth_frame)
        return self._undistorted_depth_frame

    def get_registered_color_frame(self):
        if self._registered_color_frame is None and self._color_frame is not None and self._depth_frame is not None:
            self._undistorted_depth_frame, self._registered_color_frame = self._device.registration.apply(self._color_frame, self._depth_frame)
        return self._registered_color_frame

    def get_points(self):
        if self._undistorted_depth_frame is None and self._color_frame is not None and self._depth_frame is not None:
            self._undistorted_depth_frame, self._registered_color_frame = self._device.registration.apply(self._color_frame, self._depth_frame)
        if self._points is None and self._undistorted_depth_frame is not None:
            self._points = self._device.registration.get_points_xyz_array(self._undistorted_depth_frame)
        return self._points
