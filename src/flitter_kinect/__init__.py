"""
Kinect scene node

pip3 install git+https://github.com/chris-castillo/freenect2-python.git
"""

import threading
import os; os.environ['LIBFREENECT2_LOGGER_LEVEL'] = "error"  # noqa

from freenect2 import Device, NoDeviceError, FrameType, FrameFormat
from loguru import logger
from mako.template import Template
import usb.core

from flitter.render.window.glsl import TemplateLoader
from flitter.render.window.shaders import Shader


KINECT_SHADER = """
${HEADER}

in vec2 coord;
out vec4 color;

uniform sampler2D color_frame;
uniform sampler2D depth_frame;
uniform int mode;
uniform float near;
uniform float far;
uniform float near_value;
uniform float far_value;
uniform float invalid_value;
uniform bool flip_x;
uniform bool flip_y;

void main() {
    vec2 inverted = vec2(flip_x ? 1.0 - coord.x : coord.x, flip_y ? coord.y : 1.0 - coord.y);
    vec3 rgb = texture(color_frame, inverted).rgb;
    float d = texture(depth_frame, inverted).r;
    float a = (d - near) / (far - near);
    a = d < near || d > far ? invalid_value : a * far_value + (1.0 - a) * near_value;
    if (mode == 0) {
        color = vec4(rgb, a);
    } else if (mode == 1) {
        color = vec4(rgb, 1.0);
    } else if (mode == 2) {
        color = vec4(a, a, a, 1.0);
    }
}"""


class Kinect(Shader):
    USB_VENDOR = 0x045e
    USB_PRODUCT = 0x02c4
    DEFAULT_FRAGMENT_SOURCE = Template(KINECT_SHADER, lookup=TemplateLoader)
    TEXTURE_PARAMS = {
        FrameFormat.BGRX: (4, 'f1', 'BGR1'),
        FrameFormat.Float: (1, 'f4', 'RRR1'),
        FrameFormat.Gray: (1, 'f1', 'RRR1'),
    }

    Device = None

    def __init__(self, glctx):
        super().__init__(glctx)
        self._color_texture = None
        self._depth_texture = None
        self._color_frame = None
        self._depth_frame = None
        self._frame_lock = threading.Lock()
        self._running = False

    @property
    def child_textures(self):
        textures = {}
        if self._color_texture is not None:
            textures['color_frame'] = self._color_texture
        if self._depth_texture is not None:
            textures['depth_frame'] = self._depth_texture
        return textures

    def release(self):
        super().release()
        if Kinect.Device is not None:
            logger.debug("Stopping Kinect v2 device")
            Kinect.Device.stop()
        self._color_texture = None
        self._depth_texture = None
        self._color_frame = None
        self._depth_frame = None
        self._running = False

    def create(self, engine, node, resized, **kwargs):
        super().create(engine, node, resized, **kwargs)
        if usb.core.find(idVendor=self.USB_VENDOR, idProduct=self.USB_PRODUCT) is not None:
            if Kinect.Device is None:
                try:
                    Kinect.Device = Device()
                except Exception:
                    logger.exception("Error trying to initialize Kinect v2 device")
            if not self._running and Kinect.Device is not None:
                logger.debug("Starting Kinect v2 device")
                Kinect.Device.start(self._process_frame)
                self._running = True
        elif Kinect.Device is not None:
            logger.warning("Kinect device disconnected")
            Kinect.Device.stop()
            Kinect.Device = None
            self._running = False

    def _process_frame(self, frame_type, frame):
        with self._frame_lock:
            if frame_type == FrameType.Color:
                if self._color_frame is None:
                    logger.debug("Receiving color frames")
                self._color_frame = frame
            elif frame_type == FrameType.Depth:
                if self._depth_frame is None:
                    logger.debug("Receiving depth frames")
                self._depth_frame = frame

    async def descend(self, engine, node, **kwargs):
        pass

    def render(self, node, **kwargs):
        with self._frame_lock:
            color_frame = self._color_frame
            depth_frame = self._depth_frame
        mode = {'color': 1, 'depth': 2}.get(node.get('output', 1, str, 'combined').lower(), 0)
        if mode == 0 and color_frame is not None and depth_frame is not None:
            depth_frame, color_frame = self.Device.registration.apply(color_frame, depth_frame)
        if mode in (0, 1) and color_frame is not None:
            if self._color_texture is None or self._color_texture.size != (color_frame.width, color_frame.height):
                components, dtype, swizzle = self.TEXTURE_PARAMS[color_frame.format]
                self._color_texture = self.glctx.texture((color_frame.width, color_frame.height), components, dtype=dtype)
                self._color_texture.swizzle = swizzle
                logger.debug("Create {}x{} kinect color texture", color_frame.width, color_frame.height)
            self._color_texture.write(color_frame.data)
        if mode in (0, 2) and depth_frame is not None:
            if self._depth_texture is None or self._depth_texture.size != (depth_frame.width, depth_frame.height):
                components, dtype, swizzle = self.TEXTURE_PARAMS[depth_frame.format]
                self._depth_texture = self.glctx.texture((depth_frame.width, depth_frame.height), components, dtype=dtype)
                self._depth_texture.swizzle = swizzle
                logger.debug("Create {}x{} kinect depth texture", depth_frame.width, depth_frame.height)
            self._depth_texture.write(depth_frame.data)
        super().render(node, mode=mode, near=500, far=4500, near_value=1, far_value=0, invalid_value=0, flip_x=False, flip_y=False, **kwargs)
