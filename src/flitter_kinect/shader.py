"""
Kinect scene node
"""

from freenect2 import FrameFormat
from loguru import logger
from mako.template import Template

from flitter.render.window.glsl import TemplateLoader
from flitter.render.window.shaders import Shader

from . import KinectDevice


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
    a = isnan(d) || d < near || d > far ? invalid_value : a * far_value + (1.0 - a) * near_value;
    if (mode == 0) {
        color = vec4(rgb, a);
    } else if (mode == 1) {
        color = vec4(rgb, 1.0);
    } else if (mode == 2) {
        color = vec4(a, a, a, 1.0);
    }
}"""


class Kinect(Shader):
    DEFAULT_FRAGMENT_SOURCE = Template(KINECT_SHADER, lookup=TemplateLoader)
    TEXTURE_PARAMS = {
        FrameFormat.BGRX: (4, 'f1', 'BGR1'),
        FrameFormat.Float: (1, 'f4', 'RRR1'),
        FrameFormat.Gray: (1, 'f1', 'RRR1'),
    }

    def __init__(self, glctx):
        super().__init__(glctx)
        self._color_texture = None
        self._depth_texture = None
        self._kinect_device = KinectDevice.shared()
        self._last_color_frame = None
        self._last_depth_frame = None

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
        self._last_color_frame = None
        self._last_depth_frame = None
        self._kinect_device = None
        self._color_texture = None
        self._depth_texture = None

    async def descend(self, engine, node, **kwargs):
        pass

    def render(self, node, **kwargs):
        mode = {'color': 1, 'depth': 2}.get(node.get('output', 1, str, 'combined'), 0)
        with self._kinect_device:
            if mode == 1:
                depth_frame = None
                color_frame = self._kinect_device.get_color_frame()
            elif mode == 2:
                depth_frame = self._kinect_device.get_depth_frame()
                color_frame = None
            else:
                depth_frame = self._kinect_device.get_undistorted_depth_frame()
                color_frame = self._kinect_device.get_registered_color_frame()
        if mode in (0, 1) and color_frame is not None:
            if self._color_texture is None or self._color_texture.size != (color_frame.width, color_frame.height):
                components, dtype, swizzle = self.TEXTURE_PARAMS[color_frame.format]
                self._color_texture = self.glctx.texture((color_frame.width, color_frame.height), components, dtype=dtype)
                self._color_texture.swizzle = swizzle
                logger.debug("Create {}x{} kinect color texture", color_frame.width, color_frame.height)
            if color_frame is not self._last_color_frame:
                self._color_texture.write(color_frame.data)
            self._last_color_frame = color_frame
        if mode in (0, 2) and depth_frame is not None:
            if self._depth_texture is None or self._depth_texture.size != (depth_frame.width, depth_frame.height):
                components, dtype, swizzle = self.TEXTURE_PARAMS[depth_frame.format]
                self._depth_texture = self.glctx.texture((depth_frame.width, depth_frame.height), components, dtype=dtype)
                self._depth_texture.swizzle = swizzle
                logger.debug("Create {}x{} kinect depth texture", depth_frame.width, depth_frame.height)
            if depth_frame is not self._last_depth_frame:
                self._depth_texture.write(depth_frame.data)
            self._last_depth_frame = depth_frame
        super().render(node, mode=mode, near=500, far=4500, near_value=1, far_value=0, invalid_value=0, flip_x=False, flip_y=False, **kwargs)
