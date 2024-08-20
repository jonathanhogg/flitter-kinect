
# Flitter Kinect

[![CI lint](https://github.com/jonathanhogg/flitter-kinect/actions/workflows/ci-lint.yml/badge.svg?)](https://github.com/jonathanhogg/flitter-kinect/actions/workflows/ci-lint.yml)

This package provides a plugin for using a Kinect v2 device in
[Flitter](https://flitter.readthedocs.io/).

The additional nodes provided by this plugin are:

# `!window`/`!offscreen` tree nodes

## `!kinect`

This provides access to the raw frames from the Kinect as an image. In addition
to the standard attributes (`size`, etc.), it supports the following:

`output=` [ `:color` | `:depth` | :`combined` ]
: Whether to output frames from the color camera, the depth camera, or a
combined image. The default is `:combined`.

`flip_x=` [ `true` | `false` ]
: Whether to flip the image horizontally. Default is `false`.

`flip_y=` [ `true` | `false` ]
: Whether to flip the image vertically. Default is `false`.

`near=`*DISTANCE*
: The near clip plane of the depth camera, in millimetres. Depths smaller than
this will be considered to be invalid. Default is `500`.

`far=`*DISTANCE*
: The far clip plane of the depth camera, in millimetres. Depths larger than
this will be considered to be invalid. Default is `4500`.

`near_value=`*VALUE*
: The output channel value to use for distances at `near`. Default is `1`.

`far_value=`*VALUE*
: The output channel value to use for distances at `far`. Default is `0`.

`invalid_value=`*VALUE*
: The value to use for the depth channel if the distance is nearer than `near`
or further than `far`. Default is `0`.

In `:depth` output mode, the result will be a 512x424 image with each of the
RGB channels set to the distance through that pixel and the A channel set to
`1`. Distances in the range `near` to `far` will be mapped linearly to grey
values between `near_value` and `far_value`, with the value being
`invalid_value` for distances outside of that distance range.

In `:color` output mode, the result image will be the 1920x1080 color frame as
received from the Kinect visible light camera.

For combined output, the color image will be cropped and aligned to the depth
camera's view (512x424). The A channel will contain the depth value, as
described above, and the RGB channels will contain the matching color from the
visible light camera. The RGB channels are *not* premultiplied by the A channel.
