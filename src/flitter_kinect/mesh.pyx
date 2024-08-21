"""
Kinect 3D mesh
"""

import cython
import numpy as np
import trimesh

from libc.math cimport isnan
from libc.stdint cimport uint32_t

from flitter.render.window.models cimport Model

from . import KinectDevice


# Monkey patch for freenect2:
np.product = np.prod


cdef class Kinect(Model):
    cdef object device
    cdef object points
    cdef object model

    def __init__(self, name):
        super().__init__(name)
        self.device = KinectDevice.shared()
        self.points = None
        self.model = None

    @staticmethod
    def from_node(node):
        if (model := Model.by_name('!kinect')) is not None:
            return model
        model = Kinect('!kinect')
        return model

    cpdef bint is_manifold(self):
        return False

    cpdef void check_for_changes(self):
        with self.device:
            points = self.device.get_points()
        if points is not self.points:
            self.invalidate()
            self.points = points

    @cython.cdivision(True)
    @cython.boundscheck(False)
    @cython.wraparound(False)
    cpdef object build_trimesh(self):
        if self.points is None:
            return None
        cdef uint32_t rows = self.points.shape[0]
        cdef uint32_t cols = self.points.shape[1]
        vertices_array = self.points.reshape((-1, 3))
        cdef float[:, :] vertices = vertices_array
        faces_array = np.empty(((rows - 1) * (cols - 1) * 2, 3), np.uint32)
        cdef uint32_t[:, :] faces = faces_array
        cdef uint32_t row, col, i=0, a, b, c, d
        for row in range(rows):
            for col in range(cols):
                if row < rows-1 and col < cols-1:
                    a = row * cols + col
                    b = a + 1
                    c = a + cols
                    d = c + 1
                    if not isnan(vertices[b][0]) and not isnan(vertices[b][1]) and not isnan(vertices[b][2]) \
                            and not isnan(vertices[c][0]) and not isnan(vertices[c][1]) and not isnan(vertices[c][2]):
                        if not isnan(vertices[a][0]) and not isnan(vertices[a][1]) and not isnan(vertices[a][2]):
                            faces[i][0], faces[i][1], faces[i][2] = a, b, c
                            i += 1
                        if not isnan(vertices[d][0]) and not isnan(vertices[d][1]) and not isnan(vertices[d][2]):
                            faces[i][0], faces[i][1], faces[i][2] = b, d, c
                            i += 1
        if i == 0:
            return None
        if self.model is not None:
            self.model.vertices = vertices_array
            self.model.faces = faces_array[:i]
        else:
            self.model = trimesh.Trimesh(vertices=vertices_array, faces=faces_array[:i], process=False, validate=False)
        return self.model
