"""
Kinect 3D mesh
"""

import cython
import numpy as np

from libc.math cimport isnan
from libc.stdint cimport int32_t, uint64_t, int64_t

from flitter.model cimport Node, HASH_START, HASH_UPDATE, HASH_STRING, double_long
from flitter.render.window.models cimport Model, fill_in_normals

from . import KinectDevice


cdef uint64_t KINECT = HASH_UPDATE(HASH_START, HASH_STRING('kinect'))


cdef class Kinect(Model):
    cdef uint64_t average_count
    cdef float tear_distance
    cdef float near_distance
    cdef float far_distance
    cdef int64_t index
    cdef object device
    cdef list frames
    cdef object last_points
    cdef object uv

    def __init__(self, id):
        super().__init__(id)
        self.device = KinectDevice.shared()
        self.frames = []

    @staticmethod
    def from_node(Node node):
        cdef uint64_t average_count = max(1, node.get_int('average', 1))
        cdef float tear_distance = max(0, node.get_float('tear', 0))
        cdef float near_distance = max(0, node.get_float('near', 0.5))
        cdef float far_distance = max(0, node.get_float('far', 4.5))
        cdef uint64_t id = HASH_UPDATE(KINECT, average_count)
        id = HASH_UPDATE(id, double_long(f=tear_distance).l)
        id = HASH_UPDATE(id, double_long(f=near_distance).l)
        id = HASH_UPDATE(id, double_long(f=far_distance).l)
        cdef Kinect model
        if (model := Model.by_id(id)) is not None:
            return model
        model = Kinect(id)
        model.average_count = average_count
        model.tear_distance = tear_distance
        model.near_distance = near_distance
        model.far_distance = far_distance
        return model

    @property
    def name(self):
        return f'!kinect({self.average_count}, {self.tear_distance:g}, {self.near_distance:g}, {self.far_distance:g})'

    cpdef bint is_manifold(self):
        return False

    cpdef void check_for_changes(self):
        with self.device:
            points = self.device.get_points()
        if points is not self.last_points:
            if self.index == len(self.frames):
                self.frames.append(points)
            else:
                self.frames[self.index] = points
            self.index = (self.index + 1) % self.average_count
            self.last_points = points
            self.invalidate()

    @cython.cdivision(True)
    @cython.boundscheck(False)
    @cython.wraparound(False)
    cdef object averaged_points(self):
        cdef int32_t n=len(self.frames)
        if n == 0:
            return None
        points = self.frames[0]
        cdef int32_t rows = points.shape[0]
        cdef int32_t cols = points.shape[1]
        cdef int32_t row, col, i, count
        averaged = np.full((rows, cols, 3), np.nan, np.float32)
        cdef float[:, :, :] A = averaged
        cdef const float[:, :, :] B
        cdef int32_t[:, :] counts = np.zeros((rows, cols), np.int32)
        cdef float near=-self.near_distance, far=-self.far_distance
        for i in range(n):
            B = self.frames[i]
            for row in range(rows):
                for col in range(cols):
                    Az = A[row][col][2]
                    Bz = B[row][col][2]
                    if not isnan(Bz) and near >= Bz >= far:
                        if isnan(Az):
                            A[row][col][0] = B[row][col][0]
                            A[row][col][1] = B[row][col][1]
                            A[row][col][2] = Bz
                            counts[row][col] = 1
                        else:
                            A[row][col][0] = A[row][col][0] + B[row][col][0]
                            A[row][col][1] = A[row][col][1] + B[row][col][1]
                            A[row][col][2] = A[row][col][2] + Bz
                            counts[row][col] += 1
        for row in range(rows):
            for col in range(cols):
                count = counts[row][col]
                if count > 1:
                    A[row][col][0] /= count
                    A[row][col][1] /= count
                    A[row][col][2] /= count
        return averaged

    @cython.cdivision(True)
    @cython.boundscheck(False)
    @cython.wraparound(False)
    cpdef tuple build_arrays(self):
        points = self.averaged_points()
        if points is None:
            return None
        cdef int32_t rows = points.shape[0]
        cdef int32_t cols = points.shape[1]
        cdef int32_t row, col, m=rows*cols, n=0, a, b, c, d
        vertices_array = np.zeros((m, 8), np.float32)
        vertices_array[:, :3] = points.reshape((-1, 3))
        cdef const float[:, :] vertices = vertices_array
        faces_array = np.empty(((rows - 1) * (cols - 1) * 2, 3), np.int32)
        cdef int32_t[:, :] faces = faces_array
        cdef float f, g, z1, z2, tear=self.tear_distance
        for row in range(rows):
            for col in range(cols):
                a = row * cols + col
                if row < rows-1 and col < cols-1:
                    b = a + 1
                    c = a + cols
                    d = c + 1
                    if not isnan(vertices[b][2]) and not isnan(vertices[c][2]):
                        if not isnan(vertices[a][2]):
                            z1 = min(vertices[a][2], min(vertices[b][2], vertices[c][2]))
                            z2 = max(vertices[a][2], max(vertices[b][2], vertices[c][2]))
                            if tear == 0 or z2 - z1 < tear:
                                faces[n][0], faces[n][1], faces[n][2] = a, b, c
                                n += 1
                        if not isnan(vertices[d][2]):
                            z1 = min(vertices[b][2], min(vertices[d][2], vertices[c][2]))
                            z2 = max(vertices[b][2], max(vertices[d][2], vertices[c][2]))
                            if tear == 0 or z2 - z1 < tear:
                                faces[n][0], faces[n][1], faces[n][2] = b, d, c
                                n += 1
        if n == 0:
            return None
        cdef float[:, :] uv
        if self.uv is None:
            self.uv = np.empty((m, 2), np.float32)
            uv = self.uv
            f = 1.0 / cols
            a = 0
            for row in range(rows):
                g = 1 - row / <float>rows
                for col in range(cols):
                    uv[a][0] = col * f
                    uv[a][1] = g
                    a += 1
        vertices_array[:, 6:8] = self.uv
        faces_array = faces_array[:n]
        fill_in_normals(vertices_array, faces_array)
        return vertices_array, faces_array
