"""
Kinect 3D mesh
"""

import cython
import numpy as np
import trimesh

from libc.math cimport isnan
from libc.stdint cimport uint32_t, int64_t

from flitter.model cimport Node
from flitter.render.window.models cimport Model

from . import KinectDevice


# Monkey patch for freenect2:
np.product = np.prod


cdef class Kinect(Model):
    cdef bint average_count
    cdef float tear_distance
    cdef float near_distance
    cdef float far_distance
    cdef int64_t index
    cdef object device
    cdef list frames
    cdef object last_points
    cdef object uv
    cdef object model

    def __init__(self, name):
        super().__init__(name)
        self.device = KinectDevice.shared()
        self.frames = []

    @staticmethod
    def from_node(Node node):
        cdef int64_t average_count = max(1, node.get_int('average', 1))
        cdef float tear_distance = max(0, node.get_float('tear', 0))
        cdef float near_distance = max(0, node.get_float('near', 0.5))
        cdef float far_distance = max(0, node.get_float('far', 5))
        cdef str name = f'!kinect({average_count}, {tear_distance:g}, {near_distance:g}, {far_distance:g})'
        cdef Kinect model
        if (model := Model.by_name(name)) is not None:
            return model
        model = Kinect(name)
        model.average_count = average_count
        model.tear_distance = tear_distance
        model.near_distance = near_distance
        model.far_distance = far_distance
        return model

    cpdef bint is_manifold(self):
        return False

    cpdef void check_for_changes(self):
        with self.device:
            points = self.device.get_points()
        if points is not self.last_points:
            if len(self.frames) < self.average_count:
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
        cdef int64_t n=len(self.frames)
        if n == 0:
            return None
        points = self.frames[0]
        cdef uint32_t rows = points.shape[0]
        cdef uint32_t cols = points.shape[1]
        cdef uint32_t row, col, i, count
        averaged = np.full((rows, cols, 3), np.nan, np.float32)
        cdef float[:, :, :] A = averaged
        cdef const float[:, :, :] B
        cdef uint32_t[:, :] counts = np.zeros((rows, cols), np.uint32)
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
    cpdef object build_trimesh(self):
        points = self.averaged_points()
        if points is None:
            return None
        cdef uint32_t rows = points.shape[0]
        cdef uint32_t cols = points.shape[1]
        cdef uint32_t row, col, i, m=rows*cols, n=0, a, b, c, d
        vertices_array = points.reshape((-1, 3))
        cdef const float[:, :] vertices = vertices_array
        normals_array = np.zeros((m, 3), np.float32)
        cdef float[:, :] normals = normals_array
        cdef uint32_t[:] normal_counts = np.zeros((m,), np.uint32)
        faces_array = np.empty(((rows - 1) * (cols - 1) * 2, 3), np.uint32)
        cdef uint32_t[:, :] faces = faces_array
        cdef float Ax, Ay, Az, Bx, By, Bz, Nx, Ny, Nz, z1, z2, tear=self.tear_distance
        for row in range(rows):
            for col in range(cols):
                a = row * cols + col
                if row < rows-1 and col < cols-1:
                    b = a + 1
                    c = a + cols
                    d = c + 1
                    if not isnan(vertices[b][2]) and not isnan(vertices[c][2]):
                        Ax, Ay, Az = vertices[c][0]-vertices[b][0], vertices[c][1]-vertices[b][1], vertices[c][2]-vertices[b][2]
                        if not isnan(vertices[a][2]):
                            z1 = min(vertices[a][2], min(vertices[b][2], vertices[c][2]))
                            z2 = max(vertices[a][2], max(vertices[b][2], vertices[c][2]))
                            if tear == 0 or z2 - z1 < tear:
                                Bx, By, Bz = vertices[a][0]-vertices[b][0], vertices[a][1]-vertices[b][1], vertices[a][2]-vertices[b][2]
                                Nx, Ny, Nz = Ay*Bz-Az*By, Az*Bx-Ax*Bz, Ax*By-Ay*Bx
                                normals[a][0] += Nx
                                normals[a][1] += Ny
                                normals[a][2] += Nz
                                normal_counts[a] += 1
                                normals[b][0] += Nx
                                normals[b][1] += Ny
                                normals[b][2] += Nz
                                normal_counts[b] += 1
                                normals[c][0] += Nx
                                normals[c][1] += Ny
                                normals[c][2] += Nz
                                normal_counts[c] += 1
                                faces[n][0], faces[n][1], faces[n][2] = a, b, c
                                n += 1
                        if not isnan(vertices[d][2]):
                            z1 = min(vertices[b][2], min(vertices[d][2], vertices[c][2]))
                            z2 = max(vertices[b][2], max(vertices[d][2], vertices[c][2]))
                            if tear == 0 or z2 - z1 < tear:
                                Bx, By, Bz = vertices[b][0]-vertices[d][0], vertices[b][1]-vertices[d][1], vertices[b][2]-vertices[d][2]
                                Nx, Ny, Nz = Ay*Bz-Az*By, Az*Bx-Ax*Bz, Ax*By-Ay*Bx
                                normals[b][0] += Nx
                                normals[b][1] += Ny
                                normals[b][2] += Nz
                                normal_counts[b] += 1
                                normals[d][0] += Nx
                                normals[d][1] += Ny
                                normals[d][2] += Nz
                                normal_counts[d] += 1
                                normals[c][0] += Nx
                                normals[c][1] += Ny
                                normals[c][2] += Nz
                                normal_counts[c] += 1
                                faces[n][0], faces[n][1], faces[n][2] = b, d, c
                                n += 1
        for i in range(m):
            if (c := normal_counts[i]) > 1:
                normals[i][0] /= c
                normals[i][1] /= c
                normals[i][2] /= c
        if n == 0:
            return None
        cdef float[:, :] uv
        if self.uv is None:
            self.uv = np.empty((m, 2), np.float32)
            uv = self.uv
            for row in range(rows):
                for col in range(cols):
                    a = row * cols + col
                    uv[a][0] = col / <float>cols
                    uv[a][1] = 1 - row / <float>rows
        if self.model is not None:
            self.model.vertices = vertices_array
            self.model.faces = faces_array[:n]
            self.model.vertex_normals = normals_array
        else:
            visual = trimesh.visual.texture.TextureVisuals(uv=self.uv)
            self.model = trimesh.Trimesh(vertices=vertices_array, vertex_normals=normals_array, faces=faces_array[:n],
                                         visual=visual, process=False, validate=False)
        return self.model
