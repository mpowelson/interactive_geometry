
from shape_msgs.msg import Mesh
from shape_msgs.msg import MeshTriangle
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


import numpy as np


def isNaN(num):
    return num != num

def to_shape_msgs_mesh(vertices, faces):
    mesh_msg = Mesh()
    for face in faces:
        triangle = MeshTriangle()
        triangle.vertex_indices = face
        mesh_msg.triangles.append(triangle)


    for vertex in vertices:
        pt = Point()
        pt.x = vertex[0]
        pt.y = vertex[1]
        pt.z = vertex[2]
        mesh_msg.vertices.append(pt)

    return mesh_msg

def to_triangle_marker_msg(vertices, faces):
    marker = Marker()
    for face in faces:
        # There is probably some Pythonic vectorization of this
        pt = Point()
        face = face.astype(int)
        pt.x = vertices[face[0], 0]
        pt.y = vertices[face[0], 1]
        pt.z = vertices[face[0], 2]
        if( not (isNaN(pt.x) or isNaN(pt.y) or isNaN(pt.z))):
            marker.points.append(pt)

        pt = Point()
        pt.x = vertices[face[1], 0]
        pt.y = vertices[face[1], 1]
        pt.z = vertices[face[1], 2]
        if( not (isNaN(pt.x) or isNaN(pt.y) or isNaN(pt.z))):
            marker.points.append(pt)

        pt = Point()
        pt.x = vertices[face[2], 0]
        pt.y = vertices[face[2], 1]
        pt.z = vertices[face[2], 2]
        if( not (isNaN(pt.x) or isNaN(pt.y) or isNaN(pt.z))):
            marker.points.append(pt)


    return marker


