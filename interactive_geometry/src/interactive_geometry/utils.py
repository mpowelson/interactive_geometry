""" Contains utility functions for converting python types to ROS types.

In the future this could be expanded to containt other utilities. That is just how it worked out right now.
"""

from shape_msgs.msg import Mesh
from shape_msgs.msg import MeshTriangle
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

def isNaN(num):
    """
    Returns true if a number is NaN without using numpy or math
    """
    return num != num

def to_shape_msgs_mesh(vertices, faces):
    """
    Converts a set of vertices and faces to a ROS shape_msgs Mesh message
    """
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

def to_triangle_marker_msg(vertices, faces, frame, seq, time):
    """
    Converts a set of vertices and faces to a ROS visualization_msgs triangle Marker message
    """
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

        #Fill header
        marker.header.frame_id = frame
        marker.header.seq = seq
        marker.ns = "interactive_geometry"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 0.8;
        marker.color.r = 0.0;
        marker.color.g = 0.25;
        marker.color.b = 0.75;
#        marker.lifetime = rospy.Duration(0)
#        marker.lifetime = rospy.Time.now()
        marker.frame_locked = True

    return marker


