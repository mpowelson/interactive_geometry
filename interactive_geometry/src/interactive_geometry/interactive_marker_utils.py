"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

"""
Most of these are not being used. The ones that are are pretty modified.

TODO: Replace all of this with my own code that works a little better for this purpose and is easier to read. I just used the example for prototyping.
"""



# Import ROS modules
import rospy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from tf.broadcaster import TransformBroadcaster

# Import pure Python modules
from random import random
from math import sin
import copy

# Import package modules
import interactive_geometry.global_vars as global_vars

class InteractiveMarkerUtils:
    server = None
    menu_handler = MenuHandler()
    br = None
    counter = 0
    pub_marker = None
    save_file = False
    parent_link = "world"
    mesh_link = "mesh_frame"

    def __init__(self, serv, broadcaster, pub):
        self.pub_marker = pub
        self.server = serv
        self.br = broadcaster
        self.menu_handler.insert( "Export mesh as STL", callback=self.menuCallback1)
        self.menu_handler.insert( "Do something else!", callback=self.menuCallback2 )

    def menuCallback1(self, feedback):
        print("Saving mesh as an STL")
        # Run save Mesh here
        self.save_file=True

    def menuCallback2(self, feedback):
        print("Doing something else")


    # Gets called whenever the user interacts with a marker
    def processFeedback(self, feedback ):
        """Gets called whenever the user interacts with a marker. Also updaes the global_vars to scale geometry

        Mouse info is not used
        """
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo( s + ": button click" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo( s + ": pose changed")
            if feedback.marker_name == 'moving_a':
                # Update the scale of the ellipse
                global_vars.a_scale = feedback.pose.position.x
                # Now update the mirrored marker
                pose = copy.deepcopy(feedback.pose)
                pose.position = Point(-1*global_vars.a_scale, 0, 0)
                self.server.setPose('moving_a_neg', pose)
                self.server.applyChanges()
            elif feedback.marker_name == 'moving_b':
                global_vars.b_scale = feedback.pose.position.y
                # b is one sided, so there is no other marker to update
            elif feedback.marker_name == 'moving_c':
                global_vars.c_scale = feedback.pose.position.z
                pose = copy.deepcopy(feedback.pose)
                pose.position = Point(0, 0, -1*global_vars.c_scale)
                self.server.setPose('moving_c_neg', pose)
                self.server.applyChanges()
            elif feedback.marker_name == 'moving_a_neg':
                global_vars.a_scale = -1*feedback.pose.position.x
                pose = copy.deepcopy(feedback.pose)
                pose.position = Point(global_vars.a_scale, 0, 0)
                self.server.setPose('moving_a', pose)
                self.server.applyChanges()
            elif feedback.marker_name == 'moving_c_neg':
                global_vars.c_scale = -1*feedback.pose.position.z
                pose = copy.deepcopy(feedback.pose)
                pose.position = Point(0, 0, global_vars.c_scale)
                self.server.setPose('moving_c', pose)
                self.server.applyChanges()
            else:
                # This is the 6 DOF marker that moves the whole mesh
                # Move mesh_frame to be in the center of the marker
                trans = feedback.pose.position
                rot = feedback.pose.orientation
                self.br.sendTransform( (trans.x, trans.y, trans.z), ( -rot.x, -rot.y, -rot.z, -rot.w), rospy.Time.now(),  self.mesh_link, self.parent_link )


        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo( s + ": mouse down" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo( s + ": mouse up" + mp + "." )

        self.server.applyChanges()

    def makeBox(self, msg, x_scale=0.5, y_scale=0.5, z_scale=0.5):
        """Makes a large box marker that we are calling a box"""
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * x_scale
        marker.scale.y = msg.scale * y_scale
        marker.scale.z = msg.scale * z_scale
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.0

        return marker

    def makePoint(self, msg, x_scale=0.1, y_scale=0.1, z_scale=0.1):
        """Makes a small box marker that we call a point"""
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * x_scale
        marker.scale.y = msg.scale * y_scale
        marker.scale.z = msg.scale * z_scale
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def makeBoxControl(self, msg ):
        """Make a box marker and then add a control to it"""
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.makeBox(msg) )
        msg.controls.append( control )
        return control

    #####################################################################
    # Marker Creation

    def make6DofMarker(self, fixed, interaction_mode, position, show_6dof = False):
        """
        Makes a marker with a 6 DOF control that is both draggable and has the arrows and rotation ribbon if show_6dof=true
        """
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.parent_link
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "simple_6dof"
        int_marker.description = ""

        # insert a box control that makes it draggable
        self.makeBoxControl(int_marker)
        int_marker.controls[0].interaction_mode = interaction_mode

        if fixed:
            int_marker.name += "_fixed"
            int_marker.description += "\n(fixed orientation)"

        if interaction_mode != InteractiveMarkerControl.NONE:
            control_modes_dict = {
                              InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                              InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                              InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
            int_marker.name += "_" + control_modes_dict[interaction_mode]

        # Add the arrows and rotation ribbons to make it movable that way
        if show_6dof:
            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 1
            control.orientation.y = 0
            control.orientation.z = 0
            control.name = "rotate_x"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 1
            control.orientation.y = 0
            control.orientation.z = 0
            control.name = "move_x"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 1
            control.orientation.z = 0
            control.name = "rotate_z"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 1
            control.orientation.z = 0
            control.name = "move_z"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 0
            control.orientation.z = 1
            control.name = "rotate_y"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 0
            control.orientation.z = 1
            control.name = "move_y"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

        self.server.insert(int_marker, self.processFeedback)
        self.menu_handler.apply( self.server, int_marker.name )


    def makeMovingMarker(self, position, name, axis, scale=1):
        """
        Makes a marker at a point and adds a linear control in one axis

        Used to stretch the ellipsoid, etc.
        """
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.mesh_link
        int_marker.pose.position = position
        int_marker.scale = scale

        int_marker.name = "moving_" + name
        int_marker.description = ""

        control = InteractiveMarkerControl()
        if axis == 0:
            control.orientation.w = 1
            control.orientation.x = 1
            control.orientation.y = 0
            control.orientation.z = 0
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(copy.deepcopy(control))

        if axis == 1:
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 0
            control.orientation.z = 1
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(copy.deepcopy(control))

        if axis == 2:
             control.orientation.w = 1
             control.orientation.x = 0
             control.orientation.y = 1
             control.orientation.z = 0
             control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
             int_marker.controls.append(copy.deepcopy(control))

        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.always_visible = True
        control.markers.append( self.makePoint(int_marker) )
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.processFeedback)
