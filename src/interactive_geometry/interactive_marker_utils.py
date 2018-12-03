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

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin


from interactive_geometry.utils import *
from interactive_geometry.ellipsoid_generator import *
import interactive_geometry.global_vars as global_vars

class InteractiveMarkerUtils:
    server = None
    menu_handler = MenuHandler()
    br = None
    counter = 0
    pub_marker = None
    save_file = False

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
        # Run save Mesh here

    # Gets called whenever the user interacts with a marker
    def processFeedback(self, feedback ):
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
                global_vars.a_scale = feedback.pose.position.x
            elif feedback.marker_name == 'moving_b':
                global_vars.b_scale = feedback.pose.position.y
            elif feedback.marker_name == 'moving_c':
                global_vars.c_scale = feedback.pose.position.z
            else:
                # Move mesh_frame to be in the center of the marker
                trans = feedback.pose.position
                rot = feedback.pose.orientation
                self.br.sendTransform( (trans.x, trans.y, trans.z), ( -rot.x, -rot.y, -rot.z, -rot.w), rospy.Time.now(),  "mesh_frame", "base_link" )


        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo( s + ": mouse down" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo( s + ": mouse up" + mp + "." )


        self.server.applyChanges()

    # Used only for the chess piece
    def alignMarker(self, feedback ):
        pose = feedback.pose

        pose.position.x = round(pose.position.x-0.5)+0.5
        pose.position.y = round(pose.position.y-0.5)+0.5

        rospy.loginfo( feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                                                                         str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z) )

        self.server.setPose( feedback.marker_name, pose )
        self.server.applyChanges()

    # Returns a random number between min and max
    def rand(self, min_, max_ ):
        return min_ + self.random()*(max_-min_)

    def makeBox(self, msg ):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.0

        return marker

    def makePoint(self, msg ):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.1
        marker.scale.y = msg.scale * 0.1
        marker.scale.z = msg.scale * 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    # Crates a box control
    def makeBoxControl(self, msg ):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.makeBox(msg) )
        msg.controls.append( control )
        return control

    def saveMarker(self, int_marker ):
      self.server.insert(int_marker, self.processFeedback)


    #####################################################################
    # Marker Creation

    def make1DofMarker(self, position, name):
        print("1D marker")
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "mesh_link"
        int_marker.pose.position = position
        int_marker.name = "marker_" + name
        int_marker.description = "Simple 1-DOF Control"


        # create a grey box marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 1
        box_marker.scale.y = 1
        box_marker.scale.z = 1
        box_marker.color.r = 1.0
        box_marker.color.g = 0.0
        box_marker.color.b = 0.0
        box_marker.color.a = 1.0
#        self.makeBoxControl(int_marker)

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )

        # add the control to the interactive marker
        int_marker.controls.append( box_control )

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
#        if fixed:
#            control.orientation_mode = InteractiveMarkerControl.FIXED

        # add the control to the interactive marker
#        int_marker.controls.append(control);
        int_marker.controls.append(copy.deepcopy(control))

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        self.server.insert(int_marker, self.processFeedback)


    def make6DofMarker(self, fixed, interaction_mode, position, show_6dof = False):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "simple_6dof"
        int_marker.description = ""

        # insert a box
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
#            int_marker.description = "3D Control"
#            if show_6dof:
#              int_marker.description += " + 6-DOF controls"
#            int_marker.description += "\n" + control_modes_dict[interaction_mode]

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

    def makerandomDofMarker(self, position ):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "6dof_self.random_axes"
        int_marker.description = "6-DOF\n(Arbitrary Axes)"

        self.makeBoxControl(int_marker)

        control = InteractiveMarkerControl()

        for i in range(3):
            control.orientation.w = self.rand(-1,1)
            control.orientation.x = self.rand(-1,1)
            control.orientation.y = self.rand(-1,1)
            control.orientation.z = self.rand(-1,1)
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(copy.deepcopy(control))
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(copy.deepcopy(control))

        self.server.insert(int_marker, self.processFeedback)

    def makeViewFacingMarker(self, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "view_facing"
        int_marker.description = "View Facing 6-DOF"

        # make a control that rotates around the view axis
        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1
        control.name = "rotate"
        int_marker.controls.append(control)

        # create a box in the center which should not be view facing,
        # but move in the camera plane.
        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.independent_marker_orientation = True
        control.name = "move"
        control.markers.append( self.makeBox(int_marker) )
        control.always_visible = True
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.processFeedback)

    def makeQuadrocopterMarker(self, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "quadrocopter"
        int_marker.description = "Quadrocopter"

        self.makeBoxControl(int_marker)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        int_marker.controls.append(copy.deepcopy(control))
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.processFeedback)

    def makeChessPieceMarker(self, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "chess_piece"
        int_marker.description = "Chess Piece\n(2D Move + Alignment)"

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(copy.deepcopy(control))

        # make a box which also moves in the plane
        control.markers.append( self.makeBox(int_marker) )
        control.always_visible = True
        int_marker.controls.append(control)

        # we want to use our special callback function
        self.server.insert(int_marker, self.processFeedback)

        # set different callback for POSE_UPDATE feedback
        self.server.setCallback(int_marker.name, self.alignMarker, InteractiveMarkerFeedback.POSE_UPDATE )

    def makePanTiltMarker(self, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "pan_tilt"
        int_marker.description = "Pan / Tilt"

        self.makeBoxControl(int_marker)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.INHERIT
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.processFeedback)

    def makeMenuMarker(self, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "context_menu"
        int_marker.description = "Context Menu\n(Right Click)"

        # make one control using default visuals
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.description="Options"
        control.name = "menu_only_control"
        int_marker.controls.append(copy.deepcopy(control))

        # make one control showing a box
        marker = self.makeBox( int_marker )
        control.markers.append( marker )
        control.always_visible = True
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.processFeedback)
        self.menu_handler.apply( self.server, int_marker.name )

    def makeMovingMarker(self, position, name, axis):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "mesh_frame"
        int_marker.pose.position = position
        int_marker.scale = 1

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

#        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.always_visible = True
        control.markers.append( self.makePoint(int_marker) )
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.processFeedback)
