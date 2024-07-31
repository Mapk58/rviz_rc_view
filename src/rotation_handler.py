#!/usr/bin/env python3

import rospy
from math import sqrt, atan, sin, cos
import json
from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import PointStamped, Point, Vector3Stamped, Vector3
from std_msgs.msg import String, Header

class RotationHandler:

    def __init__(self, hz, config, topic_name): # Maybe viewjson is even better
        self.hz = hz
        self.x = 0
        self.y = 0
        self.z = 0
        self.focus_x = 0
        self.focus_y = 0
        self.focus_z = 0
        self.config = config
        self.topic_name = topic_name
        self.current_config_defaults = self.config[0]
        self.view_name = "Map View"
        self.channel_a = "x"
        self.channel_b = "y"
        self.channel_a_speed = 1.0
        self.channel_b_speed = 1.0
        
    def rotate(self, a_inp, b_inp):
        # Y: positive - down, negative - up
        # Z: positive - right, negative - left
        self.rotate_one_axis(self.channel_a, self.channel_a_speed * (1 / self.hz) * a_inp)
        self.rotate_one_axis(self.channel_b, self.channel_b_speed * (1 / self.hz) * b_inp)
        return self.publish_results()

    def rotate_one_axis(self, type, value):
        if type == "x":
            self.x += value

        if type == "y":
            self.x += value

        if self.channel_a == "zoom":
            r, dy, dz = self.cartesian_to_spherical(self.x, self.y, self.z)
            r += value
            self.x, self.y, self.z = self.spherical_to_cartesian(r, dy, dz)

        if self.channel_a == "pitch":
            r, dy, dz = self.cartesian_to_spherical(self.x + self.focus_x, self.y + self.focus_y, self.z + self.focus_z)
            dz += value
            self.x, self.y, self.z = self.spherical_to_cartesian(r, dy, dz)
            self.x -= self.focus_x
            self.y -= self.focus_y
            self.z -= self.focus_z

        if self.channel_a == "yaw":
            r, dy, dz = self.cartesian_to_spherical(self.x + self.focus_x, self.y + self.focus_y, self.z + self.focus_z)
            dz += value
            self.x, self.y, self.z = self.spherical_to_cartesian(r, dy, dz)
            self.x -= self.focus_x
            self.y -= self.focus_y
            self.z -= self.focus_z
       
    def camera_placement_creator(self, duration_one_frame = True):
        placement = self.current_config_defaults['camera_placement']
        msg = CameraPlacement()

        msg.target_frame = placement["target_frame"]
        duration = 1/self.hz
        if duration_one_frame:
            duration = self.current_config_defaults["camera_placement"]["transition_time"]
        msg.time_from_start = rospy.Duration(duration)
        msg.eye = PointStamped(header=Header(frame_id=placement["eye_frame_id"]), point=Point(self.x, self.y, self.z))
        msg.focus = PointStamped(header=Header(frame_id=placement["focus_frame_id"]), point=Point(self.focus_x, self.focus_y, self.focus_z))
        msg.up = Vector3Stamped(header=Header(frame_id=placement["up_frame_id"]), vector=Vector3(x=0.0, y=0.0, z=1.0))
        return msg
        
    def publish_results(self, duration_one_frame = True):
        topic_name = self.topic_name
        pub = rospy.Publisher(topic_name, CameraPlacement, queue_size=10)

        msg = self.camera_placement_creator(duration_one_frame = True)
        
        if msg is not None:
            pub.publish(msg)
            return True
        else:
            rospy.logwarn("View name is not recognized.")
            return False

    def change_view(self, view_name, duration_one_frame = True):
        for view_coord in self.config:
            if view_coord["name"] == view_name:
                self.view_name = view_name
                self.current_config_defaults = view_coord
                coords = view_coord["camera_placement"]["eye_point"]
                self.x = coords[0]
                self.y = coords[1]
                self.z = coords[2]
                focus_coords = view_coord["camera_placement"]["focus_point"]
                self.focus_x = focus_coords[0]
                self.focus_y = focus_coords[1]
                self.focus_z = focus_coords[2]
                motion_type_config = view_coord["control"]
                for motion in motion_type_config:
                    if motion["some_system_data"] == "/mavros/control/channel_a":
                        self.channel_a = motion["motion_type"]
                        self.channel_a_speed = motion["motion_speed"]
                    if motion["some_system_data"] == "/mavros/control/channel_b":
                        self.channel_b = motion["motion_type"]
                        self.channel_b_speed = motion["motion_speed"]
        return self.publish_results(duration_one_frame = True)

    def spherical_to_cartesian(r, dy, dz):
        x = r * sin(dy) * cos(dz)
        y = r * sin(dy) * sin(dz)
        z = r * cos(dy)
        return x, y, z

    def cartesian_to_spherical(x, y, z):
        r = sqrt(x*x+y*y+z*z)
        dy = atan(sqrt(x*x+y*y)/z)
        dz = atan(y/x)
        return r, dy, dz
    
    def reset_view(self):
        self.change_view(self.view_name)

    