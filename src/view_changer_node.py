#!/usr/bin/env python3

import rospy
import json
from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import PointStamped, Point, Vector3Stamped, Vector3
from std_msgs.msg import String, Header
from rviz_rc_view.srv import ChangeView, ChangeViewResponse, NextView, NextViewResponse
from rotation_handler import RotationHandler
from input_handler_node import HZ

curr_view_id = -1
views = []

# def camera_placement_creator(view_name):
#     global views
#     for view in views:

#         if view_name == view['name']:
#             placement = view['camera_placement']
#             msg = CameraPlacement()

#             msg.target_frame = placement["target_frame"]
#             msg.time_from_start = rospy.Duration(placement["transition_time"])
#             msg.eye = PointStamped(header=Header(frame_id=placement["eye_frame_id"]), point=Point(*placement["eye_point"]))
#             msg.focus = PointStamped(header=Header(frame_id=placement["focus_frame_id"]), point=Point(*placement["focus_point"]))
#             msg.up = Vector3Stamped(header=Header(frame_id=placement["up_frame_id"]), vector=Vector3(x=0.0, y=0.0, z=1.0))
#             return msg
#     return None

def handle_change_view(req):
    rospy.loginfo("Received request to change view to: %s" % req.view_name)
    succ = rotationHandler.change_view(req.view_name, duration_one_frame=False)
    
    if succ:
        return ChangeViewResponse(True)
    else:
        return ChangeViewResponse(False)
    
def handle_next_view(req):
    global curr_view_id
    global views
    if curr_view_id == -1:
        old_view = None
    else:
        old_view = views[curr_view_id]["name"]

    if req.is_next:
        curr_view_id += 1
    else:
        curr_view_id -= 1
        
    if curr_view_id < 0:
        curr_view_id = len(views) - 1
    if curr_view_id >= len(views):
        curr_view_id = 0

    new_view = views[curr_view_id]["name"]


    rospy.loginfo(f"Received request to set {'next' if req.is_next else 'previous'} view from {old_view} to {new_view}")
    succ = rotationHandler.change_view(new_view, duration_one_frame=False)
    
    if succ:
        return NextViewResponse(True)
    else:
        return NextViewResponse(False)

def view_changer_server():
    rospy.init_node('view_changer_server')
    _ = rospy.Service('change_view', ChangeView, handle_change_view)
    _ = rospy.Service('next_view', NextView, handle_next_view)
    global views
    views = json.load(open(rospy.get_param('~views_path', 'views.json')))["views"]
    global rotationHandler
    rotationHandler = RotationHandler(HZ, views, rospy.get_param('~topic_name', '/rviz/camera_placement'))
    rotationHandler.change_view("3rd Person", duration_one_frame=False)
    global curr_view_id
    curr_view_id = 1
    rospy.loginfo("Ready to change view.")
    rospy.spin()

if __name__ == "__main__":
    view_changer_server()