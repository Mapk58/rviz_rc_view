#!/usr/bin/env python3

import rospy
import json
from mavros_msgs.msg import RCIn
from view_changer.srv import NextView  # Импортируйте правильный сервис

HZ = 5

class InputHandler:
    def __init__(self):
        rospy.init_node('input_handler', anonymous=True)
        
        topic_name = rospy.get_param('~topic_name', '/mavros/rc/in')
        self.config = json.load(open(rospy.get_param('~config_path', 'control.json')))
        
        rospy.Subscriber(topic_name, RCIn, self.callback)
        
        # self.change_view_service = rospy.ServiceProxy('change_view', ChangeView)
        
        rospy.Timer(rospy.Duration(1/HZ), self.timer_callback)
        
        self.last_msg = None
        self.queue = {}

    def callback(self, msg):
        self.last_msg = msg

    def timer_callback(self, event):
        if self.last_msg is None:
            return
        
        rc_signal = {}
        for button_name, button_info in self.config.items():
            value = self.last_msg.channels[button_info['channel']]
            result = self.process_signal(button_info, value)
            if result['publish']:
                rc_signal[button_name] = result['value']
                if button_name == "button":
                    self.next_view_service(result['value'])
        try:
            # self.change_view_service(button_name, result['value'])
            print(rc_signal)
        except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

    def process_signal(self, data, value):
        if data['type'] == 'digital':
            logical_value = {data['on'] : True, data['off'] : False}.get(value, 'error')
            if data['channel'] not in self.queue.keys():
                self.queue[data['channel']] = [logical_value]
                return {'publish' : False}
            else: 
                self.queue[data['channel']].append(logical_value)
            if len(self.queue[data['channel']]) > HZ: # one second
                self.queue[data['channel']] = self.queue[data['channel']][1:]
            if self.queue[data['channel']][0] == self.queue[data['channel']][-1] and self.queue[data['channel']][0] != self.queue[data['channel']][-2]:
                self.queue[data['channel']] = [self.queue[data['channel']][-1]]
                return {'publish' : True, 'value' : 2.0}
            if self.queue[data['channel']][0] != self.queue[data['channel']][1]:
                return {'publish' : True, 'value' : 1.0}
            if data['publish_zeroes']:
                return {'publish' : True, 'value' : 0.0}
            else:
                return {'publish' : False} 
        
        elif data['type'] == 'analog':
            if data['center'] - data['threshold'] <= value <= data['center'] + data['threshold']:
                if data['publish_zeroes']:
                    return {'publish' : True, 'value' : 0.0}
                else:
                    return {'publish' : False} 
            if value > data['center']:
                return {'publish' : True, 'value' : (value - (data['center'] + data['threshold']))/(data['max'] - (data['center'] + data['threshold']))}
            if value < data['center']:
                return {'publish' : True, 'value' : (value - (data['center'] - data['threshold']))/(- data['min'] + (data['center'] - data['threshold']))}
        else:
            return {'publish' : False}

    def next_view_service(self, button_info):
        next_view = rospy.ServiceProxy('next_view', NextView)
        if button_info == 1.0:
            next_view(True)
        else:
            next_view(False)

if __name__ == '__main__':
    try:
        handler = InputHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass