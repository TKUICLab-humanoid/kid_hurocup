#!/usr/bin/env python
#coding=utf-8
import sys
import rospy
import numpy as np
sys.path.append('/home/iclab/Desktop/adult_hurocup/src/strategy')
from Python_API import Sendmessage
import time

FORWARD_START_SPEED = 4000
BACK_START_SPEED = -2000
FORWARD_MAX_SPEED = 4000
FORWARD_MIN_SPEED = 2000
BACK_MAX_SPEED = -4000

FORWARD_SPEED_ADD = 100
FORWARD_SPEED_SUB = -100
BACK_SPEED_ADD = -100

FORWARD_ORIGIN_THETA = 0
BACK_ORIGIN_THETA = 1

HEAD_Y_HIGH = 2048
HEAD_Y_LOW = 1500

class parameter:
    def __init__(self, speed, theta):
        self.speed = speed
        self.theta = theta

class SP():
    def __init__(self, tku_ros_api):
        self.tku_ros_api = tku_ros_api 
        self.forward = parameter(FORWARD_START_SPEED, FORWARD_ORIGIN_THETA)
        self.backward = parameter(BACK_START_SPEED,BACK_ORIGIN_THETA)
        self.sp_ball = SprintBall(tku_ros_api)
        self.backward_time = 0

        self.init()

    def status_check(self):
        if 110 >= self.sp_ball.center.y >= 40:     #到球前減速
            return 'Decelerating'
        elif self.sp_ball.center.y > 110:   #準備後退
            self.backward_time = time.time()
            return 'Backward'

        return 'Forward'

    def angle_control(self, right_theta, left_theta, straight_theta, original_theta):
        yaw = self.tku_ros_api.imu_value_Yaw
        if yaw > 8:
            self.theta = right_theta    #右轉
        elif yaw < -8:
            self.theta = left_theta     #左轉
        else:
            self.theta = straight_theta     #直走
        self.theta += original_theta
        rospy.logdebug(f'theta = {self.theta}')

    def speed_control(self, speed, speed_variation, speed_limit, status):

        if status == 'Forward':                                                                      #前進取最小值
            speed = min(speed_limit, speed + speed_variation)
        elif status == 'Decelerating' or status == 'Backward':                                 #減速與後退取最大值
            speed = max(speed_limit, speed + speed_variation)

        rospy.logdebug(f'speed = {speed}')
        
        return speed
    
    def init(self):
        self.head_y = 1400
        self.sp_ball.size = 0
        self.forward.speed = FORWARD_START_SPEED
        self.backward.speed = BACK_START_SPEED
        self.tku_ros_api.sendHeadMotor(1, 2048, 50)
        self.tku_ros_api.sendHeadMotor(2, 1400, 50)
        self.tku_ros_api.sendSensorReset(1, 1, 1)
        self.tku_ros_api.drawImageFunction(1, 1, 0, 0, 0 ,0, 0, 0, 255)
        self.tku_ros_api.drawImageFunction(2, 1, 0, 0, 0, 0, 255, 0, 0)
        time.sleep(0.01)

def main():    
    aaaa = rospy.init_node('talker', anonymous=True, log_level=rospy.DEBUG)
    send = Sendmessage()
    r = rospy.Rate(30)
    sp = SP(send)
    first_in = True
    walk_status = 'Forward'
    while not rospy.is_shutdown():                                  
        
        if send.is_start:
            if first_in:
                sp.init()
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                first_in = False
            sp.sp_ball.find()
            
            if walk_status == 'Forward':
                walk_status = sp.status_check()
                sp.angle_control(-2, 2, 0, FORWARD_ORIGIN_THETA)
                sp.forward.speed = sp.speed_control(sp.forward.speed, FORWARD_SPEED_ADD, FORWARD_MAX_SPEED, walk_status)
                send.sendContinuousValue(sp.forward.speed,0,0,sp.theta,0)
            elif walk_status == 'Decelerating':
                walk_status = sp.status_check()
                sp.angle_control(-2, 2, 0, FORWARD_ORIGIN_THETA)
                sp.forward.speed = sp.speed_control(sp.forward.speed, FORWARD_SPEED_SUB, FORWARD_MIN_SPEED, walk_status)  
                send.sendContinuousValue(sp.forward.speed,0,0,sp.theta,0)              
            else:
                sp.angle_control(-2, 2, 0, BACK_ORIGIN_THETA)
                if time.time() - sp.backward_time > 1.5:
                    sp.backward.speed = sp.speed_control(sp.backward.speed, BACK_SPEED_ADD, BACK_MAX_SPEED, walk_status)
                else:
                    sp.backward.speed = sp.speed_control(sp.backward.speed, 0, BACK_MAX_SPEED, walk_status)
                send.sendContinuousValue(sp.backward.speed,0,0,sp.theta,0)

            rospy.loginfo(f'walk_status = {walk_status}')
        else:
            if not first_in:
                walk_status = 'Forward'
                sp.init()
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                first_in = True

        r.sleep()

class Coordinate:
    def __init__(self, x, y):
        self.x, self.y = x, y

    def __add__(self, other):
        return Coordinate(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Coordinate(self.x - other.x, self.y - other.y)

    def __truediv__(self, other):
        return Coordinate(self.x / other, self.y / other)

    def __abs__(self):
        return Coordinate(abs(self.x), abs(self.y))

    def __lt__(self, other):
        return self.x < other.x

class SprintBall:
    def __init__(self, tku_ros_api):
        self.tku_ros_api = tku_ros_api

        self.left_side = ObjectInfo('Blue', tku_ros_api)
        self.right_side = ObjectInfo('Red', tku_ros_api)
        self.size = 0
        self.center = Coordinate(0, 0)

    def find(self):

        find_left = self.left_side.update()
        find_right = self.right_side.update()
        rospy.loginfo(f'find_left = {find_left}, find_right = {find_right}')
        if find_left and find_right:
            rospy.loginfo(f'left_side.center.y = {self.left_side.center.y}, right_side.center.y = {self.right_side.center.y}')
            # rospy.loginfo(f'left_side.size = {self.left_side.size}, right_side.size = {self.right_side.size}')
            
            center_diff = abs(self.left_side.center - self.right_side.center)
            
            if center_diff.y < 5 and (self.left_side.edge_min < self.right_side.edge_min) \
                and (self.left_side.edge_max < self.right_side.edge_max):

                self.tku_ros_api.drawImageFunction(1, 1, *self.left_side.boundary_box, 0, 0, 255)
                self.tku_ros_api.drawImageFunction(2, 1, *self.right_side.boundary_box, 255, 0, 0)
                self.size = self.left_side.size + self.right_side.size
                self.center = (self.left_side.center + self.right_side.center) / 2
                rospy.logdebug(f'ball.center = {self.center.x, self.center.y}')

                return True
        # self.size = 0
        # center_diff = 0
        return False

class ObjectInfo:
    # !! 所有策略都統一的東西，應該要拉另一個 .py 讓所有人都共用，就不用一直寫
    color_dict = {'Orange': 0,
                  'Yellow': 1,
                  'Blue':   2,
                  'Green':  3,
                  'Black':  4,
                  'Red':    5,
                  'White':  6 }
    
    def __init__(self, color, tku_ros_api):

        self.tku_ros_api = tku_ros_api
        self.color = self.color_dict[color]

        self.edge_max = Coordinate(0, 0)
        self.edge_min = Coordinate(0, 0)
        self.center = Coordinate(0, 0)
        self.size = 0

    @property
    def boundary_box(self):
        return (self.edge_min.x, self.edge_max.x, self.edge_min.y, self.edge_max.y)

    def update(self):
        object_idx = None

        width = np.array(self.tku_ros_api.color_mask_subject_Width[self.color])
        height = np.array(self.tku_ros_api.color_mask_subject_Height[self.color])
        object_sizes = np.array(self.tku_ros_api.color_mask_subject_size[self.color])
        
        aspect_ratio = width / height

        for idx, (size, ratio) in enumerate(zip(object_sizes, aspect_ratio)):            
            if 1000 < size < 8000:      #and 0.4 < ratio < 0.6:
                object_idx = idx
        
        if object_idx is None:
            return False

        self.edge_max.x = self.tku_ros_api.color_mask_subject_XMax[self.color][object_idx]
        self.edge_min.x = self.tku_ros_api.color_mask_subject_XMin[self.color][object_idx]
        self.edge_max.y = self.tku_ros_api.color_mask_subject_YMax[self.color][object_idx]
        self.edge_min.y = self.tku_ros_api.color_mask_subject_YMin[self.color][object_idx]
        self.center.x = self.tku_ros_api.color_mask_subject_X[self.color][object_idx]
        self.center.y = self.tku_ros_api.color_mask_subject_Y[self.color][object_idx]
        self.size = self.tku_ros_api.color_mask_subject_size[self.color][object_idx]

        return True

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
