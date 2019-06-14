#!/usr/bin/env python3
import rospy
import math
from nubot_communication import Nubot_communication

class Strategy(object):
    def __init__(self, robot_number, robot_id):
        self.ros_communication = Nubot_communication(robot_number, robot_id)

    def nubot_strategy(self, robot_id):
        self.init_flag = self.ros_communication.init_flag1 and self.ros_communication.init_flag2
        if self.init_flag:
            #///////////////////////////////////////right_goal_ang///////////////////////////////////////
            alpha = math.radians(self.ros_communication.ball_ang - self.ros_communication.right_goal_ang)
            #///////////////////////////////////////left_goal_ang////////////////////////////////////////
            # alpha = math.radians(self.ros_communication.ball_ang - self.ros_communication.left_goal_ang)
            beta = 0.7
            if abs(alpha) > beta:
                angle_type = 'beta'
                if alpha > 0:
                    alpha = beta
                else:
                    alpha = -beta
            else:
                angle_type = 'alpha'
            br_x = self.ros_communication.ball_dis * math.cos(math.radians(self.ros_communication.ball_ang))
            br_y = self.ros_communication.ball_dis * math.sin(math.radians(self.ros_communication.ball_ang))

            if self.ros_communication.ballhandle_client(1) == 1:
                #/////////////////////////////////////////////////right_goal////////////////////////////////////////////////////
                v_x = (self.ros_communication.right_goal_dis)*2.8 * math.cos(math.radians(self.ros_communication.right_goal_ang))
                v_y = (self.ros_communication.right_goal_dis)*2.8 * math.sin(math.radians(self.ros_communication.right_goal_ang))
                v_yaw = self.ros_communication.right_goal_ang
                #/////////////////////////////////////////////////left_goal/////////////////////////////////////////////////////
                # v_x = (self.ros_communication.left_goal_dis)*2.8 * math.cos(math.radians(self.ros_communication.left_goal_ang))
                # v_y = (self.ros_communication.left_goal_dis)*2.8 * math.sin(math.radians(self.ros_communication.left_goal_ang))
                # v_yaw = self.ros_communication.left_goal_ang
                strategy_type = 'attack'
            else:
                v_x = br_x*2.5 * math.cos(alpha) - br_y*2.5 * math.sin(alpha)
                v_y = br_x*2.5 * math.sin(alpha) + br_y*2.5 * math.cos(alpha)
                #////////////////right_goal_yaw//////////////
                v_yaw = self.ros_communication.right_goal_ang
                #////////////////left_goal_yaw///////////////
                # v_yaw = self.ros_communication.left_goal_ang
                strategy_type = 'chase'

            if robot_id == 1:
                print('\r nubot_number: {}'.format(self.ros_communication.robot_number))
            else:
                print('\r rival_number: {}'.format(self.ros_communication.robot_number))

            self.ros_communication.pubNubotCtrl(v_x, v_y, v_yaw, robot_id)
            
            print('\r angle_type: {}\n strategy_type: {}'.format(angle_type, strategy_type))
            print('\r BallIsHolding: {}\n'.format(self.ros_communication.ballhandle_client(1)))
            # print('\r ShootIsDone: {}\n'.format(self.ros_communication.shoot_client(1)))
        else:
            print('unready')

if __name__ == '__main__':
    
    rospy.init_node('strategy', anonymous=True)
    rate = rospy.Rate(50)
    # nubot_goal_keeper = Strategy(1, 1)
    nubot_member2 = Strategy(2, 1)
    # nubot_member3 = Strategy(3, 1)
    # rival_goal_keeper = Strategy(1, 0)
    # rival_member2 = Strategy(2, 0)
    # rival_member3 = Strategy(3, 0)
    while not rospy.is_shutdown():
        # nubot_goal_keeper.nubot_strategy(1)
        nubot_member2.nubot_strategy(1)
        # nubot_member3.nubot_strategy(1)
        # rival_goal_keeper.nubot_strategy(0)
        # rival_member2.nubot_strategy(0)
        # rival_member3.nubot_strategy(0)
        rate.sleep()