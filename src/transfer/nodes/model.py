#!/usr/bin/env python3

import rospy
import math
from transfer.model import my_math
from transfer.msg import PPoint
from gazebo_msgs.msg import ModelStates

class ModelTransfer(object):

    def __init__(self):
        self.loadParam()
        self.publisher()        
        self.subscriber()

    def loadParam(self):
        if rospy.has_param('/cyan/num'):
            self.my_robot_num = rospy.get_param('/cyan/num')
        if rospy.has_param('/magenta/num'):
            self.oppo_robot_num = rospy.get_param('/magenta/num')

    def subscriber(self):
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.getModel)

    def publisher(self):
        if self.my_robot_num <= 0:
            raise('Robot number error')
            exit(1)
        if self.my_robot_num >= 1:
            self.nubot1_goal_pub = rospy.Publisher('/nubot1/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)
        if self.my_robot_num >= 2:
            self.nubot2_goal_pub = rospy.Publisher('/nubot2/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)
        if self.my_robot_num >= 3:
            self.nubot3_goal_pub = rospy.Publisher('/nubot3/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)
        if self.my_robot_num >= 4:
            self.nubot4_goal_pub = rospy.Publisher('/nubot4/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)
        if self.my_robot_num >= 5:
            self.nubot5_goal_pub = rospy.Publisher('/nubot5/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)
        
        if self.oppo_robot_num <= 0:
            raise('Rival Robot number error')
            exit(1)
        if self.oppo_robot_num >= 1:
            self.rival1_goal_pub = rospy.Publisher('/rival1/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)
        if self.oppo_robot_num >= 2:
            self.rival2_goal_pub = rospy.Publisher('/rival2/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)
        if self.oppo_robot_num >= 3:
            self.rival3_goal_pub = rospy.Publisher('/rival3/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)

    def getModel(self, models):

        for index, name in enumerate(models.name):
            if name == 'left_goal':
                left_goal_x = models.pose[index].position.x
                left_goal_y = models.pose[index].position.y
            elif name == 'right_goal':
                right_goal_x = models.pose[index].position.x
                right_goal_y = models.pose[index].position.y
            elif name == 'nubot1':
                nubot1_x = models.pose[index].position.x
                nubot1_y = models.pose[index].position.y
                _, _, nubot1_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
            elif name == 'nubot2':
                nubot2_x= models.pose[index].position.x
                nubot2_y = models.pose[index].position.y
                _, _, nubot2_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
            elif name == 'nubot3':
                nubot3_x= models.pose[index].position.x
                nubot3_y = models.pose[index].position.y
                _, _, nubot3_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
            elif name == 'nubot4':
                nubot4_x= models.pose[index].position.x
                nubot4_y = models.pose[index].position.y
                _, _, nubot4_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
            elif name == 'nubot5':
                nubot5_x= models.pose[index].position.x
                nubot5_y = models.pose[index].position.y
                _, _, nubot5_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
        
            elif name == 'rival1':
                rival1_x = models.pose[index].position.x
                rival1_y = models.pose[index].position.y
                _, _, rival1_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
            elif name == 'rival2':
                rival2_x = models.pose[index].position.x
                rival2_y = models.pose[index].position.y
                _, _, rival2_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
            elif name == 'rival3':
                rival3_x = models.pose[index].position.x
                rival3_y = models.pose[index].position.y
                _, _, rival3_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
            else:
                pass
        rospy.loginfo('nubot robot num: {}'.format(self.my_robot_num))
        rospy.loginfo('rival robot num: {}\n'.format(self.oppo_robot_num))
        if self.my_robot_num <= 0:
            raise('Robot number error')
            exit(1)
        if self.oppo_robot_num <= 0:
            raise('Rival Robot number error')
            exit(1)
        if len(models.name) == 5 + self.my_robot_num + self.oppo_robot_num:
            if self.my_robot_num >= 1:
                nubot1_left_goal_dis = my_math.calDis(
                        x=left_goal_x-nubot1_x, y=left_goal_y-nubot1_y,
                        remainder=3)
                nubot1_left_goal_ang = my_math.calAng(
                        x=left_goal_x-nubot1_x, y=left_goal_y-nubot1_y, 
                        yaw=nubot1_yaw, remainder=3)
                nubot1_right_goal_dis = my_math.calDis(
                        x=right_goal_x-nubot1_x, y=right_goal_y-nubot1_y,
                        remainder=3)
                nubot1_right_goal_ang = my_math.calAng(
                        x=right_goal_x-nubot1_x, y=right_goal_y-nubot1_y, 
                        yaw=nubot1_yaw, remainder=3)

                if nubot1_left_goal_ang > 180:
                        nubot1_left_goal_ang = nubot1_left_goal_ang - 360
                elif nubot1_left_goal_ang < -180:
                        nubot1_left_goal_ang = nubot1_left_goal_ang + 360
                else:
                        pass
                if nubot1_right_goal_ang > 180:
                        nubot1_right_goal_ang = nubot1_right_goal_ang - 360
                elif nubot1_right_goal_ang < -180:
                        nubot1_right_goal_ang = nubot1_right_goal_ang + 360
                else:
                        pass

                nubot1_goal = PPoint()
                nubot1_goal.left_angle = nubot1_left_goal_ang
                nubot1_goal.left_radius = nubot1_left_goal_dis
                nubot1_goal.right_angle = nubot1_right_goal_ang
                nubot1_goal.right_radius = nubot1_right_goal_dis
                self.nubot1_goal_pub.publish(nubot1_goal)
            if self.my_robot_num >= 2:
                nubot2_left_goal_dis = my_math.calDis(
                        x=left_goal_x-nubot2_x, y=left_goal_y-nubot2_y,
                        remainder=3)
                nubot2_left_goal_ang = my_math.calAng(
                        x=left_goal_x-nubot2_x, y=left_goal_y-nubot2_y, 
                        yaw=nubot2_yaw, remainder=3)
                nubot2_right_goal_dis = my_math.calDis(
                        x=right_goal_x-nubot2_x, y=right_goal_y-nubot2_y,
                        remainder=3)
                nubot2_right_goal_ang = my_math.calAng(
                        x=right_goal_x-nubot2_x, y=right_goal_y-nubot2_y, 
                        yaw=nubot2_yaw, remainder=3)
                
                if nubot2_left_goal_ang > 180:
                        nubot2_left_goal_ang = nubot2_left_goal_ang - 360
                elif nubot2_left_goal_ang < -180:
                        nubot2_left_goal_ang = nubot2_left_goal_ang + 360
                else:
                        pass
                if nubot2_right_goal_ang > 180:
                        nubot2_right_goal_ang = nubot2_right_goal_ang - 360
                elif nubot2_right_goal_ang < -180:
                        nubot2_right_goal_ang = nubot2_right_goal_ang + 360
                else:
                        pass

                nubot2_goal = PPoint()
                nubot2_goal.left_angle = nubot2_left_goal_ang
                nubot2_goal.left_radius = nubot2_left_goal_dis
                nubot2_goal.right_angle = nubot2_right_goal_ang
                nubot2_goal.right_radius = nubot2_right_goal_dis
                self.nubot2_goal_pub.publish(nubot2_goal)
            if self.my_robot_num >= 3:
                nubot3_left_goal_dis = my_math.calDis(
                        x=left_goal_x-nubot3_x, y=left_goal_y-nubot3_y,
                        remainder=3)
                nubot3_left_goal_ang = my_math.calAng(
                        x=left_goal_x-nubot3_x, y=left_goal_y-nubot3_y, 
                        yaw=nubot3_yaw, remainder=3)
                nubot3_right_goal_dis = my_math.calDis(
                        x=right_goal_x-nubot3_x, y=right_goal_y-nubot3_y,
                        remainder=3)
                nubot3_right_goal_ang = my_math.calAng(
                        x=right_goal_x-nubot3_x, y=right_goal_y-nubot3_y, 
                        yaw=nubot3_yaw, remainder=3)
                
                if nubot3_left_goal_ang > 180:
                        nubot3_left_goal_ang = nubot3_left_goal_ang - 360
                elif nubot3_left_goal_ang < -180:
                        nubot3_left_goal_ang = nubot3_left_goal_ang + 360
                else:
                        pass
                if nubot3_right_goal_ang > 180:
                        nubot3_right_goal_ang = nubot3_right_goal_ang - 360
                elif nubot3_right_goal_ang < -180:
                        nubot3_right_goal_ang = nubot3_right_goal_ang + 360
                else:
                        pass

                nubot3_goal = PPoint()
                nubot3_goal.left_angle = nubot3_left_goal_ang
                nubot3_goal.left_radius = nubot3_left_goal_dis
                nubot3_goal.right_angle = nubot3_right_goal_ang
                nubot3_goal.right_radius = nubot3_right_goal_dis
                self.nubot3_goal_pub.publish(nubot3_goal)
        #     if self.my_robot_num >= 4:
        #         nubot4_left_goal_dis = my_math.calDis(
        #                 x=left_goal_x-nubot4_x, y=left_goal_y-nubot4_y,
        #                 remainder=3)
        #         nubot4_left_goal_ang = my_math.calAng(
        #                 x=left_goal_x-nubot4_x, y=left_goal_y-nubot4_y, 
        #                 yaw=nubot4_yaw, remainder=3)
        #         nubot4_right_goal_dis = my_math.calDis(
        #                 x=right_goal_x-nubot4_x, y=right_goal_y-nubot4_y,
        #                 remainder=3)
        #         nubot4_right_goal_ang = my_math.calAng(
        #                 x=right_goal_x-nubot4_x, y=right_goal_y-nubot4_y, 
        #                 yaw=nubot4_yaw, remainder=3)
        #         nubot4_goal = PPoint()
        #         nubot4_goal.left_angle = nubot4_left_goal_ang
        #         nubot4_goal.left_radius = nubot4_left_goal_dis
        #         nubot4_goal.right_angle = nubot4_right_goal_ang
        #         nubot4_goal.right_radius = nubot4_right_goal_dis
        #         self.nubot4_goal_pub.publish(nubot4_goal)
        #     if self.my_robot_num >= 5:
        #         nubot5_left_goal_dis = my_math.calDis(
        #                 x=left_goal_x-nubot5_x, y=left_goal_y-nubot5_y,
        #                 remainder=3)
        #         nubot5_left_goal_ang = my_math.calAng(
        #                 x=left_goal_x-nubot5_x, y=left_goal_y-nubot5_y, 
        #                 yaw=nubot5_yaw, remainder=3)
        #         nubot5_right_goal_dis = my_math.calDis(
        #                 x=right_goal_x-nubot5_x, y=right_goal_y-nubot5_y,
        #                 remainder=3)
        #         nubot5_right_goal_ang = my_math.calAng(
        #                 x=right_goal_x-nubot5_x, y=right_goal_y-nubot5_y, 
        #                 yaw=nubot5_yaw, remainder=3)
        #         nubot5_goal = PPoint()
        #         nubot5_goal.left_angle = nubot5_left_goal_ang
        #         nubot5_goal.left_radius = nubot5_left_goal_dis
        #         nubot5_goal.right_angle = nubot5_right_goal_ang
        #         nubot5_goal.right_radius = nubot5_right_goal_dis
        #         self.nubot5_goal_pub.publish(nubot5_goal)

            if self.oppo_robot_num >= 1:
                rival1_left_goal_dis = my_math.calDis(
                        x=rival1_x-left_goal_x, y=rival1_y-left_goal_y,
                        remainder=3)
                rival1_left_goal_ang = my_math.calAng(
                        x=rival1_x-left_goal_x, y=rival1_y-left_goal_y, 
                        yaw=rival1_yaw, remainder=3)
                rival1_right_goal_dis = my_math.calDis(
                        x=rival1_x-right_goal_x, y=rival1_y-right_goal_y,
                        remainder=3)
                rival1_right_goal_ang = my_math.calAng(
                        x=rival1_x-right_goal_x, y=rival1_y-right_goal_y, 
                        yaw=rival1_yaw, remainder=3)

                if rival1_left_goal_ang > 180:
                        rival1_left_goal_ang = rival1_left_goal_ang - 360
                elif rival1_left_goal_ang < -180:
                        rival1_left_goal_ang = rival1_left_goal_ang + 360
                else:
                        pass
                if rival1_right_goal_ang > 180:
                        rival1_right_goal_ang = rival1_right_goal_ang - 360
                elif rival1_right_goal_ang < -180:
                        rival1_right_goal_ang = rival1_right_goal_ang + 360
                else:
                        pass

                rival1_goal = PPoint()
                rival1_goal.left_angle = rival1_left_goal_ang
                rival1_goal.left_radius = rival1_left_goal_dis
                rival1_goal.right_angle = rival1_right_goal_ang
                rival1_goal.right_radius = rival1_right_goal_dis
                self.rival1_goal_pub.publish(rival1_goal)
            if self.oppo_robot_num >= 2:
                rival2_left_goal_dis = my_math.calDis(
                        x=rival2_x-left_goal_x, y=rival2_y-left_goal_y,
                        remainder=3)
                rival2_left_goal_ang = my_math.calAng(
                        x=rival2_x-left_goal_x, y=rival2_y-left_goal_y, 
                        yaw=rival2_yaw, remainder=3)
                rival2_right_goal_dis = my_math.calDis(
                        x=rival2_x-right_goal_x, y=rival2_y-right_goal_y,
                        remainder=3)
                rival2_right_goal_ang = my_math.calAng(
                        x=rival2_x-right_goal_x, y=rival2_y-right_goal_y, 
                        yaw=rival2_yaw, remainder=3)

                if rival2_left_goal_ang > 180:
                        rival2_left_goal_ang = rival2_left_goal_ang - 360
                elif rival2_left_goal_ang < -180:
                        rival2_left_goal_ang = rival2_left_goal_ang + 360
                else:
                        pass
                if rival2_right_goal_ang > 180:
                        rival2_right_goal_ang = rival2_right_goal_ang - 360
                elif rival2_right_goal_ang < -180:
                        rival2_right_goal_ang = rival2_right_goal_ang + 360
                else:
                        pass

                rival2_goal = PPoint()
                rival2_goal.left_angle = rival2_left_goal_ang
                rival2_goal.left_radius = rival2_left_goal_dis
                rival2_goal.right_angle = rival2_right_goal_ang
                rival2_goal.right_radius = rival2_right_goal_dis
                self.rival2_goal_pub.publish(rival2_goal)
            if self.oppo_robot_num >= 3:
                rival3_left_goal_dis = my_math.calDis(
                        x=rival3_x-left_goal_x, y=rival3_y-left_goal_y,
                        remainder=3)
                rival3_left_goal_ang = my_math.calAng(
                        x=rival3_x-left_goal_x, y=rival3_y-left_goal_y, 
                        yaw=rival3_yaw, remainder=3)
                rival3_right_goal_dis = my_math.calDis(
                        x=rival3_x-right_goal_x, y=rival3_y-right_goal_y,
                        remainder=3)
                rival3_right_goal_ang = my_math.calAng(
                        x=rival3_x-right_goal_x, y=rival3_y-right_goal_y, 
                        yaw=rival3_yaw, remainder=3)

                if rival3_left_goal_ang > 180:
                        rival3_left_goal_ang = rival3_left_goal_ang - 360
                elif rival3_left_goal_ang < -180:
                        rival3_left_goal_ang = rival3_left_goal_ang + 360
                else:
                        pass
                if rival3_right_goal_ang > 180:
                        rival3_right_goal_ang = rival3_right_goal_ang - 360
                elif rival3_right_goal_ang < -180:
                        rival3_right_goal_ang = rival3_right_goal_ang + 360
                else:
                        pass

                rival3_goal = PPoint()
                rival3_goal.left_angle = rival3_left_goal_ang
                rival3_goal.left_radius = rival3_left_goal_dis
                rival3_goal.right_angle = rival3_right_goal_ang
                rival3_goal.right_radius = rival3_right_goal_dis
                self.rival3_goal_pub.publish(rival3_goal)
    
if __name__ == '__main__':
    rospy.init_node('transfer', anonymous=True)
    rate = rospy.Rate(50)
    model_transfer = ModelTransfer()
    while not rospy.is_shutdown():
        rate.sleep()
