import rospy
import math
from nubot_common.msg import OminiVisionInfo
from nubot_common.msg import VelCmd
from transfer.msg import PPoint
from nubot_common.srv import BallHandle
from nubot_common.srv import Shoot

class Nubot_communication(object):
    def __init__(self, robot_number, robot_id):
        self.init_flag1 = 0
        self.init_flag2 = 0
        self.robot_number = robot_number
        self.subscriber(robot_id)
        self.publisher(robot_id)

    def subscriber(self, robot_id):
        if robot_id == 1:
            rospy.Subscriber('nubot{}/omnivision/OmniVisionInfo'.format(self.robot_number), OminiVisionInfo, self.getOmniVision)
            rospy.Subscriber('nubot{}/omnivision/OmniVisionInfo/GoalInfo'.format(self.robot_number), PPoint, self.getGoalInfo)
        else:
            rospy.Subscriber('rival{}/omnivision/OmniVisionInfo'.format(self.robot_number), OminiVisionInfo, self.getOmniVision)
            rospy.Subscriber('rival{}/omnivision/OmniVisionInfo/GoalInfo'.format(self.robot_number), PPoint, self.getGoalInfo)

    def publisher(self, robot_id):
        if robot_id == 1:
            self.nubot_cmd_pub = rospy.Publisher('nubot{}/nubotcontrol/velcmd'.format(self.robot_number), VelCmd, queue_size=100)
        else:
            self.rival_cmd_pub = rospy.Publisher('rival{}/nubotcontrol/velcmd'.format(self.robot_number), VelCmd, queue_size=100)

    def ballhandle_client(self, robot_id):
        if robot_id == 1:
            rospy.wait_for_service('nubot{}/BallHandle'.format(self.robot_number))
            ballhandle_srv = rospy.ServiceProxy('nubot{}/BallHandle'.format(self.robot_number), BallHandle)
            ballhandle_response = ballhandle_srv(1)
            return ballhandle_response.BallIsHolding
        else:
            rospy.wait_for_service('rival{}/BallHandle'.format(self.robot_number))
            ballhandle_srv = rospy.ServiceProxy('rival{}/BallHandle'.format(self.robot_number), BallHandle)
            ballhandle_response = ballhandle_srv(1)
            return ballhandle_response.BallIsHolding

    def shoot_client(self, robot_id):
        if robot_id == 1:
            rospy.wait_for_service('nubot{}/Shoot'.format(self.robot_number))
            nubot_goal_keeper_shoot_srv = rospy.ServiceProxy('nubot{}/Shoot'.format(self.robot_number), Shoot)
            nubot_goal_keeper_shoot_ground_response = nubot_goal_keeper_shoot_srv(5, 1)
            return nubot_goal_keeper_shoot_ground_response.ShootIsDone
        else:
            rospy.wait_for_service('rival{}/Shoot'.format(self.robot_number))
            nubot_goal_keeper_shoot_srv = rospy.ServiceProxy('rival{}/Shoot'.format(self.robot_number), Shoot)
            nubot_goal_keeper_shoot_ground_response = nubot_goal_keeper_shoot_srv(5, 1)
            return nubot_goal_keeper_shoot_ground_response.ShootIsDone

    def getOmniVision(self, vision):
        self.init_flag1 = 1
        self.ball_dis = vision.ballinfo.real_pos.radius
        self.ball_ang = math.degrees(vision.ballinfo.real_pos.angle)
    
    def getGoalInfo(self, goal_info):
        self.init_flag2 = 1
        self.left_goal_dis = goal_info.left_radius
        self.left_goal_ang = goal_info.left_angle
        self.right_goal_dis = goal_info.right_radius
        self.right_goal_ang = goal_info.right_angle

    def pubNubotCtrl(self, x, y, yaw, robot_id):
        angle = yaw
        velocity = math.hypot(x, y)
        if x != 0:
            alpha = math.degrees(math.atan2(y, x))
        else:
            alpha = 0

        dis_max = 2
        dis_min = 0.3
        velocity_max = 70
        velocity_min = 50
        angular_velocity_max = 2
        angular_velocity_min = 0.5
        angle_max = 144
        angle_min = 20
        angle_out = angle

        if velocity == 0:
            pass
        elif velocity > dis_max:
            velocity = velocity_max
        elif velocity < dis_min:
            velocity = velocity_min
        else:
            velocity = (velocity_max - velocity_min) * (math.cos((((velocity - dis_min) / (dis_max-dis_min) - 1) * math.pi)) + 1 )/ 2 + velocity_min
        if angle == 0:
            pass
        elif abs(angle) > angle_max:
            angle_out = angular_velocity_max
        elif abs(angle) < angle_min:
            angle_out = angular_velocity_min
        else:
            angle_out = (angular_velocity_max - angular_velocity_min) * (math.cos((((angle - angle_min) / (angle_max-angle_min) - 1) * math.pi)) + 1 )/ 2 + angular_velocity_min
        if angle < 0:
            angle_out = -angle_out

        x = velocity * math.cos(math.radians(alpha))
        y = velocity * math.sin(math.radians(alpha))
        yaw = angle_out
        vel = VelCmd()
        vel.Vx = x
        vel.Vy = y
        vel.w = yaw
        
        if robot_id == 1:
            self.nubot_cmd_pub.publish(vel)
        else:
            self.rival_cmd_pub.publish(vel)

        print('\r vel.Vx: {}\n vel.Vy: {}\n vel.w: {}'.format(vel.Vx, vel.Vy, vel.w))