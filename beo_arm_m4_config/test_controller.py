#!/usr/bin/env python

import rospy
import rospkg
import sys
import moveit_commander
import pickle
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import random_quaternion

rospy.init_node('cmd_publisher')
rospack = rospkg.RosPack()
beoarm_config_path = rospack.get_path('beo_arm_m4_config')

moveit_commander.roscpp_initialize(sys.argv)
arm = moveit_commander.MoveGroupCommander('arm')

def move_around():
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    msg = JointTrajectory()
    msg.header = Header()
    msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
    # msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    msg.points = [JointTrajectoryPoint()]
    msg.points[0].positions = [3.14, 0, 3.14, 3.14]
    # msg.points[0].positions = [0, -1, 0, 0, 0, 0]
    msg.points[0].time_from_start = rospy.Duration.from_sec(10)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()

def spawn_robot():
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    while not spawn_proxy: continue;
    # FIXME: absolute path bad
    spawn_proxy(model_name='robot',
                model_xml=open('/home/ros/new_catkin_ws/src/beoarm_model/beo_arm_m4/urdf/beo_arm_m4.urdf','r').read(),
                initial_pose=Pose(position=Point(0,0,0.1),orientation=Quaternion(0,0,0,0)),
                reference_frame='world')

def mvit_rand_valid_ee_pose(iteration=50):
    # Can replan if the previous planning attempt failed(?)
    # arm.allow_replanning(True)
    rospy.sleep(2)
    result = []
    for i in range(iteration):
        rand_pose = arm.get_random_pose(arm.get_end_effector_link())
        result.append(rand_pose)
    # FIXME: pickle only works for rospy
    pickle.dump(result, open(beoarm_config_path+'ee_poses.p', 'wb'))
    # ee_poses = pickle.load(open(beoarm_config_path+'ee_poses.p', 'rb'))

def get_mvit_plan_trans(x, y, z):
    attempts = 0
    while True:
        if attempts >= 10:
            return False
        goal_pose = Pose()
        goal_pose.position.x = x
        goal_pose.position.y = y
        goal_pose.position.z = z
        rand_quat = random_quaternion()
        goal_pose.orientation.x = rand_quat[0]
        goal_pose.orientation.y = rand_quat[1]
        goal_pose.orientation.z = rand_quat[2]
        goal_pose.orientation.w = rand_quat[3]
        arm.set_pose_target(goal_pose)
        plan_success, plan1, _, _ = arm.plan()
        if plan_success:
            return plan
        else:
            attempts += 1
    
     
if __name__ == '__main__':
    mvit_rand_valid_ee_pose(1)
    ee_poses = pickle.load(open(beoarm_config_path+'ee_poses.p', 'rb'))
    plan = get_mvit_plan_trans(ee_poses[0].pose.position.x, ee_poses[0].pose.position.y, ee_poses[0].pose.position.z)
    if plan:
        arm.execute(plan)
    else:
        print('planning failed')

