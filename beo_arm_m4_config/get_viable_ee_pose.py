#!/usr/bin/env python

import rospy
import rospkg
import tf2_ros
import numpy as np
from random import uniform
from threading import Event
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteModel, SpawnModel
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Pose, Point, Quaternion
from controller_manager_msgs.srv import SwitchController, LoadController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

# constants
iterations = 10
rospack = rospkg.RosPack()
urdf_path = rospack.get_path('beo_arm_m4') + '/urdf/beo_arm_m4.urdf'

# global vars
collision = False
latest_collision_pair = None
latest_joint_states = JointState(name=['joint_1','joint_2','joint_2','joint_2'],
                                 position=[0]*4,
                                 velocity=[0]*4,
                                 effort=[0]*4)

# global objects (only call functions, so no need global keyword)
rospy.init_node('test_goal')
traj_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)
set_js_event = Event()
set_js_event.set()
collision_event = Event()
collision_event.set()
tf2_buffer = tf2_ros.Buffer()
tf2_listener = tf2_ros.TransformListener(tf2_buffer)


def action_collision_handler(data):
    global collision, latest_collision_pair
    if data.states != [] and collision_event.is_set():
        collision_event.clear()
        collision = True
        latest_collision_pair = (data.states[-1].collision1_name, data.states[-1].collision2_name)


def joint_state_handler(data):
    global latest_joint_states
    if set_js_event.is_set():
        latest_joint_states = data


def my_reset():
    global latest_joint_states

    print('deleting robot...')
    rospy.wait_for_service('/gazebo/delete_model')
    # ServiceProxy gets closed after the first call when persistent=False, so no need to close()
    delete_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel, persistent=False)
    delete_proxy(model_name='robot')
    # rospy.sleep(1)

    print('respawning robot...')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel, persistent=False)
    spawn_proxy(model_name='robot',
                model_xml=open(urdf_path,'r').read(),
                initial_pose=Pose(position=Point(0,0,0.04),orientation=Quaternion(0,0,0,0)),
                reference_frame='world')
    # rospy.sleep(1)

    print('loading arm_controller...')
    rospy.wait_for_service('/controller_manager/load_controller')
    arm_ctrlr_load_proxy = rospy.ServiceProxy('/controller_manager/load_controller', LoadController, persistent=False)
    arm_ctrlr_load_proxy(name='arm_controller')
    print('loading joint_state_controller...')
    rospy.wait_for_service('/controller_manager/load_controller')
    jts_ctrlr_load_proxy = rospy.ServiceProxy('/controller_manager/load_controller', LoadController, persistent=False)
    jts_ctrlr_load_proxy(name='joint_state_controller')
    # rospy.sleep(1)

    print('activating controllers...')
    rospy.wait_for_service('/controller_manager/switch_controller')
    controller_on_proxy = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController, persistent=False)
    controller_on_proxy(start_controllers=['arm_controller', 'joint_state_controller'],
                        stop_controllers=[],
                        strictness=1)
    rospy.sleep(4)

    latest_joint_states = JointState(name=['joint_1','joint_2','joint_2','joint_2'],
                                     position=[0]*4,
                                     velocity=[0]*4,
                                     effort=[0]*4)


def set_joint_positions(ps):
    global set_js_event
    position_reached = False
    while not position_reached:
        if collision:
            print('collision...')
            traj_pub.publish(JointTrajectory())
            return False
        g = JointTrajectory()
        g.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        g.points = [JointTrajectoryPoint(positions=ps, time_from_start=rospy.Duration.from_sec(2))]
        traj_pub.publish(g)
        rospy.sleep(1)
        set_js_event.clear()
        position_reached = np.isclose(ps, latest_joint_states.position, atol=0.03).all()
        set_js_event.set()
    print('done!')
    return True


try:
    for i in range(iterations):
        # needs to restart every iter, otherwise states from the previous iter may leak into this iter
        subs = [
            # queue_size=1, latest state
            rospy.Subscriber('/hand_link_collision', ContactsState, action_collision_handler, queue_size=1),
            rospy.Subscriber('/link1_collision', ContactsState, action_collision_handler, queue_size=1),
            rospy.Subscriber('/link2_collision', ContactsState, action_collision_handler, queue_size=1),
            rospy.Subscriber('/link3_collision', ContactsState, action_collision_handler, queue_size=1),
            rospy.Subscriber('/link4_collision', ContactsState, action_collision_handler, queue_size=1),
            rospy.Subscriber('/link5_collision', ContactsState, action_collision_handler, queue_size=1),
            rospy.Subscriber('/joint_states', JointState, joint_state_handler, queue_size=1)
        ]
        rand_pos = [uniform(-3.14, 3.14) for i in range(4)]
        print('Trying joint positions: {}'.format(rand_pos))
        result_succeeded = set_joint_positions(rand_pos)
        try:
            if result_succeeded:
                print('Success')
                print('Collision: {}'.format(collision))
                print('Latest collision pair: {}'.format(latest_collision_pair))
                print('Actural joint positions: {}'.format(latest_joint_states.position))
                ee_to_ref_trans = tf2_buffer.lookup_transform('world', 'hand', rospy.Time(0))
                ee_to_ref_trans_list = [
                    ee_to_ref_trans.transform.translation.x,
                    ee_to_ref_trans.transform.translation.y,
                    ee_to_ref_trans.transform.translation.z,
                    ee_to_ref_trans.transform.rotation.x,
                    ee_to_ref_trans.transform.rotation.y,
                    ee_to_ref_trans.transform.rotation.z,
                    ee_to_ref_trans.transform.rotation.w
                ]
                print('EE pose: {}'.format(ee_to_ref_trans_list))
            else:
                print('Fail')
                print('Collision: {}'.format(collision))
                print('Latest collision pair: {}'.format(latest_collision_pair))
                print('Actural joint positions: {}'.format(latest_joint_states.position))
        except KeyboardInterrupt:
            traj_pub.publish(JointTrajectory())
            raise
        collision = False
        latest_collision_pair = None
        collision_event.set()
        for s in subs: s.unregister();
        my_reset()
except KeyboardInterrupt:
    rospy.signal_shutdown('KeyboardInterrupt')
    raise

