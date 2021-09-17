#!/usr/bin/env python

import sys
import copy
import rospy
import rospkg
import tf2_ros
import moveit_commander
import numpy as np
from threading import Event
from std_msgs.msg import Int32MultiArray, Header, Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController, LoadController
from gazebo_msgs.msg import ContactsState, ModelState
from gazebo_msgs.srv import DeleteModel, SpawnModel
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2

'''
state_dict should look like this:
[
    'joint_1_position',
    'joint_2_position',
    'joint_3_position',
    'joint_4_position',

    'joint_1_velocity',
    'joint_2_velocity',
    'joint_3_velocity',
    'joint_4_velocity',

    'ee_to_ref_translation_x',
    'ee_to_ref_translation_y',
    'ee_to_ref_translation_z',
    'ee_to_ref_rotation_x',
    'ee_to_ref_rotation_y',
    'ee_to_ref_rotation_z',
    'ee_to_ref_rotation_w',

    'object_0_to_ref_translation_x',
    'object_0_to_ref_translation_y',
    'object_0_to_ref_translation_z',
    'object_0_to_ref_rotation_x',
    'object_0_to_ref_rotation_y',
    'object_0_to_ref_rotation_z',
    'object_0_to_ref_rotation_w',

    'in_collision'
]
'''

rospack = rospkg.RosPack()
urdf_path = rospack.get_path('beo_arm_m4') + '/urdf/beo_arm_m4.urdf'
robot_model_name = 'robot'
robot_spawn_trans = {'x':0, 'y':0, 'z':0.04}
manipulator_group_name = 'arm'
manipulator_ctrlr_name = 'arm_controller'
tf_lookup_attempts = 10

class RosBridge:

    def __init__(self):
        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

        # Joint States
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self.joint_position = dict.fromkeys(self.joint_names, 0.0)
        self.joint_velocity = dict.fromkeys(self.joint_names, 0.0)
        rospy.Subscriber("joint_states", JointState, self._on_joint_states)

        # Robot control
        self.arm_cmd_pub = rospy.Publisher('env_arm_command', JointTrajectory, queue_size=1) # joint_trajectory_command_handler publisher
        # FIXME: rospy.Rate
        self.action_cycle_rate = rospy.Rate(float(rospy.get_param("action_cycle_rate")))
        self.min_traj_duration = 0.5 # minimum trajectory duration (s)
        # FIXME: change this to reflect real velocity limits
        self.joint_velocity_limits = dict(zip(self.joint_names, [1.0] * 4))

        # Robot frames
        self.reference_frame = 'dummy_ground'
        self.ee_frame = 'hand'
        self.object_frames = rospy.get_param('object_model_names')

        # Publish new ModelState here to have object_tf node reset object state in gazebo and update TF
        # FIXME: latch
        self.set_object_state_pub = rospy.Publisher('set_object_state', ModelState, latch=True, queue_size=1)

        # TF2
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

        # Collision detection
        # FIXME: restart every _reset_robot_object?
        self.collision_subs = [
            rospy.Subscriber("hand_link_collision", ContactsState, self._on_hand_collision),
            rospy.Subscriber("link1_collision", ContactsState, self._on_link1_collision),
            rospy.Subscriber("link2_collision", ContactsState, self._on_link2_collision),
            rospy.Subscriber("link3_collision", ContactsState, self._on_link3_collision),
            rospy.Subscriber("link4_collision", ContactsState, self._on_link4_collision),
            rospy.Subscriber("link5_collision", ContactsState, self._on_link5_collision)
        ]
        # Initialization of collision sensor flags
        self.collision_sensors = dict.fromkeys(["hand", "link_1", "link_2", "link_3", "link_4", "link_5"], False)

        self.get_plan_event = Event()
        self.get_plan_event.set()
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group = moveit_commander.MoveGroupCommander(manipulator_group_name)


    def get_state(self):
        self.get_state_event.clear()
        # Get environment state
        state_dict = {}

        # Joint Positions and Joint Velocities
        joint_position = copy.deepcopy(self.joint_position)
        joint_velocity = copy.deepcopy(self.joint_velocity)
        state_dict.update(self._get_joint_states_dict(joint_position, joint_velocity))

        # ee/obj to ref transform
        tf_lookup_succeeded = False
        for i in range(tf_lookup_attempts):
            while True:
                try:
                    ee_to_ref_trans = self.tf2_buffer.lookup_transform(self.reference_frame, self.ee_frame, rospy.Time(0))
                    state_dict.update(self._get_transform_dict(ee_to_ref_trans, 'ee_to_ref'))

                    # FIXME: The pose information of ONLY 1 object (might expand to support more objects in the future)
                    object_0_trans = self.tf2_buffer.lookup_transform(self.reference_frame, self.object_frames[0], rospy.Time(0))
                    state_dict.update(self._get_transform_dict(object_0_trans, 'object_0_to_ref'))
                except tf2_ros.LookupException:
                    rospy.logwarn('TF lookup for ee_frame or object_frame failed, retrying...')
                    rospy.sleep(1)
                    continue
                tf_lookup_succeeded = True
                break
        if not tf_lookup_succeeded:
            return robot_server_pb2.State(success=False)

        # Collision sensors
        beoarm_collision = any(self.collision_sensors.values())
        state_dict['in_collision'] = float(beoarm_collision)

        self.get_state_event.set()

        # Create and fill State message
        msg = robot_server_pb2.State(state_dict=state_dict, success=True)

        return msg


    '''
    Use the new reset_robot_object function for setting state

    Only use this if you wish to set to a non-zero joint position(since reset_robot_object can only set to all-zeros)
        - might cause problems when collision
    '''
    def set_state(self, state_msg):
        # Clear reset Event
        self.reset.clear()

        # Updates object_0 position if information given in float_params
        # FIXME: support for more objects?
        try:
            object_0_x = state_msg.float_params['object_0_x']
            object_0_y = state_msg.float_params['object_0_y']
            object_0_z = state_msg.float_params['object_0_z']
            obj_state_msg = ModelState(
                model_name=self.object_frames[0],
                pose=Pose(
                    position=Point(object_0_x, object_0_y, object_0_z),
                    orientation=Quaternion(0,0,0,1)
                ),
                reference_frame=self.reference_frame
            )
            self.set_object_state_pub.publish(obj_state_msg)
        except KeyError:
            rospy.logwarn('No object_0 reset position given, skipping...')

        # Joints Positions
        goal_joint_position = [state_msg.state_dict['joint_1_position'], state_msg.state_dict['joint_2_position'], \
                                state_msg.state_dict['joint_3_position'], state_msg.state_dict['joint_4_position']]
        self.set_joint_position(goal_joint_position)

        # Reset collision sensors flags
        self.collision_sensors.update(dict.fromkeys(["hand", "link_1", "link_2", "link_3", "link_4", "link_5"], False))

        self.reset.set()
        self.action_cycle_rate.sleep()

        return 1


    '''
    resets object_0 position to that given in float_params and robot joint positions to all-zeros

    does this by deleting and respawning robot
        - tried SetModelConfiguration, doesn't work, which is also why this function currently doesn't support setting joint positions
    '''
    def reset_robot_object(self, state_msg):
        self.reset.clear()

        # ignores joint position information in state_msg

        # FIXME: support for more objects?
        try:
            object_0_x = state_msg.float_params['object_0_x']
            object_0_y = state_msg.float_params['object_0_y']
            object_0_z = state_msg.float_params['object_0_z']
            obj_state_msg = ModelState(
                model_name=self.object_frames[0],
                pose=Pose(
                    position=Point(object_0_x, object_0_y, object_0_z),
                    orientation=Quaternion(0,0,0,1)
                ),
                reference_frame='world'
            )
            self.set_object_state_pub.publish(obj_state_msg)
        except KeyError:
            rospy.logwarn('No object_0 reset position given, skipping...')

        rospy.loginfo('reset_joint_position::deleting %s...', robot_model_name)
        rospy.wait_for_service('/gazebo/delete_model')
        # ServiceProxy gets closed after the first call when persistent=False, so no need to close()
        delete_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel, persistent=False)
        delete_result = delete_proxy(model_name=robot_model_name)
        rospy.sleep(1)

        rospy.loginfo('reset_joint_position::respawning %s...', robot_model_name)
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel, persistent=False)
        spawn_result = spawn_proxy(model_name=robot_model_name,
                                    model_xml=open(urdf_path,'r').read(),
                                    initial_pose=Pose(
                                        position=Point(
                                            robot_spawn_trans['x'],
                                            robot_spawn_trans['y'],
                                            robot_spawn_trans['z']
                                        ),
                                        orientation=Quaternion(0,0,0,1)
                                    ),
                                    reference_frame='world')
        rospy.sleep(1)

        rospy.loginfo('reset_joint_position::loading %s...', manipulator_ctrlr_name)
        rospy.wait_for_service('/controller_manager/load_controller')
        arm_ctrlr_load_proxy = rospy.ServiceProxy('/controller_manager/load_controller', LoadController, persistent=False)
        arm_ctrlr_load_result = arm_ctrlr_load_proxy(name=manipulator_ctrlr_name)
        rospy.loginfo('reset_joint_position::loading joint_state_controller...')
        rospy.wait_for_service('/controller_manager/load_controller')
        jts_ctrlr_load_proxy = rospy.ServiceProxy('/controller_manager/load_controller', LoadController, persistent=False)
        jts_ctrlr_load_result = jts_ctrlr_load_proxy(name='joint_state_controller')
        rospy.sleep(1)

        rospy.loginfo('reset_joint_position::activating controllers...')
        rospy.wait_for_service('/controller_manager/switch_controller')
        controller_on_proxy = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController, persistent=False)
        controller_on_result = controller_on_proxy(start_controllers=[manipulator_ctrlr_name, 'joint_state_controller'],
                                                    stop_controllers=[],
                                                    strictness=1)
        rospy.sleep(1)

        # restart collision subscribers every iter to prevent collision states from the previous iter leaking into this iter
        #   (not sure if it's really what's happening but restarting does seem to improve performance from my testing)
        for s in self.collision_subs: s.unregister();
        self.collision_subs = [
            rospy.Subscriber("hand_link_collision", ContactsState, self._on_hand_collision),
            rospy.Subscriber("link1_collision", ContactsState, self._on_link1_collision),
            rospy.Subscriber("link2_collision", ContactsState, self._on_link2_collision),
            rospy.Subscriber("link3_collision", ContactsState, self._on_link3_collision),
            rospy.Subscriber("link4_collision", ContactsState, self._on_link4_collision),
            rospy.Subscriber("link5_collision", ContactsState, self._on_link5_collision)
        ]

        self.collision_sensors.update(dict.fromkeys(["hand", "link_1", "link_2", "link_3", "link_4", "link_5"], False))

        self.reset.set()
        self.action_cycle_rate.sleep()

        if delete_result.success and \
           spawn_result.success and \
           arm_ctrlr_load_result.ok and \
           arm_ctrlr_load_result.ok and \
           controller_on_result.ok:
            return 1
        else:
            return 0


    def set_joint_position(self, goal_joint_position):
        """Set robot joint positions to a desired value
        """

        position_reached = False
        rospy.loginfo('Setting joint positions:')
        rospy.loginfo(goal_joint_position)
        while not position_reached:
            self.publish_env_arm_cmd(goal_joint_position)
            self.get_state_event.clear()
            joint_position = copy.deepcopy(self.joint_position)
            position_reached = np.isclose(goal_joint_position, joint_position, atol=0.03).all()
            self.get_state_event.set()


    def publish_env_arm_cmd(self, position_cmd):
        """Publish environment JointTrajectory msg.
        """

        msg = JointTrajectory()
        msg.header = Header()
        msg.joint_names = self.joint_names
        msg.points=[JointTrajectoryPoint()]
        msg.points[0].positions = position_cmd
        dur = []
        for idx, name in enumerate(msg.joint_names):
            pos = self.joint_position[name]
            cmd = position_cmd[idx]
            max_vel = self.joint_velocity_limits[name]
            dur.append(max(abs(cmd-pos)/max_vel, self.min_traj_duration))
        msg.points[0].time_from_start = rospy.Duration.from_sec(max(dur))
        self.arm_cmd_pub.publish(msg)
        self.action_cycle_rate.sleep()
        return position_cmd


    def get_moveit_plan(self, moveit_goal):
        """returns a MoveitPlan to goal
        """
        pass


    def _on_joint_states(self, msg):
        if self.get_state_event.is_set():
            for idx, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.joint_position[name] = msg.position[idx]
                    self.joint_velocity[name] = msg.velocity[idx]

    def _on_hand_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["hand"] = True

    def _on_link1_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["link_1"] = True

    def _on_link2_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["link_2"] = True

    def _on_link3_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["link_3"] = True

    def _on_link4_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["link_4"] = True

    def _on_link5_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["link_5"] = True

    def _on_occupancy_state(self, msg):
        if self.get_state_event.is_set():
            # occupancy_3d_array = np.reshape(msg.data, [dim.size for dim in msg.layout.dim])
            self.voxel_occupancy = msg.data
        else:
            pass

    def _get_joint_states_dict(self, joint_position, joint_velocity):
        d = {}
        for joint in self.joint_names:
            d[joint+'_position'] = joint_position[joint]
            d[joint+'_velocity'] = joint_velocity[joint]
        return d

    def _get_transform_dict(self, transform, transform_name):
        d ={}
        d[transform_name + '_translation_x'] = transform.transform.translation.x
        d[transform_name + '_translation_y'] = transform.transform.translation.y
        d[transform_name + '_translation_z'] = transform.transform.translation.z
        d[transform_name + '_rotation_x'] = transform.transform.rotation.x
        d[transform_name + '_rotation_y'] = transform.transform.rotation.y
        d[transform_name + '_rotation_z'] = transform.transform.rotation.z
        d[transform_name + '_rotation_w'] = transform.transform.rotation.w
        return d

