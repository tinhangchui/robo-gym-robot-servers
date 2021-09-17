#!/usr/bin/env python

import rospy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates, ModelState

'''
Publishes the poses of all objects to TF tree
Can reset object position if asked and also update TF tree

NOTE: These poses are wrt. gazebo world, NOT robot base
'''
class ObjectsTF:
    def __init__(self):
        self.object_model_names = rospy.get_param('object_model_names')

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.objects_tf_handler)
        update_rate = float(rospy.get_param('~update_rate'))
        self.check_timeout = rospy.Duration(1.0 / update_rate)
        self.prev_check_time = rospy.Time.now()
        self.tf2_broadcaster = TransformBroadcaster()

        # publish once at start so tf is not empty
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        for object_name in self.object_model_names:
            try:
                object_index = model_states.name.index(object_name)
                object_pose = model_states.pose[object_index]
                t = TransformStamped()
                t.header.stamp = self.prev_check_time
                t.header.frame_id = 'world'
                t.child_frame_id = object_name
                t.transform.translation.x = object_pose.position.x
                t.transform.translation.y = object_pose.position.y
                t.transform.translation.z = object_pose.position.z
                t.transform.rotation = object_pose.orientation
                self.tf2_broadcaster.sendTransform(t)
            except ValueError:
                rospy.logerr('Object %s not found in /gazebo/model_states', object_name)

        rospy.Subscriber('set_object_state', ModelState, self.set_object_state_handler)
        self.model_state_publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)


    def objects_tf_handler(self, model_states):
        time_now = rospy.Time.now()
        if time_now - self.prev_check_time < self.check_timeout:
            return
        else:
            self.prev_check_time = time_now
            for object_name in self.object_model_names:
                try:
                    object_index = model_states.name.index(object_name)
                    object_pose = model_states.pose[object_index]
                    t = TransformStamped()
                    t.header.stamp = time_now
                    t.header.frame_id = 'world'
                    t.child_frame_id = object_name
                    t.transform.translation.x = object_pose.position.x
                    t.transform.translation.y = object_pose.position.y
                    t.transform.translation.z = object_pose.position.z
                    t.transform.rotation = object_pose.orientation
                    self.tf2_broadcaster.sendTransform(t)
                except ValueError:
                    rospy.logerr('Object %s not found in /gazebo/model_states', object_name)


    def set_object_state_handler(self, model_state):
        self.model_state_publisher.publish(model_state)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'world'
        t.child_frame_id = model_state.model_name
        t.transform.translation.x = model_state.pose.position.x
        t.transform.translation.y = model_state.pose.position.y
        t.transform.translation.z = model_state.pose.position.z
        t.transform.rotation = model_state.pose.orientation
        self.tf2_broadcaster.sendTransform(t)


if __name__ == '__main__':
    try:
        rospy.init_node('objects_tf')
        oc = ObjectsTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

