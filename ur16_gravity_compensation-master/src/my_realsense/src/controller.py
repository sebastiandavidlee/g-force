#!/usr/bin/env python

import rospy
import numpy as np
import itertools
from collections import deque
from moveit_msgs.msg import RobotTrajectory
from std_msgs.msg import Float64MultiArray


class Controller(object):

    def __init__(self, Kp, Ki, Kd, Kw, robot, topic):

        # If the node is shutdown, call this function
        rospy.on_shutdown(self.shutdown)

        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._Kw = Kw

        self._LastError = np.zeros(len(Kd))
        self._LastTime = 0
        self._IntError = np.zeros(len(Ki))
        self._ring_buff_capacity = 3
        self._ring_buff = deque([], self._ring_buff_capacity)

        self._path = RobotTrajectory()
        self._curIndex = 0
        self._maxIndex = 0

        self._robot = robot
        self._path = RobotTrajectory()
        self._pub = rospy.Publisher(topic, Float64MultiArray, queue_size=10)
        self._jointsNum = 6 #FIXME

    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety
        """
        rospy.loginfo("Stopping Controller")

        # Set velocities to zero
        self.set_velocity(np.zeros(self._jointsNum))
        rospy.sleep(0.1)

    def set_velocity(self, velocities):
        # check vel sequence in path and topic TODO
        vel = Float64MultiArray()
        vel.data = velocities
        self._pub.publish(vel)

    def execute_path(self, path, timeout=100.0):

        self._path = path

        self._curIndex = 0
        self._maxIndex = len(path.joint_trajectory.points) - 1
        startTime = rospy.Time.now()

        # Set the last error as zero for t = 0
        self._LastError = np.zeros(len(self._Kd))
        self._LastTime = 0.0

        # Set the integral of positions to zero
        self._IntError = np.zeros(len(self._Ki))

        # Set your ring buffer to zero
        self._ring_buff = deque([], self._ring_buff_capacity)
        r = rospy.Rate(200)

        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - startTime).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self.set_velocity(np.zeros(self._jointsNum))
                return False

            # Get the input for this time
            u = self.step_control(t)

            # Set the joint velocities
            self.set_velocity(u)
            # Sleep for a defined time (to let the robot move)
            r.sleep()

            # Once the end of the path has been reached, stop moving and break
            if self._curIndex >= self._maxIndex:
                # Set velocities to zero
                self.set_velocity(np.zeros(self._jointsNum))
                break
        return True

    def step_control(self, t):

        # Make sure you're using the latest time
        while (not rospy.is_shutdown() and self._curIndex < self._maxIndex and self._path.joint_trajectory.points[
            self._curIndex + 1].time_from_start.to_sec() < t + 0.001):
            self._curIndex = self._curIndex + 1

        # The state of each joint (revolute or prismatic) is defined by:
        #  * the position of the joint (rad or m),
        #  * the velocity of the joint (rad/s or m/s) and
        current_state = self._robot.get_current_state()
        position = current_state.joint_state.position
        velocity = current_state.joint_state.velocity
        current_position = np.array(
            [position[joint_name] for joint_name in self._path.joint_trajectory.joint_names])
        current_velocity = np.array(
            [velocity[joint_name] for joint_name in self._path.joint_trajectory.joint_names])

        if self._curIndex < self._maxIndex:
            time_low = self._path.joint_trajectory.points[self._curIndex].time_from_start.to_sec()
            time_high = self._path.joint_trajectory.points[self._curIndex + 1].time_from_start.to_sec()

            target_position_low = np.array(self._path.joint_trajectory.points[self._curIndex].positions)
            target_velocity_low = np.array(self._path.joint_trajectory.points[self._curIndex].velocities)

            target_position_high = np.array(self._path.joint_trajectory.points[self._curIndex + 1].positions)
            target_velocity_high = np.array(self._path.joint_trajectory.points[self._curIndex + 1].velocities)

            target_position = target_position_low + (t - time_low) / (time_high - time_low) * (
                        target_position_high - target_position_low)
            target_velocity = target_velocity_low + (t - time_low) / (time_high - time_low) * (
                        target_velocity_high - target_velocity_low)
        else:
            target_position = np.array(self._path.joint_trajectory.points[self._maxIndex].positions)
            target_velocity = np.array(self._path.joint_trajectory.points[self._maxIndex].velocities)

        # Feed Forward Term
        u_ff = target_velocity

        # Error Term
        error = target_position - current_position

        # Integral Term
        self._IntError = self._Kw * self._IntError + error

        # Derivative Term
        dt = t - self._LastTime
        # implement a moving average filter to smooth the derivative
        curr_derivative = (error - self._LastError) / dt
        self._ring_buff.append(curr_derivative)
        ed = np.mean(self._ring_buff)

        # Save terms for the next run
        self._LastError = error
        self._LastTime = t

        u = u_ff + self._Kp*error + self._Kd*curr_derivative + self._Ki*self._IntError
        # u = u + self._Kp * error + self._Kd * curr_derivative
        return  u


if __name__ == '__main__':
    pass