#!/usr/bin/env python

import numpy
import random
import sys
import time

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

	
        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)


    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def motion_planning(self, ee_goal):
        print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########

	# Get the desired set of q where to stop
	tra = tf.transformations.translation_matrix((ee_goal.translation.x,ee_goal.translation.y,ee_goal.translation.z))
	rot = tf.transformations.quaternion_matrix((ee_goal.rotation.x,ee_goal.rotation.y,ee_goal.rotation.z,ee_goal.rotation.w))	
	T_goal = numpy.dot(tra,rot)
	end_point = self.IK(T_goal)
	if end_point == []:
		print "No IK found!"
	
	self.traj=JointTrajectory()
	self.next = JointTrajectoryPoint()
	
	# Build a branch map and add the starting point as the first element
	branch_map = []
	branch_map.append(RRTBranch(self.q_current,self.q_current))
	# Time out count for different obstacle
	if self.current_obstacle == "None":
		no_time_out = 10
	elif self.current_obstacle == "Simple":
		no_time_out = 60
	elif self.current_obstacle == "Hard":
		no_time_out = 120
	elif self.current_obstacle == "Super" :	
		no_time_out = 200

	# Get a new branch
	end_time = 0
	start = time.time()
	time_count = 0
	while time_count<no_time_out:
		# generate new random point for tree
		q = numpy.zeros(self.num_joints)
		for i in range(self.num_joints):
			q[i]=random.uniform(-numpy.pi,numpy.pi)
		# Which point q will be the closest to
		length_min = self.num_joints * 4 * numpy.pi**2
		for j in range(len(branch_map)):
			length = 0
			for i in range(self.num_joints):
				length = length + (q[i]- branch_map[j].q[i])**2
			if length < length_min:
				length_min = length
				pick = j

		# Make a branch with length 0.1 starting from parent
		short_q = numpy.zeros(self.num_joints)
		for i in range(self.num_joints):
			short_q[i] = branch_map[pick].q[i]+(q[i]-branch_map[pick].q[i])/10/numpy.sqrt(length_min)

		# Add the new branch to profile

		new_end = self.is_segment_valid(branch_map[pick].q,short_q)

		branch_map.append(RRTBranch(branch_map[pick].q,new_end))
	# Test if new random q can connect to end point directly
		if max(abs(end_point - self.is_segment_valid(new_end,end_point)))<0.01:
			print "Reached end from map!"
			branch_map.append(RRTBranch(new_end,end_point))
			break;
		end_time = time.time()
		time_count = end_time-start
	# Trace from end to each corresponding parent and form a list of q's
	if time_count < no_time_out:

		print "Done building Tree"
	else:
		print "Time out for calculating"
	branch_map.append(RRTBranch(end_point,end_point))
	path = []
	prev_parent = end_point
	print "Start looking for map"
	for k in range(len(branch_map)-1,-1,-1):
		max_ran = 0
		for i in range(self.num_joints):
			if abs(branch_map[k].q[i] - prev_parent[i])>max_ran:
				max_ran = abs(branch_map[k].q[i] - prev_parent[i])
			# Trace back
		if max_ran<0.01:
			prev_parent = branch_map[k].parent
			path.append(k)
	path.reverse()
	path_short = [0]

	# Connect from start point to optimize route

	k = 0
	while k!=len(path):
		for i in range(len(path)-k):
			v_check = numpy.zeros(self.num_joints)
			for j in range(self.num_joints):
				v_check[j] = self.is_segment_valid(branch_map[path[k]].parent,branch_map[path[k+i]].q)[j]-branch_map[path[k+i]].q[j]
			if max(abs(v_check))>0.01:
				path_short.append(path[k+i])
				k = k+i
				break

		if k+i+1 == len(path):
			k = k+i+1
			path_short.append(path[len(path)-1])
			break

	
	print "Optimized route accuired!"
	# Divide route and append each point into msg for publish
	for i in range(len(path_short)-1):
		step = numpy.zeros(self.num_joints)
		for k in range(self.num_joints):
			step[k] = (branch_map[path_short[i+1]].parent[k]-branch_map[path_short[i]].parent[k])/50
		for j in range(50):
			self.next = JointTrajectoryPoint()
			self.next.positions = branch_map[path_short[i]].parent + step*(j+1)
			self.traj.points.append(self.next)
			#print self.traj.points
	self.traj.joint_names = self.joint_names
	self.pub.publish(self.traj)

	#print self.next.positions


        ######################################################


    def is_segment_valid(self, start, end):
	move = numpy.zeros(self.num_joints)
	for ii in range(self.num_joints):
		move[ii] = end[ii] - start[ii]
	# scale
	step = move/30
	check = start
	for j in range(30):
		if (self.is_state_valid(check)):
			check = check + step
			
		else :
			return check
	
	return check

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	self.parent = parent
	self.q = q

    def get_parent(self,q):
	return self.parent

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

