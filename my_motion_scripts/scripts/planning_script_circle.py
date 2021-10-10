#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import rosbag

class Line_Marker(object):
    def __init__(self, marker_topic, queue_size, orien_w, orien_x, orien_y, orien_z, pos_x, pos_y, pos_z):
        self._marker_topic = marker_topic
        self._publisher = rospy.Publisher(self._marker_topic, Marker, queue_size=queue_size)

        self._marker = Marker()
        self._marker.lifetime = rospy.Duration()
        self._marker.header.frame_id = "/base_link"
        self._marker.type = self._marker.LINE_STRIP
        self._marker.action = self._marker.ADD
        self._marker.scale.x = 0.05
        self._marker.color.a = 1.0
        self._marker.color.r = 0.0
        self._marker.color.g = 1.0
        self._marker.color.b = 0.0
        self._marker.pose.orientation.w = orien_w
        self._marker.pose.orientation.x = orien_x
        self._marker.pose.orientation.y = orien_y
        self._marker.pose.orientation.z = orien_z
        self._marker.pose.position.x = pos_x
        self._marker.pose.position.y = pos_y
        self._marker.pose.position.z = pos_z
        self._marker.points = []

    def add_new_point(self, x, y, z):
        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = z
        self._marker.points.append(pt)

    def get_marker_ptr(self):
        return self._marker

    def publish_marker(self):
        rospy.sleep(20)
        print("publishing line marker ...")
        self._publisher.publish(self._marker)

    def write_to_rosbag(self, bag_name):
        print("writing marker to rosbag (circle)...")
        bag = rosbag.Bag(bag_name, 'w')
        try:
            bag.write(self._marker_topic, self._marker, rospy.Time.now())
        finally:
            bag.close()
    
    def read_from_rosbag_and_publish(self, bag_name):
        print("reading marker from rosbag (circle)...")
        msg_from_bag = None
        marker_bag = rosbag.Bag(bag_name)
        # read all messages from the indicated topics:
        for topic, msg, t in marker_bag.read_messages(topics=[self._marker_topic]):
            if topic == self._marker_topic:
                msg_from_bag = msg
        self._publisher.publish(msg_from_bag)

# initialise the moveit_commander module (which allows us to communicate with the MoveIt Rviz interface)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_1', anonymous=True)

# create RobotCommander obj (an interface to our robot):
robot = moveit_commander.RobotCommander()
# create PlanningSceneInterface obj (an interface to the world that surrounds the robot):
scene = moveit_commander.PlanningSceneInterface()
# create MoveGroupCommander obj to allow us to interact with the 'manipulator' set of joints:
group = moveit_commander.MoveGroupCommander("manipulator")
# create tomath.pic publisher to allow MoveIt Rviz to visualise our planned motion:
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
                                                moveit_msgs.msg.DisplayTrajectory, 
                                                queue_size=20)

marker_topic = 'visualization_marker'
line_marker = Line_Marker(marker_topic=marker_topic, orien_w=1.0, 
                    orien_x=0, orien_y=0, orien_z=0, pos_x=0, 
                    pos_y=0, pos_z=0, queue_size=10)

# set joints to home position first:
print("bringing robot to home position ...")
joint_goal = group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -math.pi/2
joint_goal[2] = math.pi/2
joint_goal[3] = -math.pi/2
joint_goal[4] = -math.pi/2
joint_goal[5] = 0
group.go(joint_goal, wait=True)
group.stop()


# draw a circle:
waypoints = []
scale = 1.5
wpose = group.get_current_pose().pose
x_center = wpose.position.x
y_center = wpose.position.y
radius = 0.2

# 1st move down onto drawing plane:
wpose.position.z -= scale * 0.2
wpose.position.x += radius # move from centre to one end of the circle
waypoints.append(copy.deepcopy(wpose))
line_marker.add_new_point(wpose.position.x, wpose.position.y, wpose.position.z)

print("generating points along circumference ...")
# tolerances required to execute this is beyond capabilities of controller!
# find a way for ur5 to draw curves 
for i in range(360/2): 
    theta = i*math.pi/(180/2) # rad
    wpose.position.x = x_center + radius*math.cos(theta)
    wpose.position.y = y_center + radius*math.sin(theta)
    waypoints.append(copy.deepcopy(wpose))
    line_marker.add_new_point(wpose.position.x, wpose.position.y, wpose.position.z)
    

# 5th move away from drawing plane:
wpose.position.z += scale * 0.2 # 6th move down (z)
wpose.position.x -= radius # move from other end of circle back to centre
waypoints.append(copy.deepcopy(wpose))


# Here we want cartesian path to be interpolated at a resoltion of 1cm, 
# hence eef_step=0.01 in cartesian translation.
# Here we ignore checks for infeasible jumps in joint space
(plan, fraction) = group.compute_cartesian_path( 
                                        waypoints=waypoints, # waypoints to follow
                                        eef_step=0.01,        # eef_step
                                        jump_threshold=0.0)  # jump_threshold

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# publish:
display_trajectory_publisher.publish(display_trajectory)

'''
## ONLY USE THE BELOW CODE WHEN RUNNING .PY WITHIN SCRIPS DIRECTORY! 
# (env in shbang does not have the necessary bag_msg_<shape> folders)
## --- ROSBAG CODE (WRITE): ----
print("writing trajectory to rosbag (circle)...")
bag = rosbag.Bag('bag_msgs_circle/display_trajectory_circle.bag', 'w')
try:
    bag.write('/move_group/display_planned_path', display_trajectory, rospy.Time.now())
finally:
    bag.close()
## --- ROSBAG CODE (READ): do not run at same time with rosbag write code! ---
print("reading trajectory from rosbag (circle)...")
msg_from_bag = None
circle_bag = rosbag.Bag('bag_msgs_circle/display_trajectory_circle.bag')
# read all messages from the indicated topics:
for topic, msg, t in circle_bag.read_messages(topics=['/move_group/display_planned_path']):
    if topic == '/move_group/display_planned_path':
        msg_from_bag = msg
display_trajectory_publisher.publish(msg_from_bag)
'''

print("drawing shape trajectory ...")
line_marker.publish_marker()
'''
## ONLY USE THE BELOW CODE WHEN RUNNING .PY WITHIN SCRIPS DIRECTORY!
# (env in shbang does not have the necessary bag_msg_<shape> folders)
line_marker.write_to_rosbag('bag_msgs_circle/marker_bag_circle.bag')
line_marker.read_from_rosbag_and_publish('bag_msgs_circle/marker_bag_circle.bag')
'''

print("executing plan ...")
group.execute(plan, wait=True)

rospy.sleep(5)

print("finished, going to sleep now")
moveit_commander.roscpp_shutdown()

