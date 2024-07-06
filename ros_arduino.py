import os
import rclpy
import serial
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

ser =serial.Serial('/dev/ttyACM0',9600,timeout=1)
c=0
class RosArduino(Node):
	def __init_(self):
		super().__init__("ros_arduino")
		self.stint_pub_ = self.create_publisher(Pose, "/navbot_stint",10)
		self.pose_subs_ = self.create_subscription(Twist, "cmd_vel",self.callback_func ,10)
		self.get_logger().info("The ros_arduino node has been started")
		
	def callback_func(Self, twist:Twist):
	cmd =Pose()
	#read from arduino
	pos_values = ser.readline().decode('utf-8').rstrip()
	print(pos_values)
	for i in pos_values:
		if(i == " " and c == 0):
			y_pos.append(i)
			c =1
		else:
			x_pos.append(i)
	for i in vel_values
	left_pos =read.(motor_pos_l)
	right_pos=read.(motor_pos_r)
	left_vel=read.(motor_vel_l)
	right_vel=read.(motor_vel_r)
	#feedback publish but it recieves the position in x and y.
	cmd.linear_l_v = left_vel
	cmd.linear_r_v = right_vel
	
	self.cmd_vel_pub_.publish(cmd)
	#recieved subscription transferred to arduino
	## twist.linear.x_l
	## twist.linear.y_l
	linear_l = sqrt(twist.linear.x_l **2 ,twist.linear.y_l **2)
	angular_l = sqrt(twist.angular.x_l **2 ,twist.angular.y_l **2)	
	## twist.linear.x_r
	## twist.linear.y_r
        linear_r = sqrt(twist.linear.x_r **2, twist.linear.y_r **2)
        angular_r = sqrt(twist.angular.x_r **2, twist.angular.y_r **2) 
             
        ser.write(angular_r)
        ser.write(angular_l)
        wite(angular_l, angular_r)
	return (x,y)
	
	
def main(args=None):
	rclpy.init()
	node= RosArduino()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
