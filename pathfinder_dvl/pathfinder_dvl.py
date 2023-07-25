import serial
import time

import rclpy
from rclpy.node import Node

from kyogre_msgs.msg import DataPoint, DVL


class PathfinderDVL(Node):
    def __init__(self):
        super().__init__('pathfinder_self.dvl')

        # self.dvl data publisher setup
        self.self.dvl_data_publisher_ = self.create_publisher(self.dvl, 'Pathfinderself.dvl/data', 2)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # data point subscription setup
        self.data_point_subscription_ = self.create_subscription(
            DataPoint,
            'Kyogre/data_point',
            self.data_point_callback,
            10)
        self.data_point = DataPoint()

        # Initialize serial connection
        # self.dvl = serial.Serial("/dev/ttyUSB0", 115200) #Ubuntu Serial
        self.dvl = serial.Serial("/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0", 115200) #Ubuntu Serial

        self.dvl.write("===") #self.dvl Break (PathFinder Guide p. 24 and p.99)

        #PD6 settings --------------------------------------------------------------
        self.dvl.write("CR1\r") # set factory defaults.(Pathfinder guide p.67)
        self.dvl.write("CP1\r") # required command
        #PD6 settings --------------------------------------------------------------
        self.dvl.write("PD6\r") #pd6 data format (Pathfinder Guide p.207) <---important
        # self.dvl.write("BK0\r")
        #PD13 settings -------------------------------------------------------------
        # self.dvl.write("PD13\r")
        self.dvl.write("EX11110\r") # coordinate transformation (Pathfinder guide p.124)
        self.dvl.write("EA+4500\r") # heading alignment (Pathfinder guide 118)
        # self.dvl.write("EZ11110010\r") # internal speed of sound, depth, heading, pitch, roll, temperature
        self.dvl.write("EZ11000010\r") # internal speed of sound, depth, temperature
        # self.dvl.write("EZ10000010\r") # default sensor source (Pathfinder guide 125)
        # self.dvl.write("EZ11011010\r") # internal speed of sound, depth, pitch, roll, temperature
        # self.dvl.write("EZ10000010\r") # internal speed of sound, temperature

        self.dvl.write("CK\r") #stores present parameters (Pathfinder guide 114)
        self.dvl.write("CS\r") #start pinning (Pathfinder guide 115)

        #NOTE: the \r character is required for continuous stream i.e (PD6\r")

        self.get_logger().info('Pathfinderself.dvl node has been initialized')

        self.self.dvl_heading = 0
        self.east_trans = 0
        self.north_trans = 0
        self.up_trans = 0
        self.east_vel = 0
        self.north_vel = 0
        self.up_vel = 0
        self.status = 0

        self.pitch = 0
        self.roll = 0
        self.heading = 0

        self.loop_time = time.time()
        self.pubTimePrev = self.loop_time
        self.heading_time_prev = self.loop_time
        self.heading_time_interval = 0.014
        self.mod_val = 0

    def timer_callback(self):
        msg = DVL()
        self.self.dvl_data_publisher_.publish(msg)

        self.loop_time = time.time()

        if self.loop_time - self.heading_time_prev > self.heading_time_interval:
            self.heading_time_prev = self.loop_time
            #and mod_val % 2 == 0
            if self.yaw and self.mod_val % 2 == 0:
                self.mod_val = 1
                self.heading = self.yaw #put heading info ** heree from IMU
                self.heading *= 100 #heading needs to go from 0 to 35999 (see Heading Alignment Pathfinder p.118)
                self.heading_int = int(self.heading)
                if (self.heading - self.heading_int) >= 0.5:
                    self.heading_int += 1
                self.dvl.write("EH " + str(self.heading_int) + ", 1\r") #Update Heading
            if self.pitch and self.roll and self.mod_val % 2 == 1:
                self.mod_val = 0
                self.pitch = self.pitch 
                self.pitch *= 100 
                self.pitch_int = int(self.pitch)
                if (self.pitch - self.pitch_int) >= 0.5:
                    self.pitch_int += 1

                self.roll = self.roll
                self.roll *= 100
                self.roll_int = int(self.oll)
                if (self.roll - self.roll_int) >= 0.5:
                    self.roll_int += 1

                self.dvl.write("EP " + str(self.pitch_int) + ", " + str(self.roll_int) + ", 1\r") #Update Heading
        # print("in loop")
        if self.dvl.in_waiting > 0: #If there is a message from the self.dvl
            try:
                line = self.dvl.readline()
            except:
                rclpy.get_logger().info('Error reading from self.dvl')

            if line[:3] == ":BD": #If the message is a positional update
                line = line.split(",")
                # north_trans = float(line[1])
                # east_trans = float(line[2])
                self.east_trans = float(line[1])
                self.north_trans = float(line[2])
                self.up_trans = float(line[3])
                self.rangeToBottom = float(line[4])
                self.timeDifference = float(line[5])
                #setup msg to be published to ROS
                msg.position.x = self.east_trans
                msg.position.y = self.north_trans
                msg.position.z = self.up_trans
                self.dvl_data_publisher_.publish(msg)

            elif line[:3] == ":BS": #If the message is a velocity update
                line = line.split(",")
                port_vel = float(line[1])
                aft_vel = float(line[2])
                up_vel = float(line[3])
                status = line[4]
                msg.velocity.x = port_vel #need to change msg var names
                msg.velocity.y = aft_vel
                msg.velocity.z = up_vel
                self.dvl_data_publisher_.publish(msg)

            # elif line[:3] == ":SA": #If the message is orientation 
            #     line = line.split(",")
            #     # pitch = float(line[1])
            #     # roll = float(line[2])
            #     print(line[0])
            #     print(line[1])
            #     print(line[2])
            #     print(line[3])
            #     self.dvl_heading = float(line[3])
            #     msgHeading.data = self.dvl_heading
            #     pubHeading.publish(msgHeading)

            # elif line[:3] == ":TS": #If the message is a timestamp
                # line = line.split(",")
                # msgSS.data = float(line[5])
                # pubSS.publish(msgSS)
                
                # print line
                # print "Heading:", heading, "east_vel:", east_vel, "north_vel:", north_vel, "depth_vel:", depth_vel, "Status:", status, "\r"
                
            # print "IMU Heading:", heading, "self.dvl Heading:", self.dvl_heading, "east_vel:", east_vel, "north_vel:", north_vel, "depth_vel:", depth_vel, "Status:", status, "Xpos", east_trans, "YPos", north_trans, "ZPos", depth_trans
            
            # if (loop_time - pubTimePrev) > pubTimeInterval:
            #     pubTimePrev = loop_time

    def data_point_callback(self, data_point_msg):
        # Convert yaw from imu convention to self.dvl convention
        # TODO: Double check conventions for both imu and self.dvl
        # TODO: Use quaternions to convert to euler angles
        if 90 <= data_point_msg.rotation.yaw <= 180:
            data_point_msg.rotation.yaw -= 90
        else:
            data_point_msg.rotation.yaw += 270

        self.data_point = data_point_msg


def main(args=None):
    rclpy.init(args=args)

    pathfinder_dvl = PathfinderDVL()

    rclpy.spin(pathfinder_dvl)

    # Destroy the node explicitly
    pathfinder_dvl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()