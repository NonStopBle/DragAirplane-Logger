import rospy
from std_msgs.msg import Float32MultiArray, Int8
from geometry_msgs.msg import Pose2D, Twist

class Axis:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class IMU:
    def __init__(self):
        self.acceleration = Axis()
        self.gyro = Axis()
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

class GPS:
    def __init__(self):
        self.latitude = 0.0
        self.longitude = 0.0

class Position:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0

class Robot:
    def __init__(self):
        self.imu_sensor = IMU()
        self.position = Position()
        self.motor_left_speed = 0.0
        self.motor_right_speed = 0.0
        self.gps_sensor = GPS()
        self.mode = 0.0

class SerialTranslator:
    def robot_init(self):
        # initialization variable
        self.robot2high_info_topic = "/robot/low/info"
        self.robot2low_odom_topic = "/robot/high/odom"
        self.robot2low_p2p_topic = "/robot/low/send_p2p"
        self.robot2low_cmdVel_topic = "/robot/cmd_vel"
        self.robot2low_cmdMode_topic = "/robot/cmd_mode"

        # Robot2High
        self.robot_info_sub = rospy.Subscriber(self.robot2high_info_topic, Float32MultiArray, self.__robot_info_handler__)

        # Robot2Low
        self.__robot2low_odom_pub = rospy.Publisher(self.robot2low_odom_topic, Pose2D, queue_size=1)
        self.__robot2low_p2p_pub = rospy.Publisher(self.robot2low_p2p_topic, Pose2D, queue_size=1)
        self.__robot2low_cmdVel_pub = rospy.Publisher(self.robot2low_cmdVel_topic, Twist, queue_size=1)
        self.__robot2low_mode_pub = rospy.Publisher(self.robot2low_cmdMode_topic, Int8, queue_size=1)

        # usage variable
        self.robot = Robot()

    def __robot_info_handler__(self, data: Float32MultiArray):
        if len(data.data) > 15:
            # acceleration 
            self.robot.imu_sensor.acceleration.x = data.data[0]
            self.robot.imu_sensor.acceleration.y = data.data[1]
            self.robot.imu_sensor.acceleration.z = data.data[2]
            
            # gyroscope 
            self.robot.imu_sensor.gyro.x = data.data[3]
            self.robot.imu_sensor.gyro.y = data.data[4]
            self.robot.imu_sensor.gyro.z = data.data[5]
            
            # ypr
            self.robot.imu_sensor.yaw = data.data[6]
            self.robot.imu_sensor.pitch = data.data[7]
            self.robot.imu_sensor.roll = data.data[8]

            # robot position
            self.robot.position.x = data.data[9]
            self.robot.position.y = data.data[10]

            # robot speed
            self.robot.motor_left_speed = data.data[11]
            self.robot.motor_right_speed = data.data[12]
            
            # gps data
            self.robot.gps_sensor.latitude = data.data[13]
            self.robot.gps_sensor.longitude = data.data[14]
            self.robot.mode = data.data[15]

    def robot_deserialization(self):
        return self.robot

    def robot_set_mode(self, mode: bool):  # auto true, manual false
        data = Int8()
        data.data = 0 if not mode  else 1
        self.__robot2low_mode_pub.publish(data)

    def robot_send_position(self, position_x : float, position_y : float):
        data = Pose2D()
        data.x = position_x
        data.y = position_y
        self.__robot2low_odom_pub.publish(data)

    def robot_send_p2p(self, position_x : float, position_y : float):
        data = Pose2D()
        data.x = position_x
        data.y = position_y
        rospy.loginfo(f"Publishing position: x={data.x}, y={data.y}")

        self.__robot2low_p2p_pub.publish(data)

    def robot_send_move(self, vx : float, omega : float):
        data = Twist()
        data.linear.x = vx
        data.angular.z = omega
        # rospy.loginfo(f"Publishing velocity: x={vx}, y={omega}")

        self.__robot2low_cmdVel_pub.publish(data)

    def robot_emergency(self , activate : bool) :
        print("")