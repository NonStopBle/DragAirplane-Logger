import rospy

from SerialTranslator import *
RobotDeserilzation =  SerialTranslator()


def run() :
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown() :
        robot_data : Robot = RobotDeserilzation.robot_deserialization() 

        print(
            f"====================================\n"
            f"----Robot Position/Speed----\n"
            f"Rx: {robot_data.position.x} , Ry: {robot_data.position.y}\n"
            f"ML: {robot_data.motor_left_speed} , MR: {robot_data.motor_right_speed}\n"
            f"-----Robot IMU------\n"
            f"Y: {robot_data.imu_sensor.yaw} , P: {robot_data.imu_sensor.pitch} , R: {robot_data.imu_sensor.roll}\n"
            f"ACC: x: {robot_data.imu_sensor.acceleration.x} , y: {robot_data.imu_sensor.acceleration.y} , z: {robot_data.imu_sensor.acceleration.z}\n"
            f"GYRO: x: {robot_data.imu_sensor.gyro.x} , y: {robot_data.imu_sensor.gyro.y} , z: {robot_data.imu_sensor.gyro.z}\n"
            f"====================================\n"
        )

        RobotDeserilzation.robot_set_mode(False)
        RobotDeserilzation.robot_send_move(0 ,0)

        rate.sleep()

        # rospy.spin()

if __name__ == '__main__':
    try :
        rospy.init_node('robot_control', anonymous=True)
        RobotDeserilzation.robot_init()

        run()
    except rospy.ROSInterruptException :
        pass

