import rospy
import csv
import time

from sensor_msgs.msg import NavSatFix

from SerialTranslator import *

RobotDeserilzation = SerialTranslator()

csv_file = open("robotpmind_data_log.csv", mode="w", newline="")
csv_writer = csv.writer(csv_file)


csv_writer.writerow([
    "Time", "Lat" , "Lng" ,"Position_X", "Position_Y", "Motor_Left_Speed", "Motor_Right_Speed",
    "IMU_Yaw", "IMU_Pitch", "IMU_Roll", "Accel_X", "Accel_Y", "Accel_Z",
    "Gyro_X", "Gyro_Y", "Gyro_Z" 
])


getLatitude = 0.00
getLongitude =0.00

def gpsCallback (data : NavSatFix) :
    global getLatitude , getLongitude
    getLatitude = data.latitude
    getLongitude = data.longitude

def run():
    rate = rospy.Rate(100)  # 100 Hz
    previous_time = time.time() 
    
    while not rospy.is_shutdown():
        robot_data: Robot = RobotDeserilzation.robot_deserialization()

        current_time = time.time()
        if current_time - previous_time >= 0.1:  # Recording 1 seconds
            csv_writer.writerow([
                rospy.get_time(),
                getLatitude,getLongitude,
                robot_data.position.x, robot_data.position.y,
                robot_data.motor_left_speed, robot_data.motor_right_speed,
                robot_data.imu_sensor.yaw, robot_data.imu_sensor.pitch, robot_data.imu_sensor.roll,
                robot_data.imu_sensor.acceleration.x, robot_data.imu_sensor.acceleration.y, robot_data.imu_sensor.acceleration.z,
                robot_data.imu_sensor.gyro.x, robot_data.imu_sensor.gyro.y, robot_data.imu_sensor.gyro.z
            ])
            previous_time = current_time  

        print(
            f"=============Recording==================\n"
            f"----Robot GPS Position----\n"
            f"Lat: {getLatitude} , Lng: {getLongitude}\n"
            f"----Robot Position/Speed----\n"
            f"Rx: {robot_data.position.x:.2f} , Ry: {robot_data.position.y:.2f}\n"
            f"ML: {robot_data.motor_left_speed:.2f} , MR: {robot_data.motor_right_speed:.2f}\n"
            f"-----Robot IMU------\n"
            f"Y: {robot_data.imu_sensor.yaw:.2f} , P: {robot_data.imu_sensor.pitch:.2f} , R: {robot_data.imu_sensor.roll:.2f}\n"
            f"ACC: x: {robot_data.imu_sensor.acceleration.x:.2f} , y: {robot_data.imu_sensor.acceleration.y:.2f} , z: {robot_data.imu_sensor.acceleration.z:.2f}\n"
            f"GYRO: x: {robot_data.imu_sensor.gyro.x:.2f} , y: {robot_data.imu_sensor.gyro.y:.2f} , z: {robot_data.imu_sensor.gyro.z:.2f}\n"
            f"====================================\n"
        )



        rate.sleep()


    csv_file.close()

if __name__ == '__main__':
    try:
        rospy.init_node('robot_control', anonymous=True)
        rospy.Subscriber('/gps/fix' , NavSatFix ,gpsCallback )
        RobotDeserilzation.robot_init()

        run()
    except rospy.ROSInterruptException:
        csv_file.close()  # Ensure file is closed in case of interruption