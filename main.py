import rospy
import math
from SerialTranslator import *
RobotDeserilzation =  SerialTranslator()

previousTime = 0
prevJoyRx = 0
prevJoyRy = 0
counterTimer  =0


def run() :
    global prevJoyRx , prevJoyRy , previousTime
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown() :
        robot_data : Robot = RobotDeserilzation.robot_deserialization() 

        # joyControl.joyValueHandler()
        print(
            f"====================================\n"
            f"----Robot Position/Speed----\n"
            f"Rx: {robot_data.position.x:.2f} , Ry: {robot_data.position.y:.2f}\n"
            f"ML: {robot_data.motor_left_speed:.2f} , MR: {robot_data.motor_right_speed:.2f}\n"
            f"XL: {robot_data.motor_left_speed:.2f} , MR: {robot_data.motor_right_speed:.2f}\n"
            
            f"-----Robot IMU------\n"
            f"Y: {robot_data.imu_sensor.yaw:.2f} , P: {robot_data.imu_sensor.pitch:.2f} , R: {robot_data.imu_sensor.roll:.2f}\n"
            f"ACC: x: {robot_data.imu_sensor.acceleration.x:.2f} , y: {robot_data.imu_sensor.acceleration.y:.2f} , z: {robot_data.imu_sensor.acceleration.z:.2f}\n"
            f"GYRO: x: {robot_data.imu_sensor.gyro.x:.2f} , y: {robot_data.imu_sensor.gyro.y:.2f} , z: {robot_data.imu_sensor.gyro.z:.2f}\n"
            f"====================================\n"
        )

        # joyRx = joyControl.joyRxValue()
        # joyRy = -joyControl.joyRyValue()
        # if(abs(joyRx) > 0.03 and abs(joyRy) > 0.03):
        #     joyRx = joyRx 
        #     joyRy = joyRy

        #     joyRyError  = abs(joyRy - prevJoyRy)
        #     joyRxError = abs(joyRx - prevJoyRx)
                
        #         # print(f"{joyRy} {joyRyError} , {joyRxError}")
            
        # else :
        #     if (abs(joyRx) < 0.03) :
        #         joyRx = 0
            
        #     if(abs(joyRy) < 0.03):
        #         joyRy = 0
                


        # if(rospy.get_time() - previousTime > 1) :
            

        #     # counterTimer+=1

        #     prevJoyRx =  joyRx
        #     prevJoyRy =  joyRy

        #     # print("sampling")
        
        #     previousTime = rospy.get_time()

        
        # RobotDeserilzation.robot_send_move(joyRy ,joyRx * 0.0035)
        
        rate.sleep()

        # rospy.spin()

if __name__ == '__main__':
    try :
        rospy.init_node('robot_control', anonymous=True)
        # joyControl.joyScan()
        RobotDeserilzation.robot_init()

        run()
    except rospy.ROSInterruptException :
        pass

