import airsim
import numpy as np
# import my_controller

import sys
import math
import time
import argparse
import pprint
import numpy

def my_controller(xin, uin, xref):                            
    # xin - (2-D numpy array: 13x1): current quadrotor kinematic state
    # uin - (2-D numpy array: 4x1):  current rotor state (rotation speeds of 4 rotors / or thrust of 4 rotors)
    # xref -(2-D numpy array: 13x1)
    # u_pwm (2-D numpy array: 4x1):  Generated input for moveByMotorPWMsAsync(u_pwm)
    u_pwm = np.array()
    return u_pwm

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

airsim.wait_key('Press any key to takeoff')
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()
client.hoverAsync().join()

sampling_interval = 0.01    # 10ms frequency

while True:

    state = client.getMultirotorState()
    xin = np.array([ state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val, state.kinematics_estimated.position.z_val,
                     state.kinematics_estimated.linear_velocity.x_val, state.kinematics_estimated.linear_velocity.y_val, state.kinematics_estimated.linear_velocity.z_val,
                     state.kinematics_estimated.orientation.w_val, state.kinematics_estimated.orientation.x_val, state.kinematics_estimated.orientation.y_val, state.kinematics_estimated.orientation.z_val,
                     state.kinematics_estimated.angular_velocity.x_val, state.kinematics_estimated.angular_velocity.y_val, state.kinematics_estimated.angular_velocity.z_val])
    #print("State: ", xin)

    rotor_state = client.getRotorStates().rotors  
    uin = np.array([rotor_state[0]['speed'], rotor_state[1]['speed'], rotor_state[2]['speed'], rotor_state[3]['speed']])                # choise of attibute : speed, thrust, torque_scalar
    print("Uin :",uin)

    xref = np.zeros((13,1))      # How will we give this input?  

    # u_pwm = my_controller(xin, uin, xref)     # Generate individual thrusts

    # client.moveByMotorPWMsAsync(u_pwm[0, 0], u_pwm[1, 0], u_pwm[2, 0], u_pwm[3, 0], duration=100*sampling_interval)
    
    # set Lidar sensor parameters
    # lidar_data = airsim.LidarData()
    # lidar_data.setPointCloud(lidar_point_cloud)  # set Lidar point cloud data
    # lidar_data.pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(0, 0, 0))  # set Lidar pose
    # lidar_data.range = 10  # set Lidar range to 10 meters
    # client.simSetLidarData(lidar_data)  # send Lidar data to simula

    class LidarTest:

    # def __init__(self):

       # connect to the AirSim simulator
       # self.client = airsim.MultirotorClient()
       # self.client.confirmConnection()
       # self.client.enableApiControl(True)

     def execute(self):

        print("arming the drone...")
        self.client.armDisarm(True)

        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        #print("state: %s" % s)

        airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()

        state = self.client.getMultirotorState()
        #print("state: %s" % pprint.pformat(state))

        airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
        self.client.moveToPositionAsync(-10, 10, -10, 5).join()

        self.client.hoverAsync().join()

        airsim.wait_key('Press any key to get Lidar readings')
        
        for i in range(1,5):
            lidarData = self.client.getLidarData();
            if (len(lidarData.point_cloud) < 3):
                print("\tNo points received from Lidar data")
            else:
                points = self.parse_lidarData(lidarData)
                print("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidarData.time_stamp, len(points)))
                print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
                print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))
            time.sleep(5)

     def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

     def write_lidarData_to_disk(self, points):
        # TODO
        print("not yet implemented")

     def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")

     # main
    if __name__ == "__main__":
       args = sys.argv
       args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)
  
    args = arg_parser.parse_args(args)    
    lidarTest = LidarTest()
    try:
        lidarTest.execute()
    finally:
        lidarTest.stop()

    airsim.wait_key('Press any key to step simulation')
    client.reset()
    client.enableApiControl(False)