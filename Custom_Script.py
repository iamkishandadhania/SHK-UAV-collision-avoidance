import airsim
import numpy as np
# import my_controller

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
    print("State: ", xin)

    rotor_state = client.getRotorStates().rotors  
    uin = np.array([rotor_state[0]['speed'], rotor_state[1]['speed'], rotor_state[2]['speed'], rotor_state[3]['speed']])                # choise of attibute : speed, thrust, torque_scalar
    print("Uin :",uin)

    xref = np.zeros((13,1))      # How will we give this input?  

    u_pwm = my_controller(xin, uin, xref)     # Generate individual thrusts

    client.moveByMotorPWMsAsync(u_pwm[0, 0], u_pwm[1, 0], u_pwm[2, 0], u_pwm[3, 0], duration=100*sampling_interval)
    
    airsim.wait_key('Press any key to step simulation')
    client.reset()
    client.enableApiControl(False)