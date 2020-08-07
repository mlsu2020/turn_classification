#Grady Williams
#January 12, 2015
#Program for training an LWPR dynamics model of a fifth-scale rally car

import rosbag
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
import rospy
import roslib
import tf
from math import atan2, atan, asin, tan
import sys, os
import random
import numpy.linalg as la
import copy


#Converts a quaternion to 1-2-3 Euler Angles
def convert_quat_to_euler(quat):
    q0 = quat.w
    q1 = quat.x
    q2 = quat.y
    q3 = quat.z
    #Using the 1-2-3 euler angle convention
    roll = atan2(2*q2*q3 + 2*q0*q1, q3**2 - q2**2 - q1**2 + q0**2)
    pitch = -asin(2*q1*q3 - 2*q0*q2)
    yaw = atan2(2*q1*q2 + 2*q0*q3, q1**2 + q0**2 - q3**2 - q2**2)
    return roll, pitch, yaw


#Converts angular velocity to euler angle derivatives
def convert_angular_to_ederiv(ang_vel, r, p, y):
    conversion_mat = 1/np.cos(p)*np.array([[np.cos(p), np.sin(r)*np.sin(p), np.cos(r)*np.sin(p)],
                                            [0, np.cos(r)*np.cos(p), -np.sin(r)*np.cos(p)],
                                            [0, np.sin(r), np.cos(r)]])
    ang_vel_np = np.array([ang_vel[0], ang_vel[1], ang_vel[2]])
    derivs = np.dot(conversion_mat, ang_vel_np)
    return derivs[0], derivs[1], derivs[2]


def make_arrays():
    #Postion and velocities 
    x = []
    y = []
    x_dot = []
    y_dot = []
    x_ddot = []
    y_ddot = []
    vx = []
    vy = []
    #Euler Angles and derivatives
    roll = []
    pitch = []
    yaw = []
    angV_x = []
    angV_y = []
    angV_z = []
    #Wheels Speeds
    leftFront = []
    rightFront = []
    leftBack = []
    rightBack = []
    #Commands
    throttle_cmd = []
    steering_cmd = []
    frontBrake_cmd = []
    backBrake_cmd = []
    #Time Stamps
    pos_ts = []
    vel_ts = []
    ws_ts = []
    angle_ts = []
    imu_ts = []
    control_ts = []
    
    return x, y, x_dot, y_dot, vx, vy, x_ddot, y_ddot, roll, pitch, yaw, angV_x, angV_y, angV_z, leftFront, rightFront, leftBack, rightBack, throttle_cmd, steering_cmd, frontBrake_cmd, backBrake_cmd, pos_ts, vel_ts, ws_ts, angle_ts, imu_ts, control_ts


def get_bag_data(bag_file, servo_command = "/RC/chassisCommand"):
    x, y, x_dot, y_dot, vx, vy, x_ddot, y_ddot, roll, pitch, yaw, angV_x, angV_y, angV_z, leftFront, rightFront, leftBack, rightBack, throttle_cmd, steering_cmd, frontBrake_cmd, backBrake_cmd, pos_ts, vel_ts, ws_ts, angle_ts, imu_ts, control_ts = make_arrays()

    #Fill state arrays
    for topic, msg, t in bag_file.read_messages(topics = ['/pose_estimate']):
        pos_ts.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0)
        vel_ts.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0)
        angle_ts.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0)
        x.append(msg.pose.pose.position.x)
        y.append(msg.pose.pose.position.y)
        x_dot.append(msg.twist.twist.linear.x)
        y_dot.append(msg.twist.twist.linear.y)
        r,p,ya = convert_quat_to_euler(msg.pose.pose.orientation)
        roll.append(r)
        pitch.append(p)
        yaw.append(ya)
        rot_mat = np.array([[np.cos(ya), np.sin(ya)], [-np.sin(ya), np.cos(ya)]])
        vel_wf = np.array([x_dot[-1], y_dot[-1]])
        vel_bf = np.dot(rot_mat, vel_wf)
        vx.append(vel_bf[0])
        vy.append(vel_bf[1])
        #Get angular velocity *Not* equal to euler angle derivatives
        angV_x.append(msg.twist.twist.angular.x)
        angV_y.append(msg.twist.twist.angular.y)
        angV_z.append(msg.twist.twist.angular.z)

    for topic, msg, t in bag_file.read_messages(topics =['/wheelSpeeds']):
        ws_ts.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0)
        leftFront.append(msg.lfSpeed)
        rightFront.append(msg.rfSpeed)
        leftBack.append(msg.lbSpeed)
        rightBack.append(msg.rbSpeed)

    for topic, msg, t in bag_file.read_messages(topics = ['/imu/imu']):
        imu_ts.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0)
        #Get cartesian velocity
        x_ddot.append(msg.linear_acceleration.x )
        y_ddot.append(msg.linear_acceleration.y )

    #Fill control command arrays
    for topic, msg, t, in bag_file.read_messages(topics = [servo_command]):
        #if (msg.header.frame_id == "RC"):
        control_ts.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0)
        throttle_cmd.append(msg.throttle)
        steering_cmd.append(msg.steering)
        frontBrake_cmd.append(msg.frontBrake)
        backBrake_cmd.append(msg.frontBrake)

    return [x, y, x_dot, y_dot, vx, vy, x_ddot, y_ddot, roll, pitch, yaw, angV_x, angV_y, angV_z, leftFront, rightFront, leftBack, rightBack, throttle_cmd, steering_cmd, frontBrake_cmd, backBrake_cmd, pos_ts, vel_ts, ws_ts, angle_ts, imu_ts, control_ts]


def smooth_data(X, size, sigma):
    filter = np.zeros(size)
    mid = size/2
    filter[mid] = 1.0
    for i in range(1, mid + 1):
        w = np.exp(-.5*(i**2/sigma))
        filter[mid + i] = w
        filter[mid - i] = w
    filter = (1.0/np.sum(filter))*filter
    X_aug = np.zeros(len(X) + size - 1)
    #Fill the beginning of the augmented X matrix with X[0]
    for i in range(size/2):
        X_aug[i] = X[0]
    #Fill the end of the augmented X matrix with the last element of X
    for i in range(len(X), len(X_aug)):
        X_aug[i] = X[-1]
    #Fill the rest with values of X
    for i in range(size/2, len(X_aug) - size/2):
        X_aug[i] = X[i - size/2]
    for i in range(size/2, len(X_aug) - size/2):
        filtered_data = np.dot(filter, X_aug[i - size/2: i + size/2 + 1])
        X[i-size/2] = filtered_data


def interpolate_data(Y, T, spline_pwr = 3):
    #knots = np.linspace(T[0], T[-1], T.size/10.0)[1:-1]
    spline_params = interpolate.splrep(T, Y, k = spline_pwr, s=0)
    return spline_params


def do_smoothing_and_splining(bagfile, servo_command = "/RC/chassisCommand"):
    dataSets = get_bag_data(bagfile, servo_command = servo_command)

    #Smooth all of the data arrays (except time stamps)

    for i in range(len(dataSets) - 15):
        smooth_data(dataSets[i], 15, 5)

    x, y, x_dot, y_dot, vx, vy, x_ddot, y_ddot, roll, pitch, yaw, angV_x, angV_y, angV_z, leftFront, rightFront, leftBack, rightBack, throttle_cmd, steering_cmd, frontBrake_cmd, backBrake_cmd, pos_ts, vel_ts, ws_ts, angle_ts, imu_ts, control_ts = dataSets

    #Convert time into relative units
    t_start = min(pos_ts[0], vel_ts[0], ws_ts[0], angle_ts[0], imu_ts[0], control_ts[0])

    # pos_ts = np.array(pos_ts) - t_start
    # vel_ts = np.array(vel_ts) - t_start
    # ws_ts = np.array(ws_ts) - t_start
    # imu_ts = np.array(imu_ts) - t_start
    # angle_ts = np.array(angle_ts) - t_start
    # control_ts = np.array(control_ts) - t_start
    pos_ts = np.array(pos_ts)
    vel_ts = np.array(vel_ts)
    ws_ts = np.array(ws_ts)
    imu_ts = np.array(imu_ts)
    angle_ts = np.array(angle_ts)
    control_ts = np.array(control_ts)

    #Use splines to smoothly interpolate data
    x_spline = interpolate_data(x, pos_ts)
    y_spline = interpolate_data(y, pos_ts)
    x_dot_spline = interpolate_data(x_dot, vel_ts)
    y_dot_spline = interpolate_data(y_dot, vel_ts)
    vx_spline = interpolate_data(vx, vel_ts)
    vy_spline = interpolate_data(vy, vel_ts)
    x_ddot_spline = interpolate_data(x_ddot, imu_ts)
    y_ddot_spline = interpolate_data(y_ddot, imu_ts)
    roll_spline = interpolate_data(roll, angle_ts)
    pitch_spline = interpolate_data(pitch, angle_ts)
    yaw_spline = interpolate_data(yaw, angle_ts)
    angV_x_spline = interpolate_data(angV_x, pos_ts)
    angV_y_spline = interpolate_data(angV_y, pos_ts)
    angV_z_spline = interpolate_data(angV_z, pos_ts)
    leftFront_spline = interpolate_data(leftFront, ws_ts)
    rightFront_spline = interpolate_data(rightFront, ws_ts)
    leftBack_spline = interpolate_data(leftBack, ws_ts)
    rightBack_spline = interpolate_data(rightBack, ws_ts)
    throttle_cmd_spline = interpolate_data(throttle_cmd, control_ts)
    steering_cmd_spline = interpolate_data(steering_cmd, control_ts)
    frontBrake_cmd_spline = interpolate_data(frontBrake_cmd, control_ts)
    backBrake_cmd_spline = interpolate_data(backBrake_cmd, control_ts)
    
    return {"x": x_spline, "y":y_spline, "vx":vx_spline, "vy":vy_spline, 
            "roll":roll_spline, "pitch":pitch_spline, "yaw":yaw_spline, 
            "angV_x":angV_x_spline, "angV_y":angV_y_spline, "angV_z":angV_z_spline, 
            "leftFront":leftFront_spline, "rightFront":rightFront_spline, "leftBack":leftBack_spline, 
            "rightBack":rightBack_spline, "throttle_cmd":throttle_cmd_spline, "steering_cmd":steering_cmd_spline}, {"pos":pos_ts, "vel":vel_ts, "ws":ws_ts, "angle":angle_ts, "imu":imu_ts, "control":control_ts, "start":t_start}


def verify_smoothing_and_splining(bagfile, servo_command = "/RC/servoCommand", starting_time = None, stopping_time = None):
    dataSets = get_bag_data(bagfile, servo_command = servo_command)

    x, y, x_dot, y_dot, vx, vy, x_ddot, y_ddot, roll, pitch, yaw, angV_x, angV_y, angV_z, leftFront, rightFront, leftBack, rightBack, throttle_cmd, steering_cmd, frontBrake_cmd, backBrake_cmd, pos_ts, vel_ts, ws_ts, angle_ts, imu_ts, control_ts = dataSets

    x_spline, y_spline, x_dot_spline, y_dot_spline, vx_spline, vy_spline, x_ddot_spline, y_ddot_spline, roll_spline, pitch_spline, yaw_spline, angV_x_spline, angV_y_spline, angV_z_spline, leftFront_spline, rightFront_spline, leftBack_spline, rightBack_spline, throttle_cmd_spline, steering_cmd_spline, frontBrake_cmd_spline, backBrake_cmd_spline, pos_ts, vel_ts, ws_ts, angle_ts, imu_ts, control_ts = do_smoothing_and_splining(bagfile, servo_command = servo_command)

    if (starting_time is None):
        starting_time = max(pos_ts[0], ws_ts[0], angle_ts[0], imu_ts[0], control_ts[0])
    if (stopping_time is None):
        stopping_time = min(pos_ts[-1], ws_ts[-1], angle_ts[-1], imu_ts[-1], control_ts[-1])

    timesteps = (stopping_time - starting_time)/.02
    T = np.linspace(starting_time, stopping_time, timesteps)

    """
    x_new = interpolate.splev(T, x_spline)
    plt.plot(T, x_new) 
    plt.plot(pos_ts, x)
    plt.title('x position')
    #plt.savefig('x_pos')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, y_spline)) 
    plt.plot(pos_ts, y)
    plt.title('y position')
    #plt.savefig('y_pos')
    plt.show()   
    plt.clf()
    """

    plt.plot(x, y) 
    #plt.plot(pos_ts, y)
    plt.title('Position')
    #plt.savefig('y_pos')
    plt.show()   
    plt.clf()

    """
    plt.plot(T, interpolate.splev(T, x_dot_spline)) 
    plt.plot(vel_ts, x_dot)
    plt.title('x velocity')
    plt.show()   
    plt.clf()

    plt.plot(T, interpolate.splev(T, y_dot_spline)) 
    plt.plot(vel_ts, y_dot)
    plt.title('y velocity')
    plt.show()   
    plt.clf()

    plt.plot(T, interpolate.splev(T, vx_spline)) 
    plt.plot(vel_ts, vx)
    plt.title('forward velocity')
    plt.show()   
    plt.clf()

    plt.plot(T, interpolate.splev(T, vy_spline)) 
    plt.plot(vel_ts, vy)
    plt.title('lateral velocity')
    plt.show()   
    plt.clf()

    plt.plot(T, interpolate.splev(T, x_ddot_spline)) 
    plt.plot(imu_ts, x_ddot)
    plt.title('x acceleration')
    #plt.savefig('x_ddot')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, y_ddot_spline)) 
    plt.plot(imu_ts, y_ddot)
    plt.title('y acceleration')
    #plt.savefig('y_ddot')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, roll_spline))
    plt.plot(angle_ts, roll)
    plt.title('roll')
    #plt.savefig('roll')
    plt.show()
    plt.clf()
    
    plt.plot(T, interpolate.splev(T, pitch_spline))
    plt.plot(angle_ts, pitch)
    plt.title('pitch')
    #plt.savefig('pitch')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, yaw_spline))
    plt.plot(angle_ts, yaw)
    plt.title('yaw')
    #plt.savefig('yaw')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, angV_x_spline))
    plt.plot(angle_ts, angV_x)
    plt.title('x Angular Velocity')
    #plt.savefig('x_angular_velocity')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, angV_y_spline))
    plt.plot(angle_ts, angV_y)
    plt.title('y Angular Velocity')
    #plt.savefig('y_angular_velocity')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, angV_z_spline))
    plt.plot(angle_ts, angV_z)
    plt.title('z Angular Velocity')
    #plt.savefig('z_angular_velocity')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, leftFront_spline))
    plt.plot(ws_ts, leftFront)
    plt.title('leftFront wheel speed')
    #plt.savefig('leftFront')
    plt.show()
    plt.clf()
 
    plt.plot(T, interpolate.splev(T, rightFront_spline))
    plt.plot(ws_ts, rightFront)
    plt.title('rightFront wheel speed')
    #plt.savefig('rightFront')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, leftBack_spline))
    plt.plot(ws_ts, leftBack)
    plt.title('leftBack wheel speed')
    #plt.savefig('leftBack')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, rightBack_spline))
    plt.plot(ws_ts, rightBack)
    plt.title('rightBack wheel speed')
    #plt.savefig('rightBack')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, throttle_cmd_spline))
    #plt.plot(control_ts, throttle_cmd)
    plt.title('throttle')
    #plt.savefig('throttle')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, steering_cmd_spline))
    #plt.plot(control_ts, steering_cmd)
    plt.title('steering')
    #plt.savefig('steering')
    plt.show()
    plt.clf()
 
    plt.plot(T, interpolate.splev(T, frontBrake_cmd_spline))
    #plt.plot(control_ts, frontBrake_cmd)
    plt.title('frontBrake')
    #plt.savefig('frontBrake')
    plt.show()
    plt.clf()

    plt.plot(T, interpolate.splev(T, backBrake_cmd_spline))
    #plt.plot(control_ts, backBrake_cmd)
    plt.title('backBrake')
    #plt.savefig('backBrake')
    plt.show()
    plt.clf()
    """

if __name__ == "__main__":
    bagfile = rosbag.Bag(os.path.abspath("/home/gwilliams76/Dropbox/Grady/PlatformA_2015-06-07-17-26-23_no_images.bag"))

    x_spline, y_spline, x_dot_spline, y_dot_spline, vx_spline, vy_spline, x_ddot_spline, y_ddot_spline, roll_spline, pitch_spline, yaw_spline, angV_x_spline, angV_y_spline, angV_z_spline, leftFront_spline, rightFront_spline, leftBack_spline, rightBack_spline, throttle_cmd_spline, steering_cmd_spline, frontBrake_cmd_spline, backBrake_cmd_spline, pos_ts, vel_ts, ws_ts, angle_ts, imu_ts, control_ts = do_smoothing_and_splining(bagfile)

    #Test the splining data
    starting_time = max(pos_ts[0], ws_ts[0], angle_ts[0], imu_ts[0], control_ts[0])
    #stopping_time = min(pos_ts[-1], ws_ts[-1], angle_ts[-1], imu_ts[-1], control_ts[-1])
    stopping_time = 650
    timesteps = (stopping_time - starting_time)/.02
    T = np.linspace(starting_time, stopping_time, timesteps)
    
    throttle = interpolate.splev(T, throttle_cmd_spline)
    steering = interpolate.splev(T, steering_cmd_spline) + .076
    plt.plot(T, steering)
    plt.show()
    plt.clf()
    Ux = interpolate.splev(T, vx_spline)
    Uy = interpolate.splev(T, vy_spline) 
    roll = interpolate.splev(T, roll_spline)
    pitch = interpolate.splev(T, pitch_spline)
    yaw = interpolate.splev(T, yaw_spline)
    angV_x = interpolate.splev(T, angV_x_spline)
    angV_y = interpolate.splev(T, angV_y_spline)
    angV_z = interpolate.splev(T, angV_z_spline)
    
    frontBrake = interpolate.splev(T, frontBrake_cmd_spline)
    backBrake = interpolate.splev(T, backBrake_cmd_spline)
    
    plt.plot(T, frontBrake)
    plt.plot(T, backBrake)
    plt.show()

    roll_rate = np.zeros(len(T))
    pitch_rate = np.zeros(len(T))
    yaw_rate = np.zeros(len(T))

    for i in range(len(T)):
        roll_rate[i] = (1/np.cos(pitch[i]))*(np.cos(pitch[i])*angV_x[i] + np.sin(roll[i])*np.sin(pitch[i])*angV_y[i] + np.cos(roll[i])*np.sin(pitch[i])*angV_z[i])
        pitch_rate[i] = (1/np.cos(pitch[i]))*(np.cos(roll[i])*np.cos(pitch[i])*angV_y[i] - np.sin(roll[i])*np.cos(pitch[i])*angV_z[i])
        yaw_rate[i] = (1/np.cos(pitch[i]))*(np.sin(roll[i])*angV_y[i] + np.cos(roll[i])*angV_z[i])

    B1 = interpolate.splev(T, vx_spline, der = 1)
    B2 = interpolate.splev(T, vy_spline, der = 1)
    B3 = np.zeros(len(T))
    for i in range(0, len(T) - 1):
        B3[i] = (yaw_rate[i] - yaw_rate[i + 1])/.02
    B4 = interpolate.splev(T, roll_spline, der = 1)

    #B = interpolate.splev(T, x_ddot_spline)
    smooth_data(B1, 5, 2)
    smooth_data(B2, 5, 2)
    smooth_data(B3, 5, 2)
    smooth_data(B4, 5, 2)
    
    A = np.zeros((len(T), 22))
    for i in range(len(T)):
        A[i,0] = throttle[i]
        A[i,1] = Ux[i]
        alpha_f = 0
        alpha_r = 0
        if (Ux[i] > .1):
            alpha_f = atan(Uy[i]/Ux[i] + .45*yaw_rate[i]/Ux[i]) - steering[i]
            alpha_r = atan(Uy[i]/Ux[i] - .35*yaw_rate[i]/Ux[i])
        else:
            alpha_f = -steering[i]
        sd = np.sin(steering[i])
        A[i,2] = sd*tan(alpha_f)
        A[i,3] = sd*tan(alpha_f)*np.abs(tan(alpha_f))
        A[i,4] = sd*tan(alpha_f)**3
        A[i,5] = yaw_rate[i]*Uy[i]
        A[i,6] = yaw_rate[i]
        A[i,7] = Uy[i]
        A[i,8] = np.sin(steering[i])
        if (Ux[i] > .1):
            A[i,9] = Uy[i]/Ux[i]
        else:
            A[i,9] = 0.0
        A[i,10] = tan(alpha_f)
        A[i,11] = tan(alpha_f)*np.abs(tan(alpha_f))
        A[i,12] = tan(alpha_f)**3
        A[i,13] = tan(alpha_r)
        A[i,14] = tan(alpha_r)*np.abs(tan(alpha_r))
        A[i,15] = tan(alpha_r)**3
        A[i,16] = yaw_rate[i]*Ux[i]
        A[i,17] = 1
        A[i,18] = roll[i]
        A[i,19] = roll[i]*yaw_rate[i]
        A[i,20] = roll[i]*Ux[i]
        A[i,21] = roll[i]*Ux[i]*yaw_rate[i]
    
    """
    A = np.zeros((len(T), 5))
    for i in range(len(T)):
        A[i,0] = throttle[i]
        A[i,1] = steering[i]
        A[i,2] = Ux[i]
        A[i,3] = Uy[i]
        A[i,4] = yaw_rate[i]
    """
        
    theta1 = np.linalg.lstsq(A,B1)[0]
    theta2 = np.linalg.lstsq(A,B2)[0]
    theta3 = np.linalg.lstsq(A,B3)[0]
    theta4 = np.linalg.lstsq(A,B4)[0]

    plt.subplot(2,2,1)
    plt.plot(T, B1, label = 'Actual')
    plt.plot(T, np.dot(A,theta1), label = 'Predicted')
    plt.xlabel('Time')
    plt.ylabel('m/s^2')
    plt.legend()
    plt.title('Forward Acceleration')
 
    plt.subplot(2,2,2)
    plt.plot(T, B2, label = 'Actual')
    plt.plot(T, np.dot(A,theta2), label = 'Predicted')
    plt.xlabel('Time')
    plt.ylabel('m/s^2')
    plt.legend()
    plt.title('Lateral Acceleration')

    plt.subplot(2,2,3)
    plt.plot(T, B3, label = 'Actual')
    plt.plot(T, np.dot(A,theta3), label = 'Predicted')
    plt.xlabel('Time')
    plt.ylabel('rad/s^2')
    plt.legend()
    plt.title('Heading Angular Acceleration')

    plt.subplot(2,2,4)
    plt.plot(T, B4, label = 'Actual')
    plt.plot(T, np.dot(A,theta4), label = 'Predicted')
    plt.xlabel('Time')
    plt.ylabel('rad/sec')
    plt.legend()
    plt.title('Roll Rate')

    plt.show()

    
