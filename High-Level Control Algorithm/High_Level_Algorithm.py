#!/usr/bin/env python3
import smbus
from _datetime import datetime
import math as mt
import numpy as np
from numpy import pi
import Jetson.GPIO as GPIO
import time
import cv2
from multiprocessing import Lock, Manager, Process, Value

manager = Manager()
final_array = manager.list()
output_pin = 12
inPin = 13
servo_first_pin = 16
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(output_pin, GPIO.OUT)
GPIO.output(output_pin, GPIO.HIGH)
GPIO.setup(inPin, GPIO.IN)
GPIO.setup(servo_first_pin, GPIO.OUT)
GPIO.output(servo_first_pin, GPIO.HIGH)


# Nvidia Jetson TX2 i2c Bus 0
bus = smbus.SMBus(0)
time.sleep(1)

# This is the address we setup in the Tiva C Program
address = 0x30
start_time = datetime.now()

# I2C Data Sending
theta_SplitData = np.zeros([3])
vel_acc_SplitData = np.zeros([2])
N_DECIMAL_POINTS_PRECISION = 100

# Transformation
transformation_Matrix = np.array([[0, -1, 0, -60], [1, 0, 0, -131], [0, 0, 1, 100], [0, 0, 0, 1]])
robot_positions_reshaped = np.array([0, 0, 0])
Servo_angle = 0
camera_position = np.array([])
camera_input = np.array([])
camera_input_old = np.array([])

# Product identification
redParts = 0
greenParts = 0
redflag = 0
greenflag = 0
z_red_counter = 0
z_green_counter = 0
i_initialization = 0
servo_first_time = 0

solenoid_ON = 0
solenoid_OFF = 0

global time_counter, Sd, Vd, Ad, Jd
path_iterator = 0
old_x = 0
old_y = 0
old_z = 0
y_target = 0
z_target = 0
product_iterator = 0
test_flag = 0


def camera_input_data():
    global Servo_angle, camera_position, redParts, greenParts, camera_input, redflag, greenflag
	camera_input = np.array([])
    temp_array = np.array([0, 1])
    camera_input = camera_input[np.argsort(camera_input[:, 0])]
    camera_position = np.append(camera_position, camera_input[0][:2])
    camera_position = np.append(camera_position, temp_array)
    Servo_angle = int(0 + (96/180) * (180 - camera_input[0][2]))
    if camera_input[0][3] == 0:
        redParts += 1
        redflag = 1
        greenflag = 0
    else:
        greenParts += 1
        greenflag = 1
        redflag = 0

def transformation():
    global transformation_Matrix, robot_positions_reshaped, i_initialization, camera_position
    robot_positions = np.array([])
    robot_positions = np.delete(np.dot(transformation_Matrix, camera_position), 3)
    robot_positions_reshaped = robot_positions


def inverse_kinematics(coord_array):
    x = np.array([])
    z = np.array([])
    y = np.array([])
    theta_to_path_plan = np.array([])
    for j in range(coord_array.shape[0]):
        x = np.append(x, coord_array[j][0])
        y = np.append(y, coord_array[j][1])
        z_temp = coord_array[j][2]
        z_after = -(289.83 + z_temp + 89)
        z = np.append(z, z_after)
    theta = np.zeros(3)
    f = 150  # base radius
    e = 60  # end effector radius
    Lb = 410  # lower limb length
    La = 200  # upper limb length

    endEffectorOffset = -101.85

    for t in range(len(coord_array)):
        for n in range(0, 5, 2):
            alpha = n * pi / 3
            x_ = x[t] * mt.cos(alpha) - y[t] * mt.sin(alpha)
            y_ = x[t] * mt.sin(alpha) + y[t] * mt.cos(alpha)
            z_ = z[t] - endEffectorOffset
            T = f - e - x_
            K = (Lb ** 2) - (y_ ** 2) - (La ** 2) - (z_ ** 2) - (T ** 2)
            e1 = K + 2 * T * La
            e2 = -4 * z_ * La
            e3 = K - 2 * T * La
            p = [e1, e2, e3]
            r = np.roots(p)
            theta[int(n / 2)] = np.round(mt.atan(r[1]) * 360 / pi, 2)
        theta_to_path_plan = np.append(theta_to_path_plan, theta)
    theta_to_path_plan = np.reshape(theta_to_path_plan, (coord_array.shape[0], coord_array.shape[1]))
    return theta_to_path_plan


# Init Variables
def initialize_Variables(t0, t1, d0, d1, v0, v1, a0, a1):
    C3 = (1 / (2 * ((t1 - t0) ** 3))) * (
            (20 * (d1 - d0)) - (((8 * v1) + (12 * v0)) * (t1 - t0)) - (((3 * a1) - a0) * ((t1 - t0) ** 2)))
    C4 = (1 / (2 * ((t1 - t0) ** 4))) * (
            (30 * (d0 - d1)) + (((14 * v1) + (16 * v0)) * (t1 - t0)) + (((3 * a1) - (2 * a0)) * ((t1 - t0) ** 2)))
    C5 = (1 / (2 * ((t1 - t0) ** 5))) * (
            (12 * (d1 - d0)) - (((6 * v1) + (6 * v0)) * (t1 - t0)) - ((a1 - a0) * ((t1 - t0) ** 2)))
    return C3, C4, C5


def path_planner(t_final, d_init, d_final):
    global time_counter, Sd, Vd, Ad, Jd, path_iterator

    D0 = d_init
    D1 = d_final
    V0 = np.array([0, 0, 0])
    V1 = np.array([0, 0, 0])
    Acc0 = np.array([0, 0, 0])
    Acc1 = np.array([0, 0, 0])

    T0 = 0
    Tf = t_final
    A0 = D0
    A1 = V0
    A2 = 0.5 * Acc0

    A3, A4, A5 = initialize_Variables(T0, Tf, D0, D1, V0, V1, Acc0, Acc1)

    if path_iterator == 0:
        time_counter = 0
        Sd = D0
        Vd = V0
        Ad = Acc0
        Jd = np.array([1, 1, 1]) * (6 * A3)
        path_iterator += 1
    time_counter = time_counter + t_final

    T = np.arange(start=T0, stop=Tf + 0.001, step=0.025)

    for i in range(len(T) - 1):
        Sd = np.vstack(
            (Sd, (A0 + (A1 * (T[i + 1] - T[0])) + (A2 * ((T[i + 1] - T[0]) ** 2)) + (A3 * ((T[i + 1] - T[0]) ** 3)) + (
                    A4 * ((T[i + 1] - T[0]) ** 4)) + (A5 * ((T[i + 1] - T[0]) ** 5)))))
        Vd = np.vstack((Vd, A1 + (2 * A2 * (T[i + 1] - T[0])) + (3 * A3 * ((T[i + 1] - T[0]) ** 2)) + (
                4 * A4 * ((T[i + 1] - T[0]) ** 3)) + (5 * A5 * ((T[i + 1] - T[0]) ** 4))))  # Desired velocity
        Ad = np.vstack((Ad, (2 * A2) + (6 * A3 * (T[i + 1] - T[0])) + (12 * A4 * ((T[i + 1] - T[0]) ** 2)) + (
                20 * A5 * ((T[i + 1] - T[0]) ** 3))))  # Desired acceleration
        Jd = np.vstack((Jd, (6 * A3) + (24 * A4 * ((T[i + 1] - T[0]) ** 1)) + (
                60 * A5 * ((T[i + 1] - T[0]) ** 2))))  # Desired jerk


def splitting_data_theta(theta_data_tobe_split):
    global theta_SplitData
    if theta_data_tobe_split < 0:
        theta_SplitData[2] = 1
        theta_data_tobe_split *= -1
    elif theta_data_tobe_split >= 0:
        theta_SplitData[2] = 0

    intPart = int(theta_data_tobe_split)
    theta_SplitData[0] = (intPart & 0xFF)
    theta_SplitData[1] = intPart >> 8


def splitting_data_vel_acc(vel_acc_data_tobe_split):
    global vel_acc_SplitData
    if vel_acc_data_tobe_split < 0:
        vel_acc_data_tobe_split *= -1
    intPart = int(vel_acc_data_tobe_split)
    vel_acc_SplitData[0] = (intPart & 0xFF)
    vel_acc_SplitData[1] = intPart >> 8


def writeNumber(value):
    bus.write_byte(address, value)
    return -1


def readNumber():
    number = bus.read_byte(address)
    return number


def trajectory(target_position):
    global product_iterator, old_x, old_y, old_z, y_target, z_target
    global solenoid_OFF, solenoid_ON
    global Sd, servo_first_time
    global redParts, redflag, z_red_counter, greenflag, greenParts, z_green_counter
    speed = 500
    offset = 65
    z_variable = 97

    if (product_iterator == 0 or product_iterator % 2 == 0) and redflag == 1:
        y_target = 25 + (redParts - 1) * 50
        z_target = 100 - 4 * z_red_counter
    elif (product_iterator == 0 or product_iterator % 2 == 0) and greenflag == 1:
        y_target = -25 - (greenParts -1) * 50
        z_target = 100 - 4 * z_green_counter
    if redParts == 3 and y_target == 25 + (redParts - 1) * 50 :
        z_red_counter += 1
        redParts = 0
    if greenParts == 3 and y_target == -25 - (greenParts -1) * 50:
        z_green_counter += 1
        greenParts = 0

    if product_iterator % 2 != 0:
        points = np.array([[old_x, old_y, old_z],
                           [old_x, old_y, old_z - 35],
                           [target_position[0], target_position[1] - offset, z_variable - 35],
                           [target_position[0], target_position[1] - offset, z_variable]])
        degrees = inverse_kinematics(points)
        for i in range(len(points) - 1):
            if i == 0:
                travel_time = np.linalg.norm((points[1] - points[0])) / (0.35 * speed)
            elif i == 1:
                travel_time = np.linalg.norm((points[2] - points[1])) / (1.325 * speed) 
            else:
                solenoid_ON = Sd.shape[0] - 5
                travel_time = np.linalg.norm((points[3] - points[2])) / (0.35 * speed)
            path_planner(travel_time, degrees[i], degrees[i + 1])
            if i == 2:
                solenoid_OFF = 255
    elif product_iterator == 0:
        points = np.array([[0, 0, 0], [target_position[0] , target_position[1] - offset, z_variable - 45],
                           [target_position[0] , target_position[1] - offset, z_variable],
                           [target_position[0] , target_position[1] - offset, z_variable - 35],
                           [100, y_target, z_target - 35],
                           [100, y_target, z_target]])
        degrees = inverse_kinematics(points)
        for i in range(len(points) - 1):
            if i == 0:
                travel_time = np.linalg.norm((points[1] - points[0])) / speed
            elif i == 1:
                solenoid_ON = Sd.shape[0] - 5
                travel_time = np.linalg.norm((points[2] - points[1])) / (0.39 * speed)
            elif i == 2:
                travel_time = np.linalg.norm((points[3] - points[2])) / (0.25 * speed)
                servo_first_time = Sd.shape[0]
            elif i == 3:
                travel_time = np.linalg.norm((points[4] - points[3])) / (1.325 * speed) 
            else:
                travel_time = np.linalg.norm((points[5] - points[4])) / (0.325 * speed)
            path_planner(travel_time, degrees[i], degrees[i + 1])
            if i  == 4:
                solenoid_OFF = Sd.shape[0] - 2
    else:
        points = np.array([[target_position[0], target_position[1] - offset, z_variable],
                           [target_position[0], target_position[1] - offset, z_variable - 35],
                           [100, y_target, z_target - 35],
                           [100, y_target, z_target]])
        degrees = inverse_kinematics(points)
        for i in range(len(points) - 1):
            if i == 0:
                travel_time = np.linalg.norm((points[1] - points[0])) / (0.35 * speed)
            elif i == 1:
                travel_time = np.linalg.norm((points[2] - points[1])) / (1.325 * speed) 
            else:
                travel_time = np.linalg.norm((points[3] - points[2])) / (0.35 * speed)
            path_planner(travel_time, degrees[i], degrees[i + 1])
            if i == 2:
                solenoid_OFF = Sd.shape[0] - 2
                solenoid_On = 255
    if product_iterator % 2 == 0:
        old_x = 100
        old_y = y_target
        old_z = z_target


# path Planning
def path_planning_code(data, l_data, Path_Camera_Lock_Value, Path_Camera_LocK, Product_Index_Value, Product_Index_Lock):
    global test_flag, robot_positions_reshaped, Servo_angle
    global theta_SplitData, vel_acc_SplitData
    global Sd, Vd, Ad, path_iterator, product_iterator
    global solenoid_OFF, solenoid_ON, servo_first_time
    global camera_position
    global camera_input, camera_input_old
    global output_pin, inPin, servo_first_pin

    gearRatio = 5
    microStep = 8
    anglePerStep = 1.2

    while True:
        if Path_Camera_Lock_Value.value == 1:
            l_data.acquire()
            camera_input = np.array(data)
            l_data.release()
			if product_iterator == 0:
				camera_input_old = camera_input
			else:
				if (camera_input == camera_input_old).all():
					Path_Camera_LocK.acquire()
                    Path_Camera_Lock_Value.value = 0
                    Path_Camera_LocK.release()
				else:
					camera_input_old = camera_input
			if Path_Camera_Lock_Value.value == 1:
				camera_input_data()
				transformation()
            while Path_Camera_Lock_Value.value == 1:
                if robot_positions_reshaped.shape[0] != 0:
                    trajectory(robot_positions_reshaped)
                    Sd = Sd * ((gearRatio * microStep) / anglePerStep)
                    if product_iterator == 0 or product_iterator % 2 == 0:
                        robot_positions_reshaped = np.array([])
                        camera_position = np.array([])
                    writeNumber(int(Sd.shape[0]))
                    if product_iterator != 0 and product_iterator % 2 == 0:
                        writeNumber(45)
                    if product_iterator == 0 or product_iterator % 2 != 0:
                        writeNumber(int(Servo_angle))
                        Servo_angle = 0
                    while test_flag < 3:
                        for i in range(Sd.shape[0]):
                            for j in range(Sd.shape[1]):
                                if j == 0:
                                    splitting_data_theta(Sd[i, test_flag])
                                    for k in range(theta_SplitData.shape[0]):
                                        writeNumber(int(theta_SplitData[k]))
                                elif j == 1:
                                    splitting_data_vel_acc(Vd[i, test_flag])
                                    for k in range(vel_acc_SplitData.shape[0]):
                                        writeNumber(int(vel_acc_SplitData[k]))
                                elif j == 2:
                                    splitting_data_vel_acc(Ad[i, test_flag])
                                    for k in range(vel_acc_SplitData.shape[0]):
                                        writeNumber(int(vel_acc_SplitData[k]))
                            if i == solenoid_ON:
                                GPIO.output(output_pin, GPIO.LOW)
                            if i == solenoid_OFF:
                                GPIO.output(output_pin, GPIO.HIGH)
                            if product_iterator == 0 and i == servo_first_time:
                                GPIO.output(servo_first_pin, GPIO.LOW)
                            if product_iterator == 0 and i == servo_first_time + 1:
                                GPIO.output(servo_first_pin, GPIO.HIGH)
                        solenoid_OFF = 255
                        solenoid_ON = 255
                        test_flag += 1
                    product_iterator += 1
                    Product_Index_Lock.acquire()
                    Product_Index_Value.value = product_iterator
                    Product_Index_Lock.release()
                    test_flag = 0
                    path_iterator = 0
                if robot_positions_reshaped.shape[0] == 0:
                    Path_Camera_LocK.acquire()
                    Path_Camera_Lock_Value.value = 0
                    Path_Camera_LocK.release()


# camera
def camera_code(data, l_data, Path_Camera_Lock_Value, Path_Camera_LocK, Product_Index_Value, Product_Index_Lock):
    disW = 1280
    disH = 720
    cam = cv2.VideoCapture("/dev/video1")
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, disH)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, disW)
    
    BL_x = 218 
    BL_y = 411 
    BR_x = 1089 
    BR_y = 415 
    UL_x = 274 
    UL_y = 198
    UR_x = 1036 
    UR_y = 192

    width = 296
    height = 100
    pts1 = np.float32([[UL_x, UL_y], [UR_x, UR_y], [BL_x, BL_y], [BR_x, BR_y]])
    pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)

    hue_Lower_1 = 30
    hue_Upper_1 = 90
    hue2_Lower_1 = 30
    hue2_Upper_1 = 90
    sat_Low_1 = 0
    sat_High_1 = 202
    val_Low_1 = 172
    val_High_1 = 255
    hue_Lower_2 = 0
    hue_Upper_2 = 40
    hue2_Lower_2 = 155
    hue2_Upper_2 = 179
    sat_Low_2 = 13
    sat_High_2 = 255
    val_Low_2 = 165
    val_High_2 = 255
    
    while True:
        ret, frame_2 = cam.read()
        frame_2 = cv2.flip(frame_2,0)
        frame_2 = cv2.flip(frame_2,1)
        frame = cv2.warpPerspective(frame_2, matrix, (width, height))
        Product_Index_Lock.acquire()
        toggle_camera = Product_Index_Value.value
        Product_Index_Lock.release()
        if Path_Camera_Lock_Value.value == 0 and GPIO.input(inPin) == 1 and (toggle_camera == 0 or toggle_camera % 2 != 0):
            test_time = datetime.now()
            positions_green = np.array([])
            frame = cv2.warpPerspective(frame_2, matrix, (width, height))

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            l_b = np.array([hue_Lower_1, sat_Low_1, val_Low_1])
            u_b = np.array([hue_Upper_1, sat_High_1, val_High_1])

            l_b2 = np.array([hue2_Lower_1, sat_Low_1, val_Low_1])
            u_b2 = np.array([hue2_Upper_1, sat_High_1, val_High_1])

            l_b4 = np.array([hue_Lower_2, sat_Low_2, val_Low_2])  # start of the second color
            u_b4 = np.array([hue_Upper_2, sat_High_2, val_High_2])

            l_b24 = np.array([hue2_Lower_2, sat_Low_2, val_Low_2])
            u_b24 = np.array([hue2_Upper_2, sat_High_2, val_High_2])  # end of the second color

            FGmask = cv2.inRange(hsv, l_b, u_b)
            FGmask2 = cv2.inRange(hsv, l_b2, u_b2)

            FGmask4 = cv2.inRange(hsv, l_b4, u_b4)  # second color
            FGmask24 = cv2.inRange(hsv, l_b24, u_b24)  # second color

            FGmaskComp = cv2.add(FGmask, FGmask2)

            FGmaskComp4 = cv2.add(FGmask4, FGmask24)  # second color

            contours, _ = cv2.findContours(FGmaskComp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours2, _ = cv2.findContours(FGmaskComp4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # second color

            contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
            contours2 = sorted(contours2, key=lambda x: cv2.contourArea(x), reverse=True)  # second color
            for cnt in contours:
                area = cv2.contourArea(cnt)
                (x, y, w, h) = cv2.boundingRect(cnt)
                if area >= 2800:
                    rotrect = cv2.minAreaRect(cnt)
                    box = cv2.boxPoints(rotrect)
                    box = np.int0(box)
                    cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
                    angle = rotrect[2]
                    size = rotrect[1]
                    center = rotrect[0]
                    centerX = int(center[0])
                    centerY = int(center[1])
                    cv2.circle(frame, (centerX, centerY), 4, (0, 0, 255), -1)
                    if size[0] > size[1]:
                        angle = -angle
                    else:
                        angle = 90 - angle
                    positions_green = np.append(positions_green, [centerX, centerY, round(angle), 1])
            for cnt2 in contours2:
                area = cv2.contourArea(cnt2)
                (x, y, w, h) = cv2.boundingRect(cnt2)
                if area >= 2800:
                    rotrect = cv2.minAreaRect(cnt2)
                    box = cv2.boxPoints(rotrect)
                    box = np.int0(box)
                    cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
                    angle = rotrect[2]
                    size = rotrect[1]
                    center = rotrect[0]
                    centerX = int(center[0])
                    centerY = int(center[1])
                    cv2.circle(frame, (centerX, centerY), 4, (0, 255, 0), -1)
                    if size[0] > size[1]:
                        angle = -angle
                    else:
                        angle = 90 - angle
                    positions_green = np.append(positions_green, [centerX, centerY, round(angle), 0])

            positions_green = positions_green.reshape(int((positions_green.shape[0] / 4)), 4)
            if positions_green.shape[0] != 0:
                Path_Camera_LocK.acquire()
                Path_Camera_Lock_Value.value = 1
                l_data.acquire()
                data[:] = positions_green.__deepcopy__({})
                l_data.release()
                Path_Camera_LocK.release()

        cv2.imshow('Warped', frame)
        if cv2.waitKey(1) == ord('q'):
            break
    cam.release()
    cv2.destroyAllWindows()

def main():
    l_1 = Lock()
    v = Value('i', 0)
    l_2 = Lock()
    toggle = Value('i', 0)
    L_toggle = Lock()

    p1 = Process(target=camera_code, args=(final_array, l_1, v, l_2, toggle, L_toggle,))
    p2 = Process(target=path_planning_code, args=(final_array, l_1, v, l_2, toggle, L_toggle,))

    p1.start()
    p2.start()

    p1.join()
    p2.join()


if __name__ == '__main__':
    main()
