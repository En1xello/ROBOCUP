from controller import Robot, Motor, DistanceSensor, Camera, Emitter, GPS, Gyro
import struct
import numpy as np
import math
import random
import time
import cv2


robot = Robot()
#########################################################################################

timeStep = 32
max_velocity = 2
swamp_colour = b'\x12\x1b \xff'  # цвет болота (болото)
white = b'\xfc\xfc\xfc\xff'  # белый цвет (плиты)
black = b'<<<\xff'  # черный цвет (яма)

###############################################################################################

def getColor():
    img = colour_camera.getImage()    # Grab color sensor camera's image view
    return colour_camera.imageGetGray(img, colour_camera.getWidth(), 0, 0)
def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        # If time elapsed (converted into ms) is greater than value passed in
        if (robot.getTime() - initTime) * 1000.0 > ms:
            break
def spin_R():
    motor_L.setVelocity(0.8 * max_velocity)
    motor_R.setVelocity(-0.8 * max_velocity)
    delay(1000)

def spin_L():
    motor_L.setVelocity(-0.8 * max_velocity)
    motor_R.setVelocity(0.8 * max_velocity)
    delay(1000)

def left_wall_moving():
    while robot.step(timestep) != -1:
        img1 = camera1R.getImage()
        img2 = camera2L.getImage()
        img3 = camera3F.getImage()
        helpR = detectVisualSimple(img1, camera1R)
        helpL = detectVisualSimple(img2, camera2L)
        helpF = detectVisualSimple(img3, camera3F)

        #print(getColor())
        #print(distance_sens[0].getValue(), '\t', distance_sens[5].getValue(),'\t', \
        #    distance_sens[1].getValue(), '\t',\
        #    distance_sens[2].getValue(), '\t', distance_sens[4].getValue(),\
        #    '\t',distance_sens[3].getValue(), '\t', x, '\t', helpR, '\t', helpL, '\t', helpF)

        print(len(helpL))
        left_wall = distance_sens[0].getValue() < 0.08
        front_wall_L = distance_sens[1].getValue() < 0.08
        left_corner = distance_sens[5].getValue() < 0.08

        left_speed = max_velocity
        right_speed = max_velocity
        if getColor() > 44:
            spin_R()
        elif front_wall_L:
            print('Поворот вправо на месте')
            left_speed = max_velocity
            right_speed = -max_velocity
            delay(250)
        else:
            if left_wall:
                print('Езда вперед')
                left_speed = max_velocity
                right_speed = max_velocity
                delay(250)
            else:
                print('Поворот налево')
                left_speed = max_velocity / 8
                right_speed = max_velocity
                delay(250)
            if left_corner:
                print('Поддерживание определенной дистанции')
                left_speed = max_velocity
                right_speed = max_velocity / 8
                delay(100)

        motor_L.setVelocity(left_speed)
        motor_R.setVelocity(right_speed)

def detectVisualSimple(image_data, camera):

	coords_list = []
	img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
	img[:,:,2] = np.zeros([img.shape[0], img.shape[1]])


	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)[1]

	contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	for c in contours:
		if cv2.contourArea(c) > 1000:
			coords = list(c[0][0])
			coords_list.append(coords)
			print("Victim at x="+str(coords[0])+" y="+str(coords[1]))

	return coords_list






timestep = int(robot.getBasicTimeStep())


motor_L = robot.getDevice("wheel2 motor")
motor_R = robot.getDevice("wheel1 motor")

motor_L.setPosition(float('inf'))
motor_L.setVelocity(0.0)

motor_R.setPosition(float('inf'))
motor_R.setVelocity(0.0)

distance_sens = []
for i in range(1,7):
    distance_sens.append(robot.getDevice(f"distance sensor{i}"))
for i in range(6):
    distance_sens[i].enable(timestep)

#dis_sens_1 - left 0
#dis_sens_4 - right 3
#dis_sens_2 - frontL 1
#dis_sens_3 - frontR 2
#dis_sens_5 - 45R 4
#dis_sens_6 - 45L 5

colour_camera = robot.getCamera("colour_sensor")
colour_camera.enable(timestep)

camera1R = robot.getDevice("camera1")
camera2L = robot.getDevice("camera2")
camera3F = robot.getDevice("camera3")
camera1R.enable(timestep)
camera2L.enable(timestep)
camera3F.enable(timestep)

Gps = robot.getDevice("gps")
Gps.enable(timestep)

#####################################################################


while robot.step(timestep) != -1:
    x = Gps.getValues()[0]
    #img1 = camera1R.getImage()
    #img2 = camera2L.getImage()
    #img3 = camera3F.getImage()
    #helpR = detectVisualSimple(img1, camera1R)
    #helpL = detectVisualSimple(img2, camera2L)
    #helpF = detectVisualSimple(img3, camera3F)
    ##print(type(img1))
    #print(len(helpR))
    if (distance_sens[0].getValue() > distance_sens[3].getValue()) or\
          distance_sens[0].getValue() == distance_sens[3].getValue():
        motor_L.setVelocity(max_velocity)
        motor_R.setVelocity(-max_velocity)
        delay(700)
    else:
        left_wall_moving()

