from controller import Robot, Motor, DistanceSensor, Camera, Emitter, GPS, Gyro
import struct
import numpy as np
import math
import random
import time
import cv2




def turn_right():
    motor_L.setVelocity(-0.5 * max_velocity)
    motor_R.setVelocity(0.5 * max_velocity)
    delay(300)


def turn_left():
    motor_L.setVelocity(0.5 * max_velocity)
    motor_R.setVelocity(-0.5 * max_velocity)
    delay(300)


def spin():
    motor_L.setVelocity(0.5 * max_velocity)
    motor_R.setVelocity(-0.5 * max_velocity)
    delay(1000)


def Forward():
    motor_L.setVelocity(0.5 * max_velocity)
    motor_R.setVelocity(0.5 * max_velocity)
    delay(300)


def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        # If time elapsed (converted into ms) is greater than value passed in
        if (robot.getTime() - initTime) * 1000.0 > ms:
            break


def getColor():
    img = colour_camera.getImage()    # Grab color sensor camera's image view
    return colour_camera.imageGetGray(img, colour_camera.getWidth(), 0, 0)


def StopMotors():
    motor_L.setVelocity(0 * max_velocity)
    motor_R.setVelocity(0 * max_velocity)

def detectVisualSimple(image_data, camera):

	coords_list = []
	img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
	img[:,:,2] = np.zeros([img.shape[0], img.shape[1]])


	#convert from BGR to HSV color space
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	#apply threshold
	thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)[1]

	# draw all contours in green and accepted ones in red
	contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	for c in contours:
		if cv2.contourArea(c) > 1000:
			coords = list(c[0][0])
			coords_list.append(coords)
			print("Victim at x="+str(coords[0])+" y="+str(coords[1]))

	return coords_list
timeStep = 32
max_velocity = 3
swamp_colour = b'\x12\x1b \xff'  # цвет болота (болото)
white = b'\xfc\xfc\xfc\xff'  # белый цвет (плиты)
black = b'<<<\xff'  # черный цвет (яма)


robot = Robot()


# dis_sens_1 - right mas-0
# dis_sens_4 - left  mas -3
# dis_sens_2 - front mas-1
# dis_sens_3 - front mas-2
dictance_sens = []

for i in range(1,5):
    dictance_sens.append(robot.getDevice(f"distance sensor{i}"))

Gps = robot.getDevice("gps")

colour_camera = robot.getDevice("colour_sensor")  # иницилизация датчика цвета

motor_L = robot.getDevice("wheel1 motor")
motor_R = robot.getDevice("wheel2 motor")

camera1 = robot.getDevice("camera1")
camera2 = robot.getDevice("camera2")
camera3 = robot.getDevice("camera3")

timestep = int(robot.getBasicTimeStep())

motor_L.setPosition(float('inf'))  # установка позиции колеса
motor_R.setPosition(float('inf'))

camera1.enable(timeStep)
camera2.enable(timeStep)
camera3.enable(timeStep)
colour_camera.enable(timeStep)
Gps.enable(timestep)

for i in range(4):
    dictance_sens[i].enable(timeStep)

while robot.step(32) != -1:
    Forward()
    x = Gps.getValues()[0]
    #print(camera.getImage())
    img = camera1.getImage()
    img2 = camera2.getImage()
    img3 = camera3.getImage()
    help1 = detectVisualSimple(img, camera1)
    help2 = detectVisualSimple(img2, camera2)
    help3 = detectVisualSimple(img3, camera3)
    print(dictance_sens[1].getValue(), "\t", dictance_sens[2].getValue(), "\t", dictance_sens[0].getValue(),
                "\t", dictance_sens[3].getValue(), "\t", x, "\t", help1)
    if getColor() > 80:
        spin()
    if dictance_sens[1].getValue() < 0.25 or dictance_sens[2].getValue() < 0.25:
        if dictance_sens[0].getValue() < 0.05:
            turn_left()
        elif dictance_sens[3].getValue() < 0.05:
            turn_right()
        spin()
