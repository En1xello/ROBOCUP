from controller import Robot, Motor, DistanceSensor, Camera, Emitter, GPS, Gyro
import struct
import numpy as np
import math
import random
import time
import cv2



def turn_right():
    motor_L.setVelocity(0.5 * max_velocity)
    motor_R.setVelocity(-0.2 * max_velocity)
    delay(250)


def turn_left():
    motor_L.setVelocity(-0.2 * max_velocity)
    motor_R.setVelocity(0.5 * max_velocity)
    delay(250)


def spin():
    motor_L.setVelocity(0.5 * max_velocity)
    motor_R.setVelocity(-0.5 * max_velocity)
    delay(1000)


def Forward():
    motor_L.setVelocity(0.5 * max_velocity)
    motor_R.setVelocity(0.5 * max_velocity)
    delay(400)


def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        # If time elapsed (converted into ms) is greater than value passed in
        if (robot.getTime() - initTime) * 1000.0 > ms:
            break


def getColor():
    img = colour_camera.getImage()    # Grab color sensor camera's image view
    return colour_camera.imageGetGray(img, colour_camera.getWidth(), 0, 0)

#########################################################################################

timeStep = 32
max_velocity = 3
swamp_colour = b'\x12\x1b \xff'  # цвет болота (болото)
white = b'\xfc\xfc\xfc\xff'  # белый цвет (плиты)
black = b'<<<\xff'  # черный цвет (яма)

###############################################################################################

robot = Robot()


#dis_sens_1 - left 0
#dis_sens_4 - right 3
#dis_sens_2 - frontL 1
#dis_sens_3 - frontR 2

distance_sens = []

for i in range(1,5):
    distance_sens.append(robot.getDevice(f"distance sensor{i}"))


colour_camera = robot.getCamera("colour_sensor")  # иницилизация датчика цвета

motor_L = robot.getDevice("wheel1 motor")
motor_R = robot.getDevice("wheel2 motor")

timestep = int(robot.getBasicTimeStep())

#################################################################################

motor_L.setPosition(float('inf'))  # установка позиции колеса
motor_R.setPosition(float('inf'))


for i in range(4):
    distance_sens[i].enable(timestep)

colour_camera.enable(timestep)

#################################################################################

while robot.step(32) != 1:
    Forward()
    print(getColor())
    print(distance_sens[0].getValue(), '\t', distance_sens[1].getValue(), '\t',\
            distance_sens[2].getValue(), '\t',distance_sens[3].getValue())

    if getColor() > 44:
        spin()

    if distance_sens[1].getValue() < 0.1 or distance_sens[2].getValue() < 0.1:
        if distance_sens[0].getValue() > 0.05 and distance_sens[3].getValue() < 0.05:
            turn_left()
        elif distance_sens[0].getValue() < 0.05 and distance_sens[3].getValue() > 0.05:
            turn_right()
    #if distance_sens[1].getValue() < 0.15 or distance_sens[2].getValue() < 0.15:
    #    if distance_sens[0].getValue() > distance_sens[3].getValue():
    #        turn_left()
    #    elif distance_sens[0].getValue() < distance_sens[3].getValue():
    #        turn_right()

        spin()
