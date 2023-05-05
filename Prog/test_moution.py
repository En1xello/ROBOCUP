#################################  используемые бибилотеки

from controller import Robot, Motor, DistanceSensor, Camera, Emitter, GPS, Gyro
import struct
import numpy as np
import math
import random

################################# переменные
swamp_colour = b'\x12\x1b \xff' # цвет болота (болото)
white = b'\xfc\xfc\xfc\xff'  # белый цвет (плиты)
black = b'<<<\xff'  # черный цвет (яма)
robot = Robot() # создание объекта робот
timeStep = 32   # время обновления мира
max_velocity = 3.28 # максимальная скорость
startTime = 0  #время старта
duration = 0  # задержка

################################# Иницилизация датчиков
gps = robot.getDevice("gps") # иницилизация GPS модуля

colour_camera = robot.getCamera("colour_sensor") # иницилизация датчика цвета

distanceSensors = [] #создаем пустой список для датчиков расстояния
for i in range(8):
    distanceSensors.append(robot.getDevice("distance sensor" + str(i)))   #иницилизация датчика расстояния х8

leftMotor = robot.getDevice("wheel1")    # инициализация двигателей
rightMotor = robot.getDevice("wheel2")

# leftEncoder = leftMotor.getPositionSensor()    # иницализация инкодеров
# rightEncoder = rightMotor.getPositionSensor()

################################ Включение датчиков

timestep = int(robot.getBasicTimeStep()) # устанавливаем время обработки

gps.enable(timestep) # включение GPS модуля

colour_camera.enable(timeStep) # включение датчика цвета

for i in range(4):
    distanceSensors[i].enable(timestep) # включение датчика расстояния х8

leftMotor.setPosition(float('inf')) # установка позиции колеса
rightMotor.setPosition(float('inf'))

leftEncoder.enable(timeStep)  # включение инкодеров
rightEncoder.enable(timeStep)

################################  используемые функции

def turn_right():
    #set left wheel speed
    leftMotor.setVelocity(0.5 * max_velocity)
    #set right wheel speed
    rightMotor.setVelocity(-0.2 * max_velocity)

def turn_left():
    #set left wheel speed
    leftMotor.setVelocity(-0.2 * max_velocity)
    #set right wheel speed
    rightMotor.setVelocity( 0.5 * max_velocity)

def spin():
    #set left wheel speed
    leftMotor.setVelocity(0.5 * max_velocity)
    #set right wheel speed
    rightMotor.setVelocity(-0.5 * max_velocity)

def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > ms: # If time elapsed (converted into ms) is greater than value passed in
            break

def getColor():
    img = colorSensor.getImage()    # Grab color sensor camera's image view
    return colorSensor.imageGetGray(img, colorSensor.getWidth(), 0, 0)


################################




leftSensors = []      # создаем список для датчиков расстояния расположенных на левой части робота
rightSensors = []     # создаем список для датчиков расстояния расположенных на правой части робота
frontSensors = []     # создаем список для датчиков расстояния расположенных на передней части робота

# назначаем элементам списка какие датчики использовать
frontSensors.append(robot.getDistanceSensor("distance sensor7"))
frontSensors[0].enable(timeStep)
frontSensors.append(robot.getDistanceSensor("distance sensor0"))
frontSensors[1].enable(timeStep)

rightSensors.append(robot.getDistanceSensor("distance sensor1"))
rightSensors[0].enable(timeStep)
rightSensors.append(robot.getDistanceSensor("distance sensor2"))
rightSensors[1].enable(timeStep)

leftSensors.append(robot.getDistanceSensor("distance sensor5"))
leftSensors[0].enable(timeStep)
leftSensors.append(robot.getDistanceSensor("distance sensor6"))
leftSensors[1].enable(timeStep)

################################

while robot.step(timeStep) != -1:
    #устанавливаем скорость колес максимальной
    speedl = max_velocity  # скорость левого колеса
    speedr = max_velocity  # скорость правого колеса

#если левая стена близко немного отъезжаем вправо
    if leftSensors[0].getValue()<0.25 or leftSensors[1].getValue()<0.25:
        turn_right()

#если правая стена близко немного отъезжаем влево
    if rightSensors[0].getValue()<0.25 or rightSensors[1].getValue()<0.25:
        turn_left()

#если стена перед роботом поворачиваемся
    if frontSensors[0].getValue()<0.25 or frontSensors[0].getValue()<0.25:
        spin()
        delay(100)

#если робот приближается к яме разворачиваемся
    if colour_camera.getImage() == black :
        spin()
        delay(400)

    # устанавливаем скорости моторов
    leftMotor.setVelocity(speedl)
    rightMotor.setVelocity(speedr)
