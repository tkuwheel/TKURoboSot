#!/usr/bin/python
import math

def quaternionTEuler(x, y, z, w):
   q0 = w
   q1 = x
   q2 = y
   q3 = z
   pitch = math.asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3
   roll = math.atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3
   yaw = math.atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3)* 57.3

   return pitch, roll, yaw


def eulerToQuaternion(pitch, roll, yaw):
    w = math.cos(yaw/2)*math.cos(roll/2)*math.cos(pitch/2)-math.sin(yaw/2)*math.sin(roll/2)*math.sin(pitch/2)
    x = math.cos(yaw/2)*math.cos(roll/2)*math.sin(pitch/2)+math.sin(yaw/2)*math.sin(roll/2)*math.cos(pitch/2)
    y = math.cos(yaw/2)*math.sin(roll/2)*math.cos(pitch/2)-math.sin(yaw/2)*math.cos(roll/2)*math.sin(pitch/2)
    z = math.sin(yaw/2)*math.cos(roll/2)*math.cos(pitch/2)+math.cos(yaw/2)*math.sin(roll/2)*math.sin(pitch/2)
    return w, x, y, z


def calAng(x, y, yaw, remainder):
    return round(math.degrees(math.atan2(y, x))-yaw, remainder)

def calDis(x, y, remainder, toCM=True):
    if toCM:
        return round(math.hypot(x, y)*100, remainder)
    else:
        return round(math.hypot(x, y), remainder)