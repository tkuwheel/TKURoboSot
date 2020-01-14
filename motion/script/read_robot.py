#!/usr/bin/env python3
# -*- coding: utf-8 -*-+
import sys
import numpy as np
import matplotlib.pyplot as plt



def Read_File(filename):
    with open(filename) as f:
        time = []
        w1 = []
        w2 = []
        w3 = []
        tar1 = []
        tar2 = []
        tar3 = []
        vel_x = []
        vel_y = []
        vel_yaw = []
        traj_x = []
        traj_y = []
        for line in f.readlines():
            line = line.strip("\n,' '")
            line_list = line.split(' ')
            time.append(int(line_list[0]))
            w1.append(float(line_list[1]))
            w2.append(float(line_list[2]))
            w3.append(float(line_list[3]))
            tar1.append(float(line_list[4]))
            tar2.append(float(line_list[5]))
            tar3.append(float(line_list[6]))
            vel_x.append(float(line_list[7]))
            vel_y.append(float(line_list[8]))
            vel_yaw.append(float(line_list[9]))
            traj_x.append(float(line_list[10]))
            traj_y.append(float(line_list[11]))
                
        return time,w1,w2,w3,tar1,tar2,tar3,vel_x,vel_y,vel_yaw,traj_x,traj_y

def Plot_Trend(time,w1,w2,w3,tar1,tar2,tar3,vel_x,vel_y,vel_yaw,traj_x,traj_y):

    x = np.zeros(len(time))
    total_time = 0
    for i in range(len(time)):
        total_time += time[i] * 0.001
        x[i] = total_time
    
    fig, ax = plt.subplots()
    ax.plot(x,w1,'r',label='w1',linestyle='-')
    ax.plot(x,w2,'g',label='w2',linestyle='-')
    ax.plot(x,w3,'b',label='w3',linestyle='-')
    ax.plot(x,tar1,'r',label='target1',linestyle='--')
    ax.plot(x,tar2,'g',label='target2',linestyle='--')
    ax.plot(x,tar3,'b',label='target3',linestyle='--')
    plt.xlabel('time')
    plt.ylabel('velocity')
    fig1, ax1 = plt.subplots()
    ax1.plot(x,vel_x,'r',label='vel x',linestyle='-')
    ax1.plot(x,vel_y,'g',label='vel y',linestyle='-')
    ax1.plot(x,vel_yaw,'b',label='vel yaw',linestyle='-')
    plt.xlabel('time')
    plt.ylabel('velocity')
    fig2, ax2 = plt.subplots()
    ax2.plot(traj_y,traj_x,'black',label='trajectory',linestyle='-')
    plt.axis([-5,5,-5,5])
    plt.xlabel('x')
    plt.ylabel('y')

    ax.legend(loc = 'upper right')
    ax1.legend(loc = 'upper right')
    ax2.legend(loc = 'upper right')
    plt.grid()
    plt.show()
    

def main():
    Plot_Trend(*Read_File(sys.argv[1]))

if __name__ == '__main__':
    main()
