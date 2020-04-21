#-*- coding: UTF-8 -*-  
from __future__ import print_function
from robot import Robot

import rospy
import math
import numpy as np
import time

class Obstacle(object): 
  def state(self, ranges):
    self.raw = []
    object_dis=float(500)
    for i in range (len(ranges)):
       if ranges[i]>130: 
         self.raw.append(0)
       else :
         self.raw.append(ranges[i])
    
    return self.raw , object_dis


  def filter(self, raw ):
    mask=[1,0,1]

    self.edit = []
    if raw[len(raw)-1]==0 and raw[0]!=0 and raw[1]==0 :
        self.edit.append(0)
    else :
        self.edit.append(raw[0])
    for i in range (1,len(raw)-1):
        if raw[i-1]*mask[0]+raw[i]*mask[1]+raw[i+1]*mask[2]==0 :
            self.edit.append(0)
        else:
            self.edit.append(raw[i])
    if raw[len(raw)-2]==0 and raw[len(raw)-1] != 0 and raw[0]==0:
        self.edit.append(0)
    else :
        self.edit.append(raw[len(raw)-1]) 
    
    return self.edit    
    

  def Obstacle_segmentation(self, edit , angle_increment , object_dis):
    
    stack=[[] for num in range(360)]
    count=0
    j=0
    order=[]
    obstacle=[]
    even=[]
    odd=[]
    even_force_x1=0
    even_force_x2=0
    even_force_y1=0
    even_force_y2=0
    odd_force_x=0
    odd_force_y=0


    for i in range (len(self.edit)) :
        if self.edit[i]>0 :
            stack[j].append(self.edit[i])
            count=count+1
        elif self.edit[i]==0 and count >0 :
            j=j+1
            count=0
            order.append(i)
        else :
            pass

    if self.edit[len(self.edit)-1]>0 :
        order.append(len(self.edit))
    else :
        pass

    for k in range (360) :
        if len(stack[k]) >1 :
            obstacle.append(stack[k])
        else :
            pass


    if len(obstacle) != 0 :
        for w in range (len(obstacle)) :
            if len(obstacle[w])%2==0 :
                c=order[w]-len(obstacle[w])/2
                even.append(c)
                c=order[w]-len(obstacle[w])/2-1
                even.append(c)                
            else :
                c=order[w]-len(obstacle[w])/2-1
                odd.append(c)


        for z in range (len(even)) :
            if z%2==0 :
                even_force_x1 = even_force_x1 + ((object_dis-obstacle[z/2][len(obstacle[z/2])/2])/object_dis*2)*math.cos(even[z]*angle_increment)
                even_force_y1 = even_force_y1 + ((object_dis-obstacle[z/2][len(obstacle[z/2])/2])/object_dis*2)*math.sin(even[z]*angle_increment)
            else :
                even_force_x2 = even_force_x2 + ((object_dis-obstacle[z/2][len(obstacle[z/2])/2-1])/object_dis*2)*math.cos(even[z]*angle_increment)
                even_force_y2 = even_force_y2 + ((object_dis-obstacle[z/2][len(obstacle[z/2])/2-1])/object_dis*2)*math.sin(even[z]*angle_increment)
        for h in range (len(odd)) :
            odd_force_x = odd_force_x + ((object_dis-obstacle[h][len(obstacle[h])/2])/object_dis)*math.cos(odd[h]*angle_increment)
            odd_force_y = odd_force_y + ((object_dis-obstacle[h][len(obstacle[h])/2])/object_dis)*math.sin(odd[h]*angle_increment)
            
        even_force_x = even_force_x1 + even_force_x2
        even_force_y = even_force_y1 + even_force_y2
        obstacle_force_x = even_force_x + odd_force_x
        obstacle_force_y = even_force_y + odd_force_y

        return obstacle_force_x , obstacle_force_y
        
    
    else :

        obstacle_force_x = 0
        obstacle_force_y = 0

        return obstacle_force_x , obstacle_force_y

        

  def Force_Calculation (self , obstacle_force_x ,obstacle_force_y , goal_ang ,goal_dis ,mode) :
    
    x=np.array([1,0])
    y=np.array([-1*obstacle_force_x , -1*obstacle_force_y])
    cross = np.cross(x,y)
    
    if cross < 0 :
        k = -1
    else :
        k = 1

    if mode== 0 :
        v_x = goal_dis * math.cos(math.radians(goal_ang)) - 80 * obstacle_force_x
        v_y = goal_dis * math.sin(math.radians(goal_ang)) - 80 * obstacle_force_y

    elif mode==1 :
        v_x = goal_dis * math.cos(math.radians(goal_ang))
        v_y = goal_dis * math.sin(math.radians(goal_ang)) 

    v_yaw = k*(math.degrees(math.acos(np.dot(x,y)/np.linalg.norm(y))))
    
    print(obstacle_force_x ,obstacle_force_y)
    print(v_x,v_y,v_yaw)
    return  v_x ,v_y,v_yaw

#======================Avoid Strategy===================
  def obstacle_fileter(self, obs, robot, near_robot=None):
    obs_filter = []
    info_time = Robot.sync_last_time
    now = time.time()
    dt = abs(info_time-now)
    # print(Robot.sync_last_time)
    # print("now", time.time())
    # print("dt", dt)
    r_x = 999
    r_y = 999
    if(near_robot is not None):
        r_x = near_robot['position']['x']
        r_y = near_robot['position']['y']
        r_yaw = near_robot['position']['yaw']
    # print(r_x, r_y)
    for i in range (0,len(obs), 4):
        distance = obs[i+0]
        angle    = obs[i+1]+robot["location"]["yaw"]
        o_x      = robot["location"]["x"] + distance * math.cos(math.radians(angle))
        o_y      = robot["location"]["y"] + distance * math.sin(math.radians(angle))
        dis      = math.sqrt(math.pow((r_x-o_x),2)+math.pow((r_y-o_y),2))
        #未接到隊友資訊
        # print("o_X, o_y", o_x, o_y)
        # print("r_x, r_y", r_x, r_y)
        if(dt>5):
            dis = 999
        print("dis", dis)
        # print(o_x, o_y)
        abs_yaw = abs(robot["location"]["yaw"]-angle)
        if(abs(o_y)<200 and abs_yaw>10):
            obs_filter.append(obs[i+0])
            obs_filter.append(obs[i+1])
            obs_filter.append(obs[i+2])
            obs_filter.append(obs[i+3])
    return obs_filter

  def obstacle_roate(self, obs, robot):
    rf_x = 0
    rf_y = 0
    dis_threshold = 100
    for i in range (0, len(obs), 4):
        distance = obs[i+0]
        angle    = obs[i+1]-180
        if(angle<-180):
            angle = angle+360
        if(distance<dis_threshold):
            rf_x += (dis_threshold-distance)*math.cos(math.radians(angle))
            rf_y += (dis_threshold-distance)*math.sin(math.radians(angle))
    v_yaw = math.degrees(math.atan2(rf_y, rf_x))
    #print(v_yaw)
    return v_yaw
  
  def obstacle_escape(self, goal_dis, goal_ang, obs, robot):
    route_ang = []
    ang_threshold = 90
    robot_radius = 30
    route_filter = []
    #============判斷球門路線有無阻擋=================
    cross_flag = True
    route_angle = goal_ang
    
    for i in range (0, len(obs), 4):
        
        dis = obs[i+0]
        right_ang = obs[i+2]
        left_ang  = obs[i+3]
        right_min_ang = abs(right_ang-route_angle) if(abs(right_ang-route_angle)<abs(360-abs(right_ang-route_angle))) else abs(360-abs(right_ang-route_angle))
        left_min_ang = abs(left_ang-route_angle) if(abs(left_ang-route_angle)<abs(360-abs(left_ang-route_angle))) else abs(360-abs(left_ang-route_angle))
        min_ang = right_min_ang if(right_min_ang<left_min_ang) else left_min_ang
        min_dis = dis * math.sin(math.radians(min_ang))
        if(abs(robot["location"]["y"])>80 and abs(robot["location"]["x"])>250):

            if(min_dis < robot_radius*0.5):
                cross_flag = False
        else:
            if(min_dis < robot_radius*0.8):
                cross_flag = False
    if(cross_flag==True):
        route_filter.append(route_angle)
    #=============================================
    #find all possible angles
    
    distance_filter = 150
    for i in range (0,len(obs), 4):
        
        max_rotate_ang = 80
        distance = obs[i+0]

        if(distance>distance_filter):
            continue
        right_ang = obs[i+2]
        min_ang = abs(right_ang-goal_ang) if(abs(right_ang-goal_ang)<abs(360-abs(right_ang-goal_ang))) else abs(360-abs(right_ang-goal_ang))
        if(min_ang<ang_threshold):
            tmp = math.sqrt(abs(pow(distance,2)-pow(robot_radius,2)))
            escape_ang = math.degrees(math.atan2(robot_radius, tmp))*1.2
            if(escape_ang > max_rotate_ang or distance < robot_radius):
                escape_ang = max_rotate_ang
            right_ang = right_ang+escape_ang
            if(right_ang > 180):
                right_ang = right_ang - 360
            route_ang.append(right_ang)
            
        left_ang = obs[i+3]
        min_ang = abs(left_ang-goal_ang) if(abs(left_ang-goal_ang)<abs(360-abs(left_ang-goal_ang))) else abs(360-abs(left_ang-goal_ang))
        if(min_ang<ang_threshold):
            tmp = math.sqrt(abs(pow(distance,2)-pow(robot_radius,2)))
            escape_ang = math.degrees(math.atan2(robot_radius, tmp))*1.2
            if(escape_ang>max_rotate_ang):
                escape_ang = max_rotate_ang
            left_ang = left_ang-escape_ang
            if(left_ang < -180):
                left_ang = left_ang + 360
            route_ang.append(left_ang)

    #filter blocked angles
    for i in range (0, len(route_ang), 1):
        cross_flag = True
        #print("route_ang", route_ang[i])
        route_obs_num = i/2
        route_obs_dis = obs[route_obs_num*4+0]
        route_obs_ang = obs[route_obs_num*4+1]
        route_obs_right_ang = obs[route_obs_num*4+2]
        route_obs_left_ang = obs[route_obs_num*4+3]

        route_angle = route_ang[i]
        for j in range (0, len(obs), 4):
            dis = obs[j+0]
            ang = obs[j+1]
            right_ang = obs[j+2]
            left_ang  = obs[j+3]
            if(j/4!=route_obs_num):
                #計算路徑是否碰撞障礙物
                right_min_ang = abs(right_ang-route_angle) if(abs(right_ang-route_angle)<abs(360-abs(right_ang-route_angle))) else abs(360-abs(right_ang-route_angle))
                left_min_ang = abs(left_ang-route_angle) if(abs(left_ang-route_angle)<abs(360-abs(left_ang-route_angle))) else abs(360-abs(left_ang-route_angle))
                min_ang = right_min_ang if(right_min_ang<left_min_ang) else left_min_ang
                hypotenuse = dis/math.cos(math.radians(min_ang))
                obs_min_dis = hypotenuse*math.sin(math.radians(min_ang))
                # if(min_ang>90):
                #     obs_min_dis = 999
                #若碰撞 計算兩障礙物間距是否可以通過
                if(obs_min_dis<robot_radius*0.8 or (route_angle < right_ang and route_angle > left_ang)):
                    right_min_ang = abs(right_ang-route_obs_left_ang) if(abs(right_ang-route_obs_left_ang)<abs(360-abs(right_ang-route_obs_left_ang))) else abs(360-abs(right_ang-route_obs_left_ang))
                    left_min_ang  = abs(left_ang-route_obs_right_ang) if(abs(left_ang-route_obs_right_ang)<abs(360-abs(left_ang-route_obs_right_ang))) else abs(360-abs(left_ang-route_obs_right_ang))
                    min_ang = right_min_ang if(right_min_ang<left_min_ang) else left_min_ang
                    #SAS a^2 = b^2 + c^2 -2bc*cos(A)
                    min_dis = math.sqrt(math.pow(dis,2)+math.pow(route_obs_dis,2)-2*dis*route_obs_dis*math.cos(math.radians(min_ang)))
                    if(min_dis<robot_radius*2*0.7):
                        cross_flag = False
      
        angle = route_ang[i]+robot["location"]["yaw"]
        x =  robot["location"]["x"] + 150 * math.cos(math.radians(angle))
        y = -robot["location"]["y"] - 150 * math.sin(math.radians(angle))
        #print("x", x,"y", y)
        #if((abs(robot["location"]["y"])>150 and abs(x)>300 and abs(y)>80)or ( abs(y)>200) ):
        if((abs(robot["location"]["y"])<60 and abs(y)>150) or (abs(robot["location"]["y"])>60 and(abs(x)>300 or abs(y)>200)) ):
            cross_flag = False
        if(cross_flag==True):
            route_filter.append(route_ang[i])
    #find min angle
    min_ang = 999
    fin_ang = goal_ang
    #print ("obs size", len(obs)/4)
    #print ("route_ang size " , len(route_ang))
    #print ("route_filter size " , len(route_filter))
    for i in range(0, len(route_filter), 1):
        #print(min_ang)
        ang_tmp = abs(route_filter[i]-goal_ang) if(abs(route_filter[i]-goal_ang)<abs(360-abs(route_filter[i]-goal_ang))) else abs(360-abs(route_filter[i]-goal_ang))
        #print(ang_tmp, min_ang)
       
        if(ang_tmp<min_ang):
            min_ang = ang_tmp
            fin_ang = route_filter[i]
            #print(fin_ang)
    if(goal_dis<80):
        goal_dis = 200
    v_x = goal_dis * math.cos(math.radians(fin_ang))
    v_y = goal_dis * math.sin(math.radians(fin_ang))
    #if(goal_dis<200):
        # v_x = v_x*0.5 + goal_dis * math.cos(math.radians(goal_ang))*1.5
        # v_y = v_y*0.5 + goal_dis * math.sin(math.radians(goal_ang))*1.5
        #force attack
        #print("force attack")
        #v_x =  goal_dis * math.cos(math.radians(goal_ang))*5
        #v_y =  goal_dis * math.sin(math.radians(goal_ang))*5
    #print("fin_ang", fin_ang)
    return v_x, v_y
  def back(self, goal_dis, goal_ang, obs, back_dis, back_ang):
    x = goal_dis * math.cos(math.radians(goal_ang+180))
    y = goal_dis * math.cos(math.radians(goal_ang+180))
    yaw = goal_ang+180 #goal_ang = our side gaol angle
    if(goal_dis<200):#only blocked
        x = goal_dis * math.cos(math.radians(goal_ang+180))
        y = goal_dis * math.cos(math.radians(goal_ang+180))
        yaw = goal_ang+180 #goal_ang = our side gaol angle
    else:
        print("steal")
        for j in range (0, len(obs), 4):
            dis = obs[j+0]
            ang = obs[j+1]
            if(abs(ang) < back_ang and dis < back_dis):
                #x = (back_dis - dis) *2.5 * math.cos(math.radians(ang+180))
                #y = (back_dis - dis) *2.5 * math.cos(math.radians(ang+180))
                print("#fast back")
                x = (back_dis - dis) *10 * math.cos(math.radians(ang+180)) #fast back
                y = (back_dis - dis) *10 * math.cos(math.radians(ang+180)) 
                yaw = 0
                break
    return x, y, yaw
 
