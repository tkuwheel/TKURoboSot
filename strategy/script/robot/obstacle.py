  
from __future__ import print_function
import rospy
import math
import numpy as np


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
        v_x = goal_dis * math.cos(math.radians(goal_ang)) - 40 * obstacle_force_x
        v_y = goal_dis * math.sin(math.radians(goal_ang)) - 40 * obstacle_force_y

    elif mode==1 :
        v_x = goal_dis * math.cos(math.radians(goal_ang))
        v_y = goal_dis * math.sin(math.radians(goal_ang)) 

    v_yaw = k*(math.degrees(math.acos(np.dot(x,y)/np.linalg.norm(y))))
    
    print(obstacle_force_x ,obstacle_force_y)
    print(v_x,v_y,v_yaw)
    return  v_x ,v_y,v_yaw
