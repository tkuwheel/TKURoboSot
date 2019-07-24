#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <strategy/RobotState.h>

ros::NodeHandle nh;

float x, y, yaw;
float T;
bool ballhandle;

const int DIR_R_UP = 8;
const int DIR_R_DOWN = 9;
const int DIR_L_UP = 5;
const int DIR_L_DOWN = 6;
const int PWM_R = 10;
const int PWM_L = 7;

void messageCb( const geometry_msgs::Twist& msg){
  x   = msg.linear.x;
  y   = msg.linear.y;
  yaw = msg.angular.z;
}

void ball_handled(const strategy::RobotState& m){
  ballhandle = m.ball_is_handled;
}

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<geometry_msgs::Twist> motion_sub("motion/cmd_vel", &messageCb );
ros::Subscriber<strategy::RobotState> handle_sub("strategy/state", &ball_handled );

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(motion_sub);
  nh.subscribe(handle_sub);
  nh.advertise(chatter);
  T=millis();
  pinMode(DIR_R_UP,OUTPUT);
  pinMode(DIR_R_DOWN,OUTPUT);
  pinMode(DIR_L_UP,OUTPUT);
  pinMode(DIR_L_DOWN,OUTPUT);
  pinMode(PWM_R,OUTPUT);
  pinMode(PWM_L,OUTPUT);
};

void loop(){
  if(ballhandle){
    str_msg.data ="Ball is handled";
    chatter.publish( &str_msg );
    ballmotionPWM();
  }else{
    digitalWrite(DIR_R_UP,LOW);
    digitalWrite(DIR_L_UP,LOW);
    digitalWrite(DIR_R_DOWN,HIGH);
    digitalWrite(DIR_L_DOWN,HIGH);
    analogWrite(PWM_R,255*25/100);
    analogWrite(PWM_L,255*25/100);
    str_msg.data ="NO_BallHandle";
    chatter.publish( &str_msg );
  }
  nh.spinOnce();
  delay(10);
}

void ballmotionPWM() {
  // put your main code here, to run repeatedly:
  // PWM 0~255
  if(yaw==0 & x==0 & y==0){             //停止
    if((millis()-T)<1500){
      digitalWrite(DIR_R_UP,LOW);
      digitalWrite(DIR_L_UP,LOW);
      digitalWrite(DIR_R_DOWN,HIGH);
      digitalWrite(DIR_L_DOWN,HIGH);
      analogWrite(PWM_R,255*20/100);
      analogWrite(PWM_L,255*20/100);
      str_msg.data ="Still_1";
      chatter.publish( &str_msg );
    }else{
      digitalWrite(DIR_R_UP,LOW);
      digitalWrite(DIR_L_UP,LOW);
      digitalWrite(DIR_R_DOWN,HIGH);
      digitalWrite(DIR_L_DOWN,HIGH);
      analogWrite(PWM_R,255*15/100);
      analogWrite(PWM_L,255*15/100);
      str_msg.data ="Still_2";
      chatter.publish( &str_msg );
    }
  }
  else if(yaw==0 &(x!=0 or y!=0)){
    if(yaw==0 & x==0 & y!=0){
      if(y>0){
        if(y<30){
          digitalWrite(DIR_R_UP,HIGH);
          digitalWrite(DIR_L_UP,HIGH);
          digitalWrite(DIR_R_DOWN,LOW);
          digitalWrite(DIR_L_DOWN,LOW);
          analogWrite(PWM_R,255*0/100);
          analogWrite(PWM_L,255*0/100);
          str_msg.data ="Forward_STOP";
          chatter.publish( &str_msg );
        }else{
          digitalWrite(DIR_R_UP,HIGH);
          digitalWrite(DIR_L_UP,HIGH);
          digitalWrite(DIR_R_DOWN,LOW);
          digitalWrite(DIR_L_DOWN,LOW);
          analogWrite(PWM_R,255*7/100);
          analogWrite(PWM_L,255*7/100);
          str_msg.data ="Forward";
          chatter.publish( &str_msg );
        }
      }else if(y<0){
        if(y>-61){
          digitalWrite(DIR_R_UP,LOW);
          digitalWrite(DIR_L_UP,LOW);
          digitalWrite(DIR_R_DOWN,HIGH);
          digitalWrite(DIR_L_DOWN,HIGH);
          analogWrite(PWM_R,255*25/100);
          analogWrite(PWM_L,255*25/100);
          str_msg.data ="Back_slow";
          chatter.publish( &str_msg );
        }else{
          digitalWrite(DIR_R_UP,LOW);
          digitalWrite(DIR_L_UP,LOW);
          digitalWrite(DIR_R_DOWN,HIGH);
          digitalWrite(DIR_L_DOWN,HIGH);
          analogWrite(PWM_R,255*(25+(abs(y)-60)*13/8)/100);
          analogWrite(PWM_L,255*(25+(abs(y)-60)*13/8)/100);
          str_msg.data ="Back_fast";
          chatter.publish( &str_msg );
        }
      }
    }else if(yaw==0 & x!=0 & y==0){
      digitalWrite(DIR_R_UP,LOW);
      digitalWrite(DIR_L_UP,LOW);
      digitalWrite(DIR_R_DOWN,HIGH);
      digitalWrite(DIR_L_DOWN,HIGH);
      analogWrite(PWM_R,255*18/100);
      analogWrite(PWM_L,255*18/100);
      str_msg.data ="Traverse";
      chatter.publish( &str_msg );
    }else{
      if(y>0){
        if(x>0){
          digitalWrite(DIR_R_UP,LOW);
          digitalWrite(DIR_L_UP,LOW);
          digitalWrite(DIR_R_DOWN,HIGH);
          digitalWrite(DIR_L_DOWN,HIGH);
          analogWrite(PWM_R,255*20/100);
          analogWrite(PWM_L,255*10/100);
          str_msg.data ="Move_other";
          chatter.publish( &str_msg );
        }else{
          digitalWrite(DIR_R_UP,LOW);
          digitalWrite(DIR_L_UP,LOW);
          digitalWrite(DIR_R_DOWN,HIGH);
          digitalWrite(DIR_L_DOWN,HIGH);
          analogWrite(PWM_R,255*10/100);
          analogWrite(PWM_L,255*20/100);
          str_msg.data ="Move_other";
          chatter.publish( &str_msg );
        }
      }else{
        if(x>0){
          digitalWrite(DIR_R_UP,LOW);
          digitalWrite(DIR_L_UP,LOW);
          digitalWrite(DIR_R_DOWN,HIGH);
          digitalWrite(DIR_L_DOWN,HIGH);
          analogWrite(PWM_R,255*(30+abs(y)*0.2)/100);
          analogWrite(PWM_L,255*(15+abs(y)*0.1)/100);
          str_msg.data ="Move_other";
          chatter.publish( &str_msg );
        }else{
          digitalWrite(DIR_R_UP,LOW);
          digitalWrite(DIR_L_UP,LOW);
          digitalWrite(DIR_R_DOWN,HIGH);
          digitalWrite(DIR_L_DOWN,HIGH);
          analogWrite(PWM_R,255*(15+abs(y)*0.1)/100);
          analogWrite(PWM_L,255*(30+abs(y)*0.2)/100);
          str_msg.data ="Move_other";
          chatter.publish( &str_msg );
        }
      }
    }
    T=millis();
  }
  else if(yaw>0){
    digitalWrite(DIR_R_UP,LOW);
    digitalWrite(DIR_L_UP,LOW);
    digitalWrite(DIR_R_DOWN,HIGH);
    digitalWrite(DIR_L_DOWN,HIGH);
    analogWrite(PWM_R,255*(10+yaw*0.4)/100);
    analogWrite(PWM_L,255*(20+yaw*0.5)/100);
    str_msg.data ="Rotate_Left";
    chatter.publish( &str_msg );
    T=millis();
  }
  else if(yaw<0){
    digitalWrite(DIR_R_UP,LOW);
    digitalWrite(DIR_L_UP,LOW);
    digitalWrite(DIR_R_DOWN,HIGH);
    digitalWrite(DIR_L_DOWN,HIGH);
    analogWrite(PWM_R,255*(20+abs(yaw)*0.5)/100);
    analogWrite(PWM_L,255*(10+abs(yaw)*0.4)/100);
    str_msg.data ="Rotate_Right";
    chatter.publish( &str_msg );
    T=millis();
  }
}
