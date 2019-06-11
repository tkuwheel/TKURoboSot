/********************************
 *	Include system
 ********************************/
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
/********************************
 *	Include libraries
 ********************************/
#include "motor_data.h"

/********************************
 *	Include header files
 ********************************/
#include "motion_nodeHandle.h"
#include "base_control.h"
//#include "CTest.h"
/********************************
 *	Define	
 ********************************/
//#define DEBUG 

int main(int argc, char **argv)
{
	Motion_nodeHandle main_nodeHandle(argc, argv);
	Base_Control main_Base_Control;

	robot_command *main_robotCMD;
	robot_command *main_robotFB;
	int count = 0;
	//while(ros::ok()){
	//	if(Global_Motor_Control.mcssl_init()){
	//		break;
	//	}else{
	//		exit(EXIT_FAILURE);
	//	}
	//}
	std::cout << "ATTACK MOTION IS RUNNING!\n";
	ros::Rate loop_rate(25);
	while(ros::ok()){
		main_robotCMD = main_nodeHandle.getMotion();
		main_Base_Control.send(main_robotCMD);
		if(*main_robotCMD->shoot_power > 0){
			count++;
			if(count>10){
				main_nodeHandle.clear();
				count = 0;
			}
		}else{
			main_nodeHandle.clear();
			count = 0;
		}
		main_robotFB = main_Base_Control.get_feedback();
		main_nodeHandle.pub_robotFB(main_robotFB);
		ros::spinOnce();
		loop_rate.sleep();
	}
	delete main_robotCMD;
//	ros::shutdown();
	std::cout << "Close Attack Motion\n";
#ifdef DEBUG
#else
#endif
	return 0;
}


