function GetChooseRobot(choice){

	ChooseRobot = choice;
	console.log(ChooseRobot);
	if(document.getElementById("CameraSwitch").checked==true){
	  MonitorSwitch(true);
	}
	else{
	  MonitorSwitch(false);
	}
}

