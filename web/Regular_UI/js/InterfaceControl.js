function GetChooseRobot(choice){

	ChooseRobot = choice;
	console.log(ChooseRobot);
	if(document.getElementById("MCLmap").checked==true){
	  Mclmap(true);
	}
	else{
	  //Mclmap(false);
	}
	if(document.getElementById("CameraSwitch").checked==true){
	  MonitorSwitch(true);
	}
	else{
	  //MonitorSwitch(false);
	}

}

