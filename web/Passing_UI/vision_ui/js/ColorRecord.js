function ColorRecord(value){
	value = parseInt(value);
	switch (value) {
        case 0:
            for (var i = 0; i < 6; i++) {
                document.getElementsByName('HSVElement')[i].value = OrangeBox[i];
                document.getElementsByName('HSVElement2')[i].value  = OrangeBox[i];
            }
            break;
        case 1:
            for (var i = 0; i < 6; i++) {
            	document.getElementsByName('HSVElement')[i].value = GreenBox[i];
            	document.getElementsByName('HSVElement2')[i].value = GreenBox[i];
            }
            break;
        case 2:
            for (var i = 0; i < 6; i++) {
            	document.getElementsByName('HSVElement')[i].value = BlueBox[i];
            	document.getElementsByName('HSVElement2')[i].value = BlueBox[i];
            }
            break;
        case 3:
            for (var i = 0; i < 6; i++) {
            	document.getElementsByName('HSVElement')[i].value = YellowBox[i];
            	document.getElementsByName('HSVElement2')[i].value = YellowBox[i];
            }
            break;
        case 4:
            for (var i = 0; i < 6; i++) {
            	document.getElementsByName('HSVElement')[i].value = WhiteBox[i];
            	document.getElementsByName('HSVElement2')[i].value = WhiteBox[i];
            }
            break;
        case 5:
            for (var i = 0; i < 6; i++) {
            	document.getElementsByName('HSVElement')[i].value = RedconeBox[i];
            	document.getElementsByName('HSVElement2')[i].value = RedconeBox[i];
            }
            break;
    }
}
