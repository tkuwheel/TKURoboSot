//ParameterCamera
var ParameterCamera = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/FPS',
});

ParameterCamera.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CameraElement");
        obj[0].value = value;
        document.getElementsByName("CameraLabel")[0].value = value;
    }
});

function ParameterCameraValue() {
    let value = parseInt(document.getElementsByName('CameraElement')[0].value);
    console.log("Sent Camera Parameter: "+value);
    SendMsgs("Sent Camera Parameter: "+value);
    ParameterCamera.set(value);
}
//ParameterCenter_X
var ParameterCenter_X = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/Center/Center_X',
});

ParameterCenter_X.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        obj[0].value = value;
        document.getElementsByName("CenterLabel")[0].value = value;
    }
});


//ParameterCenter_Y
var ParameterCenter_Y = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/Center/Center_Y',
});

ParameterCenter_Y.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        obj[1].value = value;
        document.getElementsByName("CenterLabel")[1].value = value;
    }
});



//ParameterCenter_inner
var ParameterCenter_Inner = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/Center/Inner',
});

ParameterCenter_Inner.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        obj[2].value = value;
        document.getElementsByName("CenterLabel")[2].value = value;
    }
});



//ParameterCenter_Outer
var ParameterCenter_Outer = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/Center/Outer',
});

ParameterCenter_Outer.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        obj[3].value = value;
        document.getElementsByName("CenterLabel")[3].value = value;
    }
});


//ParameterCenter_Front
var ParameterCenter_Front = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/Center/Front',
});

ParameterCenter_Front.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        obj[4].value = value;
        document.getElementsByName("CenterLabel")[4].value = value;
    }
});


//ParameterCenter_Camera_high
var ParameterCenter_Camera_high = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/Center/Camera_high',
});

ParameterCenter_Camera_high.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        obj[5].value = value;
    }
});

function ParameterCenterTransfer() {
    let value = new Array();
    for(let i=0; i<6; i++){
        value[i] = parseInt(document.getElementsByName('CenterElement')[i].value);
    }
   
    ParameterCenter_X.set(value[0]);
    ParameterCenter_Y.set(value[1]);
    ParameterCenter_Inner.set(value[2]);
    ParameterCenter_Outer.set(value[3]);
    ParameterCenter_Front.set(value[4]);
    ParameterCenter_Camera_high.set(value[5]);

    let str = "Sent Center Parameter: ";
    for(let i=0; i<6; i++){
        str += value[i];
        if(i!=5){
            str +=", ";        
        }
    }
    console.log(str);
    SendMsgs(str);
}

//ParameterScan_Angel_Near_Gap
var ParameterScan1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/SCAN/Angle_Near_Gap',
});

ParameterScan1.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[0].value = value;
        document.getElementsByName("ScanLabel")[0].value = value; 
    }
});


//ParameterScan_Magn_Near_Gap
var ParameterScan2 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/SCAN/Magn_Near_Gap',
});

ParameterScan2.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[1].value = value;
        document.getElementsByName("ScanLabel")[1].value = value; 
    }
});

//ParameterScan_Magn_Near_Start
var ParameterScan3 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/SCAN/Magn_Near_Start',
});

ParameterScan3.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[2].value = value;
        document.getElementsByName("ScanLabel")[2].value = value; 
    }
});

//ParameterScan_Magn_Middel_Start
var ParameterScan4 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/SCAN/Magn_Middle_Start',
});

ParameterScan4.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[3].value = value;
        document.getElementsByName("ScanLabel")[3].value = value; 
    }
});

//ParameterScan_Magn_Far_Start
var ParameterScan5 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/SCAN/Magn_Far_Start',
});

ParameterScan5.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[4].value = value;
        document.getElementsByName("ScanLabel")[4].value = value; 
    }
});

//ParameterScan_Magn_Far_End
var ParameterScan6 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/SCAN/Magn_Far_End',
});

ParameterScan6.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[5].value = value;
        document.getElementsByName("ScanLabel")[5].value = value; 
    }
});


//ParameterScan_Dont_Search_Angle_1
var ParameterScan7 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/SCAN/Dont_Search_Angle_1',
});

ParameterScan7.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[6].value = value;
        document.getElementsByName("ScanLabel")[6].value = value; 
    }
});


//ParameterScan_Dont_Search_Angle_2
var ParameterScan8 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/SCAN/Dont_Search_Angle_2',
});

ParameterScan8.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[7].value = value;
        document.getElementsByName("ScanLabel")[7].value = value; 
    }
});

//ParameterScan_Dont_Search_Angle_3
var ParameterScan9 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/SCAN/Dont_Search_Angle_3',
});

ParameterScan9.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[8].value = value;
        document.getElementsByName("ScanLabel")[8].value = value; 
    }
});

//ParameterScan_Angle_range_1
var ParameterScan10 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/SCAN/Angle_range_1',
});

ParameterScan10.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[9].value = value;
        document.getElementsByName("ScanLabel")[9].value = value; 
    }
});

//ParameterScan_Angle_range_2_3
var ParameterScan11 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/SCAN/Angle_range_2_3',
});

ParameterScan11.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[10].value = value;
        document.getElementsByName("ScanLabel")[10].value = value; 
    }
});

function ParameterScanTransfer() {
    let value = new Array();
    for(let i=0; i<11; i++){
        value[i] = parseInt(document.getElementsByName('ScanElement')[i].value);
    }
    ParameterScan1.set(value[0]);
    ParameterScan2.set(value[1]);
    ParameterScan3.set(value[2]);
    ParameterScan4.set(value[3]);
    ParameterScan5.set(value[4]);
    ParameterScan6.set(value[5]);
    ParameterScan7.set(value[6]);
    ParameterScan8.set(value[7]);
    ParameterScan9.set(value[8]);
    ParameterScan10.set(value[9]);
    ParameterScan11.set(value[10]);

    let str = "Sent Scan Parameter: ";
    for(let i=0; i<11; i++){
        str += value[i];
        if(i!=10){
            str +=", ";        
        }
    }
    console.log(str);
    SendMsgs(str);
}


//ParameterHSV
var ParameterHSV_Ball = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/HSV/Ball',
});
var ParameterHSV_Green = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/HSV/Green',
});
var ParameterHSV_Blue = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/HSV/Blue',
});
var ParameterHSV_Yellow = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/HSV/Yellow',
});
var ParameterHSV_White = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/HSV/White',
});

ParameterHSV_Ball.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("HSVElement");
        for (var i = 0; i < obj.length; i++) {
            //obj[i].value = value[i];
            OrangeBox[i] = value[i];
            obj[i].value = value[i];
            document.getElementsByName('HSVElement2')[i].value = value[i];
        }
    }
});

ParameterHSV_Green.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("HSVElement");
        for (var i = 0; i < obj.length; i++) {
            GreenBox[i] = value[i];
        }
    }
});

ParameterHSV_Blue.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("HSVElement");
        for (var i = 0; i < obj.length; i++) {
            BlueBox[i] = value[i];
        }
    }
});

ParameterHSV_Yellow.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("HSVElement");
        for (var i = 0; i < obj.length; i++) {
            YellowBox[i] = value[i];
        }
    }
});

ParameterHSV_White.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("HSVElement");
        for (var i = 0; i < obj.length; i++) {
            WhiteBox[i] = value[i];
        }
    }
});


function ParameterHSVTransfer() {
    var mode = parseInt(document.getElementById('HSVSelect').value);
    var box = [];
    let str = "";
    switch (mode) {
        case 0:
            for (let i = 0; i < 6; i++) {
                box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            str += "Sent Red Parameter: ";
            ParameterHSV_Ball.set(box);
            break;
        case 1:
            for (var i = 0; i < 6; i++) {
                box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            str += "Sent Green Parameter: ";
            ParameterHSV_Green.set(box);
            break;
        case 2:
            for (var i = 0; i < 6; i++) {
                box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
           str += "Sent Blue Parameter: ";
            ParameterHSV_Blue.set(box);
            break;
        case 3:
            for (var i = 0; i < 6; i++) {
                box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            str += "Sent Yellow Parameter: ";
            ParameterHSV_Yellow.set(box);
            break;
        case 4:
            for (var i = 0; i < 6; i++) {
                box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            str += "Sent White Parameter: ";
            ParameterHSV_White.set(box);
            break;
    }
    for(let i=0; i<6; i++){
        str += box[i];
        if(i!=5){
            str +=", ";        
        }
    }
    console.log(str);
    SendMsgs(str);
}
//ParameterWhite
//==========================================
var ParameterWhite_gray = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/HSV/white/gray',
});

ParameterWhite_gray.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("WhiteElement");
        obj[0].value = value;
        document.getElementsByName("WhiteLabel")[0].value = value; 
    }
});

var ParameterWhite_angle = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/HSV/white/angle',
});

ParameterWhite_angle.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("WhiteElement");
        obj[1].value = value;
        document.getElementsByName("WhiteLabel")[1].value = value; 
    }
});
function ParameterWhiteTransfer() {
    let value = new Array();
    let str = "";
    value[0]=parseInt(document.getElementsByName('WhiteElement')[0].value);
    value[1]=parseInt(document.getElementsByName('WhiteElement')[1].value);
    ParameterWhite_gray.set(value[0]);
    ParameterWhite_angle.set(value[1]);
    str += "Sent White_Line Parameter: ";
    for(let i=0; i<2; i++){
        str += value[i];
        if(i!=1){
            str +=", ";        
        }
    }
    SendMsgs(str);
    
}
//=================================
var ParameterWhite_gray2 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/HSV/white/gray',
});

ParameterWhite_gray2.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("WhiteElement");
        obj[0].value = value;
        document.getElementsByName("WhiteLabel")[0].value = value; 
    }
});

var ParameterWhite_angle2 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/HSV/white/angle',
});

ParameterWhite_angle2.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("WhiteElement");
        obj[1].value = value;
        document.getElementsByName("WhiteLabel")[1].value = value; 
    }
});

function ParameterWhiteTransfer2() {
    let value = new Array();
    let str = "";
    value[0]=parseInt(document.getElementsByName('WhiteElement')[0].value);
    value[1]=parseInt(document.getElementsByName('WhiteElement')[1].value);
    ParameterWhite_gray2.set(value[0]);
    ParameterWhite_angle2.set(value[1]);
    str += "Sent White_Line Parameter: ";
    for(let i=0; i<2; i++){
        str += value[i];
        if(i!=1){
            str +=", ";        
        }
    }
    //SendMsgs(str);
    
}
//==========================================
//ParameterBlack
var ParameterBlack_angle = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/HSV/black/angle',
});

ParameterBlack_angle.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("BlackElement");
        obj[1].value = value;
        document.getElementsByName("BlackLabel")[1].value = value; 
    }
});


var ParameterBlack_gray = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/HSV/black/gray',
});

ParameterBlack_gray.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("BlackElement");
        obj[0].value = value;
        document.getElementsByName("BlackLabel")[0].value = value; 
    }
});
function ParameterBlackTransfer() {
    let value = new Array();
    let str = "";
    value[0]=parseInt(document.getElementsByName('BlackElement')[0].value);
    value[1]=parseInt(document.getElementsByName('BlackElement')[1].value);
    ParameterWhite_gray.set(value[0]);
    ParameterBlack_angle.set(value[1]);
    str += "Sent Black_Item Parameter: ";
    for(let i=0; i<2; i++){
        str += value[i];
        if(i!=1){
            str +=", ";        
        }
    }
    SendMsgs(str);
    
}
//======================================
var ParameterBlack_angle2 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/HSV/black/angle',
});

ParameterBlack_angle2.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("BlackElement");
        obj[1].value = value;
        document.getElementsByName("BlackLabel")[1].value = value; 
    }
});


var ParameterBlack_gray2 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/HSV/black/gray',
});

ParameterBlack_gray2.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("BlackElement");
        obj[0].value = value;
        document.getElementsByName("BlackLabel")[0].value = value; 
    }
});
function ParameterBlackTransfer2() {
    let value = new Array();
    let str = "";
    value[0]=parseInt(document.getElementsByName('BlackElement')[0].value);
    value[1]=parseInt(document.getElementsByName('BlackElement')[1].value);
    ParameterWhite_gray2.set(value[0]);
    ParameterBlack_angle2.set(value[1]);
    str += "Sent Black_Item Parameter: ";
    for(let i=0; i<2; i++){
        str += value[i];
        if(i!=1){
            str +=", ";        
        }
    }
    //SendMsgs(str);
    
}
