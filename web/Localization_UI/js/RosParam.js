var IsSimulator = new ROSLIB.Param({
    ros: ros,
    name: 'FIRA/IsSimulator'
    // messageType: 'std_msgs/Int32'
});
var SPlanningVelocityBox = new ROSLIB.Param({
    ros: ros,
    name: 'FIRA/SPlanning_Velocity'
});
var HoldConditionBox = new ROSLIB.Param({
    ros: ros,
    name: 'FIRA/hold_condition'
});
var param_afast = new ROSLIB.Param({
    ros: ros,
    name: 'mcl/a_fast'
});
var param_aslow = new ROSLIB.Param({
    ros: ros,
    name: 'mcl/a_slow'
});
var param_wcmps = new ROSLIB.Param({
    ros: ros,
    name: 'mcl/wcmps'
});
function GeneralTransfer(){
    let Box1 = [];
    let Box2 = [];
    let Smallbox1 = document.getElementsByName("SPlanningVelocityElement");
    let Smallbox2 = document.getElementsByName("BallElement");

    for(var i = 0 ;i < Smallbox1.length ;i++){
        temp = Smallbox1[i].value
        Box1[i] = parseFloat(temp);
    }
    for(var i = 0;i < Smallbox2.length; i++){
        temp = Smallbox2[i].value
        Box2[i] = parseFloat(temp);

    }
    SPlanningVelocityBox.set(Box1);
    HoldConditionBox.set(Box2);
    console.log(Box1);
    console.log(Box2);
    //==========================
    let mcl=[1.0, 0.0005, 0.1];
    if(document.getElementById("MCLElementLabel").checked){
        for (let i = 0; i < 3; i++) {
            mcl[i]=document.getElementsByName('MCLElement')[i].value;
            console.log(mcl[i]);
        }
        param_afast.set(mcl[0]);
        param_aslow.set(mcl[1]);
        param_wcmps.set(mcl[2]);
    }
    //==========================
}
//GeneralReset
function GeneralReset(){
    let obj1 = [2.2,0.3,80.0,50.0,20,3,144,5];
    let obj2 = [3.0,0.33,9.0,0.4];
    let obj3 = [1.0, 0.0005, 0.1];
    let Smallbox1 = document.getElementsByName("SPlanningVelocityElement");
    let Smallbox2 = document.getElementsByName("BallElement");

    for(let i = 0 ;i < Smallbox1.length ;i++){
        Smallbox1[i].value = obj1[i];
    }
    for(let i=0;i<Smallbox2.length ;i++){
        Smallbox2[i].value = obj2[i];
    }
    //GeneralTransfer();
    //==========================
    if(document.getElementById("MCLElementLabel").checked){
        for (let i = 0; i < obj3.length; i++) {
            document.getElementsByName('MCLElement')[i].value = obj3[i];
            //console.log(obj3[i]);
        }
        param_afast.set(obj3[0]);
        param_aslow.set(obj3[1]);
        param_wcmps.set(obj3[2]);
    }
    //==========================
    // console.log(obj);
}
//GeneralGet
var obj;
SPlanningVelocityBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SPlanningVelocityElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
HoldConditionBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("BallElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
param_afast.get(function(value) {
    
    if (value != null) {
        obj = document.getElementsByName("MCLElement");
        obj[0].value = value;
    }
});
param_aslow.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("MCLElement");
        obj[1].value = value;
    }
});
param_wcmps.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("MCLElement");
        obj[2].value = value;   
    }
});