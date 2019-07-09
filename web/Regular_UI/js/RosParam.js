//============================================================================
//RobotNumber
var RobotNumber1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/RobotNumber',
});
var RobotNumber2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/RobotNumber',
});
var RobotNumber3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/RobotNumber',
});

function ChooseRobotNumber(priority, value) {
    RobNum[priority] = parseInt(value);
}

function SetParamRobotNum() {
    console.log(RobNum);
    if (RobNum[0] == RobNum[1]) {
        alert('RobNum repeat');
        RobNum[0] = 0;
        RobNum[1] = 1;
        RobNum[2] = 2;
    }
    if (RobNum[0] == RobNum[2]) {
        alert('RobNum repeat');
        RobNum[0] = 0;
        RobNum[1] = 1;
        RobNum[2] = 2;
    }
    if (RobNum[1] == RobNum[2]) {
        alert('RobNum repeat');
        RobNum[0] = 0;
        RobNum[1] = 1;
        RobNum[2] = 2;
    }
    if (CheckIP[0] == 1) {
        RobotNumber1.set(RobNum[0]);
        PublishTopicSaveParam();
    }
    if (CheckIP[1] == 1) {
        RobotNumber2.set(RobNum[1]);
        PublishTopicSaveParam();
    }
    if (CheckIP[2] == 1) {
        RobotNumber3.set(RobNum[2]);
        PublishTopicSaveParam();
    }
}
//============================================================================
//General_setting

var SPlanningVelocityBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SPlanning_Velocity'
});
var PathPlanBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/General_PathPlan'
});

var SPlanningVelocityBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/SPlanning_Velocity'
});
var PathPlanBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/General_PathPlan'
});

var SPlanningVelocityBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/SPlanning_Velocity'
});
var PathPlanBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/General_PathPlan'
});

function GetGeneralValue() {

    var SVBox1 = [];
    var PPBox1 = [];
    var SVBox2 = [];
    var PPBox2 = [];
    var SVBox3 = [];
    var PPBox3 = [];

    $("[name=SPlanningVelocityElement1]").each(function() {
        SVBox1.push(parseFloat($(this).val()));
    });
    $("[name=PathPlanElement1]").each(function() {
        PPBox1.push(parseFloat($(this).val()));
    });
    localStorage.setItem("GeneralSPlanStr1", JSON.stringify(SVBox1));
    localStorage.setItem("GeneralPathPlanStr1", JSON.stringify(PPBox1));

    $("[name=SPlanningVelocityElement2]").each(function() {
        SVBox2.push(parseFloat($(this).val()));
    });
    $("[name=PathPlanElement2]").each(function() {
        PPBox2.push(parseFloat($(this).val()));
    });
    localStorage.setItem("GeneralSPlanStr2", JSON.stringify(SVBox2));
    localStorage.setItem("GeneralPathPlanStr2", JSON.stringify(PPBox2));

    $("[name=SPlanningVelocityElement3]").each(function() {
        SVBox3.push(parseFloat($(this).val()));
    });
    $("[name=PathPlanElement3]").each(function() {
        PPBox3.push(parseFloat($(this).val()));
    });
    localStorage.setItem("GeneralSPlanStr3", JSON.stringify(SVBox3));
    localStorage.setItem("GeneralPathPlanStr3", JSON.stringify(PPBox3));
    //console.log(SVBox1, DSBox1, SVBox2, DSBox2, SVBox3, DSBox3);
    SetParamGeneral(SVBox1, PPBox1, SVBox2, PPBox2, SVBox3, PPBox3);
}

function SetParamGeneral(SVBox1, PPBox1, SVBox2, PPBox2, SVBox3, PPBox3) {
    SPlanningVelocityBox1.set(SVBox1);
    PathPlanBox1.set(PPBox1);

    SPlanningVelocityBox2.set(SVBox2);
    PathPlanBox2.set(PPBox2);

    SPlanningVelocityBox3.set(SVBox3);
    PathPlanBox3.set(PPBox3);
    //console.log(SVBox1, SVBox2, SVBox3);
    console.log(PPBox1, PPBox2, PPBox3);
    up();
    PublishTopicSaveParam();
}

SPlanningVelocityBox1.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("SPlanningVelocityElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
PathPlanBox1.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("PathPlanElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});

SPlanningVelocityBox2.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("SPlanningVelocityElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
PathPlanBox2.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("PathPlanElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});

SPlanningVelocityBox3.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("SPlanningVelocityElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
PathPlanBox3.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("PathPlanElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
//============================================================================
//Pathplan_setting

var AttackStrategyBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Attack_Strategy'
});
var ChaseStrategyBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Chase_Strategy'
});
var ZoneAttackBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Zone_Attack'
});
var TypeSAttackBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/TypeS_Attack'
});
var PGoalkeeperBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Goalkeeper'
});
var SideSpeedUpBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SideSpeedUp'
});
var DorsadAttackBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Dorsad_Attack'
});
var CornerKickBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Corner_Kick'
});
var PenaltyKickBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Penalty_Kick'
});

var AttackStrategyBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/Attack_Strategy'
});
var ChaseStrategyBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/Chase_Strategy'
});
var ZoneAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/Zone_Attack'
});
var TypeSAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/TypeS_Attack'
});
var TypeUAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/TypeU_Attack'
});
var SideSpeedUpBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/SideSpeedUp'
});
var DorsadAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/Dorsad_Attack'
});
var CornerKickBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/Corner_Kick'
});
var PenaltyKickBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/Penalty_Kick'
});

var AttackStrategyBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/Attack_Strategy'
});
var ChaseStrategyBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/Chase_Strategy'
});
var ZoneAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/Zone_Attack'
});
var TypeSAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/TypeS_Attack'
});
var TypeUAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/TypeU_Attack'
});
var SideSpeedUpBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/SideSpeedUp'
});
var DorsadAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/Dorsad_Attack'
});
var CornerKickBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/Corner_Kick'
});
var PenaltyKickBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/Penalty_Kick'
});

function GetPathplanValue() {

    var ASBox1 = [];
    var CSBox1 = [];
    var ZABox1 = [];
    var TSABox1 = [];
    var PGKBox1 = [];
    var SSUBox1 = [];
    var DABox1 = [];
    var CKBox1 = [];
    var PKBox1 = [];

    var ASBox2 = [];
    var CSBox2 = [];
    var ZABox2 = [];
    var TSABox2 = [];
    var TUABox2 = [];
    var SSUBox2 = [];
    var DABox2 = [];
    var CKBox2 = [];
    var PKBox2 = [];

    var ASBox3 = [];
    var CSBox3 = [];
    var ZABox3 = [];
    var TSABox3 = [];
    var TUABox3 = [];
    var SSUBox3 = [];
    var DABox3 = [];
    var CKBox3 = [];
    var PKBox3 = [];



    $("[name=AttackStrategyElement1]").each(function() {
        ASBox1.push(parseFloat($(this).val()));
    });
    $("[name=ChaseStrategyElement1]").each(function() {
        CSBox1.push(parseFloat($(this).val()));
    });
    $("[name=ZoneAttackElement1]").each(function() {
        ZABox1.push(parseFloat($(this).val()));
    });
    $("[name=TypeSAttackElement1]").each(function() {
        TSABox1.push(parseFloat($(this).val()));
    });
    $("[name=PGoalkeeperElement1]").each(function() {
        PGKBox1.push(parseFloat($(this).val()));
    });
    $("[name=SideSpeedUpElement1]").each(function() {
        SSUBox1.push(parseFloat($(this).val()));
    });
    $("[name=DorsadAttackElement1]").each(function() {
        DABox1.push(parseFloat($(this).val()));
    });
    $("[name=CornerKickElement1]").each(function() {
        CKBox1.push(parseFloat($(this).val()));
    });
    /*$("[name=PenaltyKickElement1]").each(function() {
        PKBox1.push(parseFloat($(this).val()));
    });*/
    obj = document.getElementsByName('PenaltyKickElement1');
    PKBox1.push(parseFloat(obj[0].value));
    for (var i = 1; i < obj.length; i++) {
        if (obj[i].checked == true)
            PKBox1.push(parseFloat(i - 1));
    }

    localStorage.setItem("PathplanAtkStrategyStr1", JSON.stringify(ASBox1));
    localStorage.setItem("PathplanChaseStrategyStr1", JSON.stringify(CSBox1));
    localStorage.setItem("PathplanZoneAtkStr1", JSON.stringify(ZABox1));
    localStorage.setItem("PathplanTypeSAtkStr1", JSON.stringify(TSABox1));
    localStorage.setItem("PathplanGoalkeeperStr1", JSON.stringify(PGKBox1));
    localStorage.setItem("PathplanSideSpeedUpStr1", JSON.stringify(SSUBox1));
    localStorage.setItem("PathplanDorsadAttackStr1", JSON.stringify(DABox1));
    localStorage.setItem("PathplanCornerKickStr1", JSON.stringify(CKBox1));
    localStorage.setItem("PathplanPenaltyKickStr1", JSON.stringify(PKBox1));


    $("[name=AttackStrategyElement2]").each(function() {
        ASBox2.push(parseFloat($(this).val()));
    });
    $("[name=ChaseStrategyElement2]").each(function() {
        CSBox2.push(parseFloat($(this).val()));
    });
    $("[name=ZoneAttackElement2]").each(function() {
        ZABox2.push(parseFloat($(this).val()));
    });
    $("[name=TypeSAttackElement2]").each(function() {
        TSABox2.push(parseFloat($(this).val()));
    });
    $("[name=TypeUAttackElement2]").each(function() {
        TUABox2.push(parseFloat($(this).val()));
    });
    $("[name=SideSpeedUpElement2]").each(function() {
        SSUBox2.push(parseFloat($(this).val()));
    });
    $("[name=DorsadAttackElement2]").each(function() {
        DABox2.push(parseFloat($(this).val()));
    });
    $("[name=CornerKickElement2]").each(function() {
        CKBox2.push(parseFloat($(this).val()));
    });
    // $("[name=PenaltyKickElement2]").each(function() {
    //     PKBox2.push(parseFloat($(this).val()));
    // });
    obj = document.getElementsByName('PenaltyKickElement2');
    PKBox2.push(parseFloat(obj[0].value));
    for (var i = 1; i < obj.length; i++) {
        if (obj[i].checked == true)
            PKBox2.push(parseFloat(i - 1));
    }
    localStorage.setItem("PathplanAtkStrategyStr2", JSON.stringify(ASBox2));
    localStorage.setItem("PathplanChaseStrategyStr2", JSON.stringify(CSBox2));
    localStorage.setItem("PathplanZoneAtkStr2", JSON.stringify(ZABox2));
    localStorage.setItem("PathplanTypeSAtkStr2", JSON.stringify(TSABox2));
    localStorage.setItem("PathplanTypeUAtkStr2", JSON.stringify(TUABox2));
    localStorage.setItem("PathplanSideSpeedUpStr2", JSON.stringify(SSUBox2));
    localStorage.setItem("PathplanDorsadAttackStr2", JSON.stringify(DABox2));
    localStorage.setItem("PathplanCornerKickStr2", JSON.stringify(CKBox2));
    localStorage.setItem("PathplanPenaltyKickStr1", JSON.stringify(PKBox2));


    $("[name=AttackStrategyElement3]").each(function() {
        ASBox3.push(parseFloat($(this).val()));
    });
    $("[name=ChaseStrategyElement3]").each(function() {
        CSBox3.push(parseFloat($(this).val()));
    });
    $("[name=ZoneAttackElement3]").each(function() {
        ZABox3.push(parseFloat($(this).val()));
    });
    $("[name=TypeSAttackElement3]").each(function() {
        TSABox3.push(parseFloat($(this).val()));
    });
    $("[name=TypeUAttackElement3]").each(function() {
        TUABox3.push(parseFloat($(this).val()));
    });
    $("[name=SideSpeedUpElement3]").each(function() {
        SSUBox3.push(parseFloat($(this).val()));
    });
    $("[name=DorsadAttackElement3]").each(function() {
        DABox3.push(parseFloat($(this).val()));
    });
    $("[name=CornerKickElement3]").each(function() {
        CKBox3.push(parseFloat($(this).val()));
    });
    // $("[name=PenaltyKickElement3]").each(function() {
    //     PKBox3.push(parseFloat($(this).val()));
    // });
    obj = document.getElementsByName('PenaltyKickElement3');
    PKBox3.push(parseFloat(obj[0].value));
    for (var i = 1; i < obj.length; i++) {
        if (obj[i].checked == true)
            PKBox3.push(parseFloat(i - 1));
    }
    localStorage.setItem("PathplanAtkStrategyStr3", JSON.stringify(ASBox3));
    localStorage.setItem("PathplanChaseStrategyStr3", JSON.stringify(CSBox3));
    localStorage.setItem("PathplanZoneAtkStr3", JSON.stringify(ZABox3));
    localStorage.setItem("PathplanTypeSAtkStr3", JSON.stringify(TSABox3));
    localStorage.setItem("PathplanTypeUAtkStr3", JSON.stringify(TUABox3));
    localStorage.setItem("PathplanSideSpeedUpStr3", JSON.stringify(SSUBox3));
    localStorage.setItem("PathplanDorsadAttackStr3", JSON.stringify(DABox3));
    localStorage.setItem("PathplanCornerKickStr3", JSON.stringify(CKBox3));
    localStorage.setItem("PathplanPenaltyKickStr1", JSON.stringify(PKBox3));

    SetParamPathplan(ASBox1, CSBox1, ZABox1, TSABox1, PGKBox1, SSUBox1, DABox1, CKBox1, PKBox1,
        ASBox2, CSBox2, ZABox2, TSABox2, TUABox2, SSUBox2, DABox2, CKBox2, PKBox2,
        ASBox3, CSBox3, ZABox3, TSABox3, TUABox3, SSUBox3, DABox3, CKBox3, PKBox3);

}

function SetParamPathplan(ASBox1, CSBox1, ZABox1, TSABox1, PGKBox1, SSUBox1, DABox1, CKBox1, PKBox1,
    ASBox2, CSBox2, ZABox2, TSABox2, TUABox2, SSUBox2, DABox2, CKBox2, PKBox2,
    ASBox3, CSBox3, ZABox3, TSABox3, TUABox3, SSUBox3, DABox3, CKBox3, PKBox3) {

    console.log(PGKBox1);
    AttackStrategyBox1.set(ASBox1);
    ChaseStrategyBox1.set(CSBox1);
    ZoneAttackBox1.set(ZABox1);
    TypeSAttackBox1.set(TSABox1);
    PGoalkeeperBox1.set(PGKBox1);
    SideSpeedUpBox1.set(SSUBox1);
    DorsadAttackBox1.set(DABox1);
    CornerKickBox1.set(CKBox1);
    PenaltyKickBox1.set(PKBox1);

    AttackStrategyBox2.set(ASBox2);
    ChaseStrategyBox2.set(CSBox2);
    ZoneAttackBox2.set(ZABox2);
    TypeSAttackBox2.set(TSABox2);
    TypeUAttackBox2.set(TUABox2);
    SideSpeedUpBox2.set(SSUBox2);
    DorsadAttackBox2.set(DABox2);
    CornerKickBox2.set(CKBox2);
    PenaltyKickBox2.set(PKBox2);

    AttackStrategyBox3.set(ASBox3);
    ChaseStrategyBox3.set(CSBox3);
    ZoneAttackBox3.set(ZABox3);
    TypeSAttackBox3.set(TSABox3);
    TypeUAttackBox3.set(TUABox3);
    SideSpeedUpBox3.set(SSUBox3);
    DorsadAttackBox3.set(DABox3);
    CornerKickBox3.set(CKBox3);
    PenaltyKickBox3.set(PKBox3);

    up();
    PublishTopicSaveParam();
}

AttackStrategyBox1.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("AttackStrategyElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ChaseStrategyBox1.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ChaseStrategyElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ZoneAttackBox1.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ZoneAttackElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
TypeSAttackBox1.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("TypeSAttackElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
PGoalkeeperBox1.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("PGoalkeeperElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
SideSpeedUpBox1.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("SideSpeedUpElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
DorsadAttackBox1.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("DorsadAttackElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
CornerKickBox1.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("CornerKickElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
PenaltyKickBox1.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("PenaltyKickElement1");
        obj[0].value = value[0];
        obj[value[1] + 1].checked = true;
    }
});
AttackStrategyBox2.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("AttackStrategyElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ChaseStrategyBox2.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ChaseStrategyElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ZoneAttackBox2.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ZoneAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
TypeSAttackBox2.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("TypeSAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
TypeUAttackBox2.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("TypeUAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
SideSpeedUpBox2.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("SideSpeedUpElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
DorsadAttackBox2.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("DorsadAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
CornerKickBox2.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("CornerKickElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
PenaltyKickBox2.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("PenaltyKickElement2");
        obj[0].value = value[0];
        obj[value[1] + 1].checked = true;
    }
});

AttackStrategyBox3.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("AttackStrategyElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ChaseStrategyBox3.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ChaseStrategyElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ZoneAttackBox3.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("ZoneAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
TypeSAttackBox3.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("TypeSAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
TypeUAttackBox3.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("TypeUAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
SideSpeedUpBox3.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("SideSpeedUpElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
DorsadAttackBox3.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("DorsadAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
CornerKickBox3.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("CornerKickElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
PenaltyKickBox3.get(function(value) {
    if (value != null) {
        CheckGetParm = 1;
        obj = document.getElementsByName("PenaltyKickElement3");
        obj[0].value = value[0];
        obj[value[1] + 1].checked = true;
    }
});
//============================================================================
//Behavior_variable
var StateChaseBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/Chase_Strategy'
});
var StateAttackBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/Attack_Strategy'
});
var GoalkeeperBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/Goalkeeper'
});
var StateTypeSAttackBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/TypeS_Attack'
});
var StateSideSpeedUPBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/Side_Speed_UP'
});
var StateZoneAttackBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/Zone_Attack'
});
var StateCornerKickBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/Corner_Kick'
});
var StrategySelectBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/StrategySelection',
});
var StrategySelectPrefixBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/PrefixSetting',
});
var SupportStrategyBox1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/Support_Strategy',
});


var StateChaseBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/Chase_Strategy'
});
var StateAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/Attack_Strategy'
});
var StateTypeUChaseBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/TypeU_Chase'
});
var StateTypeSAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/TypeS_Attack'
});
var StateSideSpeedUPBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/Side_Speed_UP'
});
var StateZoneAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/Zone_Attack'
});
var StateCornerKickBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/Corner_Kick'
});
var StrategySelectBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/StrategySelection',
});
var StrategySelectPrefixBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/PrefixSetting',
});
var SupportStrategyBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/Support_Strategy',
});


var StateChaseBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/Chase_Strategy'
});
var StateAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/Attack_Strategy'
});
var StateTypeUChaseBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/TypeU_Chase'
});
var StateTypeSAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/TypeS_Attack'
});
var StateSideSpeedUPBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/Side_Speed_UP'
});
var StateZoneAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/Zone_Attack'
});
var StateCornerKickBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/Corner_Kick'
});
var StrategySelectBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/StrategySelection',
});
var StrategySelectPrefixBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/PrefixSetting',
});
var SupportStrategyBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/Support_Strategy',
});


function GetBehaviorValue() {

    var flag = 0;

    var SCBox1 = [];
    var SABox1 = [];
    var GKBox1 = [];
    var STSABox1 = [];
    var SSSUBox1 = [];
    var SZABox1 = [];
    var SCKBox1 = [];
    var SSBox1 = [];
    var SSPBox1 = [];
    var SupSBox1 = [];

    var SCBox2 = [];
    var SABox2 = [];
    var STUCBox2 = [];
    var STSABox2 = [];
    var SSSUBox2 = [];
    var SZABox2 = [];
    var SCKBox2 = [];
    var SSBox2 = [];
    var SSPBox2 = [];
    var SupSBox2 = [];

    var SCBox3 = [];
    var SABox3 = [];
    var STUCBox3 = [];
    var STSABox3 = [];
    var SSSUBox3 = [];
    var SZABox3 = [];
    var SCKBox3 = [];
    var SSBox3 = [];
    var SSPBox3 = [];
    var SupSBox3 = [];

    $("[name=StateChaseElement1]").each(function() {
        SCBox1.push(parseFloat($(this).val()));
    });
    $("[name=StateAttackElement1]").each(function() {
        SABox1.push(parseFloat($(this).val()));
    });
    $("[name=GoalkeeperElement1]").each(function() {
        GKBox1.push(parseFloat($(this).val()));
    });
    $("[name=StateTypeSAttackElement1]").each(function() {
        STSABox1.push(parseFloat($(this).val()));
    });
    $("[name=StateSideSpeedUPElement1]").each(function() {
        SSSUBox1.push(parseFloat($(this).val()));
    });
    $("[name=StateZoneAttackElement1]").each(function() {
        SZABox1.push(parseFloat($(this).val()));
    });
    $("[name=StateCornerKickElement1]").each(function() {
        SCKBox1.push(parseFloat($(this).val()));
    });
    if (document.getElementById("StrategySelectionProtect1").checked) {
        obj = document.getElementsByName("StrategySelectionElement1");
        flag = 0;
        for (var i = 0; i < 3; i++) {
            if (obj[i].checked && flag == 0) {
                SSBox1.push(parseInt(1));
                flag = 1;
            } else {
                SSBox1.push(parseInt(0));
            }
        }
        flag = 0;
        for (var i = 3; i < obj.length; i++) {
            if (obj[i].checked && flag == 0) {
                SSBox1.push(parseInt(1));
                flag = 1;
            } else {
                SSBox1.push(parseInt(0));

            }
        }
    } else {
        obj = document.getElementsByName("StrategySelectionElement1");
        for (var i = 0; i < obj.length; i++) {
            if (obj[i].checked) {
                SSBox1.push(parseInt(1));
            } else {
                SSBox1.push(parseInt(0));
            }
        }
    }
    obj = document.getElementsByName("StrategySelectionPrefixElement1");
    flag = 0;
    for (var i = 0; i < 2; i++) {
        if (obj[i].checked && flag == 0) {
            SSPBox1.push(parseFloat(1));
            flag = 1;
        } else {
            SSPBox1.push(parseFloat(0));
        }
    }
    flag = 0;
    for (var i = 2; i < 4; i++) {
        if (obj[i].checked && flag == 0) {
            SSPBox1.push(parseFloat(1));
            flag = 1;
        } else {
            SSPBox1.push(parseFloat(0));

        }
    }
    SSPBox1.push(parseFloat(obj[4].value));
    /*
    for (var i = 0; i < obj.length; i++) {
        if ((SSBox1[0] == 0) && (SSBox1[1] == 0)) {
            SSBox1[0] = parseInt(1);
        }
        if ((SSBox1[2] == 0) && (SSBox1[3] == 0) && (SSBox1[4] == 0) && (SSBox1[5] == 0) && (SSBox1[6] == 0)) {
            SSBox1[2] = parseInt(1);
        }
    }*/
    obj = document.getElementsByName("SupportStrategyElement1");
    for (var i = 0; i < obj.length; i++) {
        if (obj[i].checked) {
            SupSBox1.push(parseInt(i + 1));
        }
    }
    localStorage.setItem("BehaviorStateChaseStr1", JSON.stringify(SCBox1));
    localStorage.setItem("BehaviorStateAtkStr1", JSON.stringify(SABox1));
    localStorage.setItem("BehaviorStateGoalkeeperStr1", JSON.stringify(GKBox1));
    localStorage.setItem("BehaviorStateTypeSAtkStr1", JSON.stringify(STSABox1));
    localStorage.setItem("BehaviorStateSideSpeedUPStr1", JSON.stringify(SSSUBox1));
    localStorage.setItem("BehaviorStateZoneAtkStr1", JSON.stringify(SZABox1));
    localStorage.setItem("BehaviorStateCornerKickStr1", JSON.stringify(SCKBox1));
    localStorage.setItem("BehaviorStrategySelectionStr1", JSON.stringify(SSBox1));
    localStorage.setItem("BehaviorStrategySelectionPrefixStr1", JSON.stringify(SSPBox1));
    localStorage.setItem("BehaviorSupportStrategyStr1", JSON.stringify(SupSBox1));


    $("[name=StateChaseElement2]").each(function() {
        SCBox2.push(parseFloat($(this).val()));
    });
    $("[name=StateAttackElement2]").each(function() {
        SABox2.push(parseFloat($(this).val()));
    });
    $("[name=StateTypeUChaseElement2]").each(function() {
        STUCBox2.push(parseFloat($(this).val()));
    });
    $("[name=StateTypeSAttackElement2]").each(function() {
        STSABox2.push(parseFloat($(this).val()));
    });
    $("[name=StateSideSpeedUPElement2]").each(function() {
        SSSUBox2.push(parseFloat($(this).val()));
    });
    $("[name=StateZoneAttackElement2]").each(function() {
        SZABox2.push(parseFloat($(this).val()));
    });
    $("[name=StateCornerKickElement2]").each(function() {
        SCKBox2.push(parseFloat($(this).val()));
    });
    if (document.getElementById("StrategySelectionProtect2").checked) {
        obj = document.getElementsByName("StrategySelectionElement2");
        flag = 0;
        for (var i = 0; i < 3; i++) {
            if (obj[i].checked && flag == 0) {
                SSBox2.push(parseInt(1));
                flag = 1;
            } else {
                SSBox2.push(parseInt(0));
            }
        }
        flag = 0;
        for (var i = 3; i < obj.length; i++) {
            if (obj[i].checked && flag == 0) {
                SSBox2.push(parseInt(1));
                flag = 1;
            } else {
                SSBox2.push(parseInt(0));

            }
        }
    } else {
        obj = document.getElementsByName("StrategySelectionElement2");
        for (var i = 0; i < obj.length; i++) {
            if (obj[i].checked) {
                SSBox2.push(parseInt(1));
            } else {
                SSBox2.push(parseInt(0));
            }
        }
    }
    obj = document.getElementsByName("StrategySelectionPrefixElement2");
    flag = 0;
    for (var i = 0; i < 2; i++) {
        if (obj[i].checked && flag == 0) {
            SSPBox2.push(parseFloat(1));
            flag = 1;
        } else {
            SSPBox2.push(parseFloat(0));
        }
    }
    flag = 0;
    for (var i = 2; i < 4; i++) {
        if (obj[i].checked && flag == 0) {
            SSPBox2.push(parseFloat(1));
            flag = 1;
        } else {
            SSPBox2.push(parseFloat(0));

        }
    }
    SSPBox2.push(parseFloat(obj[4].value));
    /*
    for (var i = 0; i < obj.length; i++) {
        if (((SSBox2[0] == 0) && (SSBox2[1] == 0)) || ((SSBox2[0] == 1) && (SSBox2[1] == 1))) {
            SSBox2[0] = parseInt(1);
        }
        if ((SSBox2[2] == 0) && (SSBox2[3] == 0) && (SSBox2[4] == 0) && (SSBox2[5] == 0) && (SSBox2[6] == 0)) {
            SSBox2[2] = parseInt(1);
        }
    }*/
    obj = document.getElementsByName("SupportStrategyElement2");
    for (var i = 0; i < obj.length; i++) {
        if (obj[i].checked) {
            SupSBox2.push(parseInt(i + 1));
        }
    }
    localStorage.setItem("BehaviorStateChaseStr2", JSON.stringify(SCBox2));
    localStorage.setItem("BehaviorStateAtkStr2", JSON.stringify(SABox2));
    localStorage.setItem("BehaviorStateTypeUChaseStr2", JSON.stringify(STUCBox2));
    localStorage.setItem("BehaviorStateTypeSAtkStr2", JSON.stringify(STSABox2));
    localStorage.setItem("BehaviorStateSideSpeedUPStr2", JSON.stringify(SSSUBox2));
    localStorage.setItem("BehaviorStateZoneAtkStr2", JSON.stringify(SZABox2));
    localStorage.setItem("BehaviorStateCornerKickStr2", JSON.stringify(SCKBox2));
    localStorage.setItem("BehaviorStrategySelectionStr2", JSON.stringify(SSBox2));
    localStorage.setItem("BehaviorStrategySelectionPrefixStr2", JSON.stringify(SSPBox2));
    localStorage.setItem("BehaviorSupportStrategyStr2", JSON.stringify(SupSBox2));


    $("[name=StateChaseElement3]").each(function() {
        SCBox3.push(parseFloat($(this).val()));
    });
    $("[name=StateAttackElement3]").each(function() {
        SABox3.push(parseFloat($(this).val()));
    });
    $("[name=StateTypeUChaseElement3]").each(function() {
        STUCBox3.push(parseFloat($(this).val()));
    });
    $("[name=StateTypeSAttackElement3]").each(function() {
        STSABox3.push(parseFloat($(this).val()));
    });
    $("[name=StateSideSpeedUPElement3]").each(function() {
        SSSUBox3.push(parseFloat($(this).val()));
    });
    $("[name=StateZoneAttackElement3]").each(function() {
        SZABox3.push(parseFloat($(this).val()));
    });
    $("[name=StateCornerKickElement3]").each(function() {
        SCKBox3.push(parseFloat($(this).val()));
    });
    if (document.getElementById("StrategySelectionProtect3").checked) {
        obj = document.getElementsByName("StrategySelectionElement3");
        flag = 0;
        for (var i = 0; i < 3; i++) {
            if (obj[i].checked && flag == 0) {
                SSBox3.push(parseInt(1));
                flag = 1;
            } else {
                SSBox3.push(parseInt(0));
            }
        }
        flag = 0;
        for (var i = 3; i < obj.length; i++) {
            if (obj[i].checked && flag == 0) {
                SSBox3.push(parseInt(1));
                flag = 1;
            } else {
                SSBox3.push(parseInt(0));

            }
        }
    } else {
        obj = document.getElementsByName("StrategySelectionElement3");
        for (var i = 0; i < obj.length; i++) {
            if (obj[i].checked) {
                SSBox3.push(parseInt(1));
            } else {
                SSBox3.push(parseInt(0));
            }
        }
    }
    obj = document.getElementsByName("StrategySelectionPrefixElement3");
    flag = 0;
    for (var i = 0; i < 2; i++) {
        if (obj[i].checked && flag == 0) {
            SSPBox3.push(parseFloat(1));
            flag = 1;
        } else {
            SSPBox3.push(parseFloat(0));
        }
    }
    flag = 0;
    for (var i = 2; i < 4; i++) {
        if (obj[i].checked && flag == 0) {
            SSPBox3.push(parseFloat(1));
            flag = 1;
        } else {
            SSPBox3.push(parseFloat(0));

        }
    }
    SSPBox3.push(parseFloat(obj[4].value));
    /*
        for (var i = 0; i < obj.length; i++) {
            if ((SSBox3[0] == 0) && (SSBox3[1] == 0)) {
                SSBox3[0] = parseInt(1);
            }
            if ((SSBox3[2] == 0) && (SSBox3[3] == 0) && (SSBox3[4] == 0) && (SSBox3[5] == 0) && (SSBox3[6] == 0)) {
                SSBox3[2] = parseInt(1);
            }
        }*/
    obj = document.getElementsByName("SupportStrategyElement3");
    for (var i = 0; i < obj.length; i++) {
        if (obj[i].checked) {
            SupSBox3.push(parseInt(i + 1));
        }
    }
    localStorage.setItem("BehaviorStateChaseStr3", JSON.stringify(SCBox3));
    localStorage.setItem("BehaviorStateAtkStr3", JSON.stringify(SABox3));
    localStorage.setItem("BehaviorStateTypeUChaseStr3", JSON.stringify(STUCBox3));
    localStorage.setItem("BehaviorStateTypeSAtkStr3", JSON.stringify(STSABox3));
    localStorage.setItem("BehaviorStateSideSpeedUPStr3", JSON.stringify(SSSUBox3));
    localStorage.setItem("BehaviorStateZoneAtkStr3", JSON.stringify(SZABox3));
    localStorage.setItem("BehaviorStateCornerKickStr3", JSON.stringify(SCKBox3));
    localStorage.setItem("BehaviorStrategySelectionStr3", JSON.stringify(SSBox3));
    localStorage.setItem("BehaviorStrategySelectionPrefixStr3", JSON.stringify(SSPBox3));
    localStorage.setItem("BehaviorSupportStrategyStr3", JSON.stringify(SupSBox3));

    SetParamBehavior(SCBox1, SABox1, GKBox1, STSABox1, SSSUBox1, SZABox1, SCKBox1, SSBox1, SSPBox1, SupSBox1,
        SCBox2, SABox2, STUCBox2, STSABox2, SSSUBox2, SZABox2, SCKBox2, SSBox2, SSPBox2, SupSBox2,
        SCBox3, SABox3, STUCBox3, STSABox3, SSSUBox3, SZABox3, SCKBox3, SSBox3, SSPBox3, SupSBox3);
}

function SetParamBehavior(SCBox1, SABox1, GKBox1, STSABox1, SSSUBox1, SZABox1, SCKBox1, SSBox1, SSPBox1, SupSBox1,
    SCBox2, SABox2, STUCBox2, STSABox2, SSSUBox2, SZABox2, SCKBox2, SSBox2, SSPBox2, SupSBox2,
    SCBox3, SABox3, STUCBox3, STSABox3, SSSUBox3, SZABox3, SCKBox3, SSBox3, SSPBox3, SupSBox3) {
    console.log(SSBox1, SSBox2, SSBox3);
    StateChaseBox1.set(SCBox1);
    StateAttackBox1.set(SABox1);
    GoalkeeperBox1.set(GKBox1);
    StateTypeSAttackBox1.set(STSABox1);
    StateSideSpeedUPBox1.set(SSSUBox1);
    StateZoneAttackBox1.set(SZABox1);
    StateCornerKickBox1.set(SCKBox1);
    StrategySelectBox1.set(SSBox1);
    StrategySelectPrefixBox1.set(SSPBox1);
    SupportStrategyBox1.set(SupSBox1);

    StateChaseBox2.set(SCBox2);
    StateAttackBox2.set(SABox2);
    StateTypeUChaseBox2.set(STUCBox2);
    StateTypeSAttackBox2.set(STSABox2);
    StateSideSpeedUPBox2.set(SSSUBox2);
    StateZoneAttackBox2.set(SZABox2);
    StateCornerKickBox2.set(SCKBox2);
    StrategySelectBox2.set(SSBox2);
    StrategySelectPrefixBox2.set(SSPBox2);
    SupportStrategyBox2.set(SupSBox2);

    StateChaseBox3.set(SCBox3);
    StateAttackBox3.set(SABox3);
    StateTypeUChaseBox3.set(STUCBox3);
    StateTypeSAttackBox3.set(STSABox3);
    StateSideSpeedUPBox3.set(SSSUBox3);
    StateZoneAttackBox3.set(SZABox3);
    StateCornerKickBox3.set(SCKBox3);
    StrategySelectBox3.set(SSBox3);
    StrategySelectPrefixBox3.set(SSPBox3);
    SupportStrategyBox3.set(SupSBox3);

    up();
    PublishTopicSaveParam();
}

function SetBehaviorKeyborard(box1) {
    if (ChooseRobot == 1) {
        if (CheckIP[0] == 1)
            StrategySelectBox1.set(box1);
        console.log("1 ", box1);
    } else if (ChooseRobot == 2) {
        if (CheckIP[1] == 1)
            StrategySelectBox2.set(box1);
        console.log("2 ", box1);
    } else if (ChooseRobot == 3) {
        if (CheckIP[2] == 1)
            StrategySelectBox3.set(box1);
        console.log("3 ", box1);
    }
}


StateChaseBox1.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateChaseElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateAttackBox1.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateAttackElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
GoalkeeperBox1.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("GoalkeeperElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateTypeSAttackBox1.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateTypeSAttackElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateSideSpeedUPBox1.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateSideSpeedUPElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateZoneAttackBox1.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateZoneAttackElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateCornerKickBox1.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateCornerKickElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StrategySelectBox1.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StrategySelectionElement1");
        for (var i = 0; i < obj.length; i++) {
            if (value[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
    }
});
StrategySelectPrefixBox1.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StrategySelectionPrefixElement1");
        for (var i = 0; i < 4; i++) {
            if (value[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
        obj[4].value = value[4];
    }
});

SupportStrategyBox1.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SupportStrategyElement1");
        obj[parseInt(value[0]) - 1].checked = true;
    }
});

StateChaseBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateChaseElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateAttackBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateTypeUChaseBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateTypeUChaseElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateTypeSAttackBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateTypeSAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateSideSpeedUPBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateSideSpeedUPElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateZoneAttackBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateZoneAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateCornerKickBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateCornerKickElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StrategySelectBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StrategySelectionElement2");
        for (var i = 0; i < obj.length; i++) {
            if (value[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
    }
});
StrategySelectPrefixBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StrategySelectionPrefixElement2");
        for (var i = 0; i < 4; i++) {
            if (value[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
        obj[4].value = value[4];
    }
});
SupportStrategyBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SupportStrategyElement2");
        obj[parseInt(value[0]) - 1].checked = true;
    }
});

StateChaseBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateChaseElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateAttackBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateTypeUChaseBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateTypeUChaseElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateTypeSAttackBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateTypeSAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateSideSpeedUPBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateSideSpeedUPElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateZoneAttackBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateZoneAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateCornerKickBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateCornerKickElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StrategySelectBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StrategySelectionElement3");
        for (var i = 0; i < obj.length; i++) {
            if (value[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
    }
});
StrategySelectPrefixBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StrategySelectionPrefixElement3");
        for (var i = 0; i < 4; i++) {
            if (value[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
        obj[4].value = value[4];
    }
});
SupportStrategyBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SupportStrategyElement3");
        obj[parseInt(value[0]) - 1].checked = true;
    }
});

//============================================================================

/*========================================================*/

//IsSimulator
var IsSimulator1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/IsSimulator',
});
var IsSimulator2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/IsSimulator',
});
var IsSimulator3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/IsSimulator',
});

function SetParamIsSimulator(checked) {
    var temp;
    if (checked == true) {
        temp = 1;
        if (CheckIP[0] == 1)
            IsSimulator1.set(temp);
        if (CheckIP[1] == 1)
            IsSimulator2.set(temp);
        if (CheckIP[2] == 1)
            IsSimulator3.set(temp);
    } else {
        temp = 0;
        if (CheckIP[0] == 1)
            IsSimulator1.set(temp);
        if (CheckIP[1] == 1)
            IsSimulator2.set(temp);
        if (CheckIP[2] == 1)
            IsSimulator3.set(temp);
    }
    PublishTopicSaveParam();
}