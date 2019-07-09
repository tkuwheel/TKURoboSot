if ((typeof(Storage) !== "undefined") && (CheckGetParm == 0)) {
    var obj;
    // GenenalStorage
    //========================================================================
    //Robot1
    if (localStorage.getItem("GeneralSPlanStr1") != null) {
        obj = document.getElementsByName("SPlanningVelocityElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("GeneralSPlanStr1"))[i];
            //console.log(213);
        }
    } else {
        obj = document.getElementsByName("SPlanningVelocityElement1");
        //console.log(obj.length);
        obj[0].value = 2.2;
        obj[1].value = 0.3;
        obj[2].value = 50.0;
        obj[3].value = 30.0;
        obj[4].value = 20.0;
        obj[5].value = 3.0;
        obj[6].value = 144.0;
        obj[7].value = 5.0;
        obj[8].value = 10.0;
        obj[9].value = 1;
        obj[10].value = 0;
        obj[11].value = 0;
        //console.log(document.getElementsByName("SPlanningVelocityElement1")[11].value)
    }
    if (localStorage.getItem("GeneralPathPlanStr1") != null) {
        obj = document.getElementsByName("PathPlanElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("GeneralPathPlanStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("PathPlanElement1");
        obj[0].value = 0;
        obj[1].value = 0;
        obj[2].value = 0;
        obj[3].value = 0;
        obj[4].value = 0;
        obj[5].value = 0;
        obj[6].value = 0;
        obj[7].value = 0;
    }

    //Robot2
    if (localStorage.getItem("GeneralSPlanStr2") != null) {
        obj = document.getElementsByName("SPlanningVelocityElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("GeneralSPlanStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("SPlanningVelocityElement2");
        obj[0].value = 2.2;
        obj[1].value = 0.3;
        obj[2].value = 80.0;
        obj[3].value = 50.0;
        obj[4].value = 20.0;
        obj[5].value = 3.0;
        obj[6].value = 144.0;
        obj[7].value = 5.0;
        obj[8].value = 10.0;
        obj[9].value = 1;
        obj[10].value = 0;
        obj[11].value = 0;
    }
    if (localStorage.getItem("GeneralPathPlanStr2") != null) {
        obj = document.getElementsByName("PathPlanElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("GeneralPathPlanStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("PathPlanElement2");
        obj[0].value = 0;
        obj[1].value = 0;
        obj[2].value = 0;
        obj[3].value = 0;
        obj[4].value = 0;
        obj[5].value = 0;
        obj[6].value = 0;
        obj[7].value = 0;
    }

    //Robot3
    if (localStorage.getItem("GeneralSPlanStr3") != null) {
        obj = document.getElementsByName("SPlanningVelocityElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("GeneralSPlanStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("SPlanningVelocityElement3");
        obj[0].value = 2.2;
        obj[1].value = 0.3;
        obj[2].value = 80.0;
        obj[3].value = 50.0;
        obj[4].value = 20.0;
        obj[5].value = 3.0;
        obj[6].value = 144.0;
        obj[7].value = 5.0;
        obj[8].value = 10.0;
        obj[9].value = 1;
        obj[10].value = 0;
        obj[11].value = 0;
    }
    if (localStorage.getItem("GeneralPathPlanStr3") != null) {
        obj = document.getElementsByName("PathPlanElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("GeneralPathPlanStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("PathPlanElement3");
        obj[0].value = 0;
        obj[1].value = 0;
        obj[2].value = 0;
        obj[3].value = 0;
        obj[4].value = 0;
        obj[5].value = 0;
        obj[6].value = 0;
        obj[7].value = 0;
    }

    // PathplanStorage
    //========================================================================
    //Robot1
    if (localStorage.getItem("PathplanAtkStrategyStr1") != null) {
        obj = document.getElementsByName("AttackStrategyElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanAtkStrategyStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("AttackStrategyElement1");
        obj[0].value = 0;
    }
    if (localStorage.getItem("PathplanChaseStrategyStr1") != null) {
        obj = document.getElementsByName("ChaseStrategyElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanChaseStrategyStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("ChaseStrategyElement1");
        obj[0].value = 0;
    }
    if (localStorage.getItem("PathplanZoneAtkStr1") != null) {
        obj = document.getElementsByName("ZoneAttackElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanZoneAtkStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("ZoneAttackElement1");
        obj[0].value = 1.5;
        obj[1].value = 2.2;
    }
    if (localStorage.getItem("PathplanTypeSAtkStr1") != null) {
        obj = document.getElementsByName("TypeSAttackElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanTypeSAtkStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("TypeSAttackElement1");
        obj[0].value = 2.0;
        obj[1].value = 0.2;
    }
    if (localStorage.getItem("PathplanGoalkeeperStr1") != null) {
        obj = document.getElementsByName("PGoalkeeperElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanGoalkeeperStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("PGoalkeeperElement1");
        obj[0].value = 0;
        obj[1].value = 0;
        obj[2].value = 0;
        obj[3].value = 0;
    }
    if (localStorage.getItem("PathplanSideSpeedUpStr1") != null) {
        obj = document.getElementsByName("SideSpeedUpElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanSideSpeedUpStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("SideSpeedUpElement1");
        obj[0].value = 0.73;
        obj[1].value = 1.2;
        obj[2].value = 0.7;
        obj[3].value = 5.0;
        obj[4].value = 17.6;
    }
    if (localStorage.getItem("PathplanDorsadAttackStr1") != null) {
        obj = document.getElementsByName("DorsadAttackElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanDorsadAttackStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("DorsadAttackElement1");
        obj[0].value = 8;
        obj[1].value = 90;
        obj[2].value = 180;
        obj[3].value = 2;
        obj[4].value = 1;
        obj[5].value = 2;
    }
    if (localStorage.getItem("PathplanCornerKickStr1") != null) {
        obj = document.getElementsByName("CornerKickElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanCornerKickStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("CornerKickElement1");
        obj[0].value = 1.2;
        obj[1].value = 60;
        obj[2].value = 90;
        obj[3].value = 50;
    }

    if (localStorage.getItem("PathplanPenaltyKickStr1") != null) {
        obj = document.getElementsByName("PenaltyKickElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanPenaltyKickStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("PenaltyKickElement1");
        obj[0].value = 1.0;
        obj[1].checked = true;
        obj[2].checked = false;
        obj[3].checked = false;
    }

    //Robot 2
    if (localStorage.getItem("PathplanAtkStrategyStr2") != null) {
        obj = document.getElementsByName("AttackStrategyElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanAtkStrategyStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("AttackStrategyElement2");
        obj[0].value = 0;
    }
    if (localStorage.getItem("PathplanChaseStrategyStr2") != null) {
        obj = document.getElementsByName("ChaseStrategyElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanChaseStrategyStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("ChaseStrategyElement2");
        obj[0].value = 0;
    }
    if (localStorage.getItem("PathplanZoneAtkStr2") != null) {
        obj = document.getElementsByName("ZoneAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanZoneAtkStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("ZoneAttackElement2");
        obj[0].value = 1.5;
        obj[1].value = 2.2;
    }
    if (localStorage.getItem("PathplanTypeSAtkStr2") != null) {
        obj = document.getElementsByName("TypeSAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanTypeSAtkStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("TypeSAttackElement2");
        obj[0].value = 2.0;
        obj[1].value = 0.2;
    }
    if (localStorage.getItem("PathplanTypeUAtkStr2") != null) {
        obj = document.getElementsByName("TypeUAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanTypeUAtkStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("TypeUAttackElement2");
        obj[0].value = 0.8;
        obj[1].value = 1.0;
        obj[2].value = -90.0;
        obj[3].value = 90.0;
        obj[4].value = 0.6;
        obj[5].value = 0.3;
        obj[6].value = 10.0;
        obj[7].value = 20.0;
    }
    if (localStorage.getItem("PathplanSideSpeedUpStr2") != null) {
        obj = document.getElementsByName("SideSpeedUpElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanSideSpeedUpStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("SideSpeedUpElement2");
        obj[0].value = 0.73;
        obj[1].value = 1.2;
        obj[2].value = 0.7;
        obj[3].value = 5.0;
        obj[4].value = 17.6;
    }
    if (localStorage.getItem("PathplanDorsadAttackStr2") != null) {
        obj = document.getElementsByName("DorsadAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanDorsadAttackStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("DorsadAttackElement2");
        obj[0].value = 8;
        obj[1].value = 90;
        obj[2].value = 180;
        obj[3].value = 2;
        obj[4].value = 1;
        obj[5].value = 2;
    }
    if (localStorage.getItem("PathplanCornerKickStr2") != null) {
        obj = document.getElementsByName("CornerKickElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanCornerKickStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("CornerKickElement2");
        obj[0].value = 1.2;
        obj[1].value = 60;
        obj[2].value = 90;
        obj[3].value = 50;
    }

    if (localStorage.getItem("PathplanPenaltyKickStr2") != null) {
        obj = document.getElementsByName("PenaltyKickElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanPenaltyKickStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("PenaltyKickElement2");
        obj[0].value = 1.0;
        obj[1].checked = true;
        obj[2].checked = false;
        obj[3].checked = false;
    }

    // Robot 3
    if (localStorage.getItem("PathplanAtkStrategyStr3") != null) {
        obj = document.getElementsByName("AttackStrategyElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanAtkStrategyStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("AttackStrategyElement3");
        obj[0].value = 0;
    }
    if (localStorage.getItem("PathplanChaseStrategyStr3") != null) {
        obj = document.getElementsByName("ChaseStrategyElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanChaseStrategyStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("ChaseStrategyElement3");
        obj[0].value = 0;
    }
    if (localStorage.getItem("PathplanZoneAtkStr3") != null) {
        obj = document.getElementsByName("ZoneAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanZoneAtkStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("ZoneAttackElement3");
        obj[0].value = 1.5;
        obj[1].value = 2.2;
    }
    if (localStorage.getItem("PathplanTypeSAtkStr3") != null) {
        obj = document.getElementsByName("TypeSAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanTypeSAtkStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("TypeSAttackElement3");
        obj[0].value = 2.0;
        obj[1].value = 0.2;
    }
    if (localStorage.getItem("PathplanTypeUAtkStr3") != null) {
        obj = document.getElementsByName("TypeUAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanTypeUAtkStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("TypeUAttackElement3");
        obj[0].value = 0.8;
        obj[1].value = 1.0;
        obj[2].value = -90.0;
        obj[3].value = 90.0;
        obj[4].value = 0.6;
        obj[5].value = 0.3;
        obj[6].value = 10.0;
        obj[7].value = 20.0;
    }
    if (localStorage.getItem("PathplanSideSpeedUpStr3") != null) {
        obj = document.getElementsByName("SideSpeedUpElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanSideSpeedUpStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("SideSpeedUpElement3");
        obj[0].value = 0.73;
        obj[1].value = 1.2;
        obj[2].value = 0.7;
        obj[3].value = 5.0;
        obj[4].value = 17.6;
    }
    if (localStorage.getItem("PathplanDorsadAttackStr3") != null) {
        obj = document.getElementsByName("DorsadAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanDorsadAttackStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("DorsadAttackElement3");
        obj[0].value = 8;
        obj[1].value = 90;
        obj[2].value = 180;
        obj[3].value = 2;
        obj[4].value = 1;
        obj[5].value = 2;
    }
    if (localStorage.getItem("PathplanCornerKickStr3") != null) {
        obj = document.getElementsByName("CornerKickElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanCornerKickStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("CornerKickElement3");
        obj[0].value = 1.2;
        obj[1].value = 60;
        obj[2].value = 90;
        obj[3].value = 50;
    }

    if (localStorage.getItem("PathplanPenaltyKickStr3") != null) {
        obj = document.getElementsByName("PenaltyKickElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanPenaltyKickStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("PenaltyKickElement3");
        obj[0].value = 1.0;
        obj[1].checked = true;
        obj[2].checked = false;
        obj[3].checked = false;
    }

    // BehaviorStorage
    //========================================================================
    //Robot 1
    if (localStorage.getItem("BehaviorStateChaseStr1") != null) {
        obj = document.getElementsByName("StateChaseElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateChaseStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("StateChaseElement1");
        obj[0].value = 5.0;
        obj[1].value = 0.15;
        obj[2].value = 0.2;
        obj[3].value = 16.5;
        obj[4].value = 0.27;
    }
    if (localStorage.getItem("BehaviorStateAtkStr1") != null) {
        obj = document.getElementsByName("StateAttackElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateAtkStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("StateAttackElement1");
        obj[0].value = 30.0;
        obj[1].value = 1.0;
        obj[2].value = 1.0;
    }
    if (localStorage.getItem("BehaviorStateGoalkeeperStr1") != null) {
        obj = document.getElementsByName("GoalkeeperElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateGoalkeeperStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("GoalkeeperElement1");
        obj[0].value = 0;
        obj[1].value = 0;
        obj[2].value = 0;
        obj[3].value = 0;
        obj[4].value = 0;
    }
    if (localStorage.getItem("BehaviorStateTypeSAtkStr1") != null) {
        obj = document.getElementsByName("StateTypeSAttackElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateTypeSAtkStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("StateTypeSAttackElement1");
        obj[0].value = 20.0;
        obj[1].value = 1.0;
        obj[2].value = 1.5;
    }
    if (localStorage.getItem("BehaviorStateSideSpeedUPStr1") != null) {
        obj = document.getElementsByName("StateSideSpeedUPElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateSideSpeedUPStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("StateSideSpeedUPElement1");
        obj[0].value = 10.0;
        obj[1].value = 0.45;
    }
    if (localStorage.getItem("BehaviorStateZoneAtkStr1") != null) {
        obj = document.getElementsByName("StateZoneAttackElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateZoneAtkStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("StateZoneAttackElement1");
        obj[0].value = 0.6;
    }
    if (localStorage.getItem("BehaviorStateCornerKickStr1") != null) {
        obj = document.getElementsByName("StateCornerKickElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateCornerKickStr1"))[i];
        }
    } else {
        obj = document.getElementsByName("StateCornerKickElement1");
        obj[0].value = 10.0;
        obj[1].value = 0.45;
    }
    if (localStorage.getItem("BehaviorStrategySelectionStr1") != null) {
        obj = document.getElementsByName("StrategySelectionElement1");
        for (var i = 0; i < obj.length; i++) {
            if (JSON.parse(localStorage.getItem("BehaviorStrategySelectionStr1"))[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
    } else {
        obj = document.getElementsByName("StrategySelectionElement1");
        obj[0].checked = true;
        obj[1].checked = false;
        obj[2].checked = true;
        obj[3].checked = false;
        obj[4].checked = false;
        obj[5].checked = false;
        obj[6].checked = false;
        obj[7].checked = false;
    }
    if (localStorage.getItem("BehaviorStrategySelectionPrefixStr1") != null) {
        obj = document.getElementsByName("StrategySelectionPrefixElement1");
        for (var i = 0; i < 4; i++) {
            if (JSON.parse(localStorage.getItem("BehaviorStrategySelectionPrefixStr1"))[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
        obj[4].value = JSON.parse(localStorage.getItem("BehaviorStrategySelectionPrefixStr1"))[4];
    } else {
        obj = document.getElementsByName("StrategySelectionPrefixElement1");
        obj[0].checked = true;
        obj[1].checked = false;
        obj[2].checked = true;
        obj[3].checked = false;
        obj[4].checked = 0;
    }
    if (localStorage.getItem("BehaviorSupportStrategyStr1") != null) {
        obj = document.getElementsByName("SupportStrategyElement1");
        obj[parseInt(JSON.parse(localStorage.getItem("BehaviorSupportStrategyStr1"))[0]) - 1].checked = true;
    } else {
        obj = document.getElementsByName("SupportStrategyElement1");
        obj[0].checked = true;
    }

    //Robot 2
    if (localStorage.getItem("BehaviorStateChaseStr2") != null) {
        obj = document.getElementsByName("StateChaseElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateChaseStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("StateChaseElement2");
        obj[0].value = 5.0;
        obj[1].value = 0.15;
        obj[2].value = 0.2;
        obj[3].value = 16.5;
        obj[4].value = 0.27;
    }
    if (localStorage.getItem("BehaviorStateAtkStr2") != null) {
        obj = document.getElementsByName("StateAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateAtkStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("StateAttackElement2");
        obj[0].value = 30.0;
        obj[1].value = 1.0;
        obj[2].value = 1.0;
    }
    if (localStorage.getItem("BehaviorStateTypeUChaseStr2") != null) {
        obj = document.getElementsByName("StateTypeUChaseElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateTypeUChaseStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("StateTypeUChaseElement2");
        obj[0].value = 30.0;
        obj[1].value = 0.3;
    }
    if (localStorage.getItem("BehaviorStateTypeSAtkStr2") != null) {
        obj = document.getElementsByName("StateTypeSAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateTypeSAtkStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("StateTypeSAttackElement2");
        obj[0].value = 20.0;
        obj[1].value = 1.0;
        obj[2].value = 1.5;
    }
    if (localStorage.getItem("BehaviorStateSideSpeedUPStr2") != null) {
        obj = document.getElementsByName("StateSideSpeedUPElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateSideSpeedUPStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("StateSideSpeedUPElement2");
        obj[0].value = 10.0;
        obj[1].value = 0.45;
    }
    if (localStorage.getItem("BehaviorStateZoneAtkStr2") != null) {
        obj = document.getElementsByName("StateZoneAttackElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateZoneAtkStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("StateZoneAttackElement2");
        obj[0].value = 0.6;
    }
    if (localStorage.getItem("BehaviorStateCornerKickStr2") != null) {
        obj = document.getElementsByName("StateCornerKickElement2");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateCornerKickStr2"))[i];
        }
    } else {
        obj = document.getElementsByName("StateCornerKickElement2");
        obj[0].value = 10.0;
        obj[1].value = 0.45;
    }
    if (localStorage.getItem("BehaviorStrategySelectionStr2") != null) {
        obj = document.getElementsByName("StrategySelectionElement2");
        for (var i = 0; i < obj.length; i++) {
            if (JSON.parse(localStorage.getItem("BehaviorStrategySelectionStr2"))[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
    } else {
        obj = document.getElementsByName("StrategySelectionElement2");
        obj[0].checked = true;
        obj[1].checked = false;
        obj[2].checked = true;
        obj[3].checked = false;
        obj[4].checked = false;
        obj[5].checked = false;
        obj[6].checked = false;
        obj[7].checked = false;
    }
    if (localStorage.getItem("BehaviorStrategySelectionPrefixStr2") != null) {
        obj = document.getElementsByName("StrategySelectionPrefixElement2");
        for (var i = 0; i < 4; i++) {
            if (JSON.parse(localStorage.getItem("BehaviorStrategySelectionPrefixStr2"))[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
        obj[4].value = JSON.parse(localStorage.getItem("BehaviorStrategySelectionPrefixStr2"))[4];
    } else {
        obj = document.getElementsByName("StrategySelectionPrefixElement2");
        obj[0].checked = true;
        obj[1].checked = false;
        obj[2].checked = true;
        obj[3].checked = false;
        obj[4].checked = 0;
    }
    if (localStorage.getItem("BehaviorSupportStrategyStr2") != null) {
        obj = document.getElementsByName("SupportStrategyElement2");
        obj[parseInt(JSON.parse(localStorage.getItem("BehaviorSupportStrategyStr2"))[0]) - 1].checked = true;
    } else {
        obj = document.getElementsByName("SupportStrategyElement2");
        obj[0].checked = true;
    }

    //Robot 3

    if (localStorage.getItem("BehaviorStateChaseStr3") != null) {
        obj = document.getElementsByName("StateChaseElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateChaseStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("StateChaseElement3");
        obj[0].value = 5.0;
        obj[1].value = 0.15;
        obj[2].value = 0.2;
        obj[3].value = 16.5;
        obj[4].value = 0.27;
    }
    if (localStorage.getItem("BehaviorStateAtkStr3") != null) {
        obj = document.getElementsByName("StateAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateAtkStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("StateAttackElement3");
        obj[0].value = 30.0;
        obj[1].value = 1.0;
        obj[2].value = 1.0;
    }
    if (localStorage.getItem("BehaviorStateTypeUChaseStr3") != null) {
        obj = document.getElementsByName("StateTypeUChaseElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateTypeUChaseStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("StateTypeUChaseElement3");
        obj[0].value = 30.0;
        obj[1].value = 0.3;
    }
    if (localStorage.getItem("BehaviorStateTypeSAtkStr3") != null) {
        obj = document.getElementsByName("StateTypeSAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateTypeSAtkStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("StateTypeSAttackElement3");
        obj[0].value = 20.0;
        obj[1].value = 1.0;
        obj[2].value = 1.5;
    }
    if (localStorage.getItem("BehaviorStateSideSpeedUPStr3") != null) {
        obj = document.getElementsByName("StateSideSpeedUPElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateSideSpeedUPStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("StateSideSpeedUPElement3");
        obj[0].value = 10.0;
        obj[1].value = 0.45;
    }
    if (localStorage.getItem("BehaviorStateZoneAtkStr3") != null) {
        obj = document.getElementsByName("StateZoneAttackElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateZoneAtkStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("StateZoneAttackElement3");
        obj[0].value = 0.6;
    }
    if (localStorage.getItem("BehaviorStateCornerKickStr3") != null) {
        obj = document.getElementsByName("StateCornerKickElement3");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateCornerKickStr3"))[i];
        }
    } else {
        obj = document.getElementsByName("StateCornerKickElement3");
        obj[0].value = 10.0;
        obj[1].value = 0.45;
    }
    if (localStorage.getItem("BehaviorStrategySelectionStr3") != null) {
        obj = document.getElementsByName("StrategySelectionElement3");
        for (var i = 0; i < obj.length; i++) {
            if (JSON.parse(localStorage.getItem("BehaviorStrategySelectionStr3"))[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
    } else {
        obj = document.getElementsByName("StrategySelectionElement3");
        obj[0].checked = true;
        obj[1].checked = false;
        obj[2].checked = true;
        obj[3].checked = false;
        obj[4].checked = false;
        obj[5].checked = false;
        obj[6].checked = false;
        obj[7].checked = false;
    }
    if (localStorage.getItem("BehaviorStrategySelectionPrefixStr3") != null) {
        obj = document.getElementsByName("StrategySelectionPrefixElement3");
        for (var i = 0; i < 4; i++) {
            if (JSON.parse(localStorage.getItem("BehaviorStrategySelectionPrefixStr3"))[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
        obj[4].value = JSON.parse(localStorage.getItem("BehaviorStrategySelectionPrefixStr3"))[4];
    } else {
        obj = document.getElementsByName("StrategySelectionPrefixElement3");
        obj[0].checked = true;
        obj[1].checked = false;
        obj[2].checked = true;
        obj[3].checked = false;
        obj[4].checked = 0;
    }
    if (localStorage.getItem("BehaviorSupportStrategyStr3") != null) {
        obj = document.getElementsByName("SupportStrategyElement3");
        obj[parseInt(JSON.parse(localStorage.getItem("BehaviorSupportStrategyStr3"))[0]) - 1].checked = true;
    } else {
        obj = document.getElementsByName("SupportStrategyElement3");
        obj[0].checked = true;
    }
    //===========================================================================================================
    //RobotShow
    if (localStorage.getItem("RobotShow1") != null) {
        obj = document.getElementsByName("ChooseShowElement1");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("RobotShow1"))[i];
        }
    } else {
        obj = document.getElementsByName("ChooseShowElement1");
        obj[0].value = 0.1;
        obj[1].value = 1.8;
        obj[2].value = 2.3;
        obj[3].value = 0.5;
        obj[4].value = 3.5;
        obj[5].value = 1.8;
        obj[6].value = 0.5;
        obj[7].value = 1.5;
    }
}