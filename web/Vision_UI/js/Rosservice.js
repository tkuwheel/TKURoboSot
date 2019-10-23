var paramClient = new ROSLIB.Service({
  ros : ros,
  name : 'interface/save_srv',
  serviceType : 'std_srvs/Empty'
});

var param_request = new ROSLIB.ServiceRequest({
});
//================================
var vison_better_Client = new ROSLIB.Service({
  ros : ros,
  name : 'StrategyParam',
  serviceType : 'param_convey/strategy_param'
});

var vision_better_request = new ROSLIB.ServiceRequest({
    receive: 1
});
function savecall(){
  paramClient.callService(param_request,
    function(param_request) {
      console.log('成功儲存參數');
      SendMsgs('成功儲存參數',"blue");
      //callback(result.action_servers);
    },
    function(message){
      console.log(message);
      SendMsgs('儲存參數失敗',"red");
    }
  );
  //==============
  //vison_better_Client.callService(vision_better_request, 
  //  function(vision_better_request) {
  //    if (vision_better_request.update == 2) {
  //      console.log('Parameter is saved');
  //    }
  //  },
  //  function(message){
  //    console.log("SAVE FAILED");
  //    //SendMsgs('儲存參數失敗',"red");
  //  }
  //);
}
//================================
var connectClient = new ROSLIB.Service({
  ros : ros,
  name : 'monitor/connect',
  serviceType : 'std_srvs/Empty'
});

var connect_request = new ROSLIB.ServiceRequest({
});

let connected = false;

function service_connect(){
  
  setTimeout(service_connect, 3000);
  let monitor_checked = document.getElementById("MonitorButton").checked;
  if(monitor_checked){
    connectClient.callService(connect_request,
      function(connect_request) {
        if(connected == false){
          console.log('Monitor連接成功');
          SendMsgs('Monitor連接成功',"blue");
          connected = true;
        }
        //callback(result.action_servers);
      },
      function(message){
        if(connected == true){
          console.log('Monitor連接中斷');
          SendMsgs('Monitor連接中斷',"red");
          //console.log(message);
          connected = false;
        }
        else{
          console.log('無法連接Monitor');
          SendMsgs('無法連接Monitor',"red");
        }
      }
    )
  }
}

