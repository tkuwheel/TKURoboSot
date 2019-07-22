function SendMsgs(Msg,color) 
{ 
  let  d = new Date();
  let n = d.getHours();
  let m = d.getMinutes();
  let s = d.getSeconds();

  //var MSg= $("msg").value; 
  var newFontElem = document.createElement("font");
  
  if(color == "red"){
    newFontElem.style.color = color;
  }
  if(color == "blue"){
    newFontElem.style.color = color;
  }
  newFontElem.innerHTML = Msg+"<br/>"; 
  document.getElementById("info").appendChild(newFontElem); 

  document.getElementById("info").scrollTop= document.getElementById("info").scrollHeight;//每次收到消息滾動條拉到最下面
} 
