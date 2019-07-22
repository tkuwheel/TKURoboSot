
function DrawRectangle(number){
	var canva = document.getElementById("Rect"+number);
	var ctx = canva.getContext("2d");
	 ClearRectangle();

	ctx.strokeStyle = "rgb(0,255,255)";  // 把「填滿樣式」設為紅 200 綠 0 藍 0rgb(255, 26, 198)
	ctx.lineWidth = 23;
	ctx.strokeRect(0,0,60,60);   // 畫一個填充的長方形
    let canvas = document.getElementById('path_map');
    ctx=canvas.getContext("2d");
    ctx.clearRect(0,0,canvas.width,canvas.height); 
}
function DrawOrderRectangle() {
    var i;
    ClearRectangle();
    for (i = 0; i < 5; i++) {
        var canva = document.getElementById("Rect" + Order[i]);
        var ctx = canva.getContext("2d");
        ctx.strokeStyle = "rgb(0,255,255)"; // 把「填滿樣式」設為紅 200 綠 0 藍 0rgb(255, 26, 198)
        ctx.lineWidth = 13;
        ctx.strokeRect(0, 0, 60, 60); // 畫一個填充的長方形

        ctx.font = "50px Times New Roman";
        ctx.fillStyle = "rgb(255,255,255)";
        ctx.lineWidth = 3;
        ctx.fillText(i + 1, 15, 45);
    }
}
function ClearRectangle(){
	
	var canva;
	var ctx;
    var obj = document.getElementsByName("LocalElement1");
	for (var i=0; i<obj.length; i++) {
        if (obj[i].checked == false) {
        	canva = document.getElementById("Rect"+document.getElementsByName("LocalElement1")[i].value);
        	ctx = canva.getContext("2d");
       		ctx.clearRect(0,0,60,60);
       	}
    }
    obj = document.getElementsByName("LocalElement2");
    for (var i=0; i<obj.length; i++) {
        if (obj[i].checked == false) {
        	canva = document.getElementById("Rect"+document.getElementsByName("LocalElement2")[i].value);
        	ctx = canva.getContext("2d");
       		ctx.clearRect(0,0,60,60);
       	}
    }
    obj = document.getElementsByName("LocalElement3");
    for (var i=0; i<obj.length; i++) {
        if (obj[i].checked == false) {
        	canva = document.getElementById("Rect"+document.getElementsByName("LocalElement3")[i].value);
        	ctx = canva.getContext("2d");
       		ctx.clearRect(0,0,60,60);
       	}
    }
    obj = document.getElementsByName("LocalElement4");
    for (var i=0; i<obj.length; i++) {
        if (obj[i].checked == false) {
        	canva = document.getElementById("Rect"+document.getElementsByName("LocalElement4")[i].value);
        	ctx = canva.getContext("2d");
       		ctx.clearRect(0,0,60,60);
       	}
    }
    obj = document.getElementsByName("LocalElement5");
    for (var i=0; i<obj.length; i++) {
        if (obj[i].checked == false) {
        	canva = document.getElementById("Rect"+document.getElementsByName("LocalElement5")[i].value);
        	ctx = canva.getContext("2d");
       		ctx.clearRect(0,0,60,60);
       	}
    }
}
function ClearAllRectangle(){
  let Rect = document.getElementById("Rect1");
  let canvasMap = document.getElementById("canvasMap");
  let map_width = canvasMap.width-Rect.width;
  let map_height = canvasMap.height-Rect.height;
	let ground_reverse = document.getElementById("GroundButton").checked;
  if(ground_reverse==true){
	    for (var i=0; i<points.length; i++) {
          //str=document.getElementById("Rect"+document.getElementsByName("LocalElement1")[i].value).style.left;
          //console.log(str);
          document.getElementById("Rect"+parseInt(i+1)).style.left=parseInt(map_width-points[i][0])+"px";
          document.getElementById("Rect"+parseInt(i+1)).style.top=parseInt(map_height-points[i][1])+"px";
        	canva = document.getElementById("Rect"+parseInt(i+1));
        	ctx = canva.getContext("2d");
       		ctx.clearRect(0,0,60,60);
        }
        var obj = document.getElementsByName("LocalElement1");
        for (var i=0; i<obj.length; i++) {
            if (obj[i].checked == true) {
                DrawRectangle(document.getElementsByName("LocalElement1")[i].value);
           	}
        }
        obj = document.getElementsByName("LocalElement2");
        for (var i=0; i<obj.length; i++) {
            if (obj[i].checked == true) {
                DrawRectangle(document.getElementsByName("LocalElement2")[i].value);
           	}
        }
        obj = document.getElementsByName("LocalElement3");
        for (var i=0; i<obj.length; i++) {
            if (obj[i].checked == true) {
                DrawRectangle(document.getElementsByName("LocalElement3")[i].value);
           	}
        }
        obj = document.getElementsByName("LocalElement4");
        for (var i=0; i<obj.length; i++) {
            if (obj[i].checked == true) {
                DrawRectangle(document.getElementsByName("LocalElement4")[i].value);
           	}
        }
        obj = document.getElementsByName("LocalElement5");
        for (var i=0; i<obj.length; i++) {
            if (obj[i].checked == true) {
                DrawRectangle(document.getElementsByName("LocalElement5")[i].value);
           	}
        }

    }else{
        
        for (var i=0; i<points.length; i++) {
          //str=document.getElementById("Rect"+document.getElementsByName("LocalElement1")[i].value).style.left;
          //console.log(str);
          document.getElementById("Rect"+parseInt(i+1)).style.left=parseInt(points[i][0])+"px";
          document.getElementById("Rect"+parseInt(i+1)).style.top=parseInt(points[i][1])+"px";
        	canva = document.getElementById("Rect"+parseInt(i+1));
        	ctx = canva.getContext("2d");
       		ctx.clearRect(0,0,60,60);
        }
        
        var obj = document.getElementsByName("LocalElement1");
        for (var i=0; i<obj.length; i++) {
            if (obj[i].checked == true) {
                DrawRectangle(document.getElementsByName("LocalElement1")[i].value);
           	}
        }
        obj = document.getElementsByName("LocalElement2");
        for (var i=0; i<obj.length; i++) {
            if (obj[i].checked == true) {
                DrawRectangle(document.getElementsByName("LocalElement2")[i].value);
           	}
        }
        obj = document.getElementsByName("LocalElement3");
        for (var i=0; i<obj.length; i++) {
            if (obj[i].checked == true) {
                DrawRectangle(document.getElementsByName("LocalElement3")[i].value);
           	}
        }
        obj = document.getElementsByName("LocalElement4");
        for (var i=0; i<obj.length; i++) {
            if (obj[i].checked == true) {
                DrawRectangle(document.getElementsByName("LocalElement4")[i].value);
           	}
        }
        obj = document.getElementsByName("LocalElement5");
        for (var i=0; i<obj.length; i++) {
            if (obj[i].checked == true) {
                DrawRectangle(document.getElementsByName("LocalElement5")[i].value);
           	}
        }
        
    }
}
