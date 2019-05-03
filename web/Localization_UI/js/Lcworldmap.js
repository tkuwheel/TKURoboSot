
function DrawRectangle(number){
	var canva = document.getElementById("Rect"+number);
	var ctx = canva.getContext("2d");
	 ClearRectangle();

	ctx.strokeStyle = "rgb(0,255,255)";  // 把「填滿樣式」設為紅 200 綠 0 藍 0rgb(255, 26, 198)
	ctx.lineWidth = 23;
	ctx.strokeRect(0,0,60,60);   // 畫一個填充的長方形
}
function DrawOrderRectangle() {
    var i;
    ClearRectangle(1);
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
