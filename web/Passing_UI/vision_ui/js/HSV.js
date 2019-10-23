    sliderBarHue(0, "min");
    sliderBarHue(359, "Max");
    sliderBarSat(0, "min");
    sliderBarSat(100, "Max");
    sliderBarVal(0, "min");
    sliderBarVal(100, "Max");

    $("#webcamImage").draggable();

    function Reset(){
        sliderBarHue(0, "min");
        sliderBarHue(359, "Max");
        sliderBarSat(0, "min");
        sliderBarSat(100, "Max");
        sliderBarVal(0, "min");
        sliderBarVal(100, "Max");
        topicColorTransfer(this.value);
    }

    function RangeHSVColor(h_min, h_max, s_min, s_max, v_min, v_max) {
        var canvasHS = document.getElementById("HSVcircle");
        var ctxHS = canvasHS.getContext("2d");
        var imageHS = ctxHS.createImageData(360, 100);
        var canvasSV = document.getElementById("HSVrect");
        var ctxSV = canvasSV.getContext("2d");
        var imageSV = ctxSV.createImageData(100, 100);
        var rgbColor;
        if ((h_min <= h_max) && (s_min <= s_max)) {
            for (var y = s_min; y <= s_max - 1; y++) {
                for (var x = h_min; x <= h_max - 1; x++) {
                    rgbColor = hsv2rgb(x, y, v_max);
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 0] = rgbColor[0];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 1] = rgbColor[1];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 2] = rgbColor[2];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 3] = 255;
                }
            }
        } else if ((h_min > h_max) && (s_min <= s_max)) {
            for (var y = s_min; y <= s_max - 1; y++) {
                for (var x = h_min; x <= 360 - 1; x++) {
                    rgbColor = hsv2rgb(x, y, v_max);
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 0] = rgbColor[0];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 1] = rgbColor[1];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 2] = rgbColor[2];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 3] = 255;
                }
                for (var x = 0; x <= h_max - 1; x++) {
                    rgbColor = hsv2rgb(x, y, v_max);
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 0] = rgbColor[0];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 1] = rgbColor[1];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 2] = rgbColor[2];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 3] = 255;
                }
            }
        } else if ((h_min < h_max) && (s_min > s_max)) {
            for (var x = h_min; x <= h_max - 1; x++) {
                for (var y = s_min; y <= 100 - 1; y++) {
                    rgbColor = hsv2rgb(x, y, v_max);
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 0] = rgbColor[0];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 1] = rgbColor[1];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 2] = rgbColor[2];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 3] = 255;
                }
                for (var y = 0; y <= s_max - 1; y++) {
                    rgbColor = hsv2rgb(x, y, v_max);
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 0] = rgbColor[0];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 1] = rgbColor[1];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 2] = rgbColor[2];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 3] = 255;
                }
            }
        } else {
            for (var x = h_min; x <= 360 - 1; x++) {
                for (var y = 0; y <= s_max - 1; y++) {
                    rgbColor = hsv2rgb(x, y, v_max);
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 0] = rgbColor[0];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 1] = rgbColor[1];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 2] = rgbColor[2];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 3] = 255;
                }
                for (var y = s_min; y <= 360 - 1; y++) {
                    rgbColor = hsv2rgb(x, y, v_max);
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 0] = rgbColor[0];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 1] = rgbColor[1];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 2] = rgbColor[2];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 3] = 255;
                }
            }
            for (var x = 0; x <= h_max - 1; x++) {
                for (var y = 0; y <= s_max - 1; y++) {
                    rgbColor = hsv2rgb(x, y, v_max);
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 0] = rgbColor[0];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 1] = rgbColor[1];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 2] = rgbColor[2];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 3] = 255;
                }
                for (var y = s_min; y <= 360 - 1; y++) {
                    rgbColor = hsv2rgb(x, y, v_max);
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 0] = rgbColor[0];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 1] = rgbColor[1];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 2] = rgbColor[2];
                    imageHS.data[(x * 4) + ((imageHS.height - y) * imageHS.width * 4) + 3] = 255;
                }
            }
        }
        ctxHS.putImageData(imageHS, 0, 0);
        if ((s_min < s_max) && (v_min < v_max)) {
            for (var y = v_min; y <= v_max - 1; y++) {
                for (var x = s_min; x <= s_max - 1; x++) {
                    rgbColor = hsv2rgb(h_max, x, y);
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 0] = rgbColor[0];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 1] = rgbColor[1];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 2] = rgbColor[2];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 3] = 255;
                }
            }
        }else if ((s_min >= s_max) && (v_min < v_max)) {
            for (var y = v_min; y <= v_max - 1; y++) {
                for (var x = s_min; x <= 100 - 1; x++) {
                    rgbColor = hsv2rgb(h_max, x, y);
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 0] = rgbColor[0];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 1] = rgbColor[1];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 2] = rgbColor[2];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 3] = 255;
                }
                for (var x = 0; x <= s_max - 1; x++) {
                    rgbColor = hsv2rgb(h_max, x, y);
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 0] = rgbColor[0];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 1] = rgbColor[1];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 2] = rgbColor[2];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 3] = 255;
                }
            }
        } else if ((s_min < s_max) && (v_min >= v_max)) {
            for (var x = s_min; x <= s_max - 1; x++) {
                for (var y = v_min; y <= 100 - 1; y++) {
                    rgbColor = hsv2rgb(h_max, x, y);
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 0] = rgbColor[0];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 1] = rgbColor[1];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 2] = rgbColor[2];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 3] = 255;
                }
                for (var y = 0; y <= v_max - 1; y++) {
                    rgbColor = hsv2rgb(h_max, x, y);
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 0] = rgbColor[0];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 1] = rgbColor[1];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 2] = rgbColor[2];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 3] = 255;
                }
            }
        } else {
            for (var x = s_min; x <= 100 - 1; x++) {
                for (var y = 0; y <= v_max - 1; y++) {
                    rgbColor = hsv2rgb(h_max, x, y);
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 0] = rgbColor[0];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 1] = rgbColor[1];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 2] = rgbColor[2];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 3] = 255;
                }
                for (var y = v_min; y <= 100 - 1; y++) {
                    rgbColor = hsv2rgb(h_max, x, y);
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 0] = rgbColor[0];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 1] = rgbColor[1];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 2] = rgbColor[2];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 3] = 255;
                }
            }
            for (var x = 0; x <= s_max - 1; x++) {
                for (var y = 0; y <= v_max - 1; y++) {
                    rgbColor = hsv2rgb(h_max, x, y);
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 0] = rgbColor[0];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 1] = rgbColor[1];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 2] = rgbColor[2];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 3] = 255;
                }
                for (var y = v_min; y <= 100 - 1; y++) {
                    rgbColor = hsv2rgb(h_max, x, y);
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 0] = rgbColor[0];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 1] = rgbColor[1];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 2] = rgbColor[2];
                    imageSV.data[(x * 4) + ((imageSV.height - y) * imageSV.width * 4) + 3] = 255;
                }
            }
        }
        ctxSV.putImageData(imageSV, 0, 0);
    }
    //Listen sliderBar and show
    sliderBarHue(document.getElementById('slider_Hue_Bar_min').value, "min");
    sliderBarHue(document.getElementById('slider_Hue_Bar_Max').value, "Max");
    sliderBarSat(document.getElementById('slider_Sat_Bar_min').value, "min");
    sliderBarSat(document.getElementById('slider_Sat_Bar_Max').value, "Max");
    sliderBarVal(document.getElementById('slider_Val_Bar_min').value, "min");
    sliderBarVal(document.getElementById('slider_Val_Bar_Max').value, "Max");

    function sliderBarHue(newValue, string) {
        if (string == "min") {
            document.getElementById("inputHue_min").placeholder = newValue;
            document.getElementById('inputHue_min').value = newValue;
        } else if (string == "Max") {
            document.getElementById("inputHue_Max").placeholder = newValue;
            document.getElementById('inputHue_Max').value = newValue;
        }
        showHsvColor();
    }
    function inputHue(newValue, string) {
        if (newValue < 0)
            newValue = 0;
        else if (newValue > 360)
            newValue = 360;
        if (string == "min") {
            document.getElementById('slider_Hue_Bar_min').value = newValue;
            sliderBarHue(newValue,"min")
        } else if (string == "Max") {
            document.getElementById('slider_Hue_Bar_Max').value = newValue;
            sliderBarHue(newValue,"Max")
        }
        showHsvColor();
    }

    function sliderBarSat(newValue, string) {
        if (string == "min") {
            document.getElementById("inputSat_min").placeholder = newValue;
            document.getElementById('inputSat_min').value = newValue;
        } else if (string == "Max") {
            document.getElementById("inputSat_Max").placeholder = newValue;
            document.getElementById('inputSat_Max').value = newValue;
        }
        showHsvColor();
    }
    function inputSat(newValue, string) {
        if (newValue < 0)
            newValue = 0;
        else if (newValue > 100)
            newValue = 100;
        if (string == "min") {
            document.getElementById('slider_Sat_Bar_min').value = newValue;
            sliderBarSat(newValue,"min")
        } else if (string == "Max") {
            document.getElementById('slider_Sat_Bar_Max').value = newValue;
            sliderBarSat(newValue,"Max")
        }
        showHsvColor();
    }

    function sliderBarVal(newValue, string) {
        if (string == "min") {
            document.getElementById("inputVal_min").placeholder = newValue;
            document.getElementById('inputVal_min').value = newValue;
        } else if (string == "Max") {
            document.getElementById("inputVal_Max").placeholder = newValue;
            document.getElementById('inputVal_Max').value = newValue;
        }
        showHsvColor();
    }
    function inputVal(newValue, string) {
        if (newValue < 0)
            newValue = 0;
        else if (newValue > 100)
            newValue = 100;
        if (string == "min") {
            document.getElementById('slider_Val_Bar_min').value = newValue;
            sliderBarVal(newValue,"min")
        } else if (string == "Max") {
            document.getElementById('slider_Val_Bar_Max').value = newValue;
            sliderBarVal(newValue,"Max")
        }
        showHsvColor();
    }
    //Read input's value and show HSV color
    var Hmax, Hmin, Smax, Smin, Vmax, Vmin;

    function showHsvColor() {
        var hsvValue = readTableHSV();
        var rgbValue = hsv2rgb(hsvValue[1], hsvValue[3], hsvValue[5]);
        var hexValue = '#' + rgb2hex(rgbValue[0], rgbValue[1], rgbValue[2]);
        document.getElementById('colorBox').style.backgroundColor = hexValue;
        /*if (hsvValue[0] > hsvValue[1]) {
            Hmax = hsvValue[0];
            Hmin = hsvValue[1];
        }
        if (hsvValue[0] <= hsvValue[1]) {
            Hmax = hsvValue[1];
            Hmin = hsvValue[0];
        }
        if (hsvValue[2] > hsvValue[3]) {
            Smax = hsvValue[2];
            Smin = hsvValue[3];
        }
        if (hsvValue[2] <= hsvValue[3]) {
            Smax = hsvValue[3];
            Smin = hsvValue[2];
        }
        if (hsvValue[4] >= hsvValue[5]) {
            Vmax = hsvValue[4];
            Vmin = hsvValue[5];
        } else {
            Vmax = hsvValue[5];
            Vmin = hsvValue[4];
        }*/
        //RangeHSVColor(Hmin, Hmax, Smin, Smax, Vmin, Vmax);
        RangeHSVColor(parseInt(hsvValue[0]), parseInt(hsvValue[1]), parseInt(hsvValue[2]),
            parseInt(hsvValue[3]), parseInt(hsvValue[4]), parseInt(hsvValue[5]));
        return 1;
    }

    function readTableHSV() {
        var hsvArr = new Array(6);
        hsvArr[0] = document.getElementById('inputHue_min').value;
        hsvArr[1] = document.getElementById('inputHue_Max').value;
        hsvArr[2] = document.getElementById('inputSat_min').value;
        hsvArr[3] = document.getElementById('inputSat_Max').value;
        hsvArr[4] = document.getElementById('inputVal_min').value;
        hsvArr[5] = document.getElementById('inputVal_Max').value;
        document.getElementById('slider_Hue_Bar_min').value = hsvArr[0];
        document.getElementById('slider_Hue_Bar_Max').value = hsvArr[1];
        document.getElementById('slider_Sat_Bar_min').value = hsvArr[2];
        document.getElementById('slider_Sat_Bar_Max').value = hsvArr[3];
        document.getElementById('slider_Val_Bar_min').value = hsvArr[4];
        document.getElementById('slider_Val_Bar_Max').value = hsvArr[5];
	hsvArr[1] = hsvArr[1]/1.002;
        return hsvArr;
    }
    //Convert HSV to RGB
    function hsv2rgb(hue, saturation, value) {
        var RR, GG, BB;
        var rgbArr = new Array(3);
        saturation = saturation / 100;
        value = value / 100;

        var C = value * saturation;
        var h1 = hue / 60;
        var X = C * (1 - Math.abs((h1 % 2) - 1));
        var m = value - C;
        switch (parseInt(h1)) {
            case 0:
                RR = C;
                GG = X;
                BB = 0;
                break;
            case 1:
                RR = X;
                GG = C;
                BB = 0;
                break;
            case 2:
                RR = 0;
                GG = C;
                BB = X;
                break;
            case 3:
                RR = 0;
                GG = X;
                BB = C;
                break;
            case 4:
                RR = X;
                GG = 0;
                BB = C;
                break;
            case 5:
                RR = C;
                GG = 0;
                BB = X;
                break;
        }
        rgbArr[0] = (RR + m) * 255;
        rgbArr[1] = (GG + m) * 255;
        rgbArr[2] = (BB + m) * 255;
        return rgbArr;
    }
    //Convert RGB to Hex color
    function rgb2hex(R, G, B) {
        return toHex(R) + toHex(G) + toHex(B)
    }

    function toHex(n) {
        n = parseInt(n, 10);
        if (isNaN(n)) return "00";
        n = Math.max(0, Math.min(n, 255));
        return "0123456789ABCDEF".charAt((n - n % 16) / 16) + "0123456789ABCDEF".charAt(n % 16);
    }

    /*TESTING LEARNING
    	var tempX=150,tempY=150;
    	for(var ang=0;ang<360;ang++){
    		for(var r=0;r<100;r++){
    			var rad = ang/180*Math.PI;
    			var x = (r*Math.cos(rad))+150;
    			var y = (r*Math.sin(rad))+150;
    			ctx.strokeStyle = "rgb(255, 0, "+r+")";
    			ctx.moveTo(tempX,tempY);
    			ctx.lineTo(x,y);
    			tempX =x;
    			tempY =y;
    		}
    		ctx.moveTo(150,150);
    	}
    	ctx.stroke();*/
