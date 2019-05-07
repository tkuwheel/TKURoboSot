FIRA影像使用說明
=============


### 確認資料夾裡面有新的影像程式
image_publisher interface, monitor, black_item, white_line  
   
### 程式說明
+ image_publisher: 測試程式時使用,　會從topic　/camera/image_raw　傳送照片  
+ interface: 使用網頁影像介面調整各個影像參數,　按下save按鈕後,　會儲存parameter到config/FIRA.yaml  
+ monitor: 偵測球及球門位置  
+ black_item: 黑色物體掃描線, 負責偵測障礙物  
+ white_line: 白色物體掃描線, 掃描場地白線用於定位  

---
### 開啟程式
    $roslaunch vision vision.launch
+ 使用這個launch會開啟vision/config內的prosilica_drivier.yaml與FIRA.yaml  
+ 網頁連接部份的rosbridge_websocket.launch與web_video_server(也可以切換成mjpeg_server)  
+ 影像程式會開啟prosilica_node, interface, black_line, white_line, monitor  
有任何需要增減的部份自行到launch資料夾修改  
啟動完成後即可到web/vison資料夾內開啟interface網頁介面控制影像  
### interface網頁介面影像查看完記得關閉影像以節省效能
###   
###   
+ 如果要在電腦端測試影像程式，可以使用  

###   
    $roslaunch vision test.launch
這個launch會開啟image_publisher傳送照片代替影像  
或者可以修改各個程式內的main.cpp將影像的topic替換  
例如  
`Vision cam("/camera/image_raw");`替換成`Vision cam("/usb_cam/image_raw");`  
如果影像type不同, 請再另外修改class Vision的副函式imageCb  

--- 
### 個別啟動
+ 進入workspace後，開啟roscore

###     
    $roscore
+ 使用新的Terminal分頁，讀取yaml參數

###     
    $. devel/setup.bash
    $rosparam load src/vision/config/FIRA.yaml /FIRA
    $rosparam load src/vision/config/prosilica_driver.yaml /prosilica_driver
+ 啟動攝影機  

###    
    $rosrun prosilica_camera prosilica_node
+ 使用新的Terminal分頁，啟動rosbrideg websocket連接網頁介面

###    
    $. devel/setup.bash
    $roslaunch rosbridge_server rosbridge_websocket.launch
+ 使用新的Terminal分頁，啟動mjpeg_server連接網頁介面

###     
    $. devel/setup.bash
    $rosrun mjpeg_server mjpeg_server
或者

###     
    $rosrun web_video_server web_video_server
+ 開啟影像程式  
+ interface  

###    
    $. devel/setup.bash
    $rosrun vision interface
+ monitor

###    
    $. devel/setup.bash
    $rosrun vision monitor
+ black_item

###    
    $. devel/setup.bash
    $rosrun vision black_item
+ white_line

###    
    $. devel/setup.bash
    $rosrun vision white_line

--- 
### 馬達測試
+ 先開啟rosbridge_websocket

###    
    $roslaun rosbridge_server rosbridge_websocket.launch
+ 使用新的Terminal分頁, 開啟馬達程式

###   
    $. deverl/setup.bash
    $rosrun motion motion
+ 開啟正規賽網頁介面, 與機器人連線後按Stand by按鈕後, 調整speed(10~20), 拉動拉桿察看馬達有沒有正確反應  
