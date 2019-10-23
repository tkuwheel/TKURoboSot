#使用說明

//======================================================================
#模擬
    $roscore
    $rosrun self_localization distest
    wasd控制上下左右 qe控制旋轉
    $rosrun self_localization self_localization
    ###注意###模擬已經與實際使用整合完畢 皆使用rosrun self_localization self_localization
//======================================================================
#實際使用
    將機器人移動至球場中心,車頭朝向櫃子會比較好定位
    $roslaunch fira_launch main_launch.launch
    $rosrun vision localization_white_line
    $rosrun imu_3d imu_3d
    $rosrun self_localization self_localization

    #輸入topic:
        #馬達回授: /motion/motionFB
        #imu_3d回授: /imu_3d
        #白線座標點: /vision/mcl/WhiteRealDis
    #輸出topic:
        #定位點: /akf_pose
        #粒子標準差: /mcl/sd
        #影像: /mcl/image
    
    球場中心點為(0, 0)朝向櫃子為地圖y+,藍門為地圖x+,面向藍門為地圖0度
//======================================================================
#重置定位
    $rostopic pub /mcl/resetParticles self_localization/resetParticles "init: false
    x: 0.0
    y: 0.0
    w: 0.0" 

    init改成ture可以設置搜尋起始點(x,y,w),使用false為隨機搜尋
//======================================================================
#定位資訊輸出查看
    #定位座標
        $rostopic echo /akf_pose
    #粒子標準差
        $rostopic echo /mcl/sd
    #影像
        $rostopic echo /mcl/image
//======================================================================
#馬達回授資訊查看
    $rostopic echo /motion/motionFB
    y+為機器人車頭前方
    x+為機器人車頭右方
    距離單位 公尺
//======================================================================
