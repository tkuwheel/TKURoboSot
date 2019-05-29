1.1執行時會出現問題
	執行imu_3d時會因為執行權限不夠無法抓到device

1.2必須將ros的環境變數加入sudo
	sudo visudo

1.3將以下程式碼加到最下面
	Defaults env_keep += "ROS_MASTER_URI"

2.1執行程式
	cd catkin_ws/devel/lib/imu_3d/
	sudo ./imu_3d
