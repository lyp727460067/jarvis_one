jarvis   vio主程序，编译成.so
jarvis_pic  中  jarvis_pic_main  为读取图片和imu.txt可以在 mr527中放入数据进行离线测试时间
jarvis_pic  中 jarvis_main   在mr527中主程序沟通，包括tcp发送到PC端和写入共享内存交互
jarvis_ros 为ros1 demo，读入bag包名，在配置上面加入bag名字
jarvis_ros2 为ros2 demo，  其中jarvis_main读入图片和imu.txt,发布结果给RVIZ2，jarvis_zmp_main通过TCP接受MR527的数据进行
rviz2显示


######
ros1 
在jarvis_ros2新建CATKIN_IGNORE(可以忽略当前的包编译)
放到catkit_ws/src
读取bag的方法，bag放到配置文件里面，会离线读取
执行  catkin_make_isolate --install 
执行  ./devel/jarvis_ros/jarvis_ros_offline   配置文件
######
ros2  在jarvis_ros1新建CATKIN_IGNORE(可以忽略当前的包编译)
执行 colcon  build
./build/jarvis_ros2/jarvis_ros2_zmp_main 为和mr527 进行的通信显示
正常读取图片和.txt的指令
./build/jarvis_ros2/jarvis_ros2_main  src/jarvis/configuration/euroc_test_new/euroc_stereo_config_15.yaml  /home/lyp/project/rosbag/0429/0429_sys/0429_sys/0429/zhengchang/
####

在MR527上运行的时候 设置 yaml 中的record =1 会同时录取image和imu.txt和pose.txt，写入在/tmp/jarvis/data/日期/
