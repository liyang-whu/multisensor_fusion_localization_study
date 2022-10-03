### 1. 项目介绍
自动驾驶 多传感器融合定位学习的个人代码记录，从0到1逐步实现一个完整且实用的定位框架  
主要侧重于学习基础理论、代码框架，打好还在新手村的我的基本功

## 2. 主要内容

Tag v1.0 惯导数据可视化   
Tag v2.0 DNT里程计
Tag v3.0 DNT里程计+后端优化+回环检测(codeding 中)

具体算法流程及代码框架可见[CSDN博客](https://blog.csdn.net/weixin_37684239/article/details/126571774?spm=1001.2014.3001.5502)

## 3.环境配置
ROS melodic  
yaml-cpp 6.0  
google glog
GeographicLib  

(安装上述第三方库有遇到问题也可参考[机器人开发常见第三方库、软件安装和使用](https://blog.csdn.net/weixin_37684239/article/details/126568335?spm=1001.2014.3001.5501))

## 4.如何运行
```
rosbag play ${kitti dataset}
```
  ```shell
 source devel/setup.bash
 roslaunch multisensor_localization test_frame.launch
  ```

## 5.参考
任乾 从零开始做自动驾驶    
深蓝学院 多传感器融合定位  
李想 从零开始做定位  
