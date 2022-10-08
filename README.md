# 自动驾驶多传感器融合定位
## 1.项目介绍
自动驾驶多传感器融合定位的个人学习记录，将会从0到1逐步实现一个完整且实用的建图、定位框架   
阶段性代码将以Tag形式记录,10days左右上传新tag,如遇问题可提交issue :relaxed:  
感谢任乾、李想等大佬的开源贡献,受益良多  

## 2.主要内容

+ [x] Tag v1.0 惯导数据可视化   
+ [x] Tag v2.0 DNT里程计  
+ [ ] Tag v3.0 DNT里程计+后端优化+回环检测(codeding 中)  
+ [ ] Tag v4.0 DNT重定位
+ [ ] Tag v5.0 前端适配Fast lio
+ [ ] Tag v5.0 Lio gnss松耦合
 


流程及代码框架可见[博客:多传感器融合定位学习系列](https://blog.csdn.net/weixin_37684239/article/details/126571774?spm=1001.2014.3001.5502) (PS: 咕咕咕 尽量及时更新不鸽 :laughing:) 

## 3.环境依赖
+ ubuntu18.04 
+ ROS melodic  
+ yaml-cpp >=6.0  
+ google glog  
+ geographicLib
+ g2o  

安装方法可参考[博客:机器人开发常见第三方库、软件安装和使用](https://blog.csdn.net/weixin_37684239/article/details/126568335?spm=1001.2014.3001.5501)

## 4.如何运行
参考各Tag中的README,通常运行launch 文件即可

## 5 工程适配
 **前端里程计**
+ [x] NDT    
+ [ ] ICP
+ [ ] Fast lio2  
+ [ ] Aloam
**后端优化器**
+ [x] g2o
+ [ ] ceres
+ [ ] gtsam

**回环检测**
+ [ ] Scan Context  

**重定位**
+ [ ] DNT  

**数据集**
+ [x] kitti  
 
## 6.参考
[任乾 知乎专栏从零开始做自动驾驶](https://zhuanlan.zhihu.com/p/83775731)  
[李太白lx 从零开始学定位 ](https://blog.csdn.net/tiancailx/article/details/125785641?spm=1001.2014.3001.5501)  
深蓝学院 多传感器融合定位      
github.com/XiaotaoGuo/modular_mapping_and_localization_framework
