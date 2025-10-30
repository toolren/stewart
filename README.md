![](https://github.com/toolren/stewart/blob/main/gif/stewart.gif)

源项目来源于 https://github.com/daniel-s-ingram/stewart 源项目中获取手柄数据部分有问题，无法通过手柄控制机器人平台的移动，因此修改了src目录下的ps$_control.cpp文件并重新编译之后可以通过ps4手柄控制末端平台移动。
以下是执行步骤

```
cd ~/your_catkin_ws_src_path/  
git clone https://github.com/toolren/stewart.git  
catkin build stewart
source ~/your_catkin_ws/devel/setup.bash
```

编译控制插件:

```
cd plugin  
mkdir build  
cd build  
cmake ../  
make  
```

启动launch文件:

```
roslaunch stewart stewart.launch
```


关于ps4手柄如何控制平台移动：
[]()
