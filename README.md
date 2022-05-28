# zavtrav3

## Установка

```
pip install opencv-contrib-python==3.4.2.16
pip intall numpy
```

## Сборка

```
cd ~/catkin_ws/src
git clone https://github.com/ifonya22/zavtrav3
cd ..
catkin_make
```
## Запуск
```
roscore
```
```
roslaunch turtlebot3_gazebo turtlebot3_autorace.launch
```
```
roslaunch turtlebot3_gazebo turtlebot3_autorace_mission.launch
```
```
roslaunch zavtrav3_autorace_core autorace_core.launch
```
```
rostopic pub -1 /robot_start
```


