# cartographer_interface
simplify cartographer interface, support using ros or not

cartographer 算法接口的极度简化接口实现，能自行修改选择是否使用ros

## 使用方法

### cartographer 安装

首先参考官方网站中的教程，确保成功安装并运行cartographer中的2d Demo

### 修改launch文件

* 修改 backpack_2d.launch 文件，将原始的ros接口注释调(注意：修改文件的路径为：工作空间/install_isolated/share/cartographer_ros/launch/demo_backpack_2d.launch)

修改后的文件

```
<launch>
  <param name="robot_description"
    textfile="$(find cartographer_ros)/urdf/backpack_2d.urdf" />

  <node name="robot_state_publisher" pkg="robot_state_publisher"
    type="robot_state_publisher" />
<!--
  <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename backpack_2d.lua"
      output="screen">
    <remap from="echoes" to="horizontal_laser_2d" />
  </node>
-->
  <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros"
      type="cartographer_occupancy_grid_node" args="-resolution 0.05" />
</launch>
```
* 修改 demo_backpack_2d.launch 文件，将激光雷达数据名称进行修改

修改后的文件内容如下
```
<launch>
  <param name="/use_sim_time" value="true" />

  <include file="$(find cartographer_ros)/launch/backpack_2d.launch" />

  <node name="rviz" pkg="rviz" type="rviz" required="true"
      args="-d $(find cartographer_ros)/configuration_files/demo_2d.rviz" />
  <node name="playbag" pkg="rosbag" type="play" args="--clock $(arg bag_filename)">
      <remap from="horizontal_laser_2d" to="echoes" />
  </node>
</launch>
```

### 运行算法接口工程

1. 下载并编译本工程
2. 运行launce文件，如下命令
`roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag`
3. 运行本代码
4. 在rviz中添加数据，By topic 选择栅格地图和轨迹数据
5. 最后可以在rviz中看到运行轨迹和构建的地图，并且在终端中以激光雷达的数据频率输出当前位置信息


### 运行在自己工程项目中

本项目为演示方便，在最外层加了一个ros壳，从而兼容了官方的示例演示功能，在实际项目中可以直接将ros部分去除然后重写传感器输入读取部分代码，此时需要对整体工程较为熟悉，但是本工程代码量极小稍有基础应该不难实现。

* 自己编写代码读取传感器数据，并按照给定格式传入
* 自己修改配置文件和代码可以定制使用的传感器类型和数量

## cartographer 库接口调用逻辑



