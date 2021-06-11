#include "cartographer_interface.h"
#include "msg.h"

// demo 示例程序
int main(int argc, char** argv)
{
    // 构造 cartographer_interface 类，传入配置文件目录和文件名
    cartographer_interface localize("",
                                    0.05,
                                    "/home/lyc/cartographer_ws/src/cartographer_ros/cartographer_ros/configuration_files",
                                    "backpack_2d.lua");
    // 数据接口类初始化
    msg m(localize,0.05);

    // log 类，取消注释可以打印出cartographer算法内部的消息
    //ScopedLogSink ros_log_sink;
    while(true)
    {
    }
}


