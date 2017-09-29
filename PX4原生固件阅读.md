# PX4原生固件阅读

##PX4杂谈

###PIX/APM的差别

pixhawk是硬件平台

PX4是pixhawk的原生固件，专门为pixhawk开发的

APM（Ardupilot Mega）早期也是硬件

Ardupilot是APM的固件

后来APM3.0后挂了，ArduPilot就支持Pixhawk，

所以称ArduPilot固件也叫APM



### PX4逻辑控制流程

1. commander/navigator 产生期望位置
2. position_estimator估计当前位置
3. 通过pos_ctrl产生期望姿态
4. attitude_estimator估计当前姿态
5. 通过att_estimator产生PWM数值
6. 最后通过mixer/motor_driver控制电机

### PX4启动

​	飞控上点后自动执行Firmware/ROMFS/px4fmu_common/init.d/rcS脚本（startup script）

#### poll

 `int poll(struct pollfd fds[], nfds_t nfds, int timeout)`

​	监控文件描述符

##PX4模块

### uORB（Micro Object Request Broker，微对象请求代理器）

​	uORB是Pixhawk系统中关键的一个模块，肩负了*数据传输*任务。所有传感器，数据传输任务，GPS，PPM信号从芯片获取后通过uORB进行传输，到各个模块计算处理。（可以理解为数据中心仓库）

​	uORB是跨进程的IPC通信模块。实际多个进程打开同一设备文件，通过此文件节点进行数据交互和共享。

​	进程间通过命名（总线）交换消息成为topic

​	一个topic包含一种消息类型/数据类型。

​	每个进程可以订阅/发布topic,一个进程可以订阅多个主题，但一条总线始终只能有一条消息。

#### 公告topic


​	`extern int orb_advertise(const struct orb_metadata *meta,const void *data)`

​	meta:uORB元对象，可以看topic的ID，通过ORB_ID赋值。

​	data：指向一个被初始化，发布者要发布的数据存储变量指针。

####发布更新

`extern int orb_publish(const struct orb_metadata *meta, int handle, const void *data)`

handle：orb_advertise函数返回的句柄

data:指向发布数据的制作

#### 订阅topic

要求的满足条件：

- 调用ORB_DEFINE()或ORB_DEFINE_OPTIONAL()宏（在订阅头文件中包含他们）
- 发布到主题的数据结构定义

`extern int orb_subscribe(const struct orb_metadata *meta)`

#### 取消订阅

`extern int orb_unsubscribe(int handle)`

#### 拷贝数据

订阅者不能引用从ORB中存储数据或其他订阅共享的数据，只能拷贝到临界缓冲区

`extern int orb_copy(const struct orb_metadata *meta, int handle, void *buffer)`

#### 检查更新

`extern int orb_check(int handle, bool *updataed)`

#### 发布时间戳

`extern int orb_stat(int handle,uint64_t *time)`



### Note:

1. 必须要先orb_advertise()+orb_subscribe()后，才能使用orb_copy()	
2. 所有主题在Firmware/Build_px4fmu-v2_default/Src_Modules/uORB/Topics
3. 也可以到Firmware/Msg下找到

## Daemon



### 参数获取

```c
uintptr_t param_find(const char *name)
  对已知参数进行线性搜索
eg:param_t _param_system_id = param_find("MAV_SYS_ID");
PARAM_DEFINE_INT32(MAV_SYS_ID,1)
 int param_get(param_t param, void *val)
  获取param的地址并赋给val，调用param_get_value_ptr(param)
```

### examples

#### fixedwing_control



#### px4_daemon_app

px4_daemon_app_main:控制的主线程

px4_daemon_thread_main:运行线程

构造线程=>**px4_task_spawn_cmd**构建

usage负责输出



