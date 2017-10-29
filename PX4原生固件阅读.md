# PX4原生固件阅读

#### 学习计划

1.看懂src/example的小实例，如果有条件，跑起来

2.研究module，首先rORB的原理，搞清楚

3.可以换着看drivers的代码

4.跑起来

##PX4杂谈

###PIX/APM的差别

pixhawk是硬件平台

PX4是pixhawk的原生固件，专门为pixhawk开发的

APM（Ardupilot Mega）早期也是硬件

Ardupilot是APM的固件

后来APM3.0后挂了，ArduPilot就支持Pixhawk，

所以称ArduPilot固件也叫APM



## 固件编译&环境搭建

###Ubuntu16.04环境搭建：

````shell
预环境处理：（把用户添加到用户组　"dialout":重新登录一回使其生效）
sudo usermod -a -G dialout $USER
````



````shell
依赖环境安装：
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
# 必备软件
sudo apt-get install python-argparse git-core wget zip \
    python-empy qtcreator cmake build-essential genromfs -y
# 仿真工具
sudo add-apt-repository ppa:openjdk-r/ppa
sudo apt-get update
sudo apt-get install openjdk-8-jre
sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-8-jdk openjdk-8-jre clang-3.5 lldb-3.5 -y

#ubuntu自带端口管理会影响这个
sudo apt-get remove modemmanager

#依赖包
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo build-essential \
    libftdi-dev libtool zlib1g-dev \
    python-empy  -y

#安装交叉编译链
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa
````

如果交叉编译链没有安装好，用如下的办法。

````bash
编译链安装：
   https://launchpad.net/gcc-arm-embedded/+download 
然后执行如下命令：重启后交叉编译链接
pushd .
# => 卸载新版的gcc-arm-none-eabi
sudo apt-get remove gcc-arm-none-eabi
# => 安装下载好的gcc-arm-none-eabi
tar -jxf gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2
sudo mv gcc-arm-none-eabi-4_9-2015q3 /opt
exportline="export PATH=/opt/gcc-arm-none-eabi-4_9-2015q3/bin:\$PATH"
if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
# => 使路径生效
. ~/.profile
popd
````

接下来进行下载源码编译：

````shell
git clone https://github.com/PX4/Firmware.git
#源码里部分子模块不存在，需要更新
cd Firmware
git submodule update --init --recursive
make px4fmu-v2_default
````

一般来说，只要有交叉编译链，就可以一次编译通过

````shell
# Gazebo simulator
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
## Setup keys
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
## Update the debian database:
sudo apt-get update -y
## Install Gazebo8
sudo apt-get install gazebo8 -y
## For developers (who work on top of Gazebo) one extra package
sudo apt-get install libgazebo8-dev
````

### 问题解决：

````shell
如果出现
fatal: Needed a single revision
Unable to find current revision in submodule path 'Tools/jMAVSim'
````

是因为多次重新update的原因，删除对应的文件夹重试就好

如果update 子模块时候卡了半天，ctrl-c后重新试试

### window下环境搭建

见下面这个链接：

https://zhuanlan.zhihu.com/p/25198079

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

### uORB模块（Micro Object Request Broker，微对象请求代理器）

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

####Note:

1. 必须要先orb_advertise()+orb_subscribe()后，才能使用orb_copy()	
2. 所有主题在Firmware/Build_px4fmu-v2_default/Src_Modules/uORB/Topics
3. 也可以到Firmware/Msg下找到

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
###land_detector模块

#### class LandDetector  

​	为降落检测算法提供统一接口

- isRunning() bool public
- isLanded() bool
- shutdown() void
- start()  void
- cycle_trampoline() static void
- update() virtual bool  protected
- initialize() virtual void
  - orb_update(xxx) bool  =>为了方便统计uORB订阅信息
- cycle() void private

````c++
* 类的实现逻辑：
 * 调用LandDetector::start()方法 =>将LandDetector::cycle_trampoline()加入全局工作队列中，参数时当前对象的指针
 * cycle_trampoline()将参数转化为LandDetecotr的基类指针，然后调用LandDetector::cycle()
 * 在cycle()中，如果没有初始化，初始化！然后更新状态updata()，通过uORB输出状态，如果没有退出标识，将cycle_trampoline()再次加入工作队列，等待下次调用
````



#### class FixedwingLandDetector

- update() bool override protected
- initialize() void override  -> 订阅信息
- updateSubscriptions() void -> 更新信息 
- updateParamterCache(const bool) void ->更新cache中的参数

````c
类的描述：
	类订阅了control_state,vehicle_status,actuator_armed,airspeed信息。
	实现了updata()方法：
		更新_velocity_xy_filtered,_velocity_z_filtered,_airspeed_filtered,_accel_horz_lp四个参数，根据当前实际状态值，以不同比例，更新这几个参数
	
````

### local_position_estimator模块





### **fw_att_control模块**

#### class FixedwingAttitudeControl

- start() int **public** -> 控制程序开始入口
- task_runing() bool 
- paramters_update() int **private** -> 更新\_parameters的所有参数，并且更新\_roll_ctrl,\_yaw_ctrl,\_pitch_ctrl,\_wheel_ctrl参数
- control_update() void  -> 更新控制输出
- vehicle_control_mode_poll() void -> 检测控制模式是否改变，并从uORBcopy下来，存在private变量中，其他一样
- vehicle_manual_poll() void ->  检测输入是否改变
- vehicle_accel_poll() void  -> 检测加速度是否改变
- vehicle_setpoint_poll() void -> 检测设定点是否改变
- global_pos_poll() void  -> 检测位置是否改变
- vehicle_status_poll() void -> 检测状态是否改变
- task_main_trampoline(int argc,char* argv[]) void static
- task_main() void -> 姿态调整的主线程

类的主题实现流程：

1. fw_att_control_main的入口，实现start|stop|status功能，给namespace::g_control指针赋值。
2. 调用g_control->start()开始类中的函数
3. 调用px4_task_spawn_cmd来运行task_main_trampoline,通过这个函数调用task_main()开始姿态调整
4. 更新所有状态，然后监控\_params_sub,_\_ctrl_state_sub，最慢500ms更新一次
5. 1. perf_begin(_loop_perf)，为止，应该是有关调度和资源分配的吧

   2. 判断\_params是否需要更新

   3. 当姿态发生变化，run controller

   4. 1. 判断当前调整与上一次调整的时间间隔，如果大与1.0f，置为0.01f

      2. 保存控制信息到\_ctrl_state

      3. 从\_ctrl_state中获取旋转矩阵和欧拉角度。

      4. 更新所有状态

      5. 判断是否开始控制，否则锁定积分器

      6. 判断故障否

      7. 先获取输入襟翼舵量flaps_control(有手动自动之分)

      8. 根据与上一次输入舵量的比较，判断是否更新输入，然后记录更改的时间t_flaps_changed，变化舵量delta_flaps,

      9. 最后对输出flaps_applied做平滑处理，防止在短时间内，变化过大，*flaps_applied = flaps_control (输入量) - (1 - delta_T(时间变化量)) * delta_flaps*, 流程代码见附加1

      10. 副翼控制流程同上

      11. 根据_vcontrol_mode模式区别处理

          - 自动模式下，先限制空速最小0.5,计算airspeed_scaling,groundspeed（地速），roll_sp,pitch_sp等

          1. \_flag_control_auto_enabled，俯仰滚转姿态设定=_att_sp的参数+默认的偏移量，油门航向直接是设定的值。根据参数是否重置积分器

          2. flag_control_velocity_enabled,定速巡航模式，同上，roll改变前只是会做点小判断

          3. flag_control_altitude_enabled，定高模式，roll设定混入一定手动参数

          4. 其他模式，半手动模式，roll=手动输入\*man_roll_max+rollsp_offset_rad,pitch=手动输入\*man_roll_max+pitch_offset_rad

          5. 如果已经落地，重置积分器

             将得到的参数输入control_input中，给\roll_ctrl,\_pitch_ctrl,\_yaw_ctrl,\_wheel_ctrl,在通过函数get_desired_rate()计算得到期望速率，然后输出给actuators对应通道

          - 手动模式下，将手动输入值+\+parameter.trim_xx直接输出到_actuatuors中，作为最后的控制信号

      12. 通过uORB输出对应的信息

   5. loop_counter++;perf_end(\_loop_perf);

   ​


#####附录1

````c++

//fw_att_control_main.cpp
//line:843
			/* default flaps（振翅，应该是襟翼舵量） to center */
			float flaps_control = 0.0f;  //flaps_control 可以看作当前的输入舵量

			static float delta_flaps = 0;   // 舵量的变化值

			/* map flaps by default to manual if valid */
			/* 从手动输入通道获取输入值，判读是否合法 */
			if (PX4_ISFINITE(_manual.flaps) && _vcontrol_mode.flag_control_manual_enabled) {
				flaps_control = 0.5f * (_manual.flaps + 1.0f) * _parameters.flaps_scale;

			} else if (_vcontrol_mode.flag_control_auto_enabled) {
				flaps_control = _att_sp.apply_flaps ? 1.0f * _parameters.flaps_scale : 0.0f;
			}

			// move the actual control value continuous with time
			static hrt_abstime t_flaps_changed = 0;   //上一次更新舵量的绝对时间

			if (fabsf(flaps_control - _flaps_cmd_last) > 0.01f) {   //flaps_cmd_last 是上一次修改舵量的值
				t_flaps_changed = hrt_absolute_time();
				delta_flaps = flaps_control - _flaps_cmd_last;		//记录当前输入和上一次输入的差值
				_flaps_cmd_last = flaps_control;
			}

			static float flaps_applied = 0.0f;		//最终输出舵量

			if (fabsf(flaps_applied - flaps_control) > 0.01f) {
				//为了平滑处理，防止输出量出现阶跃的现象
				// flaps_applied = flaps_control (输入量) - (1 - delta_T(时间变化量)) * delta_flaps;
				flaps_applied = (flaps_control - delta_flaps) + (float)hrt_elapsed_time(&t_flaps_changed) * (delta_flaps) / 1000000;
			}
````

### **fw_pos_control模块**

####class landingslope 

为固定翼着陆的角度变化模块

- ``calulateSlopeValues() void private``

  更新H1,H0,d1,根据log（H0/H1）的比例调整 d1 / d1+ delta d的比例，更新其他参数

- ``getLandingSlopeRelativeAltitude(wp_landing_distance) float``

  返回在距离落航点的着陆坡上点的相对高度，调用多参数的同名函数

- ``getLandingSlopeRelativeAltitudeSave(wp_landing_distance,bearing_lastwp_currwp,bearing_airplane_currwp) float public``

  返回在距离落航点的着陆坡上点的相对高度，检查飞行器是否在航点上来避免爬升

- ``getLandingSlopeAbsoluteAltitude(wp_landing_distance,wp_altitude) float``

  返回在距离落航点的着陆坡上点的绝对高度

基本上示例都是这么一个结构
- ``getLandingSlopeAbsoluteAltitudeSave(wp_landing_distance,bearing_last_wp_currwp,bearing_airplane_currwp) float``

- 检查飞行器是否在航点上来避免爬升

- ``getLandingSlopeRelativeAltitude(wp_landing_distance,horizontal_slope_displacement,landing_slope_angle_rad) float static``

  返回h_flare.rel的高度，返回在距离落航点的着陆坡上点的相对高度

- ``getLandingSlopeAbsoluteAltitude(wp_landing_distance,wp_landing_altitude,horizontal_slope_displacement,landing_slope_angle_rad) float static``

  返回h_flare.rel + H1的高度，返回在距离落航点的着陆坡上点的绝对高度

- ``getLandigSlopeWPDistance(slope_altitude,wp_landing_altitude,horizontal_slope_displacement,landing_slope_angle_rad) float static``

  给定降落高度，返回距离预定降落点的距离

- ``getFlareCurveRelativeAltitudeSave(wp_distance, bearing_lastwp_currwp, bearing_airplane_currwp) float``

- getFlareCurveRelativeAltitudeSave(wp_distance, ``bearing_lastwp_currwp, bearing_airplane_currwp, wp_altitude) float``

  获取Flare曲线的相对高度

- ``updata(landing_slope_angle_Rad_new, flare_relative_alt_new, mmotor_lim_relative_alt_new, H1_virt_new) void``

  将值重新赋值，并且重新调用calcuateSlopeValues()

#### class lib/extern_lgpl/tecs 

这个类负责控制俯仰和油门来控制速度和高度，在后面会用到，不做深入讲究，只是为了简绍这个类的作用

````c++
/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 * 	  通过控制油门来控制能量，控制俯仰控制势能，使得速度高度结合控制，在势能和运动结合
 *    Selectable speed or height priority modes when calculating pitch angle
 * 	  当计算俯仰角度时优先选择速度/高度模式
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *    当空速测量不管用时，切换俯仰角度优先控制到高度优先控制，并设置油门基于高度速率需求
 *  - Underspeed protection that demands maximum throttle switches pitch angle control
 *    to speed priority mode
 * 	  需求最大油门的低速保护下，切换俯仰控制优先到速度优先模式
 *  - Relative ease of tuning through use of intuitive time constant, trim rate and damping parameters and the use
 *    of easy to measure aircraft performance data
 *    通过使用直观的时间常数，微调速率和阻尼参数以及使用易于测量飞机性能数据的相对易于调谐
 *   
 */
````



#### class FixedwingPositionControl

#####FixedwingPositionControl:

使用param_find()函数定位所有的参数,赋初值

#####start() int static  public

开始控制

#####task_running() bool

任务的状态

#####更新控制函数：

- parameters_update() int  *private*

  更新所有的类内参数，使用param_get()方法

- control_update() void

  更新控制输出

- vehicle_control_mode_poll()

  飞行器模式更新，通过orb_check来检测是否更新

- vehicle_status_poll()

  飞行器状态更新，通过orb_check

- vehicle_manual_control_setpoint_poll()

  飞行器手动控制设定点更新，同上

- control_state_poll()

  控制状态更新

  如果空速1s内没有更新，认定是非法的。

  将四元组的控制信息转化为欧拉角度

  更新TECS的状态

- vehicle_sensor_combined_poll()

  飞行器传感器状态更新

- vehicle_setpoint_poll()

  设置点更新

- navigation_capabilities_publish()

  导航能力输出

##### get_demanded_airspeed() float

返回需求的空速

实现：

​	根据油门输入(\_manual.z)大于0.5与否，决定速度应该变大还是变小（按照比例）

​	// 如果油门输入<0.5，输出空速应该是 最小速度+(可缩减速度)*(2×输入值)  

​        // 可缩减速度为当前速度到最小速度的差，以0.5*2为1，中值，不会变化

##### calculate_target_airspeed(airspeed_demand) float

返回目标速度，输出需求速度

实现：

​	一般只对需求速度限制在最大和最小的范围内

​	在目的空速加上最小下冲速度(\_groundspeed_undershoot)，只有在有强风时候这个最小下冲速度不为0

##### calculate_gnd_speed_undershoot(&current_position,&ground_speed_2d,&pos_sp_triplet)

计算对地下冲速度，输出到\_groundspeed_undershoot属性中

说明:

对地下冲速度（固定翼速度不够，升力<重力 产生的飞行高度下降速度） 是飞机没有到达一定地速的结果。因此，当空速大于最小地速时候，对地下冲速度为0，否则为正当小于最小地速时候。

#####get_waypoint_heading_distance(heading, distance, &wp_prev, &wp_next) void

基于方向和当前的距离获取下一个waypoint，输出到wp_prev,wp_next

实现：

如果是flag_init，根据角度和距离获取前后航点

否则，根据前一帧的前后航点说确定的直线，根据预先定义的距离，获取新的前后航点。

#####get_terrain_altitude_landing(land_setpoint_alt, &global_pos) float

返回降落点的地形高度

实现：

​	如果地形评估无效，直接使用设定点的高度

​	如果使用地形评估，并切换到地形评估，一直使用到降落为止

#####get_terrain_altitude_takeoff(takeoff_alt, &global_pos) float

返回起飞时候的地形高度

实现：

​	如果global_pos中地形高度有效返回，否则返回起飞高度takeoff_alt

#####in_takeoff_situation() bool

检查是否正在起飞中

实现：

​	判断是否刚进入空中，油门输入是否到门限，高度是否符合最小的起飞的条件

#####do_takeoff_help(\*hold_altitude,\*pitch_limit_min) 

定高模式下的起飞参数信息的获取函数，获得起飞的高度和最小俯仰，传入参数指针，直接对地址修改

实现：

​	如果正在起飞，更新定高高度为起飞最小高度和最小俯仰；否则，根据参数pitch_limit_min更新最小俯仰

#####update_desired_altitude(dt) bool

根据俯仰输入更新期望高度,输出到属性\_alt_hold,参数为Time step，单位时间,返回的bool类型是是否是爬升模式。

实现：

​	0.06 ~ -0.06 视为0， 其余1 ~ 0.06 | -0.06 ~  -1 部分按比例，更新高度 = 原高度 + 最大变化速率*dt * pitch （俯仰指数）

​	如果是垂直起降飞机，在旋翼状态/转化状态，高度是当前高度。

#####control_position(global_pos, ground_speed, pos_sp_triplet) bool

控制位置，根据输入的位置，速度，和航点，（存在全局变量的飞行器状态），输出对应的控制姿态\_att_sp（全局变量）

#####get_tecs_pitch() float

如果mTecs可行，返回getPitchSetpoint()否则返回_tecs.get_pitch_demand()

#####get_tecs_thrust() float

同上

#####get_demanded_airspeed() float

获取需要的空速大小

实现算法如下：

​	判断输入油门的大小，设定可变化速度为当前速度距最大/最小速度的差，airspeed_trim字面上是修剪的空速速度（即当前速度）

​		如果油门输入小于0.5，输出速度=最小速度+可变化速度×油门输入值×2；（如果油门输入值0.5，则速度不变化，在输入值小于0.5的情况下，油门输入×2 < 1）

​		如果油门大于0.5，输出速度 = 当前速度 + 可变化速度 × （油门输入值-0.5）*2（减去0.5的基础油门）

#####calculate_target_airspeed(airspeed_demand) float

计算目标空速，没看懂想干嘛

实现：

​	当前空速根据全局变量\_airspeed_valid是否合法，合法取当前控制状态的空速，否则取中值。

​	目标空速= airspeed_demand+\_groundsped_undershoot(期望速度+地速)，再对目标空速进行限制，使其在最大最小之间。

​	\_airspeed_error这个全局变量是目标速度和当前空速之差，应该是用来调整速度的。

​	返回目标速度

#####**calualte_gndspeed_undershoot(current_pos, ground\_speed_2d, pos_sp_triplet) void **

计算对地下降速度\_groundspeed_undershoot，对地下冲速度（固定翼速度不够，升力<重力 产生的飞行高度下降速度） 是飞机没有到达一定地速的结果。因此，当空速大于最小地速时候，对地下冲速度为0，否则为正当小于最小地速时候。

​	根据distance和delta_altitude计算期望的速度，与当前速度之差就是应该对地下降的速度，

#####task_main_trampoline()

构建FixedwingPositionControl类，赋给l1_control命名空间的g\_control指针，开始运行``task_main()``函数，然后释放。

#####task_main() void

先订阅需要的信息，设置更新速率等。

fds[2]，fds[0]负责检查参数更新，fds[1]负责位置更新

while

​	px4_poll检查是否有更新，更新control_mode,status

​	如果参数更新，获取更新参数

​	如果位置更新，control_statue,setpoint,sensor_combined,manual_control_setpoint更新，运行control_position()函数，如果控制成功，通过uorb机制输出控制结果信息，最后如果导航信息很久没有更新，输出导航信息没看懂

#####reset_takeoff_state() void

#####reset_landing_state() void

#####tecs_update_pitch_trottle() void

一个调用实现TECS的包装器函数(mTECS是只通过参数启用)

#####类的实现流程：

​	从fw_pos_control_l1_main函数开始执行，这个main方法有命令行参数，根据参数，输出是否fw_pos_control_l1这个模块的运行状态，实际运行调用l1\_control::g\_control指针所创建的对象,调用FixedwingPositionControl::start方法的px4\_task_spawn_cmd来初始化创建对象。再通过task_main_trampoline真正创建类的对象。

​	FixedwingPositionControl::task_main()正式开始：

订阅全局位置，三个一组的航点位置，控制状态，传感器状态，控制模式，飞行器状态，参数，手动输入等通过uORB订阅。设置更新速率

- 设置px4_pollfd_struct_t fds[2]这个结构体，fds[0]作为\_params\_sub，fds[1]作为\_global_pos_sub的监视，开始循环。

- 更新飞行器的控制模式，状态信息。如果参数更新了(fds[0]),调用paramters_update()更新所有的参数。

- 如果位置发生变化(fds[1]),

  调用perf_begin(\_loop_perf),将最新的位置更新出来。

  更新控制状态信息，设定点信息，传感器的信息，手动控制的信息。

  构造ground_speed(ned3个方向的速度),current_position(经纬坐标)，调用control_position(),所有的控制内容都在这个函数中实现

  控制成功，通过uORB输出\_attitude_sp_pub,

  调用perf_end(\_loop_perf)结束

注：\_global_pos是当前飞行器的位置

​	\_paramters是全局参数

​	pos_sp_triplet是航点三兄弟的结构体

​	\_att_sp设定的姿态控制

​	\_vehicle_status 飞行器的状态

​	_manual 手动控制输入的结构体

#####**control_position()的函数运行流程：**

```c++
bool control_position(
	const math::Vector<2> &current_position,  //当前位置，经纬坐标
    const math::Vector<3> &ground_speed,	  //三轴加速度
    const struct position_setpoint_triplet_s &pos_sp_triple) //航点集合
```



1. \_control_position_last_called记录上一次位置控制的时间

2. 判断是否是固定翼模式/VTOL的变形模式，否则return false

3. 默认不使用方向舵，不使用襟翼,eas2tas(基于当前的测量来计算实际值),accel_body是飞行器的加速度，accel_earth是飞行器对地的加速度，得到这个参数用于tecs.update_state()

4. 构造ground_speed_2d，(2个维度的速度)，作为calualte_gndspeed_undershoot()函数的参数

5. in_air_alt_control是判断飞行器是否滞空的标识，飞行器的控制模式在auto|velocity|altitude下 & 飞行器没有降落(!\_vehicle_status.condition_landed),是可以控制的，同样，这个参数用于tecs.update_state()

6. 如果mTecs不可用，就使用tecs.update_state()更新参数

7. 计算下冲速度\_gndspeed_undershoot

8. 计算高度差altitude_error，目标高度-当前高度

9. 设置油门上限

10. _was_in_air置为true，记录在空中的时间\_time_went_in_air和高度\_takeoff_ground_alt

11. 如果飞行器已经降落(\_vehicle_status.condition_landed)，\_was_in_air置为false

12. 根据飞行模式控制*(每一个模式下的控制都分为水平控制和垂直控制，水平控制交给l1_control类，更新yaw和roll，垂直控制交给tecs_update更新油门俯仰)*

    - 自动模式下当前航点有效 flag_control_auto_enabled & pos_sp_triplet.current.valid

    1. 如果从FW_POSCTRL_MODE_OTHER(其他模式)切换过来，重置积分器。\_control_mode_current全局变量记录当前模式，如果mTecs可用，重置mTecs,否则tecs.reset_state(),将当前模式置为FW_POSCTRL_MODE_AUTO

    2. 更新定高高度\_hold_alt为当前高度，定航航向\_hdg_hold_yaw为当前航向

    3. 获取控制对象\_l1_control的circle_mode()状态,后面在控制完成后决定是否重置积分器；

    4.  储存speed weight到tecs中，防止速度值立马改变

    5. 构造next_wp,curr_wp,prev_wp*(作为l1_control导航控制函数的参数)*

       前俩个使用pos_sp_triplet对象的current赋值，prev_wp根据pos_sp_triplet的previous是否合法：

       若是，使用previous的值;

       否则，使用current的值。

    6. 目标速度(mission_airspeed)默认是需要修改的速度(\_parameters.airspeed_trim),

       如果当前的巡航速度符合范围&巡航速度>0.1，改为当前巡航速度 (_pos_sp_triplet.current.cruising_speed)

       *(目标速度作为calculate_target_airspeed()的参数用于计算最终输出的速度)*

    7. 根据当前航点的状态控制

       - 怠机状态(SETPOINT_TYPE_IDLE),可认为在地上中

         (\_att_sp应该就是控制输出)

         油门，俯仰，滚转为0

       - 普通航点(SETPOINT_TYPE_POSITION)

         1. 使用\_l1\_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground\_speed\_2d)做普通的导航，获得\_att\_sp.roll_body,yaw_body的值(通过调用\_l1_control的方法)

            *(这个类只根据航点的信息，当前的状态去区分控制，然后分别把具体的控制类的对应函数去更新参数，比如水平控制交给l1_control,垂直控制交给mtecs/tecs;属于比较高级调用类)*

         2. 调用tecs_update_pitch_throttle()更新俯仰和油门

       - 徘徊航点(SETPOINT_TYPE_LOITER)

         1. 使用\_l1\_control.navigate\_loiter()方法，做徘徊的导航，获得\_att\_sp.roll_body,yaw_body的值(通过调用\_l1_control的方法)
         2. 根据条件设置高度：

           如果是因为中止着陆而进入徘徊模式，需要将高度设定高于降落模式

           ​否则，设置为当前航点的高度

         3. 判断是否是起飞模式 | 因为中止降落而没有低于安全高度

           ​是，限制滚转在15°内

         4. tecs_update_pitch_throttle()更新俯仰和油门

       - 降落航点(SETPOINT_TYPE_LAND)*(这个航点的内容 比较多，水平控制垂直控制分别注释)*

         1. 降落模式开启襟翼，记录开始着陆的时间\_time_started_landing

         // Horizontal landing control 

         2. 计算上一个航点到当前航点的方向bearing_lastwp_currwp,飞行器到当前航点的方向bearing_airplane_currwp，以及当前位置到当前目标航点的距离wp_distance
         3. 如果飞行器到航点的方位-上一个航点到当前航点的方位差>90°

           设置wp_distance_save=0，偏差太大

         （创建虚拟航点在期望的路径上，但是一些距离落后于航点（没看懂这句）。这样可以确保飞机即使在关闭废除降落航点时也能在期望飞行路径上）

         4. 根据方位和距离产生虚拟航点curr_wp_shifted,但是感觉后面没用啊
         5. 如果当前距离据航点距离小于着陆的指向距离|| 着陆进入无可返回的状态 

         （可以认为是准备水平方向准备就绪或者为了进入安全距离的保护，锁住方向，沿着当前路径一直走）*(程序里大多判断是 条件判断 ||  属性判断 ，条件判断符合条件后，设置对应属性为true，下次可以直接进入)*

         ​	是：

         ​	如果land_noreturn_horizontal是false，第一次进入,设置target_bearing

         ​		如果前一个航点合法，target_bearing=bearing_lastwp_Currwp;

         ​		否则，为当前航线\_yaw

         ​	通过\_l1_control.navigate_heading()做定航向的导航

         ​	否：

         ​	\_l1_control.navigate_waypoint()	只做普通的航点控制

         6. 获得\_att\_sp.roll_body,yaw_body的值(通过调用\_l1_control的方法)
         7. 如果land_noreturn_horizontal为true，水平方向无法回退了，

           限制滚转角度-10~10°

         //Vertical landing control

         (使用tecs姿态控制来降落,如果近地，使用最小的俯仰，限制滚转来，而且高度误差为负，为了降落)

         8. 计算得到降落油门(throttle_land)，降落空速(airspeed_land)，进场空速(airspeed_aproach)  

         9. 判断是否有地形评估(\_paramters.land_use_terrain_estimate)：

            是：

            ​	判断地形评估高度是否有效(\_global_pos.terrain_alt_valid):

            ​		设置地形评估高度，上一个地形评估的值，上一个更新的值

            ​	如果是第一次进入，之前没有设置过(\_time_last_t_alt==0):	

            ​		如果距开始降落10s内

            ​		是：地形评估设置为当前航点的高度

            ​		否：地形评估高度设置为当前航点的高度，但是中止导航。(abort_landing = true)

            ​	如果地形评估无效&距上一次更新的时间没有超过规定 || 垂直控制no way back

            ​		设置地形评估为上一次有效值

            ​	否则：

            ​		地形评估高度设置为当前航点的高度，但是中止导航。(abort_landing = true)

            否：

            ​	地形评估高度设为当前航点的高度(terrain_alt = pos_sp_triplet.current.lat)

          10.  计算上一个有效的航点的相对高度L_altitude_rel,并调用landslope类获得距离目标点wp_distance的距离时候的期望高度landing_slope_alt_rel_desired

          11.  （如果当前飞行器的高度<地面高度+降落曲线的高度 && 距离<降落距离+5 ） || 垂直控制no way back

               是：//可以带油门降落，因为高度比预计高度低

               ​	设置油门最大值(这一项先做)，设置设定点的航向，运行使用垂尾

               如果高度低于发动机限制高度 | 降落发动机限制开启

               ​	设置油门的最大值(在当前油门和参数中的油门中选小的)

               ​	将land_motor_lim设为true

               ​	获取flare_curve_alt_Rel

               ​	如果当前的垂直相对高度》上一个垂直相对高度 & no 	way back  认为在地上停留

               ​	使用tecs更新油门和俯仰

               ​	如果目前还没有到垂直控制no way back 的地步

               ​		是：设置目标俯仰为0，计算height_flare，设置land_noreturn_vertical=true

               ​		否：当前姿态中vel_d是否>0.1

               ​			是：设置picth_body?

               ​			否：不改变pitch_body	

               ​	更新flare_curve_alt_rel_last为当前的值

               否：//这里通过滑翔曲线来使高度过高的点下降,直到到与期待的航线的交叉点

               如果当前高度>地形高度+降落期望高度 || land_onslope

               ​	是：期待高度alttitude_desired_rel设为landing_slope_alt_rel_desired

               ​	否：altitude_desired_rel设为L_altitude_rel?1615

               使用tecs更新油门和俯仰

               ​

       - **起飞**航点(SETPOINT_TYPE_TAKEOFF)

         如果runway_takoff可用

         是：runway_takeoff的起飞模块的具体控制，猜的

         1. 如果runway_takeoff没有初始化，初始化它
         2. 获取地形高度terrain_alt
         3. 使用\_runway_takeoff模块更新
         4. 使用\_l1_control的航点导航模式(为什么要使用航点导航，难道起飞与普通导航只有爬升模式一个区别吗？不过这么想貌似也没问题)
         5. 获取\_runway_takeoff的最大俯仰角takeoff_pitch_max_deg
         6. 使用tecs更新油门俯仰
         7. 从\_runway_takeoff模块获取roll_bady,yaw_body,fw_control_yaw,pitch_body,roll_reset_integral,pitch_reset_integral的值

         否：//如果runway_takeoff模块不可用，使用起飞检测模块 

         如果起飞检测模块可用&&launch_detection_state 不是LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS

         ​	是：

         ​		如果距上一次更新时间>4e6，给控制台消息

         ​		起飞检测更新(launchDetector.update())

         ​		起飞检测状态更新（launch_detection_state）

         ​	否：

         ​		起飞检测不可用，置状态为需要控制

         //起飞控制依赖于起飞检测状态

         如果起飞检测状态 != 	LAUNCHDETECTION_RES_NONE

         ​	是：飞行器起飞

         ​		使用\_l1_control进行航点控制

         ​		获取\_l1_control的roll_body,yaw_body

         ​		选择油门：只有在LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS模式下，我们使用全油门，否则其他我们使用先前起飞的油门 

         ​		选择最大的俯仰角度：起飞检测模块可能会根据起飞状态给俯仰强加一个限制

         ​		使用最小的俯仰和最小的滚转如果目标高度不在爬升误差之内

         ​	否：

         ​		认为在地上，重置积分器，设置默认的roll,pitch

    8. 如果是降落航点，重置降落状态；

       如果是起飞航点，重置起飞状态

       如果航点前是循环模式，航点后不是循环模式，重置积分器

    -  在定速模式下高度控制模式 flag_control_velocity_enabled & flag_control_altitude_enabled

        俯仰设置高度，油门设置速度，航向设定航点

       1. 如果当前模式不是FW_POSCTRL_MODE_POSITION,需要重置一下，\_hold_alt,\_hdg_hold_yaw,

       2. 如果当前模式是FW_POSCTRL_MODE_OTHER,重置积分器（mTecs可用重置mTecs，否则重置tecs）

       3. 将控制模式改到FW_POSCTRL_MODE_POSITION

       4. 获取期望速度altctrl_airspeed和爬升climbout_requested,?do_takeoff_help

       5. 油门最大值设定为参数的油门最大值，如果飞行器有条件降落且油门值小于油门门限，油门最大值为0

       6. 使用tecs更新油门和俯仰

       7. （航向控制）如果航向的输入值小于偏航的门限

          是：

          ​	如果控制状态(\_ctrl_state)的偏航速率小于设定的门限且偏航没有被锁定，锁定偏航（\_yaw_lock_engaged设为true）

          ​	//如果用户试图在定航模式下起飞，重置航向确保起飞不滚转

          ​	如果是在起飞情况：\_hdg_hold_enable -> false，\_yaw_lock_engaged -> true(启用航线锁定)

          ​	如果是\_yaw_lock_engaged(设置对应属性锁定航向)

          ​		将\_hdg_hold_enabled设为true，设置\_hdg_hold_yaw为当前航线

          ​	前面获取到prev_wp,curr_wp,给\_l1_control做参数，使用其做航点控制，获取roll,yaw的输出值

          ​	如果是起飞情形，限制roll在15°内

          否：

          ​	\_hdg_hold_enable -> false，\_yaw_lock_engaged -> false

    -  在高度控制模式下 flag_control_altitude_enabled

       1. 如果控制模式不是位置控制模式/高度控制模式(FW_POSCTRK_MODE_POSITION/ALITUDE),设置定高(\_hold_alt)为当前高度
       2. 如果当前模式是FW_POSCTRL_MODE_OTHER,重置积分器（mTecs可用重置mTecs，否则重置tecs）
       3. 设置控制模式为FW_POSCTRL_MODE_ALTITUDE
       4. 获取期望速度altctrl_airspeed和爬升climbout_requested,?do_takeoff_help
       5. 油门最大值设定为参数的油门最大值，如果飞行器有条件降落且油门值小于油门门限，油门最大值为0
       6. 使用tecs更新油门和俯仰

    -  其他情况下

       1. 将控制模式设为FW_POSCTRL_MODE_OTHER，设置定高(\_hold_alt)为当前高度
       2. 设置\_time_started_landing 和\_time_last_t_alt为0
       3. abort_landing 为false,重置起飞降落状态

13. copy thrust output for publication，根据具体条件设置油门

    1. 如果飞行器引擎除了问题，

       输出 0

    2. 在自动模式下 & 当前点是起飞点 & 降落检测状态是ENABLEMOTORS & _runway_takeoff不可用

       根据launchDetectionEnabled的状态，可用就设置油门为起飞的油门，否则为怠机油门

    3. 如果是自动模式 & 当前航点是起飞航点 & _runway_takeoff可用

       油门设为runway_takeoff的有油门

    4. 如果自动模式 & 航点为怠机航点

       油门为0

    5. 其他

       如果飞行器有降落条件：

       ​	是：油门在最大和怠机油门中取最小

       ​	否：控制油门(get_tecs_thrust())|油门最大值中取最小

14. 如果不满足一下条件：

    (是自动模式 &&（（航点为起飞航点 &&（起飞检测为 RES_NONE || runway_takeoff 可用）） || （航点为降落航点 && 垂直方向降落nowayback））)

    输出控制的俯仰为get_tecs_pitch()

15. 根据控制模式的flag_control_position_enabled，

    ​	是：last_manual 为false

    ​	否：last_manual为 true

##### tecs_update_pitch_throttle()函数的运行流程：

```c++
void tecs_update_pitch_throttle(
	float alt_sp, 							//目标高度
  	float v_sp,								//目标速度
	float eas2tas,							//注释给出的基于当前的测量来计算实际值，给的默认值为1.0
	float throttle_min, float throttle_max, //油门范围
	float throttle_cruise,					//巡航油门
	bool  climboot_mode,					//是否是爬升模式
	float climbout_pitch_min_rad,			//爬升最小仰角
  	float altitude,							//当前高度
  	const math::Vector<3> &ground_speed,	//当前速度
  	unsigned mode = tecs_status_s::TECS_MODE_NORMAL,
  	bool pitch_max_special=false
	);
```



1. 计算距离上次运行tecs（\_last_tecs_update）的时间dt，dt最小0.01，更新\_last_tecs_update

2. 只在滞空时候运行tecs模块，当飞行器为VTOL时，不在旋翼模式/变形模式下运行，（空速太小）通过更新run_tecs这个函数内变量

3. 如果是VTOL模式的变形模式：

   ​	是：设置属性（\_was_in_transition）为true，设置变形后的速度属性（\_asp_after_transition）为控制的空速

   ​	否：

   ​		如果\_was_in_transition为true，

   ​			是：说明已经从VTOL模式改变，期望速度(\_asp_after_transition)增加（2m/s），设置v_sp，如果速度大于v_sp时候，认为变形完成重置\_was_in_transition和\_asp_after_transition

4. 更新属性\_is_tecs_runing=run_tecs，

   如果run_tecs为false，设置属性\_reinitialize_tecs为true，表示下次运行TECS需要初始化。

   根据\_reinitialize_tecs的值决定是否初始化\_tecs

5. 判断\_mTecs是否可用

   ​	是：

   ​		设置flightPathAngle角度

   ​		建立limitOverride对象，

   ​		如果飞行器引擎故障：限制俯仰-1~5

   ​		如果是爬升模式：限制最小的爬升角度

   ​		否则：不使用

   ​		如果pitch_max_special，设置最大的俯仰角度

   ​		否则：不设置

   ​		调用\_mTecs.updateAltitudeSpeed(),传入flightPAthAngle,高度，设定高度，当前空速，目标空速，模式，限制

   ​	否：

   ​		如果飞行器引擎故障：限制俯仰-1~5

   ​		除了着陆模式外，其他需要设置低速保护\_tecs

   ​		如果 是尾起的飞行器，我们需要在旋翼和固定翼模式中切换

   ​		使用\_tecs.update_pitch_throttle()更新油门俯仰，使用get_tecs_state取出得到的控制输出，再把对应信息赋给tecs_status_s结构体的实例，通过uORB以ORB_ID(tecs_status)输出

##QgroundControl安装

下载网站：

https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html

依赖包补丁（有些库没有，需要自己安装）：

``sudo apt-get install espeak libespeak-dev libudev-dev libsdl2-2.0-0``

然后按教程运行：

```shell
tar jxf QGroundControl.tar.bz2
cd qgroundcontrol
./qgroundcontrol-start.sh
```

