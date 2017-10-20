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

- calulateSlopeValues() void private

  更新H1,H0,d1,根据log（H0/H1）的比例调整 d1 / d1+ delta d的比例，更新其他参数

- getLandingSlopeRelativeAltitude(wp_landing_distance) float

  返回在距离落航点的着陆坡上点的相对高度，调用多参数的同名函数

- getLandingSlopeRelativeAltitudeSave(wp_landing_distance,bearing_lastwp_currwp,bearing_airplane_currwp) float public

  检查飞行器是否在航点上来避免爬升

- getLandingSlopeAbsoluteAltitude(wp_landing_distance,wp_altitude) float

  返回在距离落航点的着陆坡上点的绝对高度

基本上示例都是这么一个结构
- getLandingSlopeAbsoluteAltitudeSave(wp_landing_distance,bearing_last_wp_currwp,bearing_airplane_currwp) float

  检查飞行器是否在航点上来避免爬升

- getLandingSlopeRelativeAltitude(wp_landing_distance,horizontal_slope_displacement,landing_slope_angle_rad) float static

  返回h_flare.rel的高度，返回在距离落航点的着陆坡上点的相对高度

- getLandingSlopeAbsoluteAltitude(wp_landing_distance,wp_landing_altitude,horizontal_slope_displacement,landing_slope_angle_rad) float static

  返回h_flare.rel + H1的高度，返回在距离落航点的着陆坡上点的绝对高度

- getLandigSlopeWPDistance(slope_altitude,wp_landing_altitude,horizontal_slope_displacement,landing_slope_angle_rad) float static

  给定降落高度，返回距离预定降落点的距离

- getFlareCurveRelativeAltitudeSave(wp_distance, bearing_lastwp_currwp, bearing_airplane_currwp) float

- getFlareCurveRelativeAltitudeSave(wp_distance, bearing_lastwp_currwp, bearing_airplane_currwp, wp_altitude) float

  获取Flare曲线的相对高度

- updata(landing_slope_angle_Rad_new, flare_relative_alt_new, mmotor_lim_relative_alt_new, H1_virt_new) void

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

- FixedwingPositionControl:

  使用param_find()函数定位所有的参数,赋初值


- start() int static  public

  开始控制

- task_running() bool

  任务的状态

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

- get_waypoint_heading_distance(heading, distance, wp_prev, wp_nest) void

  基于方向和当前的距离获取下一个waypoint，

  如果是flag_init，根据角度和距离获取前后航点

  否则，根据前一帧的前后航点说确定的直线，根据预先定义的距离，获取新的前后航点。

- get_terrain_altitude_landing() float

  返回降落点的地形高度估计，在降落的时候：根据设定点的高度或者有可能的话使用地形估计

  如果地形评估不合法，直接使用设定点的高度

  如果使用地形评估，并切换到地形评估，一直使用到降落为止

- get_terrain_altitude_takeoff() float

  返回起飞地形或者返回高度在地形估计不可用的情况下

- in_takeoff_situation() bool

  检查是否在合法的起飞地点

  实现：

  ​	判断油门输入的时效性，油门输入是否到门限，高度是否符合起飞的条件

- do_takeoff_help(hold_altitude, pitch_limit_min) 

  定高模式下的起飞帮助

  如果in_takeoff_situation()，给hold_alt，pitch_limit_min赋值，否则只给pitch_limit_min

- update_desired_altitude(dt) bool

  根据俯仰输入更新期望高度,参数为Time step，单位时间

  实现：

  ​	0.06 ~ -0.06 视为0， 其余1 ~ 0.06 | -0.06 ~  -1 部分按比例，更新高度 = 原高度 + 最大变化速率*dt * pitch （俯仰指数）

  ​	如果是垂直起降飞机，在旋翼状态/转化状态，高度是当前高度。

- control_position(global_pos, ground_speed, pos_sp_triplet) bool

  控制位置

- get_tecs_pitch() float

  如果mTecs可行，返回getPitchSetpoint()否则返回_tecs.get_pitch_demand()

- get_tecs_thrust() float

  同上

- get_demanded_airspeed() float

  获取需要的空速大小

  实现算法如下：

  ​	判断输入油门的大小，设定可变化速度为当前速度距最大/最小速度的差，airspeed_trim字面上是修剪的空速速度（即当前速度）

  ​		如果油门输入小于0.5，输出速度=最小速度+可变化速度×油门输入值×2；（如果油门输入值0.5，则速度不变化，在输入值小于0.5的情况下，油门输入×2 < 1）

  ​		如果油门大于0.5，输出速度 = 当前速度 + 可变化速度 × （油门输入值-0.5）*2（减去0.5的基础油门）

- calculate_target_airspeed(airspeed_demand) float

  计算目标空速，没看懂想干嘛

  实现：

  ​	当前空速根据全局变量\_airspeed_valid是否合法，合法取当前控制状态的空速，否则取中值。

  ​	目标空速= airspeed_demand+\_groundsped_undershoot(期望速度+地速)，再对目标空速进行限制，使其在最大最小之间。

  ​	\_airspeed_error这个全局变量是目标速度和当前空速之差，应该是用来调整速度的。

  ​	返回目标速度

- **calualte_gndspeed_undershoot(current_pos, ground\_speed_2d, pos_sp_triplet) void **

  计算对地下降速度，

  ​	根据distance和delta_altitude计算期望的速度，与当前速度之差就是应该对地下降的速度

- task_main_trampoline()

  构建FixedwingPositionControl类，赋给l1_control命名空间的g\_control指针，开始运行``task_main()``函数，然后释放。

- task_main() void

  先订阅需要的信息，设置更新速率等。

  fds[2]，fds[0]负责检查参数更新，fds[1]负责位置更新

  while

  ​	px4_poll检查是否有更新，更新control_mode,status

  ​	如果参数更新，获取更新参数

  ​	如果位置更新，control_statue,setpoint,sensor_combined,manual_control_setpoint更新，运行control_position()函数，如果控制成功，通过uorb机制输出控制结果信息，最后如果导航信息很久没有更新，输出导航信息没看懂

- reset_takeoff_state() void

- reset_landing_state() void

- tecs_update_pitch_trottle() void

  一个调用实现TECS的包装器函数(mTECS是只通过参数启用)

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

