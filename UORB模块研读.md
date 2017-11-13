# UORB模块研读

## uORB函数解析：

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

#### 发布更新

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

#### Note:

1. 必须要先orb_advertise()+orb_subscribe()后，才能使用orb_copy()
2. 所有主题在Firmware/Build_px4fmu-v2_default/Src_Modules/uORB/Topics
3. 也可以到Firmware/Msg下找到

## uORB原理解析：

![0001](C:\Users\87095\Desktop\pix_study\0001.jpg)

####实现方式：

1. 首先模块入口是uORBMain.cpp，这里会创建唯一的MasterNode实例，负责内部的设备节点的控制
2. MasterNode简洁：继承自CDev，是一个字符设备，linux中设备都是文件，所以会在对应路径下生成文件，主要对Cdev的ioctl函数进行多态实现，类内部维护一个Nodemap的数据结构（后面有详细简绍），所有的DeviceNode都会加入其中管理。
3. 当外部调用uORB的方法时候，实际间接调用了uORB::Manger类的方法，也可以理解做Manger是外部函数的管理入口，MasterNode是内部节点的管理器。
4. 外部函数调用uORB时候都会通过uORB::Manger::get_instance()这个函数获取Manger的实例指针。这个函数是静态函数，根据判断命名空间中_Instance指针是否为空，确保全nameespace中只有这么一个Manger实例。
5. 实际节点，DeviceNode，继承自CDev，每一个字符设备都会生成对应的文件。这个类重载了open,close,read,wirte,ioctl，还有其他一些函数。
6. 当外部调用orb_advertise时，Manger会做如下的事情：
   1. 以发布者的身份调用node_open函数，最后返回对应文件的文件描述符fd。先确认一下是否这个主题对应实例的序号已经被发布过，如果是，会重新发布一下，否则（对应文件不存在，也就是这个主题对应ID的DeviceNode不存在），会调用node_advertise函数进行DeviceNode实例的创建。
   2. （调用node_open函数）
   3. 通过fd调用ioctl函数，返回DeiceNode对象的指针。
   4. 发布初始化的数据
7. 当外部调用orb_publish时
   1. 根据传入的DeviceNode的指针，调用这个类的静态的发布数据的方法。就是将数据写入DeviceNode对象的属性中。
8. 当外部调用orb_Subsctibe时，
   1. 使用Manger类的node_open函数，返回对应主题对应实例的文件描述符，其他信息都可以通过调用被DeviceNode多态实现过的ioctl函数获得。
   2. 取消订阅就将文件描述符关闭就可以了。

### uORB

*这个文件中所有的方法的实现方式都是调用ORB::Manger的方法，只是一个扩展接口的包装文件*

*因为这个没有在任何命名空间或者类中，这里将具体的ORB::Manger操作封装，使得其他类可以调用*

####orb_advertise(*meta, *data) orb_advert_t

参数列表含义：主题名称，初始数据

 作为发布者做广播，如果正常，返回一个可以用来发布这个主题的句柄

####orb_advertise(*meta, *data, *instance, priority) orb_advert_t

参数列表含义：主题名称，初始数据，实例ID的指针，实例优先级

一个主题下可以有多个实例，每个实例根据handle区分，最多5个

####orb_pubshlist_auto(*meta, *handle, *data, *instance, priority) int

参数列表含义：同上

根据handle是否有效发布数据

####orb_publish(*meta, handle, *data) int

参数列表含义：主题名称，主题的实例句柄，发布的数据

 数据发布是原子操作，任何等待更新的订阅者都会被通知。其他没有等待的可以使用orb_check/orb_stat函数检查更新

####orb_subscribe(*meta) int

参数列表含义： 主题名称

返回文件描述符，可以使用poll函数等待主题更新，就像使用topic_read/orb_check/orb_stat一样

​	即使这个主题没有发布订阅也可能成功，这种情况下，poll，复制它的值什么的操作都会失败，直到这个主题被发布未知。

​	如果系统不知道这样一个主题，那么会订阅失败

####orb_subscribe_muti(*meta,  instance) int

参数列表含义： 主题名称， 实例ID

####orb_unsubscribe(handle) int

参数列表含义：实例句柄

####orb_copy(*meta, handle, *buffer) int

参数列表含义：主题名称，实例句柄，缓冲区

这是唯一个可以重置内部主题是否更新标志的函数。一旦使用poll或者orb_check返回可用更新，必须使用这个方法更新参数

####orb_check(handle, *updated) int

参数列表含义：实例的句柄，是否被更新的标志

检查在上次orb_copy后这个主题是否被重新发布过

​	更新以每个句柄为基础进行跟踪; 

​	这个调用将一直返回true直到使用相同的句柄调用orb_copy。

​	由于stat和copy之间的竞争窗口可能导致错过更新，所以此接口应优先于调用orb_stat。

####orb_stat(handle, *time) int

返回上一次更新主题的时间

####orb_exists(*meta, instance) int 

检查主题是否存在

####orb_group_count(*meta) int

返回主题的实例数量

####orb_priority(handle, *priority) int

返回句柄的优先级

####orb_set_interval(handle, interval) int

设置最小的更新间隙



### uORBCommon

### ORBSet

- insert(*node_node) void
- find(*node_name) bool
- erase(*node_name) bool
- unlinkNext(Node*) void

实现方式与ORBMap一毛一样，唯一的区别就是这个类维护的队列中的node节点是只有节点名字，没有节点设备。

### ORBMap

- insert(*node_name, *node) void
- find(*node_name) bool
- *get(\*node) uORB::DeviceNode
- unlinkNext(*a) void

类的实现流程：*(太简单了吧，过分)*

​	类中维护一个队列，使用对象top,end俩个指针标识队列，

​	当有新的数据结点，加入到end后面，查找、获取是从头到尾顺序查找队列，使用strcmp()比较node_name

### uORBDevices_nuttx

```c++
namespace uORB
{
class DeviceNode;  //具体的节点
class DeviceMaster; //节点管理
}
```

#### DeviceNode

继承device::CDev(一切字符设备的基类)

#####DeviceNode(*meta, *name, *path, priority)

##### open(struct file *filp) int virtual

设备节点打开：算是虚拟的设备文件虽然继承与CDev类

根据文件信息，读取权限：（filp记录了这次打开文件的信息，比如权限，PID等数据）

​	如果文件写入权限，认为是发布者，设备节点属性发布者设为当前PID，使用CDev::open打开（但实际这个类也没干什么，可以忽略这一句），

​	如果是读取权限，认为是订阅者，申请SubscriberData给filp->f_priv，记录这个订阅对象fd所私有的数据，并且告诉对象属性又一个订阅者订阅了你

##### close(*filp) int virtual

设备节点关闭：

​	如果是发布者调用的关闭，将属性发布者置为0

​	如果是订阅者调用，获取filp中的SubscriberData，从调用列表中删除，移除内部调用者

##### read(*filp, *buffer, buflen) ssize_t virtual

将类中数据_data读入到buffer中

##### write(*filp, *buffer, buflen) ssize_t virtual

将buffer的数据读入到类的_data属性中

##### ioctl(*filp, cmd, arg) int virtual

根据cmd的值调整arg的类型，输出对应数据属性

##### publish(*meta, handle, *data) ssize_t static

这是静态函数，所有实例公用一个

根据传入的句柄（即DeviceNode的指针），将data的数据写入类的对象属性_data中

//以下3个函数应该是设计远程调用IChannel接口的，这里不做讨论，因为没看懂，也没有示例使用

##### process_add_subscription(rateInHz) int16_t

##### process_remove_subscription() int16_t

##### process_received_message(length, *data) int16_t

将接受的数据写入_data属性中

##### add_internal_subscriber() void

##### remove_internal_subscriber() void

##### is_published() bool

//protected:

##### poll_state(*filp) pollevent_t virtual

判断主题是否推送给订阅者，需要推送返回POLLIN，否则0

##### poll_notify_one(struct pollfd *fds,pollevent_t events )

对于fds来说，如果有更新，调用Cdev::poll_notify_one()函数进行更新

##### appears_updated(SubscriberData *sd) bool

根据sd的信息，判断是否有更新（发布者调用这个函数）

##### update_deferred() void

监控拉的需求

```c++
private:
	struct SubscriberData {
		unsigned  generation; /**< last generation the subscriber has seen */
		unsigned  update_interval; /**< if nonzero minimum interval between updates */
		struct hrt_call update_call;  /**< deferred wakeup call if update_period is nonzero */
		void    *poll_priv; /**< saved copy of fds->f_priv while poll is active */
		bool    update_reported; /**< true if we have reported the update via poll/check */
		int   priority; /**< priority of publisher */
	};

	const struct orb_metadata *_meta; /**< object metadata information */
	uint8_t     *_data;   /**< allocated object buffer */
	hrt_abstime   _last_update; /**< time the object was last updated */
	volatile unsigned   _generation;  /**< object generation count */
	pid_t     _publisher; /**< if nonzero, current publisher */
	const int   _priority;  /**< priority of topic */
	bool _published;  /**< has ever data been published */
```



#### DeviceMaster

继承device::CDev,管理设备节点的类

##### DeviceMaster(Flavor f)

##### \*GetDeviceNode(*node_name) uORB::DeviceNode

##### ioctl(*filp, cmd,arg ) int virtual

```c++
private:
	Flavor      _flavor;  //是PUBSUB还是OBJ
	static ORBMap _node_map; //所有的DeviceNode设备都在其中
```

###uORBDevices_posix

nuttx是在px4在nuttx操作系统下运行的，这个应该是裸机运行的。

### uORBManager

这个类管理着每一个UORB主题和节点，也实现了UORb的API

继承自IChannelRxHandler类，类中函数大多都在前面见过，我只大概描述一下实现，如果有必要的话

##### get_instance() Manager* static

获取生成的uORBManager实例，静态的，单个进程中只生成一个实例

##### orb_advertise(\*meta,\*data) orb_advert_t

##### orb_advertise_multi(*meta, *data, *instance, priority) orb_advert_t

根据主题和实例编号获取文件描述符，根据文件描述符获取当前发布者的指针并返回发布者的指针

##### orb_publish(*meta, handle, *data) int

发布数据，返回成功与否

实现：

​	调用DevicNode的publish方法，DeviceNode::publish是静态方法，调用handle->write()函数（write会使属性_generation++，标记数据版本的），将数据写入文件中

##### orb_subscribe(*meta) int

##### orb_subscribe_multi(*meta, instance) int

订阅的原理就是打开以只读打开文件，返回fd

（一个文件可以被打开多次，返回的fd都不一样，fd对应这file *filp这个指针（也就是你在DeivceNode类中参数filp一样，这个指针的结构体参数比较全），所以可以打开一次，有多个备份的filp）

##### orb_unsubscribe(handle) int

##### orb_copy(*meta, handle, buffer) int

从handle中读取相应长度的数据到buffer中

##### orb_check(handle, *updated) int

使用ioctl方法（ioctrl方法有DeviceNode类实现）

这个检查的办法是对比filp中维护的SubscriberData数据和对象中属性_generation是否一致，虽然filp有多个，但是文件（或者说DeviceNode对象只有一个），所以可以进行判断。（更新版本是在写入时候_generation++）

##### orb_stat(handle, *time) int

使用ioctl方法

##### orb_exists(*meta, instance) int

主题的对应实例是否有

​	根据参数生成路径，然后试图打开对应文件

##### orb_priority(handle, *priority) int

使用ioctl方法

##### orb_set_interval(handle, interval) int

使用ioctl方法

##### set_uorb_communicator(*comm_channel) void

#####get_uorb_communicator() IChannel*

##### is_remote_subscriber_present(*messageName) bool

private:

##### node_advertise(*meta, *instance, priority) int

通过调用控制设备MasterNode的ioctl函数（被重载过），让控制文件去创建对应的DeviceNode对象（有了对象，对应路径下就有了文件）

##### node_open(Flavor, *meta, *data, advertiser, *instance, priority) int

根据订阅发布的属性打开对应的设备文件，返回文件描述符

这个函数让你屏蔽是否存在这样的设备文件，根据你的需求去解决。

如果你是订阅者，存在，直接返回。不存在，那就创建一个，方法同下。

如果你是发布者，存在，关闭fd。重新打开一次，（这时node_advertise调用将自动进入下一个空闲条目）不存在，调用node_advertise发布创建一个DeviceNode对象（有了对象自然会有文件）

​	实现：

​	判断输入的逻辑合法性

​	根据订阅者还是发布者，选择对应读取权限打开文件

​	如果是发布者&&打开成功，说明这个文件存在

​	关闭文件重新调用node_advertise发布再打开文件

​	返回文件描述符fd

##### process_add_subscription(*messageName, msgRate) int16_t

处理订阅的回调函数，Manger类实现的

实现逻辑：

​	将主体加入订阅主题的集合中

​	根据对应信息获取路径，再从设备管理器DeviceMaster获取对应的设备节点

​	具体设备节点处理订阅频率

##### process_remove_subscription(*messageName) int16_t

处理移除订阅的回调函数，Manger类实现

实现逻辑：

​	将主题移除主题集合

​	获取对应的设备节点

​	具体设备节点处理移除事件

#####process_received_message(*message, length, *data) int16_t

处理接受信息的回调函数，Manger类实现

实现逻辑：

​	获取对应的设备节点

​	具体的设备节点处理接受的信息

````c++
	static Manager *_Instance; //全局唯一的管理器
	// the communicator channel instance.
	uORBCommunicator::IChannel *_comm_channel;
	ORBSet _remote_subscriber_topics;
````



### uORBUtils

工具类Utils中只有2个同名的静态函数

```c++
int node_mkpath(char* buf, Flavor f, const struct orb_metadata *meta, int *instance)
int node_mkpath(char* buf, Flavor f, const char* orbMsgName)
```

参数含义：

​	buf：输出路径的字符串

​	f:	标志，是发布订阅还是作为参数（PUBSUB/PARAM）

​	meta: 通过meta->o_name = orbMsgName

​	instance: 如果是多个实例，对应相应的，否则默认为0

​	生成路径为/(obj?param)/orbMsgName/instance ,应该是节点设备的文件路径

### uORBCommunicator

```c++
namespace uORBCommunicator
{
 //俩个类都为纯虚类，可以认为java中的接口类
class IChannel; // 可以理解为这个是客户端
class IChannelRxHandler; //这个是服务器端
}
```

#### IChannel

 *启用远程订阅的接口，接口的实现类需要管理通信信道，快速RPC/TCP/IP*

##### add_subscription(*messageName, msgRateInHz) int16_t

参数列表含义：主题名称（全局唯一），最大消息更新速率

通知有兴趣的远程实体订阅消息的接口

##### remove_subscription(*messageName) int16_t

通知远程实体移除订阅的接口

##### register_handler(IChannelRxHandler *handler) int16_t

注册消息句柄

##### send_message(*messageName, length, *data) int16_t

通过通讯链接发送数据

####IChannelRxHandler

通过通信链路回调的类

##### process_add_subscription(*messageName, msgRateInHz) int16_t

处理从远程接受添加的接口函数

##### process_remove_subscription(*messageName) int16_t

处理远程移除订阅的接口

##### process_received_message(*messageName, length, *data) int16_t

处理远程发送数据的类