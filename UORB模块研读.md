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

### 乱乱七八糟

uORB namespace中有类*ORBMap* 和 *DeviceNode*

猜测：

​	Map类负责管理维护数据结构，Node才是具体的存储类

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

### Pulication

### Subscription

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

##### close(*filp) int virtual

##### read(*filp, *buffer, buflen) ssize_t virtual

##### write(*filp, *buffer, buflen) ssize_t virtual

##### ioctl(*filp, cmd, arg) int virtual

##### publish(*meta, handle, *data) ssize_t static

##### process_add_subscription(rateInHz) int16_t

##### process_remove_subscription() int16_t

##### process_received_message(length, *data) int16_t

##### add_internal_subscriber() void

##### remove_internal_subscriber() void

##### is_published() bool

//protected:

##### poll_state(*filp) pollevent_t virtual

##### poll_notify_one(struct pollfd *fds,pollevent_t events )

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

private: // private class methods.

	SubscriberData    *filp_to_sd(struct file *filp)
	{
		SubscriberData *sd = (SubscriberData *)(filp->f_priv);
		return sd;
	}

	bool    _IsRemoteSubscriberPresent;
	int32_t _subscriber_count;

	/**
	 * Perform a deferred update for a rate-limited subscriber.
	 */
	void      update_deferred();

	/**
	 * Bridge from hrt_call to update_deferred
	 *
	 * void *arg    ORBDevNode pointer for which the deferred update is performed.
	 */
	static void   update_deferred_trampoline(void *arg);

	/**
	 * Check whether a topic appears updated to a subscriber.
	 *
	 * @param sd    The subscriber for whom to check.
	 * @return    True if the topic should appear updated to the subscriber
	 */
	bool      appears_updated(SubscriberData *sd);

	// disable copy and assignment operators
	DeviceNode(const DeviceNode &);
	DeviceNode &operator=(const DeviceNode &);
```



#### DeviceMaster

继承device::CDev,管理设备节点的类

##### DeviceMaster(Flavor f)

##### \*GetDeviceNode(*node_name) uORB::DeviceNode

##### ioctl(*filp, cmd,arg ) int virtual

```c++
private:
	Flavor      _flavor;
	static ORBMap _node_map;
```

###uORBDevices_posix

### uORBDevices

### uORBManager

### uORBTest_UnitTest

### uORBUtils

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