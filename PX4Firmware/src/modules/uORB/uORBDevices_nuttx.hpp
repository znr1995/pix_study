/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef _uORBDevices_nuttx_hpp_
#define _uORBDevices_nuttx_hpp_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ORBMap.hpp"
#include "uORBCommon.hpp"


namespace uORB
{
class DeviceNode;
class DeviceMaster;
}

/**
 * Per-object device instance.
 */
class uORB::DeviceNode : public device::CDev
{
public:
	/**
	 * Constructor
	 */
	DeviceNode
	(
		const struct orb_metadata *meta,
		const char *name,
		const char *path,
		int priority
	);

	/**
	 * Destructor
	 */
	~DeviceNode();

	/**
	 * Method to create a subscriber instance and return the struct
	 * pointing to the subscriber as a file pointer.
	 * 创建订阅实例并且返回订阅文件的指针
	 */
	virtual int  open(struct file *filp);

	/**
	 * Method to close a subscriber for this topic. 关闭这个主题的订阅
	 */
	virtual int   close(struct file *filp);

	/**
	 * reads data from a subscriber node to the buffer provided.从订阅的节点读取数据到buffer
	 * @param filp
	 *   The subscriber from which the data needs to be read from.
	 * @param buffer
	 *   The buffer into which the data is read into.
	 * @param buflen
	 *   the length of the buffer
	 * @return
	 *   ssize_t the number of bytes read.
	 */
	virtual ssize_t  read(struct file *filp, char *buffer, size_t buflen);

	/**
	 * writes the published data to the internal buffer to be read by
	 * subscribers later. 从buffer中写入数据到文件中
	 * @param filp
	 *   the subscriber; this is not used.
	 * @param buffer
	 *   The buffer for the input data
	 * @param buflen
	 *   the length of the buffer.
	 * @return ssize_t
	 *   The number of bytes that are written
	 */
	virtual ssize_t   write(struct file *filp, const char *buffer, size_t buflen);

	/**
	 * IOCTL control for the subscriber. IO控制
	 */
	virtual int   ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Method to publish a data to this node. 发布一个数据到这个节点
	 */
	static ssize_t publish
	(
		const orb_metadata *meta,
		orb_advert_t handle,
		const void *data
	);

	/**
	 * processes a request for add subscription from remote 处理远程添加订阅的请求
	 * @param rateInHz
	 *   Specifies the desired rate for the message.
	 * @return
	 *   0 = success
	 *   otherwise failure.
	 */
	int16_t process_add_subscription(int32_t rateInHz);

	/**
	 * processes a request to remove a subscription from remote. 处理远程移除订阅的请求
	 */
	int16_t process_remove_subscription();

	/**
	 * processed the received data message from remote. 处理接受从远程数据
	 */
	int16_t process_received_message(int32_t length, uint8_t *data);

	/**
	  * Add the subscriber to the node's list of subscriber.  If there is
	  * remote proxy to which this subscription needs to be sent, it will
	  * done via uORBCommunicator::IChannel interface.
	  	添加订阅者到订阅者列表，如果这个订阅需要通过代理发送，需要实现uORBCommunicator::IChannel接口
	  * @param sd
	  *   the subscriber to be added.
	  */
	void add_internal_subscriber();

	/**
	 * Removes the subscriber from the list.  Also notifies the remote
	 * if there a uORBCommunicator::IChannel instance.
	 * 移除订阅者。
	 * @param sd
	 *   the Subscriber to be removed.
	 */
	void remove_internal_subscriber();

	/**
	 * Return true if this topic has been published. 这个主题是否被发布
	 *
	 * This is used in the case of multi_pub/sub to check if it's valid to advertise
	 * and publish to this node or if another node should be tried. */
	bool is_published();

protected:
	virtual pollevent_t poll_state(struct file *filp);
	virtual void poll_notify_one(struct pollfd *fds, pollevent_t events);

private:
	struct SubscriberData {
		unsigned  generation; /**< last generation the subscriber has seen 上一次订阅 */
		unsigned  update_interval; /**< if nonzero minimum interval between updates 最小更新时间 */
		struct hrt_call update_call;  /**< deferred wakeup call if update_period is nonzero 如果update_period不为0，更新推迟*/
		void    *poll_priv; /**< saved copy of fds->f_priv while poll is active 保存fd的f_priv */
		bool    update_reported; /**< true if we have reported the update via poll/check 如果更新过状态，置为true */
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
};

/**
 * Master control device for ObjDev.
 *
 * Used primarily to create new objects via the ORBIOCCREATE
 * ioctl.
 */
class uORB::DeviceMaster : public device::CDev
{
public:
	DeviceMaster(Flavor f);
	virtual ~DeviceMaster();

	static uORB::DeviceNode *GetDeviceNode(const char *node_name);
	virtual int   ioctl(struct file *filp, int cmd, unsigned long arg);
private:
	Flavor      _flavor;
	static ORBMap _node_map;
};



#endif
