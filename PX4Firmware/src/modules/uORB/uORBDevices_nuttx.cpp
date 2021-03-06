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

#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <nuttx/arch.h>
#include "uORBDevices_nuttx.hpp"
#include "uORBUtils.hpp"
#include "uORBManager.hpp"
#include "uORBCommunicator.hpp"
#include <stdlib.h>

uORB::ORBMap uORB::DeviceMaster::_node_map;

uORB::DeviceNode::DeviceNode
(
	const struct orb_metadata *meta,
	const char *name,
	const char *path,
	int priority
) :
	CDev(name, path),
	_meta(meta),
	_data(nullptr),
	_last_update(0),
	_generation(0),
	_publisher(0),
	_priority(priority),
	_published(false),
	_IsRemoteSubscriberPresent(false),
	_subscriber_count(0)
{
	// enable debug() calls
	_debug_enabled = true;
}

uORB::DeviceNode::~DeviceNode()
{
	if (_data != nullptr) {
		delete[] _data;
	}

}

int
uORB::DeviceNode::open(struct file *filp)
{
	int ret;
	/**
	 * 流程：
	 * 判断是否是发布者的权限，
	 * 判断是否之前有同样的发布者
	 * 如果没有，将发布者属性设备当前进程PID
	 * 使用CDev类实际打开文件
	**/
	/* is this a publisher? 是否是发布者 */
	if (filp->f_oflags == O_WRONLY) {

		/* become the publisher if we can */
		lock();

		if (_publisher == 0) {
			_publisher = getpid(); //将发布者属性置为当前进程的PID
			ret = OK;

		} else {
			ret = -EBUSY;   //否则返回错误，已经被发布过了
		}

		unlock();

		/* now complete the open */
		if (ret == OK) {
			ret = CDev::open(filp);  //具体打开文件交给Cdev的open函数

			/* open failed - not the publisher anymore */
			if (ret != OK) {
				_publisher = 0;
			}
		}

		return ret;
	}

	/* is this a new subscriber? */
	/**
	 * 实现流程：
	 * 声明SubscriberData的结构体，初始化赋值后将指针赋给filp的f_priv
	 * 打开的具体步骤交给CDev::open
	 * 添加内部订阅
	 * 返回ret
	 * */
	if (filp->f_oflags == O_RDONLY) {

		/* allocate subscriber data */
		SubscriberData *sd = new SubscriberData;

		if (nullptr == sd) {
			return -ENOMEM;
		}

		memset(sd, 0, sizeof(*sd));

		/* default to no pending update */
		sd->generation = _generation;

		/* set priority */
		sd->priority = _priority;

		filp->f_priv = (void *)sd;

		ret = CDev::open(filp);

		add_internal_subscriber();

		if (ret != OK) {
			delete sd;
		}

		return ret;
	}

	/* can only be pub or sub, not both */
	return -EINVAL;
}

int
uORB::DeviceNode::close(struct file *filp)
{
	/**
	 * 关闭设备节点的文件：
	 * 如果是发布者调用，将发布者置为0
	 * 否则：就是订阅者：获取SubscriberData数据，
	 * 从调用列表中移除sd的更新调用
	 * 移除内部调用者，删除sd的信息
	 * /
	/* is this the publisher closing? 如果是发布者关闭文件，将类的属性发布者设为0 */
	if (getpid() == _publisher) {
		_publisher = 0;

	} else {
		SubscriberData *sd = filp_to_sd(filp); 

		if (sd != nullptr) {
			hrt_cancel(&sd->update_call);  //从调用列表中移除sd的更新调用
			remove_internal_subscriber();  //移除内部调用者，删除sd的信息
			delete sd;
			sd = nullptr;
		}
	}

	return CDev::close(filp);
}

ssize_t
uORB::DeviceNode::read(struct file *filp, char *buffer, size_t buflen)
{
	SubscriberData *sd = (SubscriberData *)filp_to_sd(filp);

	/* if the object has not been written yet, return zero 数据为空，对象还没有被写入 */
	if (_data == nullptr) {
		return 0;
	}

	/* if the caller's buffer is the wrong size, that's an error */
	if (buflen != _meta->o_size) {
		return -EIO;
	}

	/*
	 * Perform an atomic copy & state update 状态更新要求原子操作
	 */
	irqstate_t flags = irqsave();

	/* if the caller doesn't want the data, don't give it to them */
	if (nullptr != buffer) {
		memcpy(buffer, _data, _meta->o_size);  //将数据从_data拷贝到buffer
	}

	/* track the last generation that the file has seen 设置订阅数据的版本*/
	sd->generation = _generation;

	/* set priority 优先级*/
	sd->priority = _priority;

	/*
	 * Clear the flag that indicates that an update has been reported, as
	 * we have just collected it. 重置订阅信息是否被更新通知订阅者
	 */
	sd->update_reported = false;

	irqrestore(flags);

	return _meta->o_size;
}

ssize_t
uORB::DeviceNode::write(struct file *filp, const char *buffer, size_t buflen)
{
	/*
	 * Writes are legal from interrupt context as long as the
	 * object has already been initialised from thread context.
	 *
	 * Writes outside interrupt context will allocate the object
	 * if it has not yet been allocated.
	 *
	 * Note that filp will usually be NULL.
	 */
	if (nullptr == _data) {
		if (!up_interrupt_context()) {

			lock();

			/* re-check size */
			if (nullptr == _data) {
				_data = new uint8_t[_meta->o_size];
			}

			unlock();
		}

		/* failed or could not allocate */
		if (nullptr == _data) {
			return -ENOMEM;
		}
	}

	/* If write size does not match, that is an error 如果写入的大小与预计不符合，返回错误 */
	if (_meta->o_size != buflen) {
		return -EIO;
	}

	/* Perform an atomic copy. irqsave是加锁的吧 */
	irqstate_t flags = irqsave();
	memcpy(_data, buffer, _meta->o_size);
	irqrestore(flags);

	/* update the timestamp and generation count */
	_last_update = hrt_absolute_time();
	_generation++;  //版本叠加

	/* notify any poll waiters 通知等待POLLIN的用户/订阅者 */
	poll_notify(POLLIN);

	_published = true;

	return _meta->o_size;
}

int
uORB::DeviceNode::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	SubscriberData *sd = filp_to_sd(filp);

	switch (cmd) {
	case ORBIOCLASTUPDATE:
		*(hrt_abstime *)arg = _last_update;
		return OK;

	case ORBIOCUPDATED:
		*(bool *)arg = appears_updated(sd);
		return OK;

	case ORBIOCSETINTERVAL:
		sd->update_interval = arg;
		return OK;

	case ORBIOCGADVERTISER:
		*(uintptr_t *)arg = (uintptr_t)this;
		return OK;

	case ORBIOCGPRIORITY:
		*(int *)arg = sd->priority;
		return OK;

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
uORB::DeviceNode::publish
(
	const orb_metadata *meta,
	orb_advert_t handle,
	const void *data
)
{
	uORB::DeviceNode *DeviceNode = (uORB::DeviceNode *)handle;
	int ret;

	/* this is a bit risky, since we are trusting the handle in order to deref it
		如果实例句柄的主题不是当前主题，返回错误 */
	if (DeviceNode->_meta != meta) {
		errno = EINVAL;
		return ERROR;
	}

	/* call the DeviceNode write method with no file pointer 写入不需要filp */
	ret = DeviceNode->write(nullptr, (const char *)data, meta->o_size);

	if (ret < 0) {
		return ERROR;
	}

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return ERROR;
	}

	/*
	 * if the write is successful, send the data over the Multi-ORB link 写入成功，发生数据到多ORB链接，这里有个问题
	 */
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr) {
		if (ch->send_message(meta->o_name, meta->o_size, (uint8_t *)data) != 0) {
			warnx("[uORB::DeviceNode::publish(%d)]: Error Sending [%s] topic data over comm_channel",
			      __LINE__, meta->o_name);
			return ERROR;
		}
	}

	return OK;
}

pollevent_t
uORB::DeviceNode::poll_state(struct file *filp)
{
	SubscriberData *sd = filp_to_sd(filp);

	/*
	 * If the topic appears updated to the subscriber, say so.
	 * 将主题推送给订阅者
	 */
	if (appears_updated(sd)) {  //如果有更新，返回true
		return POLLIN;
	}

	return 0;
}

void
uORB::DeviceNode::poll_notify_one(struct pollfd *fds, pollevent_t events)
{
	SubscriberData *sd = filp_to_sd((struct file *)fds->priv);

	/*
	 * If the topic looks updated to the subscriber, go ahead and notify them.
	 */
	if (appears_updated(sd)) {
		CDev::poll_notify_one(fds, events);
	}
}

bool
uORB::DeviceNode::appears_updated(SubscriberData *sd)
{
	/* assume it doesn't look updated */
	bool ret = false;

	/* avoid racing between interrupt and non-interrupt context calls */
	irqstate_t state = irqsave();

	/* check if this topic has been published yet, if not bail out */
	if (_data == nullptr) {//没有发布数据，直接输出false
		ret = false;
		goto out;
	}

	/*
	 * If the subscriber's generation count matches the update generation
	 * count, there has been no update from their perspective; if they
	 * don't match then we might have a visible update.
	 * 通过数据的代数判断是否需要更新，是while循环
	 */
	while (sd->generation != _generation) {

		/*
		 * Handle non-rate-limited subscribers. 如果更新速率没有延迟，直接返回true
		 */
		if (sd->update_interval == 0) {
			ret = true;
			break;
		}

		/*
		 * If we have previously told the subscriber that there is data,
		 * and they have not yet collected it, continue to tell them
		 * that there has been an update.  This mimics the non-rate-limited
		 * behaviour where checking / polling continues to report an update
		 * until the topic is read.
		 * 我们之前告诉订阅者有更新，但是订阅者并没有接受更新
		 */
		if (sd->update_reported) {
			ret = true;
			break;
		}

		/*
		 * If the interval timer is still running, the topic should not
		 * appear updated, even though at this point we know that it has.
		 * We have previously been through here, so the subscriber
		 * must have collected the update we reported, otherwise
		 * update_reported would still be true.
		 * 如下情况是因为更新速率设置的原因，我们即使知道数据更新了，也不能告诉订阅者，需要订阅者自己来更新
		 */
		if (!hrt_called(&sd->update_call)) { //sd的更新调用已经从调用列表中删除或者永远不会调用 
			break;  //重复标注
		}

		/*
		 * Make sure that we don't consider the topic to be updated again
		 * until the interval has passed once more by restarting the interval
		 * timer and thereby re-scheduling a poll notification at that time.
		 * 在延时后调用这个更新
		 */
		hrt_call_after(&sd->update_call,
			       sd->update_interval,
			       &uORB::DeviceNode::update_deferred_trampoline,
			       (void *)this);

		/*
		 * Remember that we have told the subscriber that there is data.
		 */
		sd->update_reported = true;
		ret = true;

		break;
	}

out:
	irqrestore(state);

	/* consider it updated */
	return ret;
}

void
uORB::DeviceNode::update_deferred()
{
	/*
	 * Instigate a poll notification; any subscribers whose intervals have
	 * expired will be woken.
	 * 任何定时器到期订阅者都可以运行
	 */
	poll_notify(POLLIN);
}

void
uORB::DeviceNode::update_deferred_trampoline(void *arg)
{
	uORB::DeviceNode *node = (uORB::DeviceNode *)arg;

	node->update_deferred(); //调用延时更新的对象
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void uORB::DeviceNode::add_internal_subscriber()
{
	_subscriber_count++;
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count > 0) {
		ch->add_subscription(_meta->o_name, 1);
	}
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void uORB::DeviceNode::remove_internal_subscriber()
{
	_subscriber_count--;
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count == 0) {
		ch->remove_subscription(_meta->o_name);
	}
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool uORB::DeviceNode::is_published()
{
	return _published;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::DeviceNode::process_add_subscription(int32_t rateInHz)
{
	// if there is already data in the node, send this out to
	// the remote entity.
	// send the data to the remote entity.
	//使用多态实现订阅的这些流程，Manager的实例
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (_data != nullptr && ch != nullptr) { // _data will not be null if there is a publisher.
		ch->send_message(_meta->o_name, _meta->o_size, _data);
	}

	return OK;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::DeviceNode::process_remove_subscription()
{
	return OK;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::DeviceNode::process_received_message(int32_t length, uint8_t *data)
{
	int16_t ret = -1;

	if (length != (int32_t)(_meta->o_size)) {
		warnx("[uORB::DeviceNode::process_received_message(%d)]Error:[%s] Received DataLength[%d] != ExpectedLen[%d]",
		      __LINE__, _meta->o_name, (int)length, (int)_meta->o_size);
		return ERROR;
	}

	/* call the devnode write method with no file pointer */
	ret = write(nullptr, (const char *)data, _meta->o_size);

	if (ret < 0) {
		return ERROR;
	}

	if (ret != (int)_meta->o_size) {
		errno = EIO;
		return ERROR;
	}

	return OK;
}

uORB::DeviceMaster::DeviceMaster(Flavor f) :
	CDev((f == PUBSUB) ? "obj_master" : "param_master",
	     (f == PUBSUB) ? TOPIC_MASTER_DEVICE_PATH : PARAM_MASTER_DEVICE_PATH),
	_flavor(f)
{
	// enable debug() calls
	_debug_enabled = true;

}

uORB::DeviceMaster::~DeviceMaster()
{
}

int
uORB::DeviceMaster::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
	case ORBIOCADVERTISE: {
			const struct orb_advertdata *adv = (const struct orb_advertdata *)arg;
			const struct orb_metadata *meta = adv->meta;
			const char *objname;
			const char *devpath;
			char nodepath[orb_maxpath];
			uORB::DeviceNode *node;

			/* set instance to zero - we could allow selective multi-pubs later based on value */
			if (adv->instance != nullptr) {
				*(adv->instance) = 0;
			}

			/* construct a path to the node - this also checks the node name */
			ret = uORB::Utils::node_mkpath(nodepath, _flavor, meta, adv->instance);

			if (ret != OK) {
				return ret;
			}

			/* ensure that only one advertiser runs through this critical section */
			lock();

			ret = ERROR;

			/* try for topic groups */
			const unsigned max_group_tries = (adv->instance != nullptr) ? ORB_MULTI_MAX_INSTANCES : 1;
			unsigned group_tries = 0;

			do {
				/* if path is modifyable change try index */
				if (adv->instance != nullptr) {
					/* replace the number at the end of the string */
					nodepath[strlen(nodepath) - 1] = '0' + group_tries;
					*(adv->instance) = group_tries;
				}

				/* driver wants a permanent copy of the node name, so make one here */
				objname = strdup(meta->o_name);

				if (objname == nullptr) {
					return -ENOMEM;
				}

				/* driver wants a permanent copy of the path, so make one here */
				devpath = strdup(nodepath);

				if (devpath == nullptr) {
					return -ENOMEM;
				}

				/* construct the new node */
				node = new uORB::DeviceNode(meta, objname, devpath, adv->priority);

				/* if we didn't get a device, that's bad */
				if (node == nullptr) {
					unlock();
					return -ENOMEM;
				}

				/* initialise the node - this may fail if e.g. a node with this name already exists */
				ret = node->init();

				if (ret != OK) {
					/* if init failed, discard the node */
					delete node;

					if (ret == -EEXIST) {
						/* if the node exists already, get the existing one and check if
						 * something has been published yet. */
						uORB::DeviceNode *existing_node = GetDeviceNode(devpath);

						if ((existing_node != nullptr) && !(existing_node->is_published())) {
							/* nothing has been published yet, lets claim it */
							ret = OK;

						} else {
							/* otherwise: data has already been published, keep looking */
						}
					}

					/* also discard the name now */
					free((void *)objname);
					free((void *)devpath);

				} else {
					// add to the node map;.
					_node_map.insert(nodepath, node);
				}

				group_tries++;

			} while (ret != OK && (group_tries < max_group_tries));

			if (group_tries > max_group_tries) {
				ret = -ENOMEM;
			}

			/* the file handle for the driver has been created, unlock */
			unlock();

			return ret;
		}

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

uORB::DeviceNode *uORB::DeviceMaster::GetDeviceNode(const char *nodepath)
{
	uORB::DeviceNode *rc = nullptr;

	if (_node_map.find(nodepath)) {
		rc = _node_map.get(nodepath);
	}

	return rc;
}
