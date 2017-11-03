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

#ifndef _UORB_UORB_H
#define _UORB_UORB_H

/**
 * @file uORB.h
 * API for the uORB lightweight object broker.
 */

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

// Hack until everything is using this header
#include <systemlib/visibility.h>

/**
 * Object metadata.
 */
struct orb_metadata {
	const char *o_name;		/**< unique object name */
	const size_t o_size;		/**< object size */
};

typedef const struct orb_metadata *orb_id_t;

/**
 * Maximum number of multi topic instances
 */
#define ORB_MULTI_MAX_INSTANCES	4

/**
 * Topic priority.
 * Relevant for multi-topics / topic groups
 */
enum ORB_PRIO {
	ORB_PRIO_MIN = 0,
	ORB_PRIO_VERY_LOW = 25,
	ORB_PRIO_LOW = 50,
	ORB_PRIO_DEFAULT = 75,
	ORB_PRIO_HIGH = 100,
	ORB_PRIO_VERY_HIGH = 125,
	ORB_PRIO_MAX = 255
};

/**
 * Generates a pointer to the uORB metadata structure for
 * a given topic.
 *
 * The topic must have been declared previously in scope
 * with ORB_DECLARE().
 *
 * @param _name		The name of the topic.
 */
#define ORB_ID(_name)		&__orb_##_name  //_orb##_name = _orb_name（name就是参数拼接）ORB_ID(hello) = _orb_hello

/**
 * Declare (prototype) the uORB metadata for a topic.
 *
 * @param _name		The name of the topic.
 */
#if defined(__cplusplus)
# define ORB_DECLARE(_name)		extern "C" const struct orb_metadata __orb_##_name __EXPORT
# define ORB_DECLARE_OPTIONAL(_name)	extern "C" const struct orb_metadata __orb_##_name __EXPORT
#else
# define ORB_DECLARE(_name)		extern const struct orb_metadata __orb_##_name __EXPORT
# define ORB_DECLARE_OPTIONAL(_name)	extern const struct orb_metadata __orb_##_name __EXPORT
#endif

/**
 * Define (instantiate) the uORB metadata for a topic.
 *
 * The uORB metadata is used to help ensure that updates and
 * copies are accessing the right data.
 *
 * Note that there must be no more than one instance of this macro
 * for each topic.
 *
 * @param _name		The name of the topic.
 * @param _struct	The structure the topic provides.
 */
#define ORB_DEFINE(_name, _struct)			\
	const struct orb_metadata __orb_##_name = {	\
		#_name,					\
		sizeof(_struct)				\
	}; struct hack

__BEGIN_DECLS

/**
 * ORB topic advertiser handle.
 *
 * Advertiser handles are global; once obtained they can be shared freely
 * and do not need to be closed or released.
 *
 * This permits publication from interrupt context and other contexts where
 * a file-descriptor-based handle would not otherwise be in scope for the
 * publisher.
 */
typedef void 	*orb_advert_t;

/**
 * Advertise as the publisher of a topic. 作为发布者做广播
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node in /obj if required and publishes the initial data.
 	执行初始化发布；在obj中创建主题节点并在需要的时候发布初始数据
 * Any number of advertisers may publish to a topic; publications are atomic
 * but co-ordination between publishers is not provided by the ORB.
 * 任何发布者都可以发布到这个主题，发布是原子操作，发布的一致性ORB不做保证
 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
 *			for the topic. 元数据（就是主题名称）
 * @param data		A pointer to the initial data to be published. 发布的初始数据的指针
 *			For topics updated by interrupt handlers, the advertisement
 *			must be performed from non-interrupt context. 更新需要指针句柄，
 * @return		ERROR on error, otherwise returns a handle 如果正常，返回一个可以用来发布这个主题的句柄
 *			that can be used to publish to the topic.
 *			If the topic in question is not known (due to an
 *			ORB_DEFINE with no corresponding ORB_DECLARE)
 *			this function will return -1 and set errno to ENOENT.
 */
extern orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data) __EXPORT;

/**
 * Advertise as the publisher of a topic.作为发布者做广播
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node in /obj if required and publishes the initial data.
 *
 * Any number of advertisers may publish to a topic; publications are atomic
 * but co-ordination between publishers is not provided by the ORB.
 *
 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
 *			for the topic.
 * @param data		A pointer to the initial data to be published.
 *			For topics updated by interrupt handlers, the advertisement
 *			must be performed from non-interrupt context.
 * @param instance	Pointer to an integer which will yield the instance ID (0-based,
 *			limited by ORB_MULTI_MAX_INSTANCES) of the publication. 
			是一个指向产生实例ID的数字的指针（实例ID的数字的指针，最多5（0，1，2，3，4）个实例）
 * @param priority	The priority of the instance. If a subscriber subscribes multiple
 *			instances, the priority allows the subscriber to prioritize the best
 *			data source as long as its available.
				这个实例的优先级，如果同一个订阅多个实例，根据可用的最高优先级的
 * @return		ERROR on error, otherwise returns a handle
 *			that can be used to publish to the topic.
 *			If the topic in question is not known (due to an
 *			ORB_DEFINE with no corresponding ORB_DECLARE)
 *			this function will return -1 and set errno to ENOENT.
 */
extern orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
					int priority) __EXPORT;

/**
 * Advertise and publish as the publisher of a topic. 同上
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node in /obj if required and publishes the initial data.
 *
 * Any number of advertisers may publish to a topic; publications are atomic
 * but co-ordination between publishers is not provided by the ORB.
 *
 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
 *			for the topic.
 * @param data		A pointer to the initial data to be published.
 *			For topics updated by interrupt handlers, the advertisement
 *			must be performed from non-interrupt context.
 * @param instance	Pointer to an integer which will yield the instance ID (0-based,
 *			limited by ORB_MULTI_MAX_INSTANCES) of the publication.
 * @param priority	The priority of the instance. If a subscriber subscribes multiple
 *			instances, the priority allows the subscriber to prioritize the best
 *			data source as long as its available.
 * @return	zero on success, error number on failure 0是成功，否则返回错误码
 */
extern int orb_publish_auto(const struct orb_metadata *meta, orb_advert_t *handle, const void *data, int *instance,
			    int priority);

/**
 * Publish new data to a topic. 发布新数据到主题上
 *
 * The data is atomically published to the topic and any waiting subscribers
 * will be notified.  Subscribers that are not waiting can check the topic
 * for updates using orb_check and/or orb_stat.
 * 数据发布是原子操作，任何等待更新的订阅者都会被通知。其他没有等待的可以使用orb_check/orb_stat函数检查更新
 *
 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
 *			for the topic.  uORB的主题
 * @handle		The handle returned from orb_advertise. 主题的句柄（避免一个主题多个实例）
 * @param data		A pointer to the data to be published. 要发布的数据
 * @return		OK on success, ERROR otherwise with errno set accordingly.  
 */
extern int	orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data) __EXPORT;

/**
 * Subscribe to a topic.
 * 订阅这个主题
 * The returned value is a file descriptor that can be passed to poll()
 * in order to wait for updates to a topic, as well as topic_read,
 * orb_check and orb_stat.
 * 这个函数返回文件描述符，可以使用poll函数等待主题更新，就像使用topic_read/orb_check/orb_stat一样
 * Subscription will succeed even if the topic has not been advertised; 即使这个主题没有发布订阅也可能成功
 * in this case the topic will have a timestamp of zero, it will never 
 * signal a poll() event, checking will always return false and it cannot
 * be copied. When the topic is subsequently advertised, poll, check,
 * stat and copy calls will react to the initial publication that is
 * performed as part of the advertisement. 这种情况下，poll，复制它的值什么的操作都会失败，直到这个主题被发布未知。
 *
 * Subscription will fail if the topic is not known to the system, i.e. 如果系统不知道这样一个主题，那么会订阅失败
 * there is nothing in the system that has declared the topic and thus it
 * can never be published.
 * 系统中没有关于这个主题的声明，并且它将永远不会被发布。
 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
 *			for the topic.
 * @return		ERROR on error, otherwise returns a handle
 *			that can be used to read and update the topic.
 *			If the topic in question is not known (due to an
 *			ORB_DEFINE_OPTIONAL with no corresponding ORB_DECLARE)
 *			this function will return -1 and set errno to ENOENT.
 */
extern int	orb_subscribe(const struct orb_metadata *meta) __EXPORT;

/**
 * Subscribe to a multi-instance of a topic. 订阅多实例的主题
 *
 * The returned value is a file descriptor that can be passed to poll()
 * in order to wait for updates to a topic, as well as topic_read,
 * orb_check and orb_stat.
 *
 * Subscription will succeed even if the topic has not been advertised;
 * in this case the topic will have a timestamp of zero, it will never
 * signal a poll() event, checking will always return false and it cannot
 * be copied. When the topic is subsequently advertised, poll, check,
 * stat and copy calls will react to the initial publication that is
 * performed as part of the advertisement.
 *
 * Subscription will fail if the topic is not known to the system, i.e.
 * there is nothing in the system that has declared the topic and thus it
 * can never be published.
 *
 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
 *			for the topic.
 * @param instance	The instance of the topic. Instance 0 matches the
 *			topic of the orb_subscribe() call, higher indices
 *			are for topics created with orb_publish_multi().
 *			Instance is limited by ORB_MULTI_MAX_INSTANCES.
 * @return		ERROR on error, otherwise returns a handle
 *			that can be used to read and update the topic.
 *			If the topic in question is not known (due to an
 *			ORB_DEFINE_OPTIONAL with no corresponding ORB_DECLARE)
 *			this function will return -1 and set errno to ENOENT.
 */
extern int	orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance) __EXPORT;

/**
 * Unsubscribe from a topic. 取消订阅，参数为订阅成功返回的文件描述符
 *
 * @param handle	A handle returned from orb_subscribe.
 * @return		OK on success, ERROR otherwise with errno set accordingly.
 */
extern int	orb_unsubscribe(int handle) __EXPORT;

/**
 * Fetch data from a topic. 从主题中获取数据
 *
 * This is the only operation that will reset the internal marker that
 * indicates that a topic has been updated for a subscriber. Once poll
 * or check return indicating that an updaet is available, this call
 * must be used to update the subscription.
 * 这是唯一个可以重置内部主题是否更新标志的函数。一旦使用poll或者orb_check返回可用更新，必须使用这个方法更新参数
 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
 *			for the topic.
 * @param handle	A handle returned from orb_subscribe.
 * @param buffer	Pointer to the buffer receiving the data, or NULL
 *			if the caller wants to clear the updated flag without
 *			using the data.
 * @return		OK on success, ERROR otherwise with errno set accordingly.
 */
extern int	orb_copy(const struct orb_metadata *meta, int handle, void *buffer) __EXPORT;

/**
 * Check whether a topic has been published to since the last orb_copy.
 * 检查在上次orb_copy后这个主题是否被重新发布过
 * This check can be used to determine whether to copy the topic when
 * not using poll(), or to avoid the overhead of calling poll() when the
 * topic is likely to have updated.
 * 在不使用poll函数的情况下检查是否有更新
 * Updates are tracked on a per-handle basis; this call will continue to
 * return true until orb_copy is called using the same handle. This interface
 * should be preferred over calling orb_stat due to the race window between
 * stat and copy that can lead to missed updates.
 * 更新以每个句柄为基础进行跟踪; 
 * 这个调用将一直返回true直到使用相同的句柄调用orb_copy。
 * 由于stat和copy之间的竞争窗口可能导致错过更新，所以此接口应优先于调用orb_stat。
 *
 * @param handle	A handle returned from orb_subscribe.
 * @param updated	Set to true if the topic has been updated since the
 *			last time it was copied using this handle.
 * @return		OK if the check was successful, ERROR otherwise with
 *			errno set accordingly.
 */
extern int	orb_check(int handle, bool *updated) __EXPORT;

/**
 * Return the last time that the topic was updated. 返回上一次更新主题的时间
 *
 * @param handle	A handle returned from orb_subscribe.
 * @param time		Returns the absolute time that the topic was updated, or zero if it has
 *			never been updated. Time is measured in microseconds.
 * @return		OK on success, ERROR otherwise with errno set accordingly.
 */
extern int	orb_stat(int handle, uint64_t *time) __EXPORT;

/**
 * Check if a topic has already been created. 检查主题是否存在
 *
 * @param meta		ORB topic metadata.
 * @param instance	ORB instance
 * @return		OK if the topic exists, ERROR otherwise with errno set accordingly.
 */
extern int	orb_exists(const struct orb_metadata *meta, int instance) __EXPORT;

/**
 * Get the number of published instances of a topic group 返回主题的实例数量
 *
 * @param meta    ORB topic metadata.
 * @return    The number of published instances of this topic
 */
extern int	orb_group_count(const struct orb_metadata *meta) __EXPORT;

/**
 * Return the priority of the topic 返回句柄的优先级
 *
 * @param handle	A handle returned from orb_subscribe.
 * @param priority	Returns the priority of this topic. This is only relevant for
 *			topics which are published by multiple publishers (e.g. mag0, mag1, etc.)
 *			and allows a subscriber to automatically pick the topic with the highest
 *			priority, independent of the startup order of the associated publishers.
 * @return		OK on success, ERROR otherwise with errno set accordingly.
 */
extern int	orb_priority(int handle, int32_t *priority) __EXPORT;

/**
 * Set the minimum interval between which updates are seen for a subscription.
 * 设置最小的更新间隙
 * If this interval is set, the subscriber will not see more than one update
 * within the period.
 *
 * Specifically, the first time an update is reported to the subscriber a timer
 * is started. The update will continue to be reported via poll and orb_check, but
 * once fetched via orb_copy another update will not be reported until the timer
 * expires.
 *
 * This feature can be used to pace a subscriber that is watching a topic that
 * would otherwise update too quickly.
 *
 * @param handle	A handle returned from orb_subscribe.
 * @param interval	An interval period in milliseconds.
 * @return		OK on success, ERROR otherwise with ERRNO set accordingly.
 */
extern int	orb_set_interval(int handle, unsigned interval) __EXPORT;

__END_DECLS

/* Diverse uORB header defines */ //XXX: move to better location
#define ORB_ID_VEHICLE_ATTITUDE_CONTROLS    ORB_ID(actuator_controls_0)
typedef uint8_t arming_state_t;
typedef uint8_t main_state_t;
typedef uint8_t hil_state_t;
typedef uint8_t navigation_state_t;
typedef uint8_t switch_pos_t;

#endif /* _UORB_UORB_H */
