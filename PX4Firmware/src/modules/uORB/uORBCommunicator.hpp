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

#ifndef _uORBCommunicator_hpp_
#define _uORBCommunicator_hpp_

#include <stdint.h>


namespace uORBCommunicator
{
class IChannel;
class IChannelRxHandler;
}

/**
 * Interface to enable remote subscriptions.  The implementor of this interface
 * shall manage the communication channel. It can be fastRPC or tcp or ip.
 * 启用远程订阅的接口，接口的实现类需要管理通信信道，快速RPC/TCP/IP
 */

class uORBCommunicator::IChannel
{
public:

	//=========================================================================
	//     INTERFACES FOR Control messages over a channel. 通过信道控制信息的接口
	//=========================================================================

	/**
	 * @brief Interface to notify the remote entity of interest of a
	 * subscription for a message.通知有兴趣的远程实体订阅消息的接口
	 *
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique. 需要uORB的消息名称，这个名称是全局唯一的
	 * @param msgRate
	 * 	The max rate at which the subscriber can accept the messages. 订阅者可以接受的最大速率
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */

	virtual int16_t add_subscription(const char *messageName, int32_t msgRateInHz) = 0;



	/**
	 * @brief Interface to notify the remote entity of removal of a subscription
	 *  通知远程实体移除订阅的接口
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not necessarily mean that the receiver as received it.
	 *  otherwise = failure.
	 */

	virtual int16_t remove_subscription(const char *messageName) = 0;


	/**
	 * Register Message Handler.  This is internal for the IChannel implementer* 注册消息句柄
	 */
	virtual int16_t register_handler(uORBCommunicator::IChannelRxHandler *handler) = 0;


	//=========================================================================
	//     INTERFACES FOR Data messages 数据消息接口
	//=========================================================================

	/**
	 * @brief Sends the data message over the communication link. 
	 * 通过通讯链接发送数据
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @param length
	 * 	The length of the data buffer to be sent. 数据长度
	 * @param data
	 * 	The actual data to be sent. 要发生的数据
	 * @return
	 *  0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */

	virtual int16_t send_message(const char *messageName, int32_t length, uint8_t *data) = 0;

};

/**
 * Class passed to the communication link implement to provide callback for received
 * messages over a channel.
 * 通过通信链路回调的类
 */
class uORBCommunicator::IChannelRxHandler
{
public:

	/**
	 * Interface to process a received AddSubscription from remote.
	 * 处理从远程接受AddSubscription的接口函数
	 * @param messageName
	 * 	This represents the uORB message Name; This message Name should be
	 * 	globally unique.
	 * @param msgRate
	 * 	The max rate at which the subscriber can accept the messages.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *  	handler.
	 *  otherwise = failure.
	 */

	virtual int16_t process_add_subscription(const char *messageName, int32_t msgRateInHz) = 0;


	/**
	 * Interface to process a received control msg to remove subscription
	 * @param messageName
	 * 	This represents the uORB message Name; This message Name should be
	 * 	globally unique.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *  	handler.
	 *  otherwise = failure.
	 */

	virtual int16_t process_remove_subscription(const char *messageName) = 0;


	/**
	 * Interface to process the received data message.
	 * @param messageName
	 * 	This represents the uORB message Name; This message Name should be
	 * 	globally unique.
	 * @param length
	 * 	The length of the data buffer to be sent.
	 * @param data
	 * 	The actual data to be sent.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *  	handler.
	 *  otherwise = failure.
	 */

	virtual int16_t process_received_message(const char *messageName, int32_t length, uint8_t *data) = 0;

};

#endif /* _uORBCommunicator_hpp_ */
