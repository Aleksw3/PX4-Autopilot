/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/**
 * @file MavlinkCanForward.hpp
 *
 * CAN bus forwarding over MAVLink: MAV_CMD_CAN_FORWARD, CAN_FRAME and CAN_FILTER_MODIFY.
 * Lets a ground station tool such as the DroneCAN GUI tool use a CAN bus of the vehicle.
 */

#pragma once

#include "CanFrameFilter.hpp"
#include "mavlink_bridge_header.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/can_frame.h>

using namespace time_literals;

class Mavlink;

/**
 * @class MavlinkCanForward
 *
 * Holds the forwarding session of one MAVLink instance: the component that sent
 * MAV_CMD_CAN_FORWARD and the bus it asked for. The uavcan driver acknowledges the
 * command, publishes the frames of the bus on can_frame_from_bus, and sends the
 * frames from can_frame_to_bus.
 */
class MavlinkCanForward
{
public:
	explicit MavlinkCanForward(Mavlink &mavlink) : _mavlink(mavlink) {}

	/**
	 * Start, refresh or stop the session for a MAV_CMD_CAN_FORWARD addressed to this system.
	 *
	 * @param sysid MAVLink system ID of the sender, which receives the frames
	 * @param compid MAVLink component ID of the sender, which receives the frames
	 * @param bus_number param1 of the command: 1-based bus number, 0 to stop
	 */
	void handleForwardCommand(uint8_t sysid, uint8_t compid, float bus_number);

	/**
	 * Handle CAN_FRAME and CAN_FILTER_MODIFY. Other messages are ignored.
	 */
	void handleMessage(const mavlink_message_t *msg);

	/**
	 * Send the frames of the forwarded bus as CAN_FRAME messages. Call with the send lock held.
	 */
	void send();

private:
	void handleCanFrame(const mavlink_message_t *msg);
	void handleCanFilterModify(const mavlink_message_t *msg);

	bool sessionActive() const;

	static bool addressedToUs(uint8_t target_system, uint8_t target_component);

	// The DroneCAN GUI tool repeats MAV_CMD_CAN_FORWARD every second. ArduPilot uses the same timeout.
	static constexpr hrt_abstime REQUEST_TIMEOUT{5_s};

	Mavlink &_mavlink;

	hrt_abstime _last_request{0}; ///< [us] time of the last MAV_CMD_CAN_FORWARD, 0 if stopped
	uint8_t _target_system{0};
	uint8_t _target_component{0};
	uint8_t _bus{0}; ///< 0-based CAN interface index

	CanFrameFilter _filter{};

	uORB::Subscription _can_frame_from_bus_sub{ORB_ID(can_frame_from_bus)};
	uORB::Publication<can_frame_s> _can_frame_to_bus_pub{ORB_ID(can_frame_to_bus)};
};
