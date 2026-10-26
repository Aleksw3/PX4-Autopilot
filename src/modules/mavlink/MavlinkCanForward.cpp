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

#include "MavlinkCanForward.hpp"
#include "mavlink_main.h"

#include <math.h>
#include <string.h>

static_assert(sizeof(can_frame_s::data) == sizeof(mavlink_can_frame_t::data), "can_frame must hold a MAVLink CAN_FRAME");

void MavlinkCanForward::handleForwardCommand(uint8_t sysid, uint8_t compid, float bus_number)
{
	if (!(bus_number >= 1.f && bus_number <= UINT8_MAX)) {
		_last_request = 0;
		return;
	}

	if (!sessionActive()) {
		_filter.clear();
	}

	_target_system = sysid;
	_target_component = compid;
	_bus = static_cast<uint8_t>(roundf(bus_number)) - 1;
	_last_request = hrt_absolute_time();
}

void MavlinkCanForward::handleMessage(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_CAN_FRAME:
		handleCanFrame(msg);
		break;

	case MAVLINK_MSG_ID_CAN_FILTER_MODIFY:
		handleCanFilterModify(msg);
		break;

	default:
		break;
	}
}

void MavlinkCanForward::handleCanFrame(const mavlink_message_t *msg)
{
	mavlink_can_frame_t mavlink_frame;
	mavlink_msg_can_frame_decode(msg, &mavlink_frame);

	if (!addressedToUs(mavlink_frame.target_system, mavlink_frame.target_component)
	    || !sessionActive() || mavlink_frame.bus != _bus || mavlink_frame.len > sizeof(mavlink_frame.data)) {
		return;
	}

	can_frame_s can_frame{};
	can_frame.interface = mavlink_frame.bus;
	can_frame.id = mavlink_frame.id;
	can_frame.len = mavlink_frame.len;
	memcpy(can_frame.data, mavlink_frame.data, sizeof(can_frame.data));
	can_frame.timestamp = hrt_absolute_time();
	_can_frame_to_bus_pub.publish(can_frame);
}

void MavlinkCanForward::handleCanFilterModify(const mavlink_message_t *msg)
{
	mavlink_can_filter_modify_t filter_modify;
	mavlink_msg_can_filter_modify_decode(msg, &filter_modify);

	// Unlike CAN_FRAME, CAN_FILTER_MODIFY numbers the bus from 1
	if (!addressedToUs(filter_modify.target_system, filter_modify.target_component)
	    || !sessionActive() || filter_modify.bus != _bus + 1
	    || filter_modify.num_ids > sizeof(filter_modify.ids) / sizeof(filter_modify.ids[0])) {
		return;
	}

	// The packed MAVLink struct does not align ids
	uint16_t ids[sizeof(filter_modify.ids) / sizeof(filter_modify.ids[0])];
	memcpy(ids, filter_modify.ids, sizeof(ids));

	switch (filter_modify.operation) {
	case CAN_FILTER_REPLACE:
		_filter.replace(ids, filter_modify.num_ids);
		break;

	case CAN_FILTER_ADD:
		_filter.add(ids, filter_modify.num_ids);
		break;

	case CAN_FILTER_REMOVE:
		_filter.remove(ids, filter_modify.num_ids);
		break;

	default:
		break;
	}
}

void MavlinkCanForward::send()
{
	static constexpr unsigned CAN_FRAME_PACKET_SIZE = MAVLINK_MSG_ID_CAN_FRAME_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	can_frame_s can_frame;

	while (_can_frame_from_bus_sub.update(&can_frame)) {
		if (!sessionActive() || can_frame.interface != _bus || !_filter.accepts(can_frame.id)) {
			continue;
		}

		// A full link drops frames, like the CAN bus would with a full receive buffer
		if (_mavlink.get_free_tx_buf() < CAN_FRAME_PACKET_SIZE) {
			continue;
		}

		mavlink_can_frame_t mavlink_frame{};
		mavlink_frame.target_system = _target_system;
		mavlink_frame.target_component = _target_component;
		mavlink_frame.bus = can_frame.interface;
		mavlink_frame.len = can_frame.len;
		mavlink_frame.id = can_frame.id;
		memcpy(mavlink_frame.data, can_frame.data, sizeof(mavlink_frame.data));
		mavlink_msg_can_frame_send_struct(_mavlink.get_channel(), &mavlink_frame);
	}
}

bool MavlinkCanForward::sessionActive() const
{
	return _last_request != 0 && hrt_elapsed_time(&_last_request) < REQUEST_TIMEOUT;
}

bool MavlinkCanForward::addressedToUs(uint8_t target_system, uint8_t target_component)
{
	return (target_system == 0 || target_system == mavlink_system.sysid)
	       && (target_component == MAV_COMP_ID_ALL || target_component == mavlink_system.compid);
}
