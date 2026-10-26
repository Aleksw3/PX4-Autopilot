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

#include "can_frame_forwarder.hpp"

#include <mathlib/math/Limits.hpp>
#include <string.h>

static_assert(sizeof(can_frame_s::data) == uavcan::CanFrame::MaxDataLen, "can_frame data must hold a classic CAN frame");

uavcan::ICanIface *CanFrameForwarder::getIface(uint8_t iface_index)
{
	if (iface_index >= uavcan::MaxCanIfaces) {
		return nullptr;
	}

	uavcan::ICanIface *iface = _driver.getIface(iface_index);

	if (iface == nullptr) {
		return nullptr;
	}

	_ifaces[iface_index].bind(*this, *iface, iface_index);
	return &_ifaces[iface_index];
}

bool CanFrameForwarder::requestForwarding(uint8_t iface_index)
{
	if (iface_index >= getNumIfaces() || iface_index >= uavcan::MaxCanIfaces || _driver.getIface(iface_index) == nullptr) {
		return false;
	}

	_last_request[iface_index] = hrt_absolute_time();
	return true;
}

void CanFrameForwarder::stopForwarding()
{
	for (hrt_abstime &last_request : _last_request) {
		last_request = 0;
	}
}

bool CanFrameForwarder::isForwarding(uint8_t iface_index) const
{
	return _last_request[iface_index] != 0 && hrt_elapsed_time(&_last_request[iface_index]) < REQUEST_TIMEOUT;
}

void CanFrameForwarder::publishFrame(uint8_t iface_index, const uavcan::CanFrame &frame)
{
	if (!isForwarding(iface_index)) {
		return;
	}

	can_frame_s can_frame{};
	can_frame.interface = iface_index;
	can_frame.id = frame.id;
	// Classic CAN data length codes 9 to 15 also mean 8 bytes
	can_frame.len = math::min(frame.dlc, uavcan::CanFrame::MaxDataLen);
	memcpy(can_frame.data, frame.data, sizeof(can_frame.data));
	can_frame.timestamp = hrt_absolute_time();
	_can_frame_from_bus_pub.publish(can_frame);
}

void CanFrameForwarder::sendFramesToBus(uavcan::Dispatcher &dispatcher)
{
	can_frame_s can_frame;

	while (_can_frame_to_bus_sub.update(&can_frame)) {
		if (can_frame.interface >= getNumIfaces() || can_frame.len > uavcan::CanFrame::MaxDataLen) {
			continue;
		}

		// Keep only the identifier and the extended flag: a ground station cannot send remote or error frames
		const bool extended = (can_frame.id & uavcan::CanFrame::FlagEFF) != 0;
		const uint32_t id = extended ? ((can_frame.id & uavcan::CanFrame::MaskExtID) | uavcan::CanFrame::FlagEFF)
				    : (can_frame.id & uavcan::CanFrame::MaskStdID);

		const uavcan::CanFrame frame(id, can_frame.data, can_frame.len);
		const uavcan::MonotonicTime tx_deadline = dispatcher.getSystemClock().getMonotonic()
				+ uavcan::MonotonicDuration::fromMSec(TO_BUS_TX_DEADLINE_MS);

		const uint8_t iface_mask = static_cast<uint8_t>(1U << can_frame.interface);

		dispatcher.getCanIOManager().send(frame, tx_deadline, uavcan::MonotonicTime(), iface_mask, uavcan::CanTxQueue::Volatile,
						  CAN_IO_FLAG_FROM_MAVLINK);
	}
}

int16_t CanFrameForwarder::ForwardingIface::send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline,
		uavcan::CanIOFlags flags)
{
	const bool from_mavlink = (flags & CAN_IO_FLAG_FROM_MAVLINK) != 0;
	const int16_t result = _iface->send(frame, tx_deadline, static_cast<uavcan::CanIOFlags>(flags & ~CAN_IO_FLAG_FROM_MAVLINK));

	if (result > 0 && !from_mavlink) {
		_forwarder->publishFrame(_iface_index, frame);
	}

	return result;
}

int16_t CanFrameForwarder::ForwardingIface::receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic,
		uavcan::UtcTime &out_ts_utc, uavcan::CanIOFlags &out_flags)
{
	const int16_t result = _iface->receive(out_frame, out_ts_monotonic, out_ts_utc, out_flags);

	// A loopback frame is a copy of one this node sent, which send() published already
	if (result > 0 && (out_flags & uavcan::CanIOFlagLoopback) == 0) {
		_forwarder->publishFrame(_iface_index, out_frame);
	}

	return result;
}
