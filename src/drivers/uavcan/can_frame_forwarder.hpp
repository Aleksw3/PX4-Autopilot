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
 * @file can_frame_forwarder.hpp
 *
 * Forwards raw CAN frames between the CAN driver and MAVLink (MAV_CMD_CAN_FORWARD).
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>
#include <uavcan/driver/can.hpp>
#include <uavcan/transport/dispatcher.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/can_frame.h>

using namespace time_literals;

/**
 * @class CanFrameForwarder
 *
 * Wraps the CAN driver the node uses. Every frame received on a forwarded interface,
 * and every frame this node sends on it, is published on can_frame_from_bus.
 * Frames from can_frame_to_bus are sent on the bus and not published back.
 *
 * All calls must hold the node mutex, like any other use of the node.
 */
class CanFrameForwarder : public uavcan::ICanDriver
{
public:
	explicit CanFrameForwarder(uavcan::ICanDriver &driver) : _driver(driver) {}

	uavcan::ICanIface *getIface(uint8_t iface_index) override;

	uint8_t getNumIfaces() const override { return _driver.getNumIfaces(); }

	int16_t select(uavcan::CanSelectMasks &inout_masks, const uavcan::CanFrame * (&pending_tx)[uavcan::MaxCanIfaces],
		       uavcan::MonotonicTime blocking_deadline) override
	{
		return _driver.select(inout_masks, pending_tx, blocking_deadline);
	}

	/**
	 * Start forwarding an interface, or keep it going. Forwarding stops
	 * when no request arrives for REQUEST_TIMEOUT.
	 *
	 * @param iface_index 0-based CAN interface index
	 * @return false if the interface does not exist
	 */
	bool requestForwarding(uint8_t iface_index);

	void stopForwarding();

	/**
	 * Send the frames queued on can_frame_to_bus.
	 */
	void sendFramesToBus(uavcan::Dispatcher &dispatcher);

private:
	class ForwardingIface : public uavcan::ICanIface
	{
	public:
		void bind(CanFrameForwarder &forwarder, uavcan::ICanIface &iface, uint8_t iface_index)
		{
			_forwarder = &forwarder;
			_iface = &iface;
			_iface_index = iface_index;
		}

		int16_t send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags) override;

		int16_t receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic, uavcan::UtcTime &out_ts_utc,
				uavcan::CanIOFlags &out_flags) override;

		int16_t configureFilters(const uavcan::CanFilterConfig *filter_configs, uint16_t num_configs) override
		{
			return _iface->configureFilters(filter_configs, num_configs);
		}

		uint16_t getNumFilters() const override { return _iface->getNumFilters(); }

		uint64_t getErrorCount() const override { return _iface->getErrorCount(); }

	private:
		CanFrameForwarder *_forwarder{nullptr};
		uavcan::ICanIface *_iface{nullptr};
		uint8_t _iface_index{0};
	};

	bool isForwarding(uint8_t iface_index) const;

	void publishFrame(uint8_t iface_index, const uavcan::CanFrame &frame);

	// The DroneCAN GUI tool repeats MAV_CMD_CAN_FORWARD every second. ArduPilot uses the same timeout.
	static constexpr hrt_abstime REQUEST_TIMEOUT{5_s};

	static constexpr uint32_t TO_BUS_TX_DEADLINE_MS{100}; ///< [ms] how long a frame from MAVLink may wait in the transmit queue

	// libuavcan uses bits 0 and 1. The flag is removed before the frame reaches the driver.
	static constexpr uavcan::CanIOFlags CAN_IO_FLAG_FROM_MAVLINK{1U << 15};

	uavcan::ICanDriver &_driver;
	ForwardingIface _ifaces[uavcan::MaxCanIfaces] {};
	hrt_abstime _last_request[uavcan::MaxCanIfaces] {}; ///< [us] time of the last MAV_CMD_CAN_FORWARD per interface, 0 if stopped

	uORB::Publication<can_frame_s> _can_frame_from_bus_pub{ORB_ID(can_frame_from_bus)};
	uORB::Subscription _can_frame_to_bus_sub{ORB_ID(can_frame_to_bus)};
};
