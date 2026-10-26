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
 * @file CanFrameFilter.hpp
 *
 * DroneCAN data type filter for CAN frames forwarded over MAVLink (CAN_FILTER_MODIFY).
 */

#pragma once

#include <stdint.h>

/**
 * @class CanFrameFilter
 *
 * Holds the DroneCAN data type IDs a ground station wants to receive.
 * An empty filter accepts every frame.
 */
class CanFrameFilter
{
public:
	static constexpr uint8_t MAX_IDS{32}; ///< [-] DroneCAN data type IDs the filter holds, twice the 16 of one CAN_FILTER_MODIFY

	/**
	 * Replace the filter with the given IDs. No IDs clears the filter, so it accepts every frame.
	 *
	 * @param ids DroneCAN data type IDs
	 * @param num_ids number of entries in ids
	 */
	void replace(const uint16_t *ids, uint8_t num_ids);

	/**
	 * Add IDs. Duplicates are skipped and IDs beyond MAX_IDS are dropped.
	 *
	 * @param ids DroneCAN data type IDs
	 * @param num_ids number of entries in ids
	 */
	void add(const uint16_t *ids, uint8_t num_ids);

	/**
	 * Remove IDs. IDs not in the filter are ignored.
	 *
	 * @param ids DroneCAN data type IDs
	 * @param num_ids number of entries in ids
	 */
	void remove(const uint16_t *ids, uint8_t num_ids);

	void clear() { _num_ids = 0; }

	uint8_t size() const { return _num_ids; }

	/**
	 * @param can_id CAN identifier of the frame, bit 31 set for an extended identifier
	 * @return true if the frame passes the filter
	 */
	bool accepts(uint32_t can_id) const;

	/**
	 * DroneCAN data type ID of a frame, the value CAN_FILTER_MODIFY filters on:
	 * the message type ID for a message frame, the service type ID for a service
	 * frame, and 0 for an anonymous frame.
	 */
	static uint16_t dataTypeId(uint32_t can_id);

private:
	bool contains(uint16_t id) const;

	uint16_t _ids[MAX_IDS] {};
	uint8_t _num_ids{0};
};
