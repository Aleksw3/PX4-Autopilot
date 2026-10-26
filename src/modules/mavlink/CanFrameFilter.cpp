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

#include "CanFrameFilter.hpp"

// DroneCAN 29 bit identifier layout
static constexpr uint32_t SOURCE_NODE_ID_MASK{0x7F};
static constexpr uint32_t SERVICE_NOT_MESSAGE_BIT{1U << 7};
static constexpr uint32_t MESSAGE_TYPE_ID_SHIFT{8};
static constexpr uint32_t MESSAGE_TYPE_ID_MASK{0xFFFF};
static constexpr uint32_t SERVICE_TYPE_ID_SHIFT{16};
static constexpr uint32_t SERVICE_TYPE_ID_MASK{0xFF};

void CanFrameFilter::replace(const uint16_t *ids, uint8_t num_ids)
{
	clear();
	add(ids, num_ids);
}

void CanFrameFilter::add(const uint16_t *ids, uint8_t num_ids)
{
	for (uint8_t i = 0; i < num_ids && _num_ids < MAX_IDS; i++) {
		if (!contains(ids[i])) {
			_ids[_num_ids++] = ids[i];
		}
	}
}

void CanFrameFilter::remove(const uint16_t *ids, uint8_t num_ids)
{
	for (uint8_t i = 0; i < num_ids; i++) {
		for (uint8_t j = 0; j < _num_ids; j++) {
			if (_ids[j] == ids[i]) {
				_ids[j] = _ids[--_num_ids];
				break;
			}
		}
	}
}

bool CanFrameFilter::accepts(uint32_t can_id) const
{
	return _num_ids == 0 || contains(dataTypeId(can_id));
}

uint16_t CanFrameFilter::dataTypeId(uint32_t can_id)
{
	if (can_id & SERVICE_NOT_MESSAGE_BIT) {
		return (can_id >> SERVICE_TYPE_ID_SHIFT) & SERVICE_TYPE_ID_MASK;
	}

	if ((can_id & SOURCE_NODE_ID_MASK) == 0) {
		// Anonymous message, its frames carry only a 2 bit type ID. ArduPilot filters them as ID 0.
		return 0;
	}

	return (can_id >> MESSAGE_TYPE_ID_SHIFT) & MESSAGE_TYPE_ID_MASK;
}

bool CanFrameFilter::contains(uint16_t id) const
{
	for (uint8_t i = 0; i < _num_ids; i++) {
		if (_ids[i] == id) {
			return true;
		}
	}

	return false;
}
