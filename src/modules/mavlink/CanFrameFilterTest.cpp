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

#include <gtest/gtest.h>

namespace
{

constexpr uint32_t EXTENDED_ID_FLAG = 1U << 31;
constexpr uint32_t PRIORITY = 16;

constexpr uint16_t NODE_STATUS = 341;
constexpr uint16_t ESC_STATUS = 1034;
constexpr uint8_t GET_NODE_INFO = 1;

uint32_t messageFrameId(uint16_t message_type_id, uint8_t source_node_id)
{
	return EXTENDED_ID_FLAG | (PRIORITY << 24) | (static_cast<uint32_t>(message_type_id) << 8) | source_node_id;
}

uint32_t serviceFrameId(uint8_t service_type_id, uint8_t destination_node_id, uint8_t source_node_id)
{
	constexpr uint32_t request_not_response = 1U << 15;
	constexpr uint32_t service_not_message = 1U << 7;
	return EXTENDED_ID_FLAG | (PRIORITY << 24) | (static_cast<uint32_t>(service_type_id) << 16) | request_not_response
	       | (static_cast<uint32_t>(destination_node_id) << 8) | service_not_message | source_node_id;
}

uint32_t anonymousFrameId(uint8_t discriminator)
{
	return EXTENDED_ID_FLAG | (PRIORITY << 24) | (static_cast<uint32_t>(discriminator) << 10);
}

} // namespace

TEST(CanFrameFilterTest, DataTypeIdOfDroneCanFrames)
{
	EXPECT_EQ(CanFrameFilter::dataTypeId(messageFrameId(NODE_STATUS, 10)), NODE_STATUS);
	EXPECT_EQ(CanFrameFilter::dataTypeId(serviceFrameId(GET_NODE_INFO, 10, 127)), GET_NODE_INFO);
	EXPECT_EQ(CanFrameFilter::dataTypeId(anonymousFrameId(0x55)), 0);
}

TEST(CanFrameFilterTest, EmptyFilterAcceptsEverything)
{
	CanFrameFilter filter;
	EXPECT_TRUE(filter.accepts(messageFrameId(NODE_STATUS, 10)));
	EXPECT_TRUE(filter.accepts(serviceFrameId(GET_NODE_INFO, 10, 127)));
	EXPECT_TRUE(filter.accepts(anonymousFrameId(0x55)));
}

TEST(CanFrameFilterTest, ReplaceAcceptsOnlyListedIds)
{
	CanFrameFilter filter;
	const uint16_t ids[] = {NODE_STATUS, GET_NODE_INFO};
	filter.replace(ids, 2);

	EXPECT_TRUE(filter.accepts(messageFrameId(NODE_STATUS, 10)));
	EXPECT_TRUE(filter.accepts(serviceFrameId(GET_NODE_INFO, 10, 127)));
	EXPECT_FALSE(filter.accepts(messageFrameId(ESC_STATUS, 10)));

	const uint16_t replacement[] = {ESC_STATUS};
	filter.replace(replacement, 1);
	EXPECT_FALSE(filter.accepts(messageFrameId(NODE_STATUS, 10)));
	EXPECT_TRUE(filter.accepts(messageFrameId(ESC_STATUS, 10)));
}

TEST(CanFrameFilterTest, ReplaceWithNoIdsAcceptsEverything)
{
	CanFrameFilter filter;
	const uint16_t ids[] = {NODE_STATUS};
	filter.replace(ids, 1);
	filter.replace(nullptr, 0);

	EXPECT_EQ(filter.size(), 0);
	EXPECT_TRUE(filter.accepts(messageFrameId(ESC_STATUS, 10)));
}

TEST(CanFrameFilterTest, AddIgnoresDuplicates)
{
	CanFrameFilter filter;
	const uint16_t ids[] = {NODE_STATUS, NODE_STATUS, ESC_STATUS};
	filter.add(ids, 3);
	filter.add(ids, 1);

	EXPECT_EQ(filter.size(), 2);
}

TEST(CanFrameFilterTest, AddStopsAtCapacity)
{
	CanFrameFilter filter;
	uint16_t ids[CanFrameFilter::MAX_IDS + 1];

	for (uint16_t i = 0; i < CanFrameFilter::MAX_IDS + 1; i++) {
		ids[i] = 100 + i;
	}

	filter.add(ids, CanFrameFilter::MAX_IDS + 1);

	EXPECT_EQ(filter.size(), CanFrameFilter::MAX_IDS);
	EXPECT_TRUE(filter.accepts(messageFrameId(ids[CanFrameFilter::MAX_IDS - 1], 10)));
	EXPECT_FALSE(filter.accepts(messageFrameId(ids[CanFrameFilter::MAX_IDS], 10)));
}

TEST(CanFrameFilterTest, RemoveLastIdAcceptsEverything)
{
	CanFrameFilter filter;
	const uint16_t ids[] = {NODE_STATUS, ESC_STATUS};
	filter.add(ids, 2);

	const uint16_t node_status[] = {NODE_STATUS};
	filter.remove(node_status, 1);
	EXPECT_FALSE(filter.accepts(messageFrameId(NODE_STATUS, 10)));
	EXPECT_TRUE(filter.accepts(messageFrameId(ESC_STATUS, 10)));

	const uint16_t esc_status[] = {ESC_STATUS};
	filter.remove(esc_status, 1);
	EXPECT_EQ(filter.size(), 0);
	EXPECT_TRUE(filter.accepts(messageFrameId(NODE_STATUS, 10)));
}
