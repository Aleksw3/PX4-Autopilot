# Raw CAN frame forwarded between a CAN bus and MAVLink
#
# A ground station starts forwarding of a bus with MAV_CMD_CAN_FORWARD.
# can_frame_from_bus: published by the uavcan driver for every frame received on the forwarded bus, and for every frame this node sends on it.
# can_frame_to_bus: published by mavlink for every CAN_FRAME message from the ground station, sent on the bus by the uavcan driver.

uint64 timestamp # [us] Time since system start

uint8 interface # [-] CAN interface index, 0-based, as MAVLink CAN_FRAME.bus
uint32 id       # [-] CAN identifier, bit 31 set for an extended (29 bit) identifier
uint8 len       # [-] Number of valid bytes in data, 0 to 8
uint8[8] data   # [-] Frame payload

uint8 ORB_QUEUE_LENGTH = 64 # A multi-frame DroneCAN transfer arrives as a burst

# TOPICS can_frame_from_bus can_frame_to_bus
