// MESSAGE ACTION PACKING

#define MAVLINK_MSG_ID_ACTION 10
#define MAVLINK_MSG_ID_ACTION_LEN 3
#define MAVLINK_MSG_10_LEN 3
#define MAVLINK_MSG_ID_ACTION_KEY 0x2C
#define MAVLINK_MSG_10_KEY 0x2C

typedef struct __mavlink_action_t 
{
	uint8_t target;	///< The system executing the action
	uint8_t target_component;	///< The component executing the action
	uint8_t action;	///< The action id

} mavlink_action_t;

/**
 * @brief Pack a action message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system executing the action
 * @param target_component The component executing the action
 * @param action The action id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_action_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, uint8_t target_component, uint8_t action)
{
	mavlink_action_t *p = (mavlink_action_t *)&msg->payload[0];
	msg->msgid = MAVLINK_MSG_ID_ACTION;

	p->target = target;	// uint8_t:The system executing the action
	p->target_component = target_component;	// uint8_t:The component executing the action
	p->action = action;	// uint8_t:The action id

	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACTION_LEN);
}

/**
 * @brief Pack a action message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system executing the action
 * @param target_component The component executing the action
 * @param action The action id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_action_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target, uint8_t target_component, uint8_t action)
{
	mavlink_action_t *p = (mavlink_action_t *)&msg->payload[0];
	msg->msgid = MAVLINK_MSG_ID_ACTION;

	p->target = target;	// uint8_t:The system executing the action
	p->target_component = target_component;	// uint8_t:The component executing the action
	p->action = action;	// uint8_t:The action id

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACTION_LEN);
}

/**
 * @brief Encode a action struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param action C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_action_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_action_t* action)
{
	return mavlink_msg_action_pack(system_id, component_id, msg, action->target, action->target_component, action->action);
}


#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
/**
 * @brief Send a action message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system executing the action
 * @param target_component The component executing the action
 * @param action The action id
 */
static inline void mavlink_msg_action_send(mavlink_channel_t chan, uint8_t target, uint8_t target_component, uint8_t action)
{
	mavlink_header_t hdr;
	mavlink_action_t payload;

	MAVLINK_BUFFER_CHECK_START( chan, MAVLINK_MSG_ID_ACTION_LEN )
	payload.target = target;	// uint8_t:The system executing the action
	payload.target_component = target_component;	// uint8_t:The component executing the action
	payload.action = action;	// uint8_t:The action id

	hdr.STX = MAVLINK_STX;
	hdr.len = MAVLINK_MSG_ID_ACTION_LEN;
	hdr.msgid = MAVLINK_MSG_ID_ACTION;
	hdr.sysid = mavlink_system.sysid;
	hdr.compid = mavlink_system.compid;
	hdr.seq = mavlink_get_channel_status(chan)->current_tx_seq;
	mavlink_get_channel_status(chan)->current_tx_seq = hdr.seq + 1;
	mavlink_send_mem(chan, (uint8_t *)&hdr.STX, MAVLINK_NUM_HEADER_BYTES );
	mavlink_send_mem(chan, (uint8_t *)&payload, sizeof(payload) );

	crc_init(&hdr.ck);
	crc_calculate_mem((uint8_t *)&hdr.len, &hdr.ck, MAVLINK_CORE_HEADER_LEN);
	crc_calculate_mem((uint8_t *)&payload, &hdr.ck, hdr.len );
	crc_accumulate( 0x2C, &hdr.ck); /// include key in X25 checksum
	mavlink_send_mem(chan, (uint8_t *)&hdr.ck, MAVLINK_NUM_CHECKSUM_BYTES);
	MAVLINK_BUFFER_CHECK_END
}

#endif
// MESSAGE ACTION UNPACKING

/**
 * @brief Get field target from action message
 *
 * @return The system executing the action
 */
static inline uint8_t mavlink_msg_action_get_target(const mavlink_message_t* msg)
{
	mavlink_action_t *p = (mavlink_action_t *)&msg->payload[0];
	return (uint8_t)(p->target);
}

/**
 * @brief Get field target_component from action message
 *
 * @return The component executing the action
 */
static inline uint8_t mavlink_msg_action_get_target_component(const mavlink_message_t* msg)
{
	mavlink_action_t *p = (mavlink_action_t *)&msg->payload[0];
	return (uint8_t)(p->target_component);
}

/**
 * @brief Get field action from action message
 *
 * @return The action id
 */
static inline uint8_t mavlink_msg_action_get_action(const mavlink_message_t* msg)
{
	mavlink_action_t *p = (mavlink_action_t *)&msg->payload[0];
	return (uint8_t)(p->action);
}

/**
 * @brief Decode a action message into a struct
 *
 * @param msg The message to decode
 * @param action C-struct to decode the message contents into
 */
static inline void mavlink_msg_action_decode(const mavlink_message_t* msg, mavlink_action_t* action)
{
	memcpy( action, msg->payload, sizeof(mavlink_action_t));
}
