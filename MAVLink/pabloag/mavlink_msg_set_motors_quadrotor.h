// MESSAGE SET_MOTORS_QUADROTOR PACKING

#define MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR 188

typedef struct __mavlink_set_motors_quadrotor_t
{
 float ref_motor_1; ///< Reference for motor 1, normalized to 0 .. 1
 float ref_motor_2; ///< Reference for motor 2, normalized to 0 .. 1
 float ref_motor_3; ///< Reference for motor 3, normalized to 0 .. 1
 float ref_motor_4; ///< Reference for motor 4, normalized to 0 .. 1
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
} mavlink_set_motors_quadrotor_t;

#define MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN 18
#define MAVLINK_MSG_ID_188_LEN 18

#define MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_CRC 83
#define MAVLINK_MSG_ID_188_CRC 83



#define MAVLINK_MESSAGE_INFO_SET_MOTORS_QUADROTOR { \
	"SET_MOTORS_QUADROTOR", \
	6, \
	{  { "ref_motor_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_motors_quadrotor_t, ref_motor_1) }, \
         { "ref_motor_2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_motors_quadrotor_t, ref_motor_2) }, \
         { "ref_motor_3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_motors_quadrotor_t, ref_motor_3) }, \
         { "ref_motor_4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_motors_quadrotor_t, ref_motor_4) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_set_motors_quadrotor_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_set_motors_quadrotor_t, target_component) }, \
         } \
}


/**
 * @brief Pack a set_motors_quadrotor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ref_motor_1 Reference for motor 1, normalized to 0 .. 1
 * @param ref_motor_2 Reference for motor 2, normalized to 0 .. 1
 * @param ref_motor_3 Reference for motor 3, normalized to 0 .. 1
 * @param ref_motor_4 Reference for motor 4, normalized to 0 .. 1
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_motors_quadrotor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float ref_motor_1, float ref_motor_2, float ref_motor_3, float ref_motor_4, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN];
	_mav_put_float(buf, 0, ref_motor_1);
	_mav_put_float(buf, 4, ref_motor_2);
	_mav_put_float(buf, 8, ref_motor_3);
	_mav_put_float(buf, 12, ref_motor_4);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN);
#else
	mavlink_set_motors_quadrotor_t packet;
	packet.ref_motor_1 = ref_motor_1;
	packet.ref_motor_2 = ref_motor_2;
	packet.ref_motor_3 = ref_motor_3;
	packet.ref_motor_4 = ref_motor_4;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN);
#endif
}

/**
 * @brief Pack a set_motors_quadrotor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ref_motor_1 Reference for motor 1, normalized to 0 .. 1
 * @param ref_motor_2 Reference for motor 2, normalized to 0 .. 1
 * @param ref_motor_3 Reference for motor 3, normalized to 0 .. 1
 * @param ref_motor_4 Reference for motor 4, normalized to 0 .. 1
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_motors_quadrotor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float ref_motor_1,float ref_motor_2,float ref_motor_3,float ref_motor_4,uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN];
	_mav_put_float(buf, 0, ref_motor_1);
	_mav_put_float(buf, 4, ref_motor_2);
	_mav_put_float(buf, 8, ref_motor_3);
	_mav_put_float(buf, 12, ref_motor_4);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN);
#else
	mavlink_set_motors_quadrotor_t packet;
	packet.ref_motor_1 = ref_motor_1;
	packet.ref_motor_2 = ref_motor_2;
	packet.ref_motor_3 = ref_motor_3;
	packet.ref_motor_4 = ref_motor_4;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN);
#endif
}

/**
 * @brief Encode a set_motors_quadrotor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_motors_quadrotor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_motors_quadrotor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_motors_quadrotor_t* set_motors_quadrotor)
{
	return mavlink_msg_set_motors_quadrotor_pack(system_id, component_id, msg, set_motors_quadrotor->ref_motor_1, set_motors_quadrotor->ref_motor_2, set_motors_quadrotor->ref_motor_3, set_motors_quadrotor->ref_motor_4, set_motors_quadrotor->target_system, set_motors_quadrotor->target_component);
}

/**
 * @brief Encode a set_motors_quadrotor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_motors_quadrotor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_motors_quadrotor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_motors_quadrotor_t* set_motors_quadrotor)
{
	return mavlink_msg_set_motors_quadrotor_pack_chan(system_id, component_id, chan, msg, set_motors_quadrotor->ref_motor_1, set_motors_quadrotor->ref_motor_2, set_motors_quadrotor->ref_motor_3, set_motors_quadrotor->ref_motor_4, set_motors_quadrotor->target_system, set_motors_quadrotor->target_component);
}

/**
 * @brief Send a set_motors_quadrotor message
 * @param chan MAVLink channel to send the message
 *
 * @param ref_motor_1 Reference for motor 1, normalized to 0 .. 1
 * @param ref_motor_2 Reference for motor 2, normalized to 0 .. 1
 * @param ref_motor_3 Reference for motor 3, normalized to 0 .. 1
 * @param ref_motor_4 Reference for motor 4, normalized to 0 .. 1
 * @param target_system System ID
 * @param target_component Component ID
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_motors_quadrotor_send(mavlink_channel_t chan, float ref_motor_1, float ref_motor_2, float ref_motor_3, float ref_motor_4, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN];
	_mav_put_float(buf, 0, ref_motor_1);
	_mav_put_float(buf, 4, ref_motor_2);
	_mav_put_float(buf, 8, ref_motor_3);
	_mav_put_float(buf, 12, ref_motor_4);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR, buf, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR, buf, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN);
#endif
#else
	mavlink_set_motors_quadrotor_t packet;
	packet.ref_motor_1 = ref_motor_1;
	packet.ref_motor_2 = ref_motor_2;
	packet.ref_motor_3 = ref_motor_3;
	packet.ref_motor_4 = ref_motor_4;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR, (const char *)&packet, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR, (const char *)&packet, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_motors_quadrotor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float ref_motor_1, float ref_motor_2, float ref_motor_3, float ref_motor_4, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, ref_motor_1);
	_mav_put_float(buf, 4, ref_motor_2);
	_mav_put_float(buf, 8, ref_motor_3);
	_mav_put_float(buf, 12, ref_motor_4);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR, buf, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR, buf, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN);
#endif
#else
	mavlink_set_motors_quadrotor_t *packet = (mavlink_set_motors_quadrotor_t *)msgbuf;
	packet->ref_motor_1 = ref_motor_1;
	packet->ref_motor_2 = ref_motor_2;
	packet->ref_motor_3 = ref_motor_3;
	packet->ref_motor_4 = ref_motor_4;
	packet->target_system = target_system;
	packet->target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR, (const char *)packet, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR, (const char *)packet, MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SET_MOTORS_QUADROTOR UNPACKING


/**
 * @brief Get field ref_motor_1 from set_motors_quadrotor message
 *
 * @return Reference for motor 1, normalized to 0 .. 1
 */
static inline float mavlink_msg_set_motors_quadrotor_get_ref_motor_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ref_motor_2 from set_motors_quadrotor message
 *
 * @return Reference for motor 2, normalized to 0 .. 1
 */
static inline float mavlink_msg_set_motors_quadrotor_get_ref_motor_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field ref_motor_3 from set_motors_quadrotor message
 *
 * @return Reference for motor 3, normalized to 0 .. 1
 */
static inline float mavlink_msg_set_motors_quadrotor_get_ref_motor_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field ref_motor_4 from set_motors_quadrotor message
 *
 * @return Reference for motor 4, normalized to 0 .. 1
 */
static inline float mavlink_msg_set_motors_quadrotor_get_ref_motor_4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field target_system from set_motors_quadrotor message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_motors_quadrotor_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from set_motors_quadrotor message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_motors_quadrotor_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Decode a set_motors_quadrotor message into a struct
 *
 * @param msg The message to decode
 * @param set_motors_quadrotor C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_motors_quadrotor_decode(const mavlink_message_t* msg, mavlink_set_motors_quadrotor_t* set_motors_quadrotor)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_motors_quadrotor->ref_motor_1 = mavlink_msg_set_motors_quadrotor_get_ref_motor_1(msg);
	set_motors_quadrotor->ref_motor_2 = mavlink_msg_set_motors_quadrotor_get_ref_motor_2(msg);
	set_motors_quadrotor->ref_motor_3 = mavlink_msg_set_motors_quadrotor_get_ref_motor_3(msg);
	set_motors_quadrotor->ref_motor_4 = mavlink_msg_set_motors_quadrotor_get_ref_motor_4(msg);
	set_motors_quadrotor->target_system = mavlink_msg_set_motors_quadrotor_get_target_system(msg);
	set_motors_quadrotor->target_component = mavlink_msg_set_motors_quadrotor_get_target_component(msg);
#else
	memcpy(set_motors_quadrotor, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_MOTORS_QUADROTOR_LEN);
#endif
}
