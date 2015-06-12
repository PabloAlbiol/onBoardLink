// MESSAGE SET_REFERENCES_QUADROTOR PACKING

#define MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR 186

typedef struct __mavlink_set_references_quadrotor_t
{
 float roll; ///< Roll angle in degrees
 float pitch; ///< Pitch angle in degrees
 float yaw; ///< Yaw angle in degrees
 float thrust; ///< Thrust, normalized to 0 .. 1
 float x; ///< x position in meteres
 float y; ///< y position in meteres
 float z; ///< z position in meteres
 float velocity; ///< Velocity in km/h
 uint8_t bitmask; ///< Custom bitmask
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
} mavlink_set_references_quadrotor_t;

#define MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN 35
#define MAVLINK_MSG_ID_186_LEN 35

#define MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_CRC 232
#define MAVLINK_MSG_ID_186_CRC 232



#define MAVLINK_MESSAGE_INFO_SET_REFERENCES_QUADROTOR { \
	"SET_REFERENCES_QUADROTOR", \
	11, \
	{  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_references_quadrotor_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_references_quadrotor_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_references_quadrotor_t, yaw) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_references_quadrotor_t, thrust) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_set_references_quadrotor_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_set_references_quadrotor_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_set_references_quadrotor_t, z) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_set_references_quadrotor_t, velocity) }, \
         { "bitmask", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_set_references_quadrotor_t, bitmask) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_set_references_quadrotor_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_set_references_quadrotor_t, target_component) }, \
         } \
}


/**
 * @brief Pack a set_references_quadrotor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll Roll angle in degrees
 * @param pitch Pitch angle in degrees
 * @param yaw Yaw angle in degrees
 * @param thrust Thrust, normalized to 0 .. 1
 * @param x x position in meteres
 * @param y y position in meteres
 * @param z z position in meteres
 * @param velocity Velocity in km/h
 * @param bitmask Custom bitmask
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_references_quadrotor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float roll, float pitch, float yaw, float thrust, float x, float y, float z, float velocity, uint8_t bitmask, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, thrust);
	_mav_put_float(buf, 16, x);
	_mav_put_float(buf, 20, y);
	_mav_put_float(buf, 24, z);
	_mav_put_float(buf, 28, velocity);
	_mav_put_uint8_t(buf, 32, bitmask);
	_mav_put_uint8_t(buf, 33, target_system);
	_mav_put_uint8_t(buf, 34, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN);
#else
	mavlink_set_references_quadrotor_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.velocity = velocity;
	packet.bitmask = bitmask;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN);
#endif
}

/**
 * @brief Pack a set_references_quadrotor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll Roll angle in degrees
 * @param pitch Pitch angle in degrees
 * @param yaw Yaw angle in degrees
 * @param thrust Thrust, normalized to 0 .. 1
 * @param x x position in meteres
 * @param y y position in meteres
 * @param z z position in meteres
 * @param velocity Velocity in km/h
 * @param bitmask Custom bitmask
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_references_quadrotor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float roll,float pitch,float yaw,float thrust,float x,float y,float z,float velocity,uint8_t bitmask,uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, thrust);
	_mav_put_float(buf, 16, x);
	_mav_put_float(buf, 20, y);
	_mav_put_float(buf, 24, z);
	_mav_put_float(buf, 28, velocity);
	_mav_put_uint8_t(buf, 32, bitmask);
	_mav_put_uint8_t(buf, 33, target_system);
	_mav_put_uint8_t(buf, 34, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN);
#else
	mavlink_set_references_quadrotor_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.velocity = velocity;
	packet.bitmask = bitmask;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN);
#endif
}

/**
 * @brief Encode a set_references_quadrotor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_references_quadrotor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_references_quadrotor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_references_quadrotor_t* set_references_quadrotor)
{
	return mavlink_msg_set_references_quadrotor_pack(system_id, component_id, msg, set_references_quadrotor->roll, set_references_quadrotor->pitch, set_references_quadrotor->yaw, set_references_quadrotor->thrust, set_references_quadrotor->x, set_references_quadrotor->y, set_references_quadrotor->z, set_references_quadrotor->velocity, set_references_quadrotor->bitmask, set_references_quadrotor->target_system, set_references_quadrotor->target_component);
}

/**
 * @brief Encode a set_references_quadrotor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_references_quadrotor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_references_quadrotor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_references_quadrotor_t* set_references_quadrotor)
{
	return mavlink_msg_set_references_quadrotor_pack_chan(system_id, component_id, chan, msg, set_references_quadrotor->roll, set_references_quadrotor->pitch, set_references_quadrotor->yaw, set_references_quadrotor->thrust, set_references_quadrotor->x, set_references_quadrotor->y, set_references_quadrotor->z, set_references_quadrotor->velocity, set_references_quadrotor->bitmask, set_references_quadrotor->target_system, set_references_quadrotor->target_component);
}

/**
 * @brief Send a set_references_quadrotor message
 * @param chan MAVLink channel to send the message
 *
 * @param roll Roll angle in degrees
 * @param pitch Pitch angle in degrees
 * @param yaw Yaw angle in degrees
 * @param thrust Thrust, normalized to 0 .. 1
 * @param x x position in meteres
 * @param y y position in meteres
 * @param z z position in meteres
 * @param velocity Velocity in km/h
 * @param bitmask Custom bitmask
 * @param target_system System ID
 * @param target_component Component ID
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_references_quadrotor_send(mavlink_channel_t chan, float roll, float pitch, float yaw, float thrust, float x, float y, float z, float velocity, uint8_t bitmask, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, thrust);
	_mav_put_float(buf, 16, x);
	_mav_put_float(buf, 20, y);
	_mav_put_float(buf, 24, z);
	_mav_put_float(buf, 28, velocity);
	_mav_put_uint8_t(buf, 32, bitmask);
	_mav_put_uint8_t(buf, 33, target_system);
	_mav_put_uint8_t(buf, 34, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR, buf, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR, buf, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN);
#endif
#else
	mavlink_set_references_quadrotor_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.velocity = velocity;
	packet.bitmask = bitmask;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR, (const char *)&packet, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR, (const char *)&packet, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_references_quadrotor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw, float thrust, float x, float y, float z, float velocity, uint8_t bitmask, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, thrust);
	_mav_put_float(buf, 16, x);
	_mav_put_float(buf, 20, y);
	_mav_put_float(buf, 24, z);
	_mav_put_float(buf, 28, velocity);
	_mav_put_uint8_t(buf, 32, bitmask);
	_mav_put_uint8_t(buf, 33, target_system);
	_mav_put_uint8_t(buf, 34, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR, buf, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR, buf, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN);
#endif
#else
	mavlink_set_references_quadrotor_t *packet = (mavlink_set_references_quadrotor_t *)msgbuf;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;
	packet->thrust = thrust;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->velocity = velocity;
	packet->bitmask = bitmask;
	packet->target_system = target_system;
	packet->target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR, (const char *)packet, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR, (const char *)packet, MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SET_REFERENCES_QUADROTOR UNPACKING


/**
 * @brief Get field roll from set_references_quadrotor message
 *
 * @return Roll angle in degrees
 */
static inline float mavlink_msg_set_references_quadrotor_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from set_references_quadrotor message
 *
 * @return Pitch angle in degrees
 */
static inline float mavlink_msg_set_references_quadrotor_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from set_references_quadrotor message
 *
 * @return Yaw angle in degrees
 */
static inline float mavlink_msg_set_references_quadrotor_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field thrust from set_references_quadrotor message
 *
 * @return Thrust, normalized to 0 .. 1
 */
static inline float mavlink_msg_set_references_quadrotor_get_thrust(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field x from set_references_quadrotor message
 *
 * @return x position in meteres
 */
static inline float mavlink_msg_set_references_quadrotor_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field y from set_references_quadrotor message
 *
 * @return y position in meteres
 */
static inline float mavlink_msg_set_references_quadrotor_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field z from set_references_quadrotor message
 *
 * @return z position in meteres
 */
static inline float mavlink_msg_set_references_quadrotor_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field velocity from set_references_quadrotor message
 *
 * @return Velocity in km/h
 */
static inline float mavlink_msg_set_references_quadrotor_get_velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field bitmask from set_references_quadrotor message
 *
 * @return Custom bitmask
 */
static inline uint8_t mavlink_msg_set_references_quadrotor_get_bitmask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field target_system from set_references_quadrotor message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_references_quadrotor_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field target_component from set_references_quadrotor message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_references_quadrotor_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Decode a set_references_quadrotor message into a struct
 *
 * @param msg The message to decode
 * @param set_references_quadrotor C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_references_quadrotor_decode(const mavlink_message_t* msg, mavlink_set_references_quadrotor_t* set_references_quadrotor)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_references_quadrotor->roll = mavlink_msg_set_references_quadrotor_get_roll(msg);
	set_references_quadrotor->pitch = mavlink_msg_set_references_quadrotor_get_pitch(msg);
	set_references_quadrotor->yaw = mavlink_msg_set_references_quadrotor_get_yaw(msg);
	set_references_quadrotor->thrust = mavlink_msg_set_references_quadrotor_get_thrust(msg);
	set_references_quadrotor->x = mavlink_msg_set_references_quadrotor_get_x(msg);
	set_references_quadrotor->y = mavlink_msg_set_references_quadrotor_get_y(msg);
	set_references_quadrotor->z = mavlink_msg_set_references_quadrotor_get_z(msg);
	set_references_quadrotor->velocity = mavlink_msg_set_references_quadrotor_get_velocity(msg);
	set_references_quadrotor->bitmask = mavlink_msg_set_references_quadrotor_get_bitmask(msg);
	set_references_quadrotor->target_system = mavlink_msg_set_references_quadrotor_get_target_system(msg);
	set_references_quadrotor->target_component = mavlink_msg_set_references_quadrotor_get_target_component(msg);
#else
	memcpy(set_references_quadrotor, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_REFERENCES_QUADROTOR_LEN);
#endif
}
