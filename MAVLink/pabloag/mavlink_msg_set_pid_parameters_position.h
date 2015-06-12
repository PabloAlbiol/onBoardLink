// MESSAGE SET_PID_PARAMETERS_POSITION PACKING

#define MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION 183

typedef struct __mavlink_set_pid_parameters_position_t
{
 float x_p; ///< Proportional value for the x coordinate
 float x_d; ///< Derivative value for the x coordinate
 float x_i; ///< Integral value for the x coordinate
 float x_p_sat; ///< Saturation for the proportional value of the x coordinate
 float x_d_sat; ///< Saturation for the derivative value of the x coordinate
 float x_i_sat; ///< Saturation for the integral value of the x coordinate
 float y_p; ///< Proportional value for the y coordinate
 float y_d; ///< Derivative value for the y coordinate
 float y_i; ///< Integral value for the y coordinate
 float y_p_sat; ///< Saturation for the proportional value of the y coordinate
 float y_d_sat; ///< Saturation for the derivative value of the y coordinate
 float y_i_sat; ///< Saturation for the integral value of the y coordinate
 float vel_p; ///< Proportional value for the the velocity
 float vel_d; ///< Derivative value for the velocity
 float vel_i; ///< Integral value for the velocity
 float vel_p_sat; ///< Saturation for the proportional value of the velocity
 float vel_d_sat; ///< Saturation for the derivative value of the velocity
 float vel_i_sat; ///< Saturation for the integral value of the velocity
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
} mavlink_set_pid_parameters_position_t;

#define MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN 74
#define MAVLINK_MSG_ID_183_LEN 74

#define MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_CRC 232
#define MAVLINK_MSG_ID_183_CRC 232



#define MAVLINK_MESSAGE_INFO_SET_PID_PARAMETERS_POSITION { \
	"SET_PID_PARAMETERS_POSITION", \
	20, \
	{  { "x_p", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_pid_parameters_position_t, x_p) }, \
         { "x_d", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_pid_parameters_position_t, x_d) }, \
         { "x_i", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_pid_parameters_position_t, x_i) }, \
         { "x_p_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_pid_parameters_position_t, x_p_sat) }, \
         { "x_d_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_set_pid_parameters_position_t, x_d_sat) }, \
         { "x_i_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_set_pid_parameters_position_t, x_i_sat) }, \
         { "y_p", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_set_pid_parameters_position_t, y_p) }, \
         { "y_d", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_set_pid_parameters_position_t, y_d) }, \
         { "y_i", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_set_pid_parameters_position_t, y_i) }, \
         { "y_p_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_set_pid_parameters_position_t, y_p_sat) }, \
         { "y_d_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_set_pid_parameters_position_t, y_d_sat) }, \
         { "y_i_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_set_pid_parameters_position_t, y_i_sat) }, \
         { "vel_p", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_set_pid_parameters_position_t, vel_p) }, \
         { "vel_d", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_set_pid_parameters_position_t, vel_d) }, \
         { "vel_i", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_set_pid_parameters_position_t, vel_i) }, \
         { "vel_p_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_set_pid_parameters_position_t, vel_p_sat) }, \
         { "vel_d_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_set_pid_parameters_position_t, vel_d_sat) }, \
         { "vel_i_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_set_pid_parameters_position_t, vel_i_sat) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_set_pid_parameters_position_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 73, offsetof(mavlink_set_pid_parameters_position_t, target_component) }, \
         } \
}


/**
 * @brief Pack a set_pid_parameters_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x_p Proportional value for the x coordinate
 * @param x_d Derivative value for the x coordinate
 * @param x_i Integral value for the x coordinate
 * @param x_p_sat Saturation for the proportional value of the x coordinate
 * @param x_d_sat Saturation for the derivative value of the x coordinate
 * @param x_i_sat Saturation for the integral value of the x coordinate
 * @param y_p Proportional value for the y coordinate
 * @param y_d Derivative value for the y coordinate
 * @param y_i Integral value for the y coordinate
 * @param y_p_sat Saturation for the proportional value of the y coordinate
 * @param y_d_sat Saturation for the derivative value of the y coordinate
 * @param y_i_sat Saturation for the integral value of the y coordinate
 * @param vel_p Proportional value for the the velocity
 * @param vel_d Derivative value for the velocity
 * @param vel_i Integral value for the velocity
 * @param vel_p_sat Saturation for the proportional value of the velocity
 * @param vel_d_sat Saturation for the derivative value of the velocity
 * @param vel_i_sat Saturation for the integral value of the velocity
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_pid_parameters_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float x_p, float x_d, float x_i, float x_p_sat, float x_d_sat, float x_i_sat, float y_p, float y_d, float y_i, float y_p_sat, float y_d_sat, float y_i_sat, float vel_p, float vel_d, float vel_i, float vel_p_sat, float vel_d_sat, float vel_i_sat, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN];
	_mav_put_float(buf, 0, x_p);
	_mav_put_float(buf, 4, x_d);
	_mav_put_float(buf, 8, x_i);
	_mav_put_float(buf, 12, x_p_sat);
	_mav_put_float(buf, 16, x_d_sat);
	_mav_put_float(buf, 20, x_i_sat);
	_mav_put_float(buf, 24, y_p);
	_mav_put_float(buf, 28, y_d);
	_mav_put_float(buf, 32, y_i);
	_mav_put_float(buf, 36, y_p_sat);
	_mav_put_float(buf, 40, y_d_sat);
	_mav_put_float(buf, 44, y_i_sat);
	_mav_put_float(buf, 48, vel_p);
	_mav_put_float(buf, 52, vel_d);
	_mav_put_float(buf, 56, vel_i);
	_mav_put_float(buf, 60, vel_p_sat);
	_mav_put_float(buf, 64, vel_d_sat);
	_mav_put_float(buf, 68, vel_i_sat);
	_mav_put_uint8_t(buf, 72, target_system);
	_mav_put_uint8_t(buf, 73, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN);
#else
	mavlink_set_pid_parameters_position_t packet;
	packet.x_p = x_p;
	packet.x_d = x_d;
	packet.x_i = x_i;
	packet.x_p_sat = x_p_sat;
	packet.x_d_sat = x_d_sat;
	packet.x_i_sat = x_i_sat;
	packet.y_p = y_p;
	packet.y_d = y_d;
	packet.y_i = y_i;
	packet.y_p_sat = y_p_sat;
	packet.y_d_sat = y_d_sat;
	packet.y_i_sat = y_i_sat;
	packet.vel_p = vel_p;
	packet.vel_d = vel_d;
	packet.vel_i = vel_i;
	packet.vel_p_sat = vel_p_sat;
	packet.vel_d_sat = vel_d_sat;
	packet.vel_i_sat = vel_i_sat;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN);
#endif
}

/**
 * @brief Pack a set_pid_parameters_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x_p Proportional value for the x coordinate
 * @param x_d Derivative value for the x coordinate
 * @param x_i Integral value for the x coordinate
 * @param x_p_sat Saturation for the proportional value of the x coordinate
 * @param x_d_sat Saturation for the derivative value of the x coordinate
 * @param x_i_sat Saturation for the integral value of the x coordinate
 * @param y_p Proportional value for the y coordinate
 * @param y_d Derivative value for the y coordinate
 * @param y_i Integral value for the y coordinate
 * @param y_p_sat Saturation for the proportional value of the y coordinate
 * @param y_d_sat Saturation for the derivative value of the y coordinate
 * @param y_i_sat Saturation for the integral value of the y coordinate
 * @param vel_p Proportional value for the the velocity
 * @param vel_d Derivative value for the velocity
 * @param vel_i Integral value for the velocity
 * @param vel_p_sat Saturation for the proportional value of the velocity
 * @param vel_d_sat Saturation for the derivative value of the velocity
 * @param vel_i_sat Saturation for the integral value of the velocity
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_pid_parameters_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float x_p,float x_d,float x_i,float x_p_sat,float x_d_sat,float x_i_sat,float y_p,float y_d,float y_i,float y_p_sat,float y_d_sat,float y_i_sat,float vel_p,float vel_d,float vel_i,float vel_p_sat,float vel_d_sat,float vel_i_sat,uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN];
	_mav_put_float(buf, 0, x_p);
	_mav_put_float(buf, 4, x_d);
	_mav_put_float(buf, 8, x_i);
	_mav_put_float(buf, 12, x_p_sat);
	_mav_put_float(buf, 16, x_d_sat);
	_mav_put_float(buf, 20, x_i_sat);
	_mav_put_float(buf, 24, y_p);
	_mav_put_float(buf, 28, y_d);
	_mav_put_float(buf, 32, y_i);
	_mav_put_float(buf, 36, y_p_sat);
	_mav_put_float(buf, 40, y_d_sat);
	_mav_put_float(buf, 44, y_i_sat);
	_mav_put_float(buf, 48, vel_p);
	_mav_put_float(buf, 52, vel_d);
	_mav_put_float(buf, 56, vel_i);
	_mav_put_float(buf, 60, vel_p_sat);
	_mav_put_float(buf, 64, vel_d_sat);
	_mav_put_float(buf, 68, vel_i_sat);
	_mav_put_uint8_t(buf, 72, target_system);
	_mav_put_uint8_t(buf, 73, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN);
#else
	mavlink_set_pid_parameters_position_t packet;
	packet.x_p = x_p;
	packet.x_d = x_d;
	packet.x_i = x_i;
	packet.x_p_sat = x_p_sat;
	packet.x_d_sat = x_d_sat;
	packet.x_i_sat = x_i_sat;
	packet.y_p = y_p;
	packet.y_d = y_d;
	packet.y_i = y_i;
	packet.y_p_sat = y_p_sat;
	packet.y_d_sat = y_d_sat;
	packet.y_i_sat = y_i_sat;
	packet.vel_p = vel_p;
	packet.vel_d = vel_d;
	packet.vel_i = vel_i;
	packet.vel_p_sat = vel_p_sat;
	packet.vel_d_sat = vel_d_sat;
	packet.vel_i_sat = vel_i_sat;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN);
#endif
}

/**
 * @brief Encode a set_pid_parameters_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_pid_parameters_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_pid_parameters_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_pid_parameters_position_t* set_pid_parameters_position)
{
	return mavlink_msg_set_pid_parameters_position_pack(system_id, component_id, msg, set_pid_parameters_position->x_p, set_pid_parameters_position->x_d, set_pid_parameters_position->x_i, set_pid_parameters_position->x_p_sat, set_pid_parameters_position->x_d_sat, set_pid_parameters_position->x_i_sat, set_pid_parameters_position->y_p, set_pid_parameters_position->y_d, set_pid_parameters_position->y_i, set_pid_parameters_position->y_p_sat, set_pid_parameters_position->y_d_sat, set_pid_parameters_position->y_i_sat, set_pid_parameters_position->vel_p, set_pid_parameters_position->vel_d, set_pid_parameters_position->vel_i, set_pid_parameters_position->vel_p_sat, set_pid_parameters_position->vel_d_sat, set_pid_parameters_position->vel_i_sat, set_pid_parameters_position->target_system, set_pid_parameters_position->target_component);
}

/**
 * @brief Encode a set_pid_parameters_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_pid_parameters_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_pid_parameters_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_pid_parameters_position_t* set_pid_parameters_position)
{
	return mavlink_msg_set_pid_parameters_position_pack_chan(system_id, component_id, chan, msg, set_pid_parameters_position->x_p, set_pid_parameters_position->x_d, set_pid_parameters_position->x_i, set_pid_parameters_position->x_p_sat, set_pid_parameters_position->x_d_sat, set_pid_parameters_position->x_i_sat, set_pid_parameters_position->y_p, set_pid_parameters_position->y_d, set_pid_parameters_position->y_i, set_pid_parameters_position->y_p_sat, set_pid_parameters_position->y_d_sat, set_pid_parameters_position->y_i_sat, set_pid_parameters_position->vel_p, set_pid_parameters_position->vel_d, set_pid_parameters_position->vel_i, set_pid_parameters_position->vel_p_sat, set_pid_parameters_position->vel_d_sat, set_pid_parameters_position->vel_i_sat, set_pid_parameters_position->target_system, set_pid_parameters_position->target_component);
}

/**
 * @brief Send a set_pid_parameters_position message
 * @param chan MAVLink channel to send the message
 *
 * @param x_p Proportional value for the x coordinate
 * @param x_d Derivative value for the x coordinate
 * @param x_i Integral value for the x coordinate
 * @param x_p_sat Saturation for the proportional value of the x coordinate
 * @param x_d_sat Saturation for the derivative value of the x coordinate
 * @param x_i_sat Saturation for the integral value of the x coordinate
 * @param y_p Proportional value for the y coordinate
 * @param y_d Derivative value for the y coordinate
 * @param y_i Integral value for the y coordinate
 * @param y_p_sat Saturation for the proportional value of the y coordinate
 * @param y_d_sat Saturation for the derivative value of the y coordinate
 * @param y_i_sat Saturation for the integral value of the y coordinate
 * @param vel_p Proportional value for the the velocity
 * @param vel_d Derivative value for the velocity
 * @param vel_i Integral value for the velocity
 * @param vel_p_sat Saturation for the proportional value of the velocity
 * @param vel_d_sat Saturation for the derivative value of the velocity
 * @param vel_i_sat Saturation for the integral value of the velocity
 * @param target_system System ID
 * @param target_component Component ID
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_pid_parameters_position_send(mavlink_channel_t chan, float x_p, float x_d, float x_i, float x_p_sat, float x_d_sat, float x_i_sat, float y_p, float y_d, float y_i, float y_p_sat, float y_d_sat, float y_i_sat, float vel_p, float vel_d, float vel_i, float vel_p_sat, float vel_d_sat, float vel_i_sat, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN];
	_mav_put_float(buf, 0, x_p);
	_mav_put_float(buf, 4, x_d);
	_mav_put_float(buf, 8, x_i);
	_mav_put_float(buf, 12, x_p_sat);
	_mav_put_float(buf, 16, x_d_sat);
	_mav_put_float(buf, 20, x_i_sat);
	_mav_put_float(buf, 24, y_p);
	_mav_put_float(buf, 28, y_d);
	_mav_put_float(buf, 32, y_i);
	_mav_put_float(buf, 36, y_p_sat);
	_mav_put_float(buf, 40, y_d_sat);
	_mav_put_float(buf, 44, y_i_sat);
	_mav_put_float(buf, 48, vel_p);
	_mav_put_float(buf, 52, vel_d);
	_mav_put_float(buf, 56, vel_i);
	_mav_put_float(buf, 60, vel_p_sat);
	_mav_put_float(buf, 64, vel_d_sat);
	_mav_put_float(buf, 68, vel_i_sat);
	_mav_put_uint8_t(buf, 72, target_system);
	_mav_put_uint8_t(buf, 73, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION, buf, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION, buf, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN);
#endif
#else
	mavlink_set_pid_parameters_position_t packet;
	packet.x_p = x_p;
	packet.x_d = x_d;
	packet.x_i = x_i;
	packet.x_p_sat = x_p_sat;
	packet.x_d_sat = x_d_sat;
	packet.x_i_sat = x_i_sat;
	packet.y_p = y_p;
	packet.y_d = y_d;
	packet.y_i = y_i;
	packet.y_p_sat = y_p_sat;
	packet.y_d_sat = y_d_sat;
	packet.y_i_sat = y_i_sat;
	packet.vel_p = vel_p;
	packet.vel_d = vel_d;
	packet.vel_i = vel_i;
	packet.vel_p_sat = vel_p_sat;
	packet.vel_d_sat = vel_d_sat;
	packet.vel_i_sat = vel_i_sat;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION, (const char *)&packet, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION, (const char *)&packet, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_pid_parameters_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x_p, float x_d, float x_i, float x_p_sat, float x_d_sat, float x_i_sat, float y_p, float y_d, float y_i, float y_p_sat, float y_d_sat, float y_i_sat, float vel_p, float vel_d, float vel_i, float vel_p_sat, float vel_d_sat, float vel_i_sat, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, x_p);
	_mav_put_float(buf, 4, x_d);
	_mav_put_float(buf, 8, x_i);
	_mav_put_float(buf, 12, x_p_sat);
	_mav_put_float(buf, 16, x_d_sat);
	_mav_put_float(buf, 20, x_i_sat);
	_mav_put_float(buf, 24, y_p);
	_mav_put_float(buf, 28, y_d);
	_mav_put_float(buf, 32, y_i);
	_mav_put_float(buf, 36, y_p_sat);
	_mav_put_float(buf, 40, y_d_sat);
	_mav_put_float(buf, 44, y_i_sat);
	_mav_put_float(buf, 48, vel_p);
	_mav_put_float(buf, 52, vel_d);
	_mav_put_float(buf, 56, vel_i);
	_mav_put_float(buf, 60, vel_p_sat);
	_mav_put_float(buf, 64, vel_d_sat);
	_mav_put_float(buf, 68, vel_i_sat);
	_mav_put_uint8_t(buf, 72, target_system);
	_mav_put_uint8_t(buf, 73, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION, buf, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION, buf, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN);
#endif
#else
	mavlink_set_pid_parameters_position_t *packet = (mavlink_set_pid_parameters_position_t *)msgbuf;
	packet->x_p = x_p;
	packet->x_d = x_d;
	packet->x_i = x_i;
	packet->x_p_sat = x_p_sat;
	packet->x_d_sat = x_d_sat;
	packet->x_i_sat = x_i_sat;
	packet->y_p = y_p;
	packet->y_d = y_d;
	packet->y_i = y_i;
	packet->y_p_sat = y_p_sat;
	packet->y_d_sat = y_d_sat;
	packet->y_i_sat = y_i_sat;
	packet->vel_p = vel_p;
	packet->vel_d = vel_d;
	packet->vel_i = vel_i;
	packet->vel_p_sat = vel_p_sat;
	packet->vel_d_sat = vel_d_sat;
	packet->vel_i_sat = vel_i_sat;
	packet->target_system = target_system;
	packet->target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION, (const char *)packet, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION, (const char *)packet, MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SET_PID_PARAMETERS_POSITION UNPACKING


/**
 * @brief Get field x_p from set_pid_parameters_position message
 *
 * @return Proportional value for the x coordinate
 */
static inline float mavlink_msg_set_pid_parameters_position_get_x_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field x_d from set_pid_parameters_position message
 *
 * @return Derivative value for the x coordinate
 */
static inline float mavlink_msg_set_pid_parameters_position_get_x_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field x_i from set_pid_parameters_position message
 *
 * @return Integral value for the x coordinate
 */
static inline float mavlink_msg_set_pid_parameters_position_get_x_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field x_p_sat from set_pid_parameters_position message
 *
 * @return Saturation for the proportional value of the x coordinate
 */
static inline float mavlink_msg_set_pid_parameters_position_get_x_p_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field x_d_sat from set_pid_parameters_position message
 *
 * @return Saturation for the derivative value of the x coordinate
 */
static inline float mavlink_msg_set_pid_parameters_position_get_x_d_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field x_i_sat from set_pid_parameters_position message
 *
 * @return Saturation for the integral value of the x coordinate
 */
static inline float mavlink_msg_set_pid_parameters_position_get_x_i_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field y_p from set_pid_parameters_position message
 *
 * @return Proportional value for the y coordinate
 */
static inline float mavlink_msg_set_pid_parameters_position_get_y_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field y_d from set_pid_parameters_position message
 *
 * @return Derivative value for the y coordinate
 */
static inline float mavlink_msg_set_pid_parameters_position_get_y_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field y_i from set_pid_parameters_position message
 *
 * @return Integral value for the y coordinate
 */
static inline float mavlink_msg_set_pid_parameters_position_get_y_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field y_p_sat from set_pid_parameters_position message
 *
 * @return Saturation for the proportional value of the y coordinate
 */
static inline float mavlink_msg_set_pid_parameters_position_get_y_p_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field y_d_sat from set_pid_parameters_position message
 *
 * @return Saturation for the derivative value of the y coordinate
 */
static inline float mavlink_msg_set_pid_parameters_position_get_y_d_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field y_i_sat from set_pid_parameters_position message
 *
 * @return Saturation for the integral value of the y coordinate
 */
static inline float mavlink_msg_set_pid_parameters_position_get_y_i_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field vel_p from set_pid_parameters_position message
 *
 * @return Proportional value for the the velocity
 */
static inline float mavlink_msg_set_pid_parameters_position_get_vel_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field vel_d from set_pid_parameters_position message
 *
 * @return Derivative value for the velocity
 */
static inline float mavlink_msg_set_pid_parameters_position_get_vel_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field vel_i from set_pid_parameters_position message
 *
 * @return Integral value for the velocity
 */
static inline float mavlink_msg_set_pid_parameters_position_get_vel_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field vel_p_sat from set_pid_parameters_position message
 *
 * @return Saturation for the proportional value of the velocity
 */
static inline float mavlink_msg_set_pid_parameters_position_get_vel_p_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field vel_d_sat from set_pid_parameters_position message
 *
 * @return Saturation for the derivative value of the velocity
 */
static inline float mavlink_msg_set_pid_parameters_position_get_vel_d_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field vel_i_sat from set_pid_parameters_position message
 *
 * @return Saturation for the integral value of the velocity
 */
static inline float mavlink_msg_set_pid_parameters_position_get_vel_i_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field target_system from set_pid_parameters_position message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_pid_parameters_position_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  72);
}

/**
 * @brief Get field target_component from set_pid_parameters_position message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_pid_parameters_position_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  73);
}

/**
 * @brief Decode a set_pid_parameters_position message into a struct
 *
 * @param msg The message to decode
 * @param set_pid_parameters_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_pid_parameters_position_decode(const mavlink_message_t* msg, mavlink_set_pid_parameters_position_t* set_pid_parameters_position)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_pid_parameters_position->x_p = mavlink_msg_set_pid_parameters_position_get_x_p(msg);
	set_pid_parameters_position->x_d = mavlink_msg_set_pid_parameters_position_get_x_d(msg);
	set_pid_parameters_position->x_i = mavlink_msg_set_pid_parameters_position_get_x_i(msg);
	set_pid_parameters_position->x_p_sat = mavlink_msg_set_pid_parameters_position_get_x_p_sat(msg);
	set_pid_parameters_position->x_d_sat = mavlink_msg_set_pid_parameters_position_get_x_d_sat(msg);
	set_pid_parameters_position->x_i_sat = mavlink_msg_set_pid_parameters_position_get_x_i_sat(msg);
	set_pid_parameters_position->y_p = mavlink_msg_set_pid_parameters_position_get_y_p(msg);
	set_pid_parameters_position->y_d = mavlink_msg_set_pid_parameters_position_get_y_d(msg);
	set_pid_parameters_position->y_i = mavlink_msg_set_pid_parameters_position_get_y_i(msg);
	set_pid_parameters_position->y_p_sat = mavlink_msg_set_pid_parameters_position_get_y_p_sat(msg);
	set_pid_parameters_position->y_d_sat = mavlink_msg_set_pid_parameters_position_get_y_d_sat(msg);
	set_pid_parameters_position->y_i_sat = mavlink_msg_set_pid_parameters_position_get_y_i_sat(msg);
	set_pid_parameters_position->vel_p = mavlink_msg_set_pid_parameters_position_get_vel_p(msg);
	set_pid_parameters_position->vel_d = mavlink_msg_set_pid_parameters_position_get_vel_d(msg);
	set_pid_parameters_position->vel_i = mavlink_msg_set_pid_parameters_position_get_vel_i(msg);
	set_pid_parameters_position->vel_p_sat = mavlink_msg_set_pid_parameters_position_get_vel_p_sat(msg);
	set_pid_parameters_position->vel_d_sat = mavlink_msg_set_pid_parameters_position_get_vel_d_sat(msg);
	set_pid_parameters_position->vel_i_sat = mavlink_msg_set_pid_parameters_position_get_vel_i_sat(msg);
	set_pid_parameters_position->target_system = mavlink_msg_set_pid_parameters_position_get_target_system(msg);
	set_pid_parameters_position->target_component = mavlink_msg_set_pid_parameters_position_get_target_component(msg);
#else
	memcpy(set_pid_parameters_position, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_PID_PARAMETERS_POSITION_LEN);
#endif
}
