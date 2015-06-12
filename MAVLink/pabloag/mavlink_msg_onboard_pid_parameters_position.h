// MESSAGE ONBOARD_PID_PARAMETERS_POSITION PACKING

#define MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION 184

typedef struct __mavlink_onboard_pid_parameters_position_t
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
} mavlink_onboard_pid_parameters_position_t;

#define MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN 72
#define MAVLINK_MSG_ID_184_LEN 72

#define MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_CRC 227
#define MAVLINK_MSG_ID_184_CRC 227



#define MAVLINK_MESSAGE_INFO_ONBOARD_PID_PARAMETERS_POSITION { \
	"ONBOARD_PID_PARAMETERS_POSITION", \
	18, \
	{  { "x_p", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_onboard_pid_parameters_position_t, x_p) }, \
         { "x_d", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_onboard_pid_parameters_position_t, x_d) }, \
         { "x_i", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_onboard_pid_parameters_position_t, x_i) }, \
         { "x_p_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_onboard_pid_parameters_position_t, x_p_sat) }, \
         { "x_d_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_onboard_pid_parameters_position_t, x_d_sat) }, \
         { "x_i_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_onboard_pid_parameters_position_t, x_i_sat) }, \
         { "y_p", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_onboard_pid_parameters_position_t, y_p) }, \
         { "y_d", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_onboard_pid_parameters_position_t, y_d) }, \
         { "y_i", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_onboard_pid_parameters_position_t, y_i) }, \
         { "y_p_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_onboard_pid_parameters_position_t, y_p_sat) }, \
         { "y_d_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_onboard_pid_parameters_position_t, y_d_sat) }, \
         { "y_i_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_onboard_pid_parameters_position_t, y_i_sat) }, \
         { "vel_p", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_onboard_pid_parameters_position_t, vel_p) }, \
         { "vel_d", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_onboard_pid_parameters_position_t, vel_d) }, \
         { "vel_i", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_onboard_pid_parameters_position_t, vel_i) }, \
         { "vel_p_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_onboard_pid_parameters_position_t, vel_p_sat) }, \
         { "vel_d_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_onboard_pid_parameters_position_t, vel_d_sat) }, \
         { "vel_i_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_onboard_pid_parameters_position_t, vel_i_sat) }, \
         } \
}


/**
 * @brief Pack a onboard_pid_parameters_position message
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
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_onboard_pid_parameters_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float x_p, float x_d, float x_i, float x_p_sat, float x_d_sat, float x_i_sat, float y_p, float y_d, float y_i, float y_p_sat, float y_d_sat, float y_i_sat, float vel_p, float vel_d, float vel_i, float vel_p_sat, float vel_d_sat, float vel_i_sat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN];
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN);
#else
	mavlink_onboard_pid_parameters_position_t packet;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN);
#endif
}

/**
 * @brief Pack a onboard_pid_parameters_position message on a channel
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
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_onboard_pid_parameters_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float x_p,float x_d,float x_i,float x_p_sat,float x_d_sat,float x_i_sat,float y_p,float y_d,float y_i,float y_p_sat,float y_d_sat,float y_i_sat,float vel_p,float vel_d,float vel_i,float vel_p_sat,float vel_d_sat,float vel_i_sat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN];
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN);
#else
	mavlink_onboard_pid_parameters_position_t packet;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN);
#endif
}

/**
 * @brief Encode a onboard_pid_parameters_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param onboard_pid_parameters_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_onboard_pid_parameters_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_onboard_pid_parameters_position_t* onboard_pid_parameters_position)
{
	return mavlink_msg_onboard_pid_parameters_position_pack(system_id, component_id, msg, onboard_pid_parameters_position->x_p, onboard_pid_parameters_position->x_d, onboard_pid_parameters_position->x_i, onboard_pid_parameters_position->x_p_sat, onboard_pid_parameters_position->x_d_sat, onboard_pid_parameters_position->x_i_sat, onboard_pid_parameters_position->y_p, onboard_pid_parameters_position->y_d, onboard_pid_parameters_position->y_i, onboard_pid_parameters_position->y_p_sat, onboard_pid_parameters_position->y_d_sat, onboard_pid_parameters_position->y_i_sat, onboard_pid_parameters_position->vel_p, onboard_pid_parameters_position->vel_d, onboard_pid_parameters_position->vel_i, onboard_pid_parameters_position->vel_p_sat, onboard_pid_parameters_position->vel_d_sat, onboard_pid_parameters_position->vel_i_sat);
}

/**
 * @brief Encode a onboard_pid_parameters_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param onboard_pid_parameters_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_onboard_pid_parameters_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_onboard_pid_parameters_position_t* onboard_pid_parameters_position)
{
	return mavlink_msg_onboard_pid_parameters_position_pack_chan(system_id, component_id, chan, msg, onboard_pid_parameters_position->x_p, onboard_pid_parameters_position->x_d, onboard_pid_parameters_position->x_i, onboard_pid_parameters_position->x_p_sat, onboard_pid_parameters_position->x_d_sat, onboard_pid_parameters_position->x_i_sat, onboard_pid_parameters_position->y_p, onboard_pid_parameters_position->y_d, onboard_pid_parameters_position->y_i, onboard_pid_parameters_position->y_p_sat, onboard_pid_parameters_position->y_d_sat, onboard_pid_parameters_position->y_i_sat, onboard_pid_parameters_position->vel_p, onboard_pid_parameters_position->vel_d, onboard_pid_parameters_position->vel_i, onboard_pid_parameters_position->vel_p_sat, onboard_pid_parameters_position->vel_d_sat, onboard_pid_parameters_position->vel_i_sat);
}

/**
 * @brief Send a onboard_pid_parameters_position message
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
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_onboard_pid_parameters_position_send(mavlink_channel_t chan, float x_p, float x_d, float x_i, float x_p_sat, float x_d_sat, float x_i_sat, float y_p, float y_d, float y_i, float y_p_sat, float y_d_sat, float y_i_sat, float vel_p, float vel_d, float vel_i, float vel_p_sat, float vel_d_sat, float vel_i_sat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN];
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

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION, buf, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION, buf, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN);
#endif
#else
	mavlink_onboard_pid_parameters_position_t packet;
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

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION, (const char *)&packet, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION, (const char *)&packet, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_onboard_pid_parameters_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x_p, float x_d, float x_i, float x_p_sat, float x_d_sat, float x_i_sat, float y_p, float y_d, float y_i, float y_p_sat, float y_d_sat, float y_i_sat, float vel_p, float vel_d, float vel_i, float vel_p_sat, float vel_d_sat, float vel_i_sat)
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

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION, buf, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION, buf, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN);
#endif
#else
	mavlink_onboard_pid_parameters_position_t *packet = (mavlink_onboard_pid_parameters_position_t *)msgbuf;
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

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION, (const char *)packet, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION, (const char *)packet, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ONBOARD_PID_PARAMETERS_POSITION UNPACKING


/**
 * @brief Get field x_p from onboard_pid_parameters_position message
 *
 * @return Proportional value for the x coordinate
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_x_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field x_d from onboard_pid_parameters_position message
 *
 * @return Derivative value for the x coordinate
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_x_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field x_i from onboard_pid_parameters_position message
 *
 * @return Integral value for the x coordinate
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_x_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field x_p_sat from onboard_pid_parameters_position message
 *
 * @return Saturation for the proportional value of the x coordinate
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_x_p_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field x_d_sat from onboard_pid_parameters_position message
 *
 * @return Saturation for the derivative value of the x coordinate
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_x_d_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field x_i_sat from onboard_pid_parameters_position message
 *
 * @return Saturation for the integral value of the x coordinate
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_x_i_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field y_p from onboard_pid_parameters_position message
 *
 * @return Proportional value for the y coordinate
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_y_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field y_d from onboard_pid_parameters_position message
 *
 * @return Derivative value for the y coordinate
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_y_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field y_i from onboard_pid_parameters_position message
 *
 * @return Integral value for the y coordinate
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_y_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field y_p_sat from onboard_pid_parameters_position message
 *
 * @return Saturation for the proportional value of the y coordinate
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_y_p_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field y_d_sat from onboard_pid_parameters_position message
 *
 * @return Saturation for the derivative value of the y coordinate
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_y_d_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field y_i_sat from onboard_pid_parameters_position message
 *
 * @return Saturation for the integral value of the y coordinate
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_y_i_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field vel_p from onboard_pid_parameters_position message
 *
 * @return Proportional value for the the velocity
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_vel_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field vel_d from onboard_pid_parameters_position message
 *
 * @return Derivative value for the velocity
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_vel_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field vel_i from onboard_pid_parameters_position message
 *
 * @return Integral value for the velocity
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_vel_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field vel_p_sat from onboard_pid_parameters_position message
 *
 * @return Saturation for the proportional value of the velocity
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_vel_p_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field vel_d_sat from onboard_pid_parameters_position message
 *
 * @return Saturation for the derivative value of the velocity
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_vel_d_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field vel_i_sat from onboard_pid_parameters_position message
 *
 * @return Saturation for the integral value of the velocity
 */
static inline float mavlink_msg_onboard_pid_parameters_position_get_vel_i_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Decode a onboard_pid_parameters_position message into a struct
 *
 * @param msg The message to decode
 * @param onboard_pid_parameters_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_onboard_pid_parameters_position_decode(const mavlink_message_t* msg, mavlink_onboard_pid_parameters_position_t* onboard_pid_parameters_position)
{
#if MAVLINK_NEED_BYTE_SWAP
	onboard_pid_parameters_position->x_p = mavlink_msg_onboard_pid_parameters_position_get_x_p(msg);
	onboard_pid_parameters_position->x_d = mavlink_msg_onboard_pid_parameters_position_get_x_d(msg);
	onboard_pid_parameters_position->x_i = mavlink_msg_onboard_pid_parameters_position_get_x_i(msg);
	onboard_pid_parameters_position->x_p_sat = mavlink_msg_onboard_pid_parameters_position_get_x_p_sat(msg);
	onboard_pid_parameters_position->x_d_sat = mavlink_msg_onboard_pid_parameters_position_get_x_d_sat(msg);
	onboard_pid_parameters_position->x_i_sat = mavlink_msg_onboard_pid_parameters_position_get_x_i_sat(msg);
	onboard_pid_parameters_position->y_p = mavlink_msg_onboard_pid_parameters_position_get_y_p(msg);
	onboard_pid_parameters_position->y_d = mavlink_msg_onboard_pid_parameters_position_get_y_d(msg);
	onboard_pid_parameters_position->y_i = mavlink_msg_onboard_pid_parameters_position_get_y_i(msg);
	onboard_pid_parameters_position->y_p_sat = mavlink_msg_onboard_pid_parameters_position_get_y_p_sat(msg);
	onboard_pid_parameters_position->y_d_sat = mavlink_msg_onboard_pid_parameters_position_get_y_d_sat(msg);
	onboard_pid_parameters_position->y_i_sat = mavlink_msg_onboard_pid_parameters_position_get_y_i_sat(msg);
	onboard_pid_parameters_position->vel_p = mavlink_msg_onboard_pid_parameters_position_get_vel_p(msg);
	onboard_pid_parameters_position->vel_d = mavlink_msg_onboard_pid_parameters_position_get_vel_d(msg);
	onboard_pid_parameters_position->vel_i = mavlink_msg_onboard_pid_parameters_position_get_vel_i(msg);
	onboard_pid_parameters_position->vel_p_sat = mavlink_msg_onboard_pid_parameters_position_get_vel_p_sat(msg);
	onboard_pid_parameters_position->vel_d_sat = mavlink_msg_onboard_pid_parameters_position_get_vel_d_sat(msg);
	onboard_pid_parameters_position->vel_i_sat = mavlink_msg_onboard_pid_parameters_position_get_vel_i_sat(msg);
#else
	memcpy(onboard_pid_parameters_position, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_POSITION_LEN);
#endif
}
