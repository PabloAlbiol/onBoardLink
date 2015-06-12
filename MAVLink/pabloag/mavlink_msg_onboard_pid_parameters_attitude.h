// MESSAGE ONBOARD_PID_PARAMETERS_ATTITUDE PACKING

#define MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE 181

typedef struct __mavlink_onboard_pid_parameters_attitude_t
{
 float pitch_p; ///< Proportional value for the pitch
 float pitch_d; ///< Derivative value for the pitch
 float pitch_i; ///< Integral value for the pitch
 float pitch_p_sat; ///< Saturation for the proportional value of the pitch
 float pitch_d_sat; ///< Saturation for the derivative value of the pitch
 float pitch_i_sat; ///< Saturation for the integral value of the pitch
 float roll_p; ///< Proportional value for the roll
 float roll_d; ///< Derivative value for the roll
 float roll_i; ///< Integral value for the roll
 float roll_p_sat; ///< Saturation for the proportional value of the roll
 float roll_d_sat; ///< Saturation for the derivative value of the roll
 float roll_i_sat; ///< Saturation for the integral value of the roll
 float yaw_p; ///< Proportional value for the yaw
 float yaw_d; ///< Derivative value for the yaw
 float yaw_i; ///< Integral value for the yaw
 float yaw_p_sat; ///< Saturation for the proportional value of the yaw
 float yaw_d_sat; ///< Saturation for the derivative value of the yaw
 float yaw_i_sat; ///< Saturation for the integral value of the yaw
 float height_p; ///< Proportional value for the height
 float height_d; ///< Derivative value for the height
 float height_i; ///< Integral value for the height
 float height_p_sat; ///< Saturation for the proportional value of the height
 float height_d_sat; ///< Saturation for the derivative value of the height
 float height_i_sat; ///< Saturation for the integral value of the height
} mavlink_onboard_pid_parameters_attitude_t;

#define MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN 96
#define MAVLINK_MSG_ID_181_LEN 96

#define MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_CRC 230
#define MAVLINK_MSG_ID_181_CRC 230



#define MAVLINK_MESSAGE_INFO_ONBOARD_PID_PARAMETERS_ATTITUDE { \
	"ONBOARD_PID_PARAMETERS_ATTITUDE", \
	24, \
	{  { "pitch_p", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_onboard_pid_parameters_attitude_t, pitch_p) }, \
         { "pitch_d", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_onboard_pid_parameters_attitude_t, pitch_d) }, \
         { "pitch_i", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_onboard_pid_parameters_attitude_t, pitch_i) }, \
         { "pitch_p_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_onboard_pid_parameters_attitude_t, pitch_p_sat) }, \
         { "pitch_d_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_onboard_pid_parameters_attitude_t, pitch_d_sat) }, \
         { "pitch_i_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_onboard_pid_parameters_attitude_t, pitch_i_sat) }, \
         { "roll_p", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_onboard_pid_parameters_attitude_t, roll_p) }, \
         { "roll_d", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_onboard_pid_parameters_attitude_t, roll_d) }, \
         { "roll_i", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_onboard_pid_parameters_attitude_t, roll_i) }, \
         { "roll_p_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_onboard_pid_parameters_attitude_t, roll_p_sat) }, \
         { "roll_d_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_onboard_pid_parameters_attitude_t, roll_d_sat) }, \
         { "roll_i_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_onboard_pid_parameters_attitude_t, roll_i_sat) }, \
         { "yaw_p", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_onboard_pid_parameters_attitude_t, yaw_p) }, \
         { "yaw_d", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_onboard_pid_parameters_attitude_t, yaw_d) }, \
         { "yaw_i", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_onboard_pid_parameters_attitude_t, yaw_i) }, \
         { "yaw_p_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_onboard_pid_parameters_attitude_t, yaw_p_sat) }, \
         { "yaw_d_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_onboard_pid_parameters_attitude_t, yaw_d_sat) }, \
         { "yaw_i_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_onboard_pid_parameters_attitude_t, yaw_i_sat) }, \
         { "height_p", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_onboard_pid_parameters_attitude_t, height_p) }, \
         { "height_d", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_onboard_pid_parameters_attitude_t, height_d) }, \
         { "height_i", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_onboard_pid_parameters_attitude_t, height_i) }, \
         { "height_p_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_onboard_pid_parameters_attitude_t, height_p_sat) }, \
         { "height_d_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_onboard_pid_parameters_attitude_t, height_d_sat) }, \
         { "height_i_sat", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_onboard_pid_parameters_attitude_t, height_i_sat) }, \
         } \
}


/**
 * @brief Pack a onboard_pid_parameters_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pitch_p Proportional value for the pitch
 * @param pitch_d Derivative value for the pitch
 * @param pitch_i Integral value for the pitch
 * @param pitch_p_sat Saturation for the proportional value of the pitch
 * @param pitch_d_sat Saturation for the derivative value of the pitch
 * @param pitch_i_sat Saturation for the integral value of the pitch
 * @param roll_p Proportional value for the roll
 * @param roll_d Derivative value for the roll
 * @param roll_i Integral value for the roll
 * @param roll_p_sat Saturation for the proportional value of the roll
 * @param roll_d_sat Saturation for the derivative value of the roll
 * @param roll_i_sat Saturation for the integral value of the roll
 * @param yaw_p Proportional value for the yaw
 * @param yaw_d Derivative value for the yaw
 * @param yaw_i Integral value for the yaw
 * @param yaw_p_sat Saturation for the proportional value of the yaw
 * @param yaw_d_sat Saturation for the derivative value of the yaw
 * @param yaw_i_sat Saturation for the integral value of the yaw
 * @param height_p Proportional value for the height
 * @param height_d Derivative value for the height
 * @param height_i Integral value for the height
 * @param height_p_sat Saturation for the proportional value of the height
 * @param height_d_sat Saturation for the derivative value of the height
 * @param height_i_sat Saturation for the integral value of the height
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_onboard_pid_parameters_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float pitch_p, float pitch_d, float pitch_i, float pitch_p_sat, float pitch_d_sat, float pitch_i_sat, float roll_p, float roll_d, float roll_i, float roll_p_sat, float roll_d_sat, float roll_i_sat, float yaw_p, float yaw_d, float yaw_i, float yaw_p_sat, float yaw_d_sat, float yaw_i_sat, float height_p, float height_d, float height_i, float height_p_sat, float height_d_sat, float height_i_sat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN];
	_mav_put_float(buf, 0, pitch_p);
	_mav_put_float(buf, 4, pitch_d);
	_mav_put_float(buf, 8, pitch_i);
	_mav_put_float(buf, 12, pitch_p_sat);
	_mav_put_float(buf, 16, pitch_d_sat);
	_mav_put_float(buf, 20, pitch_i_sat);
	_mav_put_float(buf, 24, roll_p);
	_mav_put_float(buf, 28, roll_d);
	_mav_put_float(buf, 32, roll_i);
	_mav_put_float(buf, 36, roll_p_sat);
	_mav_put_float(buf, 40, roll_d_sat);
	_mav_put_float(buf, 44, roll_i_sat);
	_mav_put_float(buf, 48, yaw_p);
	_mav_put_float(buf, 52, yaw_d);
	_mav_put_float(buf, 56, yaw_i);
	_mav_put_float(buf, 60, yaw_p_sat);
	_mav_put_float(buf, 64, yaw_d_sat);
	_mav_put_float(buf, 68, yaw_i_sat);
	_mav_put_float(buf, 72, height_p);
	_mav_put_float(buf, 76, height_d);
	_mav_put_float(buf, 80, height_i);
	_mav_put_float(buf, 84, height_p_sat);
	_mav_put_float(buf, 88, height_d_sat);
	_mav_put_float(buf, 92, height_i_sat);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN);
#else
	mavlink_onboard_pid_parameters_attitude_t packet;
	packet.pitch_p = pitch_p;
	packet.pitch_d = pitch_d;
	packet.pitch_i = pitch_i;
	packet.pitch_p_sat = pitch_p_sat;
	packet.pitch_d_sat = pitch_d_sat;
	packet.pitch_i_sat = pitch_i_sat;
	packet.roll_p = roll_p;
	packet.roll_d = roll_d;
	packet.roll_i = roll_i;
	packet.roll_p_sat = roll_p_sat;
	packet.roll_d_sat = roll_d_sat;
	packet.roll_i_sat = roll_i_sat;
	packet.yaw_p = yaw_p;
	packet.yaw_d = yaw_d;
	packet.yaw_i = yaw_i;
	packet.yaw_p_sat = yaw_p_sat;
	packet.yaw_d_sat = yaw_d_sat;
	packet.yaw_i_sat = yaw_i_sat;
	packet.height_p = height_p;
	packet.height_d = height_d;
	packet.height_i = height_i;
	packet.height_p_sat = height_p_sat;
	packet.height_d_sat = height_d_sat;
	packet.height_i_sat = height_i_sat;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN);
#endif
}

/**
 * @brief Pack a onboard_pid_parameters_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pitch_p Proportional value for the pitch
 * @param pitch_d Derivative value for the pitch
 * @param pitch_i Integral value for the pitch
 * @param pitch_p_sat Saturation for the proportional value of the pitch
 * @param pitch_d_sat Saturation for the derivative value of the pitch
 * @param pitch_i_sat Saturation for the integral value of the pitch
 * @param roll_p Proportional value for the roll
 * @param roll_d Derivative value for the roll
 * @param roll_i Integral value for the roll
 * @param roll_p_sat Saturation for the proportional value of the roll
 * @param roll_d_sat Saturation for the derivative value of the roll
 * @param roll_i_sat Saturation for the integral value of the roll
 * @param yaw_p Proportional value for the yaw
 * @param yaw_d Derivative value for the yaw
 * @param yaw_i Integral value for the yaw
 * @param yaw_p_sat Saturation for the proportional value of the yaw
 * @param yaw_d_sat Saturation for the derivative value of the yaw
 * @param yaw_i_sat Saturation for the integral value of the yaw
 * @param height_p Proportional value for the height
 * @param height_d Derivative value for the height
 * @param height_i Integral value for the height
 * @param height_p_sat Saturation for the proportional value of the height
 * @param height_d_sat Saturation for the derivative value of the height
 * @param height_i_sat Saturation for the integral value of the height
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_onboard_pid_parameters_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float pitch_p,float pitch_d,float pitch_i,float pitch_p_sat,float pitch_d_sat,float pitch_i_sat,float roll_p,float roll_d,float roll_i,float roll_p_sat,float roll_d_sat,float roll_i_sat,float yaw_p,float yaw_d,float yaw_i,float yaw_p_sat,float yaw_d_sat,float yaw_i_sat,float height_p,float height_d,float height_i,float height_p_sat,float height_d_sat,float height_i_sat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN];
	_mav_put_float(buf, 0, pitch_p);
	_mav_put_float(buf, 4, pitch_d);
	_mav_put_float(buf, 8, pitch_i);
	_mav_put_float(buf, 12, pitch_p_sat);
	_mav_put_float(buf, 16, pitch_d_sat);
	_mav_put_float(buf, 20, pitch_i_sat);
	_mav_put_float(buf, 24, roll_p);
	_mav_put_float(buf, 28, roll_d);
	_mav_put_float(buf, 32, roll_i);
	_mav_put_float(buf, 36, roll_p_sat);
	_mav_put_float(buf, 40, roll_d_sat);
	_mav_put_float(buf, 44, roll_i_sat);
	_mav_put_float(buf, 48, yaw_p);
	_mav_put_float(buf, 52, yaw_d);
	_mav_put_float(buf, 56, yaw_i);
	_mav_put_float(buf, 60, yaw_p_sat);
	_mav_put_float(buf, 64, yaw_d_sat);
	_mav_put_float(buf, 68, yaw_i_sat);
	_mav_put_float(buf, 72, height_p);
	_mav_put_float(buf, 76, height_d);
	_mav_put_float(buf, 80, height_i);
	_mav_put_float(buf, 84, height_p_sat);
	_mav_put_float(buf, 88, height_d_sat);
	_mav_put_float(buf, 92, height_i_sat);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN);
#else
	mavlink_onboard_pid_parameters_attitude_t packet;
	packet.pitch_p = pitch_p;
	packet.pitch_d = pitch_d;
	packet.pitch_i = pitch_i;
	packet.pitch_p_sat = pitch_p_sat;
	packet.pitch_d_sat = pitch_d_sat;
	packet.pitch_i_sat = pitch_i_sat;
	packet.roll_p = roll_p;
	packet.roll_d = roll_d;
	packet.roll_i = roll_i;
	packet.roll_p_sat = roll_p_sat;
	packet.roll_d_sat = roll_d_sat;
	packet.roll_i_sat = roll_i_sat;
	packet.yaw_p = yaw_p;
	packet.yaw_d = yaw_d;
	packet.yaw_i = yaw_i;
	packet.yaw_p_sat = yaw_p_sat;
	packet.yaw_d_sat = yaw_d_sat;
	packet.yaw_i_sat = yaw_i_sat;
	packet.height_p = height_p;
	packet.height_d = height_d;
	packet.height_i = height_i;
	packet.height_p_sat = height_p_sat;
	packet.height_d_sat = height_d_sat;
	packet.height_i_sat = height_i_sat;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN);
#endif
}

/**
 * @brief Encode a onboard_pid_parameters_attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param onboard_pid_parameters_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_onboard_pid_parameters_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_onboard_pid_parameters_attitude_t* onboard_pid_parameters_attitude)
{
	return mavlink_msg_onboard_pid_parameters_attitude_pack(system_id, component_id, msg, onboard_pid_parameters_attitude->pitch_p, onboard_pid_parameters_attitude->pitch_d, onboard_pid_parameters_attitude->pitch_i, onboard_pid_parameters_attitude->pitch_p_sat, onboard_pid_parameters_attitude->pitch_d_sat, onboard_pid_parameters_attitude->pitch_i_sat, onboard_pid_parameters_attitude->roll_p, onboard_pid_parameters_attitude->roll_d, onboard_pid_parameters_attitude->roll_i, onboard_pid_parameters_attitude->roll_p_sat, onboard_pid_parameters_attitude->roll_d_sat, onboard_pid_parameters_attitude->roll_i_sat, onboard_pid_parameters_attitude->yaw_p, onboard_pid_parameters_attitude->yaw_d, onboard_pid_parameters_attitude->yaw_i, onboard_pid_parameters_attitude->yaw_p_sat, onboard_pid_parameters_attitude->yaw_d_sat, onboard_pid_parameters_attitude->yaw_i_sat, onboard_pid_parameters_attitude->height_p, onboard_pid_parameters_attitude->height_d, onboard_pid_parameters_attitude->height_i, onboard_pid_parameters_attitude->height_p_sat, onboard_pid_parameters_attitude->height_d_sat, onboard_pid_parameters_attitude->height_i_sat);
}

/**
 * @brief Encode a onboard_pid_parameters_attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param onboard_pid_parameters_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_onboard_pid_parameters_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_onboard_pid_parameters_attitude_t* onboard_pid_parameters_attitude)
{
	return mavlink_msg_onboard_pid_parameters_attitude_pack_chan(system_id, component_id, chan, msg, onboard_pid_parameters_attitude->pitch_p, onboard_pid_parameters_attitude->pitch_d, onboard_pid_parameters_attitude->pitch_i, onboard_pid_parameters_attitude->pitch_p_sat, onboard_pid_parameters_attitude->pitch_d_sat, onboard_pid_parameters_attitude->pitch_i_sat, onboard_pid_parameters_attitude->roll_p, onboard_pid_parameters_attitude->roll_d, onboard_pid_parameters_attitude->roll_i, onboard_pid_parameters_attitude->roll_p_sat, onboard_pid_parameters_attitude->roll_d_sat, onboard_pid_parameters_attitude->roll_i_sat, onboard_pid_parameters_attitude->yaw_p, onboard_pid_parameters_attitude->yaw_d, onboard_pid_parameters_attitude->yaw_i, onboard_pid_parameters_attitude->yaw_p_sat, onboard_pid_parameters_attitude->yaw_d_sat, onboard_pid_parameters_attitude->yaw_i_sat, onboard_pid_parameters_attitude->height_p, onboard_pid_parameters_attitude->height_d, onboard_pid_parameters_attitude->height_i, onboard_pid_parameters_attitude->height_p_sat, onboard_pid_parameters_attitude->height_d_sat, onboard_pid_parameters_attitude->height_i_sat);
}

/**
 * @brief Send a onboard_pid_parameters_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param pitch_p Proportional value for the pitch
 * @param pitch_d Derivative value for the pitch
 * @param pitch_i Integral value for the pitch
 * @param pitch_p_sat Saturation for the proportional value of the pitch
 * @param pitch_d_sat Saturation for the derivative value of the pitch
 * @param pitch_i_sat Saturation for the integral value of the pitch
 * @param roll_p Proportional value for the roll
 * @param roll_d Derivative value for the roll
 * @param roll_i Integral value for the roll
 * @param roll_p_sat Saturation for the proportional value of the roll
 * @param roll_d_sat Saturation for the derivative value of the roll
 * @param roll_i_sat Saturation for the integral value of the roll
 * @param yaw_p Proportional value for the yaw
 * @param yaw_d Derivative value for the yaw
 * @param yaw_i Integral value for the yaw
 * @param yaw_p_sat Saturation for the proportional value of the yaw
 * @param yaw_d_sat Saturation for the derivative value of the yaw
 * @param yaw_i_sat Saturation for the integral value of the yaw
 * @param height_p Proportional value for the height
 * @param height_d Derivative value for the height
 * @param height_i Integral value for the height
 * @param height_p_sat Saturation for the proportional value of the height
 * @param height_d_sat Saturation for the derivative value of the height
 * @param height_i_sat Saturation for the integral value of the height
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_onboard_pid_parameters_attitude_send(mavlink_channel_t chan, float pitch_p, float pitch_d, float pitch_i, float pitch_p_sat, float pitch_d_sat, float pitch_i_sat, float roll_p, float roll_d, float roll_i, float roll_p_sat, float roll_d_sat, float roll_i_sat, float yaw_p, float yaw_d, float yaw_i, float yaw_p_sat, float yaw_d_sat, float yaw_i_sat, float height_p, float height_d, float height_i, float height_p_sat, float height_d_sat, float height_i_sat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN];
	_mav_put_float(buf, 0, pitch_p);
	_mav_put_float(buf, 4, pitch_d);
	_mav_put_float(buf, 8, pitch_i);
	_mav_put_float(buf, 12, pitch_p_sat);
	_mav_put_float(buf, 16, pitch_d_sat);
	_mav_put_float(buf, 20, pitch_i_sat);
	_mav_put_float(buf, 24, roll_p);
	_mav_put_float(buf, 28, roll_d);
	_mav_put_float(buf, 32, roll_i);
	_mav_put_float(buf, 36, roll_p_sat);
	_mav_put_float(buf, 40, roll_d_sat);
	_mav_put_float(buf, 44, roll_i_sat);
	_mav_put_float(buf, 48, yaw_p);
	_mav_put_float(buf, 52, yaw_d);
	_mav_put_float(buf, 56, yaw_i);
	_mav_put_float(buf, 60, yaw_p_sat);
	_mav_put_float(buf, 64, yaw_d_sat);
	_mav_put_float(buf, 68, yaw_i_sat);
	_mav_put_float(buf, 72, height_p);
	_mav_put_float(buf, 76, height_d);
	_mav_put_float(buf, 80, height_i);
	_mav_put_float(buf, 84, height_p_sat);
	_mav_put_float(buf, 88, height_d_sat);
	_mav_put_float(buf, 92, height_i_sat);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE, buf, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE, buf, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN);
#endif
#else
	mavlink_onboard_pid_parameters_attitude_t packet;
	packet.pitch_p = pitch_p;
	packet.pitch_d = pitch_d;
	packet.pitch_i = pitch_i;
	packet.pitch_p_sat = pitch_p_sat;
	packet.pitch_d_sat = pitch_d_sat;
	packet.pitch_i_sat = pitch_i_sat;
	packet.roll_p = roll_p;
	packet.roll_d = roll_d;
	packet.roll_i = roll_i;
	packet.roll_p_sat = roll_p_sat;
	packet.roll_d_sat = roll_d_sat;
	packet.roll_i_sat = roll_i_sat;
	packet.yaw_p = yaw_p;
	packet.yaw_d = yaw_d;
	packet.yaw_i = yaw_i;
	packet.yaw_p_sat = yaw_p_sat;
	packet.yaw_d_sat = yaw_d_sat;
	packet.yaw_i_sat = yaw_i_sat;
	packet.height_p = height_p;
	packet.height_d = height_d;
	packet.height_i = height_i;
	packet.height_p_sat = height_p_sat;
	packet.height_d_sat = height_d_sat;
	packet.height_i_sat = height_i_sat;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_onboard_pid_parameters_attitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pitch_p, float pitch_d, float pitch_i, float pitch_p_sat, float pitch_d_sat, float pitch_i_sat, float roll_p, float roll_d, float roll_i, float roll_p_sat, float roll_d_sat, float roll_i_sat, float yaw_p, float yaw_d, float yaw_i, float yaw_p_sat, float yaw_d_sat, float yaw_i_sat, float height_p, float height_d, float height_i, float height_p_sat, float height_d_sat, float height_i_sat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, pitch_p);
	_mav_put_float(buf, 4, pitch_d);
	_mav_put_float(buf, 8, pitch_i);
	_mav_put_float(buf, 12, pitch_p_sat);
	_mav_put_float(buf, 16, pitch_d_sat);
	_mav_put_float(buf, 20, pitch_i_sat);
	_mav_put_float(buf, 24, roll_p);
	_mav_put_float(buf, 28, roll_d);
	_mav_put_float(buf, 32, roll_i);
	_mav_put_float(buf, 36, roll_p_sat);
	_mav_put_float(buf, 40, roll_d_sat);
	_mav_put_float(buf, 44, roll_i_sat);
	_mav_put_float(buf, 48, yaw_p);
	_mav_put_float(buf, 52, yaw_d);
	_mav_put_float(buf, 56, yaw_i);
	_mav_put_float(buf, 60, yaw_p_sat);
	_mav_put_float(buf, 64, yaw_d_sat);
	_mav_put_float(buf, 68, yaw_i_sat);
	_mav_put_float(buf, 72, height_p);
	_mav_put_float(buf, 76, height_d);
	_mav_put_float(buf, 80, height_i);
	_mav_put_float(buf, 84, height_p_sat);
	_mav_put_float(buf, 88, height_d_sat);
	_mav_put_float(buf, 92, height_i_sat);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE, buf, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE, buf, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN);
#endif
#else
	mavlink_onboard_pid_parameters_attitude_t *packet = (mavlink_onboard_pid_parameters_attitude_t *)msgbuf;
	packet->pitch_p = pitch_p;
	packet->pitch_d = pitch_d;
	packet->pitch_i = pitch_i;
	packet->pitch_p_sat = pitch_p_sat;
	packet->pitch_d_sat = pitch_d_sat;
	packet->pitch_i_sat = pitch_i_sat;
	packet->roll_p = roll_p;
	packet->roll_d = roll_d;
	packet->roll_i = roll_i;
	packet->roll_p_sat = roll_p_sat;
	packet->roll_d_sat = roll_d_sat;
	packet->roll_i_sat = roll_i_sat;
	packet->yaw_p = yaw_p;
	packet->yaw_d = yaw_d;
	packet->yaw_i = yaw_i;
	packet->yaw_p_sat = yaw_p_sat;
	packet->yaw_d_sat = yaw_d_sat;
	packet->yaw_i_sat = yaw_i_sat;
	packet->height_p = height_p;
	packet->height_d = height_d;
	packet->height_i = height_i;
	packet->height_p_sat = height_p_sat;
	packet->height_d_sat = height_d_sat;
	packet->height_i_sat = height_i_sat;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ONBOARD_PID_PARAMETERS_ATTITUDE UNPACKING


/**
 * @brief Get field pitch_p from onboard_pid_parameters_attitude message
 *
 * @return Proportional value for the pitch
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_pitch_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch_d from onboard_pid_parameters_attitude message
 *
 * @return Derivative value for the pitch
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_pitch_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch_i from onboard_pid_parameters_attitude message
 *
 * @return Integral value for the pitch
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_pitch_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch_p_sat from onboard_pid_parameters_attitude message
 *
 * @return Saturation for the proportional value of the pitch
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_pitch_p_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitch_d_sat from onboard_pid_parameters_attitude message
 *
 * @return Saturation for the derivative value of the pitch
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_pitch_d_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitch_i_sat from onboard_pid_parameters_attitude message
 *
 * @return Saturation for the integral value of the pitch
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_pitch_i_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field roll_p from onboard_pid_parameters_attitude message
 *
 * @return Proportional value for the roll
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_roll_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field roll_d from onboard_pid_parameters_attitude message
 *
 * @return Derivative value for the roll
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_roll_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field roll_i from onboard_pid_parameters_attitude message
 *
 * @return Integral value for the roll
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_roll_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field roll_p_sat from onboard_pid_parameters_attitude message
 *
 * @return Saturation for the proportional value of the roll
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_roll_p_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field roll_d_sat from onboard_pid_parameters_attitude message
 *
 * @return Saturation for the derivative value of the roll
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_roll_d_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field roll_i_sat from onboard_pid_parameters_attitude message
 *
 * @return Saturation for the integral value of the roll
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_roll_i_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field yaw_p from onboard_pid_parameters_attitude message
 *
 * @return Proportional value for the yaw
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_yaw_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field yaw_d from onboard_pid_parameters_attitude message
 *
 * @return Derivative value for the yaw
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_yaw_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field yaw_i from onboard_pid_parameters_attitude message
 *
 * @return Integral value for the yaw
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_yaw_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field yaw_p_sat from onboard_pid_parameters_attitude message
 *
 * @return Saturation for the proportional value of the yaw
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_yaw_p_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field yaw_d_sat from onboard_pid_parameters_attitude message
 *
 * @return Saturation for the derivative value of the yaw
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_yaw_d_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field yaw_i_sat from onboard_pid_parameters_attitude message
 *
 * @return Saturation for the integral value of the yaw
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_yaw_i_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field height_p from onboard_pid_parameters_attitude message
 *
 * @return Proportional value for the height
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_height_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field height_d from onboard_pid_parameters_attitude message
 *
 * @return Derivative value for the height
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_height_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field height_i from onboard_pid_parameters_attitude message
 *
 * @return Integral value for the height
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_height_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field height_p_sat from onboard_pid_parameters_attitude message
 *
 * @return Saturation for the proportional value of the height
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_height_p_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field height_d_sat from onboard_pid_parameters_attitude message
 *
 * @return Saturation for the derivative value of the height
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_height_d_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field height_i_sat from onboard_pid_parameters_attitude message
 *
 * @return Saturation for the integral value of the height
 */
static inline float mavlink_msg_onboard_pid_parameters_attitude_get_height_i_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Decode a onboard_pid_parameters_attitude message into a struct
 *
 * @param msg The message to decode
 * @param onboard_pid_parameters_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_onboard_pid_parameters_attitude_decode(const mavlink_message_t* msg, mavlink_onboard_pid_parameters_attitude_t* onboard_pid_parameters_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP
	onboard_pid_parameters_attitude->pitch_p = mavlink_msg_onboard_pid_parameters_attitude_get_pitch_p(msg);
	onboard_pid_parameters_attitude->pitch_d = mavlink_msg_onboard_pid_parameters_attitude_get_pitch_d(msg);
	onboard_pid_parameters_attitude->pitch_i = mavlink_msg_onboard_pid_parameters_attitude_get_pitch_i(msg);
	onboard_pid_parameters_attitude->pitch_p_sat = mavlink_msg_onboard_pid_parameters_attitude_get_pitch_p_sat(msg);
	onboard_pid_parameters_attitude->pitch_d_sat = mavlink_msg_onboard_pid_parameters_attitude_get_pitch_d_sat(msg);
	onboard_pid_parameters_attitude->pitch_i_sat = mavlink_msg_onboard_pid_parameters_attitude_get_pitch_i_sat(msg);
	onboard_pid_parameters_attitude->roll_p = mavlink_msg_onboard_pid_parameters_attitude_get_roll_p(msg);
	onboard_pid_parameters_attitude->roll_d = mavlink_msg_onboard_pid_parameters_attitude_get_roll_d(msg);
	onboard_pid_parameters_attitude->roll_i = mavlink_msg_onboard_pid_parameters_attitude_get_roll_i(msg);
	onboard_pid_parameters_attitude->roll_p_sat = mavlink_msg_onboard_pid_parameters_attitude_get_roll_p_sat(msg);
	onboard_pid_parameters_attitude->roll_d_sat = mavlink_msg_onboard_pid_parameters_attitude_get_roll_d_sat(msg);
	onboard_pid_parameters_attitude->roll_i_sat = mavlink_msg_onboard_pid_parameters_attitude_get_roll_i_sat(msg);
	onboard_pid_parameters_attitude->yaw_p = mavlink_msg_onboard_pid_parameters_attitude_get_yaw_p(msg);
	onboard_pid_parameters_attitude->yaw_d = mavlink_msg_onboard_pid_parameters_attitude_get_yaw_d(msg);
	onboard_pid_parameters_attitude->yaw_i = mavlink_msg_onboard_pid_parameters_attitude_get_yaw_i(msg);
	onboard_pid_parameters_attitude->yaw_p_sat = mavlink_msg_onboard_pid_parameters_attitude_get_yaw_p_sat(msg);
	onboard_pid_parameters_attitude->yaw_d_sat = mavlink_msg_onboard_pid_parameters_attitude_get_yaw_d_sat(msg);
	onboard_pid_parameters_attitude->yaw_i_sat = mavlink_msg_onboard_pid_parameters_attitude_get_yaw_i_sat(msg);
	onboard_pid_parameters_attitude->height_p = mavlink_msg_onboard_pid_parameters_attitude_get_height_p(msg);
	onboard_pid_parameters_attitude->height_d = mavlink_msg_onboard_pid_parameters_attitude_get_height_d(msg);
	onboard_pid_parameters_attitude->height_i = mavlink_msg_onboard_pid_parameters_attitude_get_height_i(msg);
	onboard_pid_parameters_attitude->height_p_sat = mavlink_msg_onboard_pid_parameters_attitude_get_height_p_sat(msg);
	onboard_pid_parameters_attitude->height_d_sat = mavlink_msg_onboard_pid_parameters_attitude_get_height_d_sat(msg);
	onboard_pid_parameters_attitude->height_i_sat = mavlink_msg_onboard_pid_parameters_attitude_get_height_i_sat(msg);
#else
	memcpy(onboard_pid_parameters_attitude, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ONBOARD_PID_PARAMETERS_ATTITUDE_LEN);
#endif
}
