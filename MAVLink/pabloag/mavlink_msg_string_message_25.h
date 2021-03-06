// MESSAGE STRING_MESSAGE_25 PACKING

#define MAVLINK_MSG_ID_STRING_MESSAGE_25 190

typedef struct __mavlink_string_message_25_t
{
 char Message[25]; ///< Message
} mavlink_string_message_25_t;

#define MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN 25
#define MAVLINK_MSG_ID_190_LEN 25

#define MAVLINK_MSG_ID_STRING_MESSAGE_25_CRC 119
#define MAVLINK_MSG_ID_190_CRC 119

#define MAVLINK_MSG_STRING_MESSAGE_25_FIELD_MESSAGE_LEN 25

#define MAVLINK_MESSAGE_INFO_STRING_MESSAGE_25 { \
	"STRING_MESSAGE_25", \
	1, \
	{  { "Message", NULL, MAVLINK_TYPE_CHAR, 25, 0, offsetof(mavlink_string_message_25_t, Message) }, \
         } \
}


/**
 * @brief Pack a string_message_25 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Message Message
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_string_message_25_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const char *Message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN];

	_mav_put_char_array(buf, 0, Message, 25);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN);
#else
	mavlink_string_message_25_t packet;

	mav_array_memcpy(packet.Message, Message, sizeof(char)*25);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_STRING_MESSAGE_25;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN, MAVLINK_MSG_ID_STRING_MESSAGE_25_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN);
#endif
}

/**
 * @brief Pack a string_message_25 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Message Message
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_string_message_25_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const char *Message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN];

	_mav_put_char_array(buf, 0, Message, 25);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN);
#else
	mavlink_string_message_25_t packet;

	mav_array_memcpy(packet.Message, Message, sizeof(char)*25);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_STRING_MESSAGE_25;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN, MAVLINK_MSG_ID_STRING_MESSAGE_25_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN);
#endif
}

/**
 * @brief Encode a string_message_25 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param string_message_25 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_string_message_25_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_string_message_25_t* string_message_25)
{
	return mavlink_msg_string_message_25_pack(system_id, component_id, msg, string_message_25->Message);
}

/**
 * @brief Encode a string_message_25 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param string_message_25 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_string_message_25_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_string_message_25_t* string_message_25)
{
	return mavlink_msg_string_message_25_pack_chan(system_id, component_id, chan, msg, string_message_25->Message);
}

/**
 * @brief Send a string_message_25 message
 * @param chan MAVLink channel to send the message
 *
 * @param Message Message
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_string_message_25_send(mavlink_channel_t chan, const char *Message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN];

	_mav_put_char_array(buf, 0, Message, 25);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STRING_MESSAGE_25, buf, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN, MAVLINK_MSG_ID_STRING_MESSAGE_25_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STRING_MESSAGE_25, buf, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN);
#endif
#else
	mavlink_string_message_25_t packet;

	mav_array_memcpy(packet.Message, Message, sizeof(char)*25);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STRING_MESSAGE_25, (const char *)&packet, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN, MAVLINK_MSG_ID_STRING_MESSAGE_25_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STRING_MESSAGE_25, (const char *)&packet, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_string_message_25_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *Message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;

	_mav_put_char_array(buf, 0, Message, 25);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STRING_MESSAGE_25, buf, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN, MAVLINK_MSG_ID_STRING_MESSAGE_25_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STRING_MESSAGE_25, buf, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN);
#endif
#else
	mavlink_string_message_25_t *packet = (mavlink_string_message_25_t *)msgbuf;

	mav_array_memcpy(packet->Message, Message, sizeof(char)*25);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STRING_MESSAGE_25, (const char *)packet, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN, MAVLINK_MSG_ID_STRING_MESSAGE_25_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STRING_MESSAGE_25, (const char *)packet, MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE STRING_MESSAGE_25 UNPACKING


/**
 * @brief Get field Message from string_message_25 message
 *
 * @return Message
 */
static inline uint16_t mavlink_msg_string_message_25_get_Message(const mavlink_message_t* msg, char *Message)
{
	return _MAV_RETURN_char_array(msg, Message, 25,  0);
}

/**
 * @brief Decode a string_message_25 message into a struct
 *
 * @param msg The message to decode
 * @param string_message_25 C-struct to decode the message contents into
 */
static inline void mavlink_msg_string_message_25_decode(const mavlink_message_t* msg, mavlink_string_message_25_t* string_message_25)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_string_message_25_get_Message(msg, string_message_25->Message);
#else
	memcpy(string_message_25, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_STRING_MESSAGE_25_LEN);
#endif
}
