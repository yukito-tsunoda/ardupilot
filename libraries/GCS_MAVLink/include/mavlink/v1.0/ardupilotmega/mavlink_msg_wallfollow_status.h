// MESSAGE WALLFOLLOW_STATUS PACKING

#define MAVLINK_MSG_ID_WALLFOLLOW_STATUS 227

typedef struct __mavlink_wallfollow_status_t
{
 float target_dist; /*< Target dist to wall (cm)*/
 float wall_rng_raw; /*< Sensor dist to wall raw (cm)*/
 float wall_rng_flt; /*< Filtered wall distance (cm)*/
 float target_yaw; /*< Wall yaw normal (deg)*/
 float loiter_pt_x; /*< Loiter target X (m)*/
 float loiter_pt_y; /*< Loiter target Y (m)*/
 float loiter_pt_z; /*< Loiter target Z (m)*/
 uint8_t status; /*< Status (see WALLFOLLOW_STATUS_FLAGS enum)*/
} mavlink_wallfollow_status_t;

#define MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN 29
#define MAVLINK_MSG_ID_227_LEN 29

#define MAVLINK_MSG_ID_WALLFOLLOW_STATUS_CRC 48
#define MAVLINK_MSG_ID_227_CRC 48



#define MAVLINK_MESSAGE_INFO_WALLFOLLOW_STATUS { \
	"WALLFOLLOW_STATUS", \
	8, \
	{  { "target_dist", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_wallfollow_status_t, target_dist) }, \
         { "wall_rng_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_wallfollow_status_t, wall_rng_raw) }, \
         { "wall_rng_flt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_wallfollow_status_t, wall_rng_flt) }, \
         { "target_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_wallfollow_status_t, target_yaw) }, \
         { "loiter_pt_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_wallfollow_status_t, loiter_pt_x) }, \
         { "loiter_pt_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_wallfollow_status_t, loiter_pt_y) }, \
         { "loiter_pt_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_wallfollow_status_t, loiter_pt_z) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_wallfollow_status_t, status) }, \
         } \
}


/**
 * @brief Pack a wallfollow_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status Status (see WALLFOLLOW_STATUS_FLAGS enum)
 * @param target_dist Target dist to wall (cm)
 * @param wall_rng_raw Sensor dist to wall raw (cm)
 * @param wall_rng_flt Filtered wall distance (cm)
 * @param target_yaw Wall yaw normal (deg)
 * @param loiter_pt_x Loiter target X (m)
 * @param loiter_pt_y Loiter target Y (m)
 * @param loiter_pt_z Loiter target Z (m)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wallfollow_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t status, float target_dist, float wall_rng_raw, float wall_rng_flt, float target_yaw, float loiter_pt_x, float loiter_pt_y, float loiter_pt_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN];
	_mav_put_float(buf, 0, target_dist);
	_mav_put_float(buf, 4, wall_rng_raw);
	_mav_put_float(buf, 8, wall_rng_flt);
	_mav_put_float(buf, 12, target_yaw);
	_mav_put_float(buf, 16, loiter_pt_x);
	_mav_put_float(buf, 20, loiter_pt_y);
	_mav_put_float(buf, 24, loiter_pt_z);
	_mav_put_uint8_t(buf, 28, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN);
#else
	mavlink_wallfollow_status_t packet;
	packet.target_dist = target_dist;
	packet.wall_rng_raw = wall_rng_raw;
	packet.wall_rng_flt = wall_rng_flt;
	packet.target_yaw = target_yaw;
	packet.loiter_pt_x = loiter_pt_x;
	packet.loiter_pt_y = loiter_pt_y;
	packet.loiter_pt_z = loiter_pt_z;
	packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_WALLFOLLOW_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN);
#endif
}

/**
 * @brief Pack a wallfollow_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status Status (see WALLFOLLOW_STATUS_FLAGS enum)
 * @param target_dist Target dist to wall (cm)
 * @param wall_rng_raw Sensor dist to wall raw (cm)
 * @param wall_rng_flt Filtered wall distance (cm)
 * @param target_yaw Wall yaw normal (deg)
 * @param loiter_pt_x Loiter target X (m)
 * @param loiter_pt_y Loiter target Y (m)
 * @param loiter_pt_z Loiter target Z (m)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wallfollow_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t status,float target_dist,float wall_rng_raw,float wall_rng_flt,float target_yaw,float loiter_pt_x,float loiter_pt_y,float loiter_pt_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN];
	_mav_put_float(buf, 0, target_dist);
	_mav_put_float(buf, 4, wall_rng_raw);
	_mav_put_float(buf, 8, wall_rng_flt);
	_mav_put_float(buf, 12, target_yaw);
	_mav_put_float(buf, 16, loiter_pt_x);
	_mav_put_float(buf, 20, loiter_pt_y);
	_mav_put_float(buf, 24, loiter_pt_z);
	_mav_put_uint8_t(buf, 28, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN);
#else
	mavlink_wallfollow_status_t packet;
	packet.target_dist = target_dist;
	packet.wall_rng_raw = wall_rng_raw;
	packet.wall_rng_flt = wall_rng_flt;
	packet.target_yaw = target_yaw;
	packet.loiter_pt_x = loiter_pt_x;
	packet.loiter_pt_y = loiter_pt_y;
	packet.loiter_pt_z = loiter_pt_z;
	packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_WALLFOLLOW_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN);
#endif
}

/**
 * @brief Encode a wallfollow_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wallfollow_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wallfollow_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wallfollow_status_t* wallfollow_status)
{
	return mavlink_msg_wallfollow_status_pack(system_id, component_id, msg, wallfollow_status->status, wallfollow_status->target_dist, wallfollow_status->wall_rng_raw, wallfollow_status->wall_rng_flt, wallfollow_status->target_yaw, wallfollow_status->loiter_pt_x, wallfollow_status->loiter_pt_y, wallfollow_status->loiter_pt_z);
}

/**
 * @brief Encode a wallfollow_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wallfollow_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wallfollow_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_wallfollow_status_t* wallfollow_status)
{
	return mavlink_msg_wallfollow_status_pack_chan(system_id, component_id, chan, msg, wallfollow_status->status, wallfollow_status->target_dist, wallfollow_status->wall_rng_raw, wallfollow_status->wall_rng_flt, wallfollow_status->target_yaw, wallfollow_status->loiter_pt_x, wallfollow_status->loiter_pt_y, wallfollow_status->loiter_pt_z);
}

/**
 * @brief Send a wallfollow_status message
 * @param chan MAVLink channel to send the message
 *
 * @param status Status (see WALLFOLLOW_STATUS_FLAGS enum)
 * @param target_dist Target dist to wall (cm)
 * @param wall_rng_raw Sensor dist to wall raw (cm)
 * @param wall_rng_flt Filtered wall distance (cm)
 * @param target_yaw Wall yaw normal (deg)
 * @param loiter_pt_x Loiter target X (m)
 * @param loiter_pt_y Loiter target Y (m)
 * @param loiter_pt_z Loiter target Z (m)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wallfollow_status_send(mavlink_channel_t chan, uint8_t status, float target_dist, float wall_rng_raw, float wall_rng_flt, float target_yaw, float loiter_pt_x, float loiter_pt_y, float loiter_pt_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN];
	_mav_put_float(buf, 0, target_dist);
	_mav_put_float(buf, 4, wall_rng_raw);
	_mav_put_float(buf, 8, wall_rng_flt);
	_mav_put_float(buf, 12, target_yaw);
	_mav_put_float(buf, 16, loiter_pt_x);
	_mav_put_float(buf, 20, loiter_pt_y);
	_mav_put_float(buf, 24, loiter_pt_z);
	_mav_put_uint8_t(buf, 28, status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WALLFOLLOW_STATUS, buf, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WALLFOLLOW_STATUS, buf, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN);
#endif
#else
	mavlink_wallfollow_status_t packet;
	packet.target_dist = target_dist;
	packet.wall_rng_raw = wall_rng_raw;
	packet.wall_rng_flt = wall_rng_flt;
	packet.target_yaw = target_yaw;
	packet.loiter_pt_x = loiter_pt_x;
	packet.loiter_pt_y = loiter_pt_y;
	packet.loiter_pt_z = loiter_pt_z;
	packet.status = status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WALLFOLLOW_STATUS, (const char *)&packet, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WALLFOLLOW_STATUS, (const char *)&packet, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_wallfollow_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t status, float target_dist, float wall_rng_raw, float wall_rng_flt, float target_yaw, float loiter_pt_x, float loiter_pt_y, float loiter_pt_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, target_dist);
	_mav_put_float(buf, 4, wall_rng_raw);
	_mav_put_float(buf, 8, wall_rng_flt);
	_mav_put_float(buf, 12, target_yaw);
	_mav_put_float(buf, 16, loiter_pt_x);
	_mav_put_float(buf, 20, loiter_pt_y);
	_mav_put_float(buf, 24, loiter_pt_z);
	_mav_put_uint8_t(buf, 28, status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WALLFOLLOW_STATUS, buf, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WALLFOLLOW_STATUS, buf, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN);
#endif
#else
	mavlink_wallfollow_status_t *packet = (mavlink_wallfollow_status_t *)msgbuf;
	packet->target_dist = target_dist;
	packet->wall_rng_raw = wall_rng_raw;
	packet->wall_rng_flt = wall_rng_flt;
	packet->target_yaw = target_yaw;
	packet->loiter_pt_x = loiter_pt_x;
	packet->loiter_pt_y = loiter_pt_y;
	packet->loiter_pt_z = loiter_pt_z;
	packet->status = status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WALLFOLLOW_STATUS, (const char *)packet, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WALLFOLLOW_STATUS, (const char *)packet, MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE WALLFOLLOW_STATUS UNPACKING


/**
 * @brief Get field status from wallfollow_status message
 *
 * @return Status (see WALLFOLLOW_STATUS_FLAGS enum)
 */
static inline uint8_t mavlink_msg_wallfollow_status_get_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field target_dist from wallfollow_status message
 *
 * @return Target dist to wall (cm)
 */
static inline float mavlink_msg_wallfollow_status_get_target_dist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field wall_rng_raw from wallfollow_status message
 *
 * @return Sensor dist to wall raw (cm)
 */
static inline float mavlink_msg_wallfollow_status_get_wall_rng_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field wall_rng_flt from wallfollow_status message
 *
 * @return Filtered wall distance (cm)
 */
static inline float mavlink_msg_wallfollow_status_get_wall_rng_flt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field target_yaw from wallfollow_status message
 *
 * @return Wall yaw normal (deg)
 */
static inline float mavlink_msg_wallfollow_status_get_target_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field loiter_pt_x from wallfollow_status message
 *
 * @return Loiter target X (m)
 */
static inline float mavlink_msg_wallfollow_status_get_loiter_pt_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field loiter_pt_y from wallfollow_status message
 *
 * @return Loiter target Y (m)
 */
static inline float mavlink_msg_wallfollow_status_get_loiter_pt_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field loiter_pt_z from wallfollow_status message
 *
 * @return Loiter target Z (m)
 */
static inline float mavlink_msg_wallfollow_status_get_loiter_pt_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a wallfollow_status message into a struct
 *
 * @param msg The message to decode
 * @param wallfollow_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_wallfollow_status_decode(const mavlink_message_t* msg, mavlink_wallfollow_status_t* wallfollow_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	wallfollow_status->target_dist = mavlink_msg_wallfollow_status_get_target_dist(msg);
	wallfollow_status->wall_rng_raw = mavlink_msg_wallfollow_status_get_wall_rng_raw(msg);
	wallfollow_status->wall_rng_flt = mavlink_msg_wallfollow_status_get_wall_rng_flt(msg);
	wallfollow_status->target_yaw = mavlink_msg_wallfollow_status_get_target_yaw(msg);
	wallfollow_status->loiter_pt_x = mavlink_msg_wallfollow_status_get_loiter_pt_x(msg);
	wallfollow_status->loiter_pt_y = mavlink_msg_wallfollow_status_get_loiter_pt_y(msg);
	wallfollow_status->loiter_pt_z = mavlink_msg_wallfollow_status_get_loiter_pt_z(msg);
	wallfollow_status->status = mavlink_msg_wallfollow_status_get_status(msg);
#else
	memcpy(wallfollow_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_WALLFOLLOW_STATUS_LEN);
#endif
}
