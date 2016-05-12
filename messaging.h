/* Last modified: <25-Nov-2015 14:55:23 CET by Dmitry Ebel> */
#ifndef MESSAGING_H
#define MESSAGING_H

#include <stdint.h>

#define MSG_START    0x7e   /** Start Of Header */
#define MSG_TYPE_DST 0x44   /** Message is uint16_t (distance) */
#define MSG_TYPE_CMD 0x43   /** Message is command */
#define MSG_TYPE_RAW 0x52   /** Message is raw data */
#define MSG_TYPE_CMP 0x53   /** Message is compressed raw data */
#define MSG_TYPE_TMP 0x54   /** Message is temperature */

/** Message data type */
typedef struct {
    uint8_t  header;    ///< 'Start of Header' byte, flags first byte of the message
    uint8_t  type;      ///< message type (DIST, COMMAND, RAWDATA)
    uint8_t  beacon_id; /** Beacon ID */
    uint8_t  sensor_id; /** Sensor ID */
    uint16_t data_size; /** Data size in bytes. Data is to be sent after the header. */
    uint32_t timestamp; /** Timestamp */
}  __attribute__((packed)) message_t;

/** Reads message from file or stream and copies the data to a buffer */
int msg_read_next_message(message_t* const msg /** message */,
			  const int fd         /** file descriptor */,
			  uint8_t* const       /** data buffer */);

#endif
