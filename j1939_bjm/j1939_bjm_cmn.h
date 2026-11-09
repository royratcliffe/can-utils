// SPDX-License-Identifier: LGPL-2.0-only
// SPDX-FileCopyrightText: 2026 Roy Ratcliffe <roy@ratcliffe.me>

#ifndef _J1939_BJM_H_
#define _J1939_BJM_H_

#include <stdint.h>
#include <endian.h>
#include <stdbool.h>
#include <sys/epoll.h>

#include <linux/can.h>
#include <linux/kernel.h>
#include "../libj1939.h"
#include "../lib.h"

#define J1939_PGN_REQUEST_PGN 0x0ea00 /* 59904 */

#define J1939_BJM_MAX_TRANSFER_LENGH 1024

struct j1939_bjm_stats {
	int err;
	uint32_t tskey_sch;
	uint32_t tskey_ack;
	uint32_t send;
};

struct j1939_bjm_msg {
	uint8_t buf[J1939_BJM_MAX_TRANSFER_LENGH];
	size_t buf_size;
	size_t len; /* length of received message */
	struct sockaddr_can peername;
	socklen_t peer_addr_len;
	int sock;
};

/* SAE J1939 specific definitions */

/* SAE J1939-71 - pgn64982 - Basic Joystick Message 1 - BJM1 - */
#define J1939_PGN_BJM1 0x0fdd6 /* 64982 */
/* SAE J1939-71 - pgn64984 - Basic Joystick Message 2 - BJM2 - */
#define J1939_PGN_BJM2 0x0fdd8 /* 64984 */

#define J1939_BJM_PRIO_DEFAULT 3
#define J1939_BJM_MAX_TRANSFER_LENGTH sizeof(struct j1939_bjm_packet)
#define J1939_BJM_REPETITION_RATE_MS 100
#define J1939_BJM_JITTER_MS 20

/* 10 bit position: 0.1 %/bit, valid range 0 to 1000 (0 to 100 %) */
#define J1939_BJM_POSITION_MAX 1000
#define J1939_BJM_POSITION_NOT_AVAILABLE 0x3ff

/**
 * enum j1939_bjm_status - Two bit discrete status used by the BJM PGNs
 *
 * Used both for lever position statuses (neutral, negative and positive
 * direction) and for button pressed statuses.
 */
enum j1939_bjm_status {
	J1939_BJM_STATUS_OFF = 0, /* not in position / not pressed */
	J1939_BJM_STATUS_ON = 1, /* in position / pressed */
	J1939_BJM_STATUS_ERROR = 2,
	J1939_BJM_STATUS_NOT_AVAILABLE = 3,
};

/**
 * struct j1939_bjm_packet - Represents a PGN 64982/64984 Basic Joystick
 *			     Message packet
 *
 * @x_status_pos: X-axis statuses and the two LSBs of the X-axis position
 *           - Bits 1-2: Joystick X-Axis Neutral Position Status
 *             (SPN 1792 for BJM1, SPN 1812 for BJM2)
 *           - Bits 3-4: Joystick X-Axis Lever Left Negative Position Status
 *             (SPN 1793 / 1813)
 *           - Bits 5-6: Joystick X-Axis Lever Right Positive Position Status
 *             (SPN 1794 / 1814)
 *           - Bits 7-8: Joystick X-Axis Position, 2 LSBs (SPN 1795 / 1815)
 *
 * @x_pos_msb: Joystick X-Axis Position, 8 MSBs (SPN 1795 / 1815)
 *           - Resolution: 0.1 %/bit, Offset: 0
 *           - Range: 0 to 100 % lever travel from neutral
 *
 * @y_status_pos: Y-axis statuses and the two LSBs of the Y-axis position
 *           - Bits 1-2: Joystick Y-Axis Neutral Position Status
 *             (SPN 1796 / 1816)
 *           - Bits 3-4: Joystick Y-Axis Lever Back Negative Position Status
 *             (SPN 1797 / 1817)
 *           - Bits 5-6: Joystick Y-Axis Lever Forward Positive Position
 *             Status (SPN 1798 / 1818)
 *           - Bits 7-8: Joystick Y-Axis Position, 2 LSBs (SPN 1799 / 1819)
 *
 * @y_pos_msb: Joystick Y-Axis Position, 8 MSBs (SPN 1799 / 1819)
 *
 * @buttons: Button 1 to 12 Pressed Status, 2 bits each, 4 buttons per byte
 *           - Byte 0: Buttons 1 to 4 (SPNs 1800-1803 / 1820-1823)
 *           - Byte 1: Buttons 5 to 8 (SPNs 1804-1807 / 1824-1827)
 *           - Byte 2: Buttons 9 to 12 (SPNs 1808-1811 / 1828-1831)
 *
 * @reserved: Not assigned, must be set to 0xFF as per J1939-71, section 5.2
 *
 * This structure defines each component of the Basic Joystick Message as
 * described in PGN 64982 (BJM1) and PGN 64984 (BJM2). Both PGNs share the
 * same layout; they differ only in the SPN numbers assigned to the fields.
 */
struct j1939_bjm_packet {
	uint8_t x_status_pos;
	uint8_t x_pos_msb;
	uint8_t y_status_pos;
	uint8_t y_pos_msb;
	uint8_t reserved;
	uint8_t buttons[3];
} __attribute__((__packed__));

/**
 * j1939_bjm_set_x_statuses - Set the X-axis position statuses in the packet
 * @packet: Pointer to the J1939 BJM packet
 * @neutral: Neutral position status
 * @negative: Lever left negative position status
 * @positive: Lever right positive position status
 */
static inline void j1939_bjm_set_x_statuses(struct j1939_bjm_packet *packet, enum j1939_bjm_status neutral, enum j1939_bjm_status negative, enum j1939_bjm_status positive)
{
	packet->x_status_pos = (packet->x_status_pos & 0xc0) | (neutral & 0x3) | ((negative & 0x3) << 2) | ((positive & 0x3) << 4);
}

/**
 * j1939_bjm_get_x_neutral - Get the X-axis neutral position status
 * @packet: Pointer to the J1939 BJM packet
 *
 * Return: X-axis neutral position status
 */
static inline enum j1939_bjm_status j1939_bjm_get_x_neutral(const struct j1939_bjm_packet *packet)
{
	return packet->x_status_pos & 0x3;
}

/**
 * j1939_bjm_set_x_position - Set the X-axis position in the packet
 * @packet: Pointer to the J1939 BJM packet
 * @position: Lever travel from neutral in 0.1 % units (0 to 1000)
 */
static inline void j1939_bjm_set_x_position(struct j1939_bjm_packet *packet, uint16_t position)
{
	packet->x_status_pos = (packet->x_status_pos & 0x3f) | ((position & 0x3) << 6);
	packet->x_pos_msb = (position >> 2) & 0xff;
}

/**
 * j1939_bjm_get_x_position - Get the X-axis position from the packet
 * @packet: Pointer to the J1939 BJM packet
 *
 * Return: Lever travel from neutral in 0.1 % units (0 to 1000)
 */
static inline uint16_t j1939_bjm_get_x_position(const struct j1939_bjm_packet *packet)
{
	return (packet->x_status_pos >> 6) | (packet->x_pos_msb << 2);
}

/**
 * j1939_bjm_set_y_statuses - Set the Y-axis position statuses in the packet
 * @packet: Pointer to the J1939 BJM packet
 * @neutral: Neutral position status
 * @negative: Lever back negative position status
 * @positive: Lever forward positive position status
 */
static inline void j1939_bjm_set_y_statuses(struct j1939_bjm_packet *packet, enum j1939_bjm_status neutral, enum j1939_bjm_status negative, enum j1939_bjm_status positive)
{
	packet->y_status_pos = (packet->y_status_pos & 0xc0) | (neutral & 0x3) | ((negative & 0x3) << 2) | ((positive & 0x3) << 4);
}

/**
 * j1939_bjm_get_y_neutral - Get the Y-axis neutral position status
 * @packet: Pointer to the J1939 BJM packet
 *
 * Return: Y-axis neutral position status
 */
static inline enum j1939_bjm_status j1939_bjm_get_y_neutral(const struct j1939_bjm_packet *packet)
{
	return packet->y_status_pos & 0x3;
}

/**
 * j1939_bjm_set_y_position - Set the Y-axis position in the packet
 * @packet: Pointer to the J1939 BJM packet
 * @position: Lever travel from neutral in 0.1 % units (0 to 1000)
 */
static inline void j1939_bjm_set_y_position(struct j1939_bjm_packet *packet, uint16_t position)
{
	packet->y_status_pos = (packet->y_status_pos & 0x3f) | ((position & 0x3) << 6);
	packet->y_pos_msb = (position >> 2) & 0xff;
}

/**
 * j1939_bjm_get_y_position - Get the Y-axis position from the packet
 * @packet: Pointer to the J1939 BJM packet
 *
 * Return: Lever travel from neutral in 0.1 % units (0 to 1000)
 */
static inline uint16_t j1939_bjm_get_y_position(const struct j1939_bjm_packet *packet)
{
	return (packet->y_status_pos >> 6) | (packet->y_pos_msb << 2);
}

#define J1939_BJM_NUM_BUTTONS 12

/**
 * j1939_bjm_set_button - Set a button pressed status in the packet
 * @packet: Pointer to the J1939 BJM packet
 * @button: Button number, 1 to 12
 * @status: Button pressed status
 */
static inline void j1939_bjm_set_button(struct j1939_bjm_packet *packet, unsigned int button, enum j1939_bjm_status status)
{
	unsigned int shift = ((button - 1) % 4) * 2;
	uint8_t *byte;

	if (button < 1 || button > J1939_BJM_NUM_BUTTONS)
		return;

	byte = &packet->buttons[(button - 1) / 4];
	*byte = (*byte & ~(0x3 << shift)) | ((status & 0x3) << shift);
}

/**
 * j1939_bjm_get_button - Get a button pressed status from the packet
 * @packet: Pointer to the J1939 BJM packet
 * @button: Button number, 1 to 12
 *
 * Return: Button pressed status, or J1939_BJM_STATUS_NOT_AVAILABLE for an
 *	   out of range button number
 */
static inline enum j1939_bjm_status j1939_bjm_get_button(const struct j1939_bjm_packet *packet, unsigned int button)
{
	unsigned int shift = ((button - 1) % 4) * 2;
	const uint8_t *byte;

	if (button < 1 || button > J1939_BJM_NUM_BUTTONS)
		return J1939_BJM_STATUS_NOT_AVAILABLE;

	byte = &packet->buttons[(button - 1) / 4];
	return (*byte >> shift) & 0x3;
}

#endif /* !_J1939_BJM_H_ */
