// SPDX-License-Identifier: LGPL-2.0-only
// SPDX-FileCopyrightText: 2026 Roy Ratcliffe

#ifndef _ISOBUS_HRTBTM_CMN_H_
#define _ISOBUS_HRTBTM_CMN_H_

#include <stdint.h>

#include <linux/can.h>
#include <linux/kernel.h>
#include "../libj1939.h"
#include "../lib.h"

/*
 * ISO 11783-7 - Heartbeat message - HRTBTM
 *
 * The heartbeat message is transmitted cyclically by a control function
 * (CF) so that other CFs can monitor the integrity of its communication.
 * It carries a single sequence counter byte which is incremented by one
 * for each transmission and wraps from 250 back to 0, allowing receivers
 * to detect missed or repeated messages.
 */
#define ISOBUS_PGN_HRTBTM 0x0f0e4 /* 61668 */

#define ISOBUS_HRTBTM_PRIO_DEFAULT 3
#define ISOBUS_HRTBTM_MAX_TRANSFER_LENGH sizeof(struct isobus_hrtbtm_packet)
#define ISOBUS_HRTBTM_REPETITION_RATE_MS 100

/* Sequence counter range is 0..250; values 251..255 are reserved. */
#define ISOBUS_HRTBTM_SEQ_MAX 250
#define ISOBUS_HRTBTM_SEQ_MODULO (ISOBUS_HRTBTM_SEQ_MAX + 1)

/*
 * A receiver should consider the heartbeat lost if no message arrives
 * within the repetition rate plus a tolerance. Default to three missed
 * cycles before declaring a timeout.
 */
#define ISOBUS_HRTBTM_TIMEOUT_MS (3 * ISOBUS_HRTBTM_REPETITION_RATE_MS)

/**
 * struct isobus_hrtbtm_packet - ISO 11783-7 Heartbeat message
 * @sequence_counter: incremented by one per transmission, 0..250, then
 *                    wraps to 0. Values 251..255 are reserved.
 */
struct isobus_hrtbtm_packet {
	uint8_t sequence_counter;
} __attribute__((__packed__));

static inline uint8_t isobus_hrtbtm_get_sequence_counter(const struct isobus_hrtbtm_packet *packet)
{
	return packet->sequence_counter;
}

static inline void isobus_hrtbtm_set_sequence_counter(struct isobus_hrtbtm_packet *packet, uint8_t seq)
{
	packet->sequence_counter = seq;
}

#endif /* !_ISOBUS_HRTBTM_CMN_H_ */
