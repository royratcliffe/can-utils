// SPDX-License-Identifier: LGPL-2.0-only
// SPDX-FileCopyrightText: 2026 Roy Ratcliffe <roy@ratcliffe.me>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/joystick.h>
#include <linux/kernel.h>
#include <net/if.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "j1939_bjm_cmn.h"

#define J1939_BJM_SRV_MAX_EPOLL_EVENTS 10

/* Lever travel below this fraction of full scale counts as neutral */
#define J1939_BJM_SRV_DEADBAND 1024 /* ~3 % of 32767 */

/* Lever travel within this margin of full scale counts as in detent */
#define J1939_BJM_SRV_DETENT_BAND (32767 / 1000) /* 0.1 % of 32767 */

#define J1939_BJM_SRV_JOY1_DEFAULT "/dev/input/js0"
#define J1939_BJM_SRV_JOY2_DEFAULT "/dev/input/js1"

enum j1939_bjm_joy_state {
	J1939_BJM_JOY_ABSENT, /* no device configured or attached */
	J1939_BJM_JOY_OK, /* device open and delivering events */
	J1939_BJM_JOY_ERROR, /* device was lost at runtime */
};

struct j1939_bjm_joystick {
	const char *path;
	bool explicit_path;
	enum j1939_bjm_joy_state state;
	int fd;
	uint8_t num_axes;
	uint8_t num_buttons;
	int16_t x;
	int16_t y;
	uint16_t buttons; /* bitmask, bit 0 is button 1 */
	bool updated; /* state changed since last transmission */
};

struct j1939_bjm_srv_priv;

struct j1939_pgn_handler {
	uint32_t pgn;
	int (*prepare_data)(struct j1939_bjm_srv_priv *priv, void *data);
	int sock;
	int sock_priority;
	struct timespec last_time;
	struct timespec next_time;
	int repetition_rate_ms;
	int jitter_ms;
	size_t data_size;
	unsigned int joy_index;
	bool enabled;
};

struct j1939_bjm_srv_priv {
	struct sockaddr_can sockname;
	struct j1939_bjm_stats stats;
	struct libj1939_cmn cmn;
	bool sim_mode;
	struct j1939_bjm_joystick joy[2];
	struct j1939_pgn_handler *handlers;
	size_t num_handlers;
};

/**
 * simulate_joystick_data - Generate simulated joystick data for testing.
 * @priv: Pointer to the private data structure of the J1939 BJM server.
 *
 * This function sweeps both simulated joysticks through their X and Y axis
 * ranges with a triangle wave and cycles through the buttons, so that the
 * broadcast BJM1 and BJM2 messages exercise all fields without real
 * hardware being attached.
 */
static void simulate_joystick_data(struct j1939_bjm_srv_priv *priv)
{
	static int32_t sweep;
	static unsigned int button;
	int32_t value;

	sweep += 1024;
	if (sweep > 4 * 32767)
		sweep = 0;

	/* Triangle wave from -32767 to 32767 and back */
	value = sweep <= 2 * 32767 ? sweep - 32767 : 3 * 32767 - sweep;

	for (size_t i = 0; i < ARRAY_SIZE(priv->joy); ++i) {
		struct j1939_bjm_joystick *joy = &priv->joy[i];

		joy->state = J1939_BJM_JOY_OK;
		joy->num_axes = 2;
		joy->num_buttons = J1939_BJM_NUM_BUTTONS;
		/* Second joystick sweeps in the opposite direction */
		joy->x = i ? -value : value;
		joy->y = i ? value : -value;
		joy->buttons = 1 << button;
	}

	button = (button + 1) % J1939_BJM_NUM_BUTTONS;
}

/**
 * j1939_bjm_joy_open - Open a joystick device and add it to epoll.
 * @priv: Pointer to the private data structure of the J1939 BJM server.
 * @joy: Pointer to the joystick slot to open.
 *
 * This function opens the configured joystick device in non-blocking mode,
 * queries the number of axes and buttons, and registers the file descriptor
 * with the epoll instance so that joystick events wake the main loop.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int j1939_bjm_joy_open(struct j1939_bjm_srv_priv *priv, struct j1939_bjm_joystick *joy)
{
	int ret, fd;

	fd = open(joy->path, O_RDONLY | O_NONBLOCK);
	if (fd < 0)
		return -errno;

	if (ioctl(fd, JSIOCGAXES, &joy->num_axes) < 0 || ioctl(fd, JSIOCGBUTTONS, &joy->num_buttons) < 0) {
		ret = -errno;
		pr_err("%s: failed to query joystick properties: %d (%s)\n", joy->path, ret, strerror(-ret));
		close(fd);
		return ret;
	}

	ret = libj1939_add_socket_to_epoll(priv->cmn.epoll_fd, fd, EPOLLIN);
	if (ret < 0) {
		pr_err("%s: failed to add joystick to epoll: %d\n", joy->path, ret);
		close(fd);
		return ret;
	}

	joy->fd = fd;
	joy->state = J1939_BJM_JOY_OK;

	pr_info("%s: joystick attached with %u axes and %u buttons\n", joy->path, joy->num_axes, joy->num_buttons);

	return 0;
}

/**
 * j1939_bjm_joy_lost - Mark a joystick device as lost.
 * @priv: Pointer to the private data structure of the J1939 BJM server.
 * @joy: Pointer to the joystick slot that was lost.
 *
 * This function closes the file descriptor of a joystick that disappeared
 * at runtime and flags the slot as being in error, so the corresponding
 * BJM message reports error statuses instead of stale data.
 */
static void j1939_bjm_joy_lost(struct j1939_bjm_srv_priv *priv, struct j1939_bjm_joystick *joy)
{
	pr_warn("%s: joystick lost\n", joy->path);

	epoll_ctl(priv->cmn.epoll_fd, EPOLL_CTL_DEL, joy->fd, NULL);
	close(joy->fd);
	joy->fd = -1;
	joy->state = J1939_BJM_JOY_ERROR;
	/* Broadcast the error statuses without waiting for the next
	 * periodic transmission */
	joy->updated = true;
}

/**
 * j1939_bjm_joy_rx - Drain pending events from a joystick device.
 * @priv: Pointer to the private data structure of the J1939 BJM server.
 * @joy: Pointer to the joystick slot to read from.
 *
 * This function reads all pending events from the Linux joystick device and
 * updates the cached axis positions and button states. Axis 0 maps to the
 * BJM X-axis and axis 1 to the BJM Y-axis; buttons 0 to 11 map to BJM
 * buttons 1 to 12. If the device disappears the slot is marked as lost.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int j1939_bjm_joy_rx(struct j1939_bjm_srv_priv *priv, struct j1939_bjm_joystick *joy)
{
	struct js_event ev;
	ssize_t ret;

	while (true) {
		ret = read(joy->fd, &ev, sizeof(ev));
		if (ret < 0) {
			if (errno == EAGAIN)
				return 0;

			ret = -errno;
			j1939_bjm_joy_lost(priv, joy);
			return ret;
		}

		if (ret != sizeof(ev)) {
			pr_warn("%s: short joystick event read: %zi\n", joy->path, ret);
			return -EIO;
		}

		switch (ev.type & ~JS_EVENT_INIT) {
		case JS_EVENT_AXIS:
			if (ev.number == 0 && joy->x != ev.value) {
				joy->x = ev.value;
				joy->updated = true;
			} else if (ev.number == 1 && joy->y != ev.value) {
				joy->y = ev.value;
				joy->updated = true;
			}
			break;
		case JS_EVENT_BUTTON: {
			uint16_t buttons = joy->buttons;

			if (ev.number >= J1939_BJM_NUM_BUTTONS)
				break;
			if (ev.value)
				buttons |= 1 << ev.number;
			else
				buttons &= ~(1 << ev.number);
			if (joy->buttons != buttons) {
				joy->buttons = buttons;
				joy->updated = true;
			}
			break;
		}
		default:
			break;
		}
	}
}

/* ----------------- PGN handlers start ----------------- */

/**
 * j1939_bjm_axis_fill - Fill one BJM axis from a raw joystick axis value.
 * @value: Raw joystick axis value in the range -32767 to 32767.
 * @neutral: Pointer to store the neutral position status.
 * @negative: Pointer to store the negative direction position status.
 * @positive: Pointer to store the positive direction position status.
 *
 * This function converts a signed Linux joystick axis value into the BJM
 * representation: a 10 bit lever travel magnitude in 0.1 % units plus three
 * discrete statuses indicating whether the lever is in the neutral position
 * or deflected in the negative or positive direction. A small dead band
 * around the centre counts as neutral.
 *
 * Return: Lever travel from neutral in 0.1 % units (0 to 1000).
 */
static uint16_t j1939_bjm_axis_fill(int16_t value, enum j1939_bjm_status *neutral, enum j1939_bjm_status *negative, enum j1939_bjm_status *positive)
{
	int32_t magnitude = value < 0 ? -(int32_t)value : value;

	if (magnitude <= J1939_BJM_SRV_DEADBAND) {
		*neutral = J1939_BJM_STATUS_ON;
		*negative = J1939_BJM_STATUS_OFF;
		*positive = J1939_BJM_STATUS_OFF;
		return 0;
	}

	*neutral = J1939_BJM_STATUS_OFF;
	*negative = value < 0 ? J1939_BJM_STATUS_ON : J1939_BJM_STATUS_OFF;
	*positive = value < 0 ? J1939_BJM_STATUS_OFF : J1939_BJM_STATUS_ON;

	return magnitude * J1939_BJM_POSITION_MAX / 32767;
}

/**
 * j1939_bjm_axis_in_detent - Test whether a raw axis value is in detent.
 * @value: Raw joystick axis value in the range -32767 to 32767.
 *
 * The BJM detent position status reports a lever latched in an end-of-travel
 * detent. A PC joystick has no physical detent, so report "in detent" when
 * the axis magnitude is within 0.1 % of full travel in either direction.
 *
 * Return: true if the axis is at full travel, false otherwise.
 */
static bool j1939_bjm_axis_in_detent(int16_t value)
{
	int32_t magnitude = value < 0 ? -(int32_t)value : value;

	return magnitude >= 32767 - J1939_BJM_SRV_DETENT_BAND;
}

/**
 * j1939_bjm_get_data - Populate a BJM packet from a joystick slot.
 * @priv: Pointer to the server's private data structure.
 * @joy: Pointer to the joystick slot to encode.
 * @bjmp: Pointer to the BJM packet structure to populate.
 *
 * This function fills the provided BJM packet from the cached joystick
 * state. An attached joystick reports its axis positions and button
 * statuses; axes and buttons beyond what the device provides are reported
 * as not available. A joystick lost at runtime reports error statuses so
 * that consumers can fail safe.
 *
 * Return: 0 on success.
 */
static int j1939_bjm_get_data(struct j1939_bjm_srv_priv *priv, struct j1939_bjm_joystick *joy, struct j1939_bjm_packet *bjmp)
{
	enum j1939_bjm_status neutral, negative, positive;
	enum j1939_bjm_status fail_status;
	uint16_t position;
	unsigned int i;

	if (joy->state != J1939_BJM_JOY_OK) {
		fail_status = joy->state == J1939_BJM_JOY_ERROR ? J1939_BJM_STATUS_ERROR : J1939_BJM_STATUS_NOT_AVAILABLE;

		j1939_bjm_set_x_statuses(bjmp, fail_status, fail_status, fail_status);
		j1939_bjm_set_x_position(bjmp, J1939_BJM_POSITION_NOT_AVAILABLE);
		j1939_bjm_set_y_statuses(bjmp, fail_status, fail_status, fail_status);
		j1939_bjm_set_y_position(bjmp, J1939_BJM_POSITION_NOT_AVAILABLE);
		for (i = 1; i <= J1939_BJM_NUM_BUTTONS; ++i)
			j1939_bjm_set_button(bjmp, i, fail_status);

		bjmp->detent_reserved = 0xf0;

		return 0;
	}

	if (joy->num_axes >= 1) {
		position = j1939_bjm_axis_fill(joy->x, &neutral, &negative, &positive);
		j1939_bjm_set_x_statuses(bjmp, neutral, negative, positive);
		j1939_bjm_set_x_position(bjmp, position);
	} else {
		j1939_bjm_set_x_statuses(bjmp, J1939_BJM_STATUS_NOT_AVAILABLE, J1939_BJM_STATUS_NOT_AVAILABLE, J1939_BJM_STATUS_NOT_AVAILABLE);
		j1939_bjm_set_x_position(bjmp, J1939_BJM_POSITION_NOT_AVAILABLE);
	}

	if (joy->num_axes >= 2) {
		position = j1939_bjm_axis_fill(joy->y, &neutral, &negative, &positive);
		j1939_bjm_set_y_statuses(bjmp, neutral, negative, positive);
		j1939_bjm_set_y_position(bjmp, position);
	} else {
		j1939_bjm_set_y_statuses(bjmp, J1939_BJM_STATUS_NOT_AVAILABLE, J1939_BJM_STATUS_NOT_AVAILABLE, J1939_BJM_STATUS_NOT_AVAILABLE);
		j1939_bjm_set_y_position(bjmp, J1939_BJM_POSITION_NOT_AVAILABLE);
	}

	for (i = 1; i <= J1939_BJM_NUM_BUTTONS; ++i) {
		enum j1939_bjm_status status;

		if (i > joy->num_buttons)
			status = J1939_BJM_STATUS_NOT_AVAILABLE;
		else if (joy->buttons & (1 << (i - 1)))
			status = J1939_BJM_STATUS_ON;
		else
			status = J1939_BJM_STATUS_OFF;

		j1939_bjm_set_button(bjmp, i, status);
	}

	/*
	 * Byte 5 contains the X and Y axis detent position status values.
	 * - Bits 7-8: X-axis detent position status (SPN 2680 / 2717)
	 * - Bits 5-6: Y-axis detent position status (SPN 2681 / 2718)
	 * - Bits 1-4: Reserved, transmitted as all 1s per J1939-71
	 * Report "in detent" when the axis sits at exactly full travel in
	 * either direction, the nearest analogue of a lever latched in its
	 * end detent.
	 */
	bjmp->detent_reserved = (j1939_bjm_axis_in_detent(joy->x) << 6) | (j1939_bjm_axis_in_detent(joy->y) << 4);

	return 0;
}

static int j1939_bjm1_prepare_data(struct j1939_bjm_srv_priv *priv, void *data)
{
	struct j1939_bjm_packet *bjmp = data;

	return j1939_bjm_get_data(priv, &priv->joy[0], bjmp);
}

static int j1939_bjm2_prepare_data(struct j1939_bjm_srv_priv *priv, void *data)
{
	struct j1939_bjm_packet *bjmp = data;

	return j1939_bjm_get_data(priv, &priv->joy[1], bjmp);
}

/* ----------------- PGN handlers end ----------------- */

/**
 * prepare_and_send_message - Handles data preparation and transmission for a
 *			      PGN handler.
 * @priv: Pointer to the server's private data structure.
 * @handler: Pointer to the PGN handler with the necessary data functions.
 *
 * This function is responsible for preparing the data associated with a
 * specific PGN handler and sending it via the corresponding socket.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int prepare_and_send_message(struct j1939_bjm_srv_priv *priv, struct j1939_pgn_handler *handler)
{
	uint8_t data[J1939_BJM_MAX_TRANSFER_LENGTH];
	int ret;

	if (sizeof(data) < handler->data_size) {
		pr_warn("Data buffer too small for PGN %u: %zu < %zu\n", handler->pgn, sizeof(data), handler->data_size);
		return -EINVAL;
	}

	memset(data, 0, handler->data_size);
	ret = handler->prepare_data(priv, data);
	if (ret < 0) {
		pr_warn("Failed to prepare data for PGN %u: %i\n", handler->pgn, ret);
		return ret;
	}

	ret = send(handler->sock, data, handler->data_size, MSG_DONTWAIT);
	if (ret == -1) {
		ret = -errno;
		pr_warn("Failed to send data for PGN %u: %i (%s)\n", handler->pgn, ret, strerror(-ret));
		return ret;
	}

	return 0;
}

/**
 * j1939_bjm_srv_process_pgn_request - Processes a PGN request message.
 * @priv: Pointer to the server's private data structure.
 * @msg: Pointer to the received J1939 BJM message.
 *
 * This function processes a PGN request message and sends the corresponding
 * BJM message back to the requester. It iterates over the available PGN
 * handlers to find the appropriate one based on the requested PGN.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int j1939_bjm_srv_process_pgn_request(struct j1939_bjm_srv_priv *priv, struct j1939_bjm_msg *msg)
{
	uint32_t requested_pgn = (msg->buf[2] << 16) | (msg->buf[1] << 8) | msg->buf[0];
	int ret = -EINVAL;

	/* Iterate over all handlers to find the appropriate PGN */
	for (size_t i = 0; i < priv->num_handlers; ++i) {
		struct j1939_pgn_handler *handler = &priv->handlers[i];

		if (handler->pgn != requested_pgn)
			continue;

		if (!handler->enabled)
			continue;

		if (priv->sim_mode)
			simulate_joystick_data(priv);

		ret = prepare_and_send_message(priv, handler);
		if (ret < 0) {
			pr_warn("Handler for PGN %u returned error %d\n", handler->pgn, ret);
		}
		return ret;
	}

	pr_warn("No handler found for PGN %u\n", requested_pgn);
	return ret;
}

/**
 * j1939_bjm_srv_rx_buf - Processes a received J1939 message.
 * @priv: Pointer to the server's private data structure.
 * @msg: Pointer to the received J1939 BJM message.
 *
 * This function processes a received J1939 message by checking the PGN
 * and calling the appropriate handler function.
 * Currently, only the PGN_REQUEST_PGN message is supported, and other
 * messages are ignored with a warning message.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int j1939_bjm_srv_rx_buf(struct j1939_bjm_srv_priv *priv, struct j1939_bjm_msg *msg)
{
	pgn_t pgn = msg->peername.can_addr.j1939.pgn;
	int ret = 0;

	switch (pgn) {
	case J1939_PGN_REQUEST_PGN:
		ret = j1939_bjm_srv_process_pgn_request(priv, msg);
		break;
	default:
		pr_warn("%s: unsupported PGN: %x\n", __func__, pgn);
		/* Not a critical error */
		break;
	}

	return ret;
}

/**
 * j1939_bjm_srv_rx_one - Receives a single J1939 message from a socket.
 * @priv: Pointer to the server's private data structure.
 * @sock: The file descriptor of the socket to receive from.
 *
 * This function receives a single J1939 message from the specified socket
 * and processes it by calling j1939_bjm_srv_rx_buf.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int j1939_bjm_srv_rx_one(struct j1939_bjm_srv_priv *priv, int sock)
{
	struct j1939_bjm_msg msg = { 0 };
	int flags = 0;
	int ret;

	msg.buf_size = J1939_BJM_MAX_TRANSFER_LENGTH;
	msg.peer_addr_len = sizeof(msg.peername);
	msg.sock = sock;

	ret = recvfrom(sock, &msg.buf[0], msg.buf_size, flags, (struct sockaddr *)&msg.peername, &msg.peer_addr_len);

	if (ret < 0) {
		ret = -errno;
		pr_warn("recvfrom() failed: %i %s\n", ret, strerror(-ret));
		return ret;
	}

	if (ret < 3) {
		pr_warn("received too short message: %i\n", ret);
		return -EINVAL;
	}

	msg.len = ret;

	ret = j1939_bjm_srv_rx_buf(priv, &msg);
	if (ret < 0) {
		pr_warn("failed to process rx buf: %i (%s)\n", ret, strerror(-ret));
		return ret;
	}

	return 0;
}

/**
 * j1939_bjm_srv_handle_events - Handles events for the J1939 BJM server.
 * @priv: Pointer to the server's private data structure.
 * @nfds: The number of file descriptors to handle.
 *
 * This function processes events for the J1939 BJM server by checking for
 * joystick input events and incoming messages on the sockets and calling
 * the appropriate handler functions.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int j1939_bjm_srv_handle_events(struct j1939_bjm_srv_priv *priv, unsigned int nfds)
{
	unsigned int n;
	int ret;

	for (n = 0; n < nfds && n < priv->cmn.epoll_events_size; ++n) {
		struct epoll_event *ev = &priv->cmn.epoll_events[n];
		struct j1939_bjm_joystick *joy = NULL;

		if (!ev->events) {
			warn("no events");
			continue;
		}

		for (size_t i = 0; i < ARRAY_SIZE(priv->joy); ++i) {
			if (priv->joy[i].fd == ev->data.fd) {
				joy = &priv->joy[i];
				break;
			}
		}

		if (joy) {
			ret = j1939_bjm_joy_rx(priv, joy);
			if (ret) {
				pr_warn("failed to read joystick events: %i (%s)\n", ret, strerror(-ret));
				/* Keep broadcasting error statuses */
			}
			continue;
		}

		if (ev->events & POLLIN) {
			ret = j1939_bjm_srv_rx_one(priv, ev->data.fd);
			if (ret) {
				warn("recv one");
				return ret;
			}
		}
	}
	return 0;
}

/**
 * handler_next_send_time - Determine the effective next send time for a
 *			    PGN handler.
 * @priv: Pointer to the server's private data structure.
 * @handler: Pointer to the PGN handler structure.
 *
 * This function returns the handler's scheduled periodic send time, pulled
 * forward when the wired joystick has new events pending: an updated
 * joystick is broadcast as soon as the minimum event interval has elapsed
 * since the last transmission, rather than waiting for the full repetition
 * rate.
 *
 * Return: the effective next send time as a struct timespec.
 */
static struct timespec handler_next_send_time(struct j1939_bjm_srv_priv *priv, struct j1939_pgn_handler *handler)
{
	struct timespec next = handler->next_time;
	struct timespec event_next;

	if (!priv->joy[handler->joy_index].updated)
		return next;

	event_next = handler->last_time;
	timespec_add_ms(&event_next, J1939_BJM_EVENT_RATE_MS);

	if (timespec_diff_ms(&next, &event_next) > 0)
		next = event_next;

	return next;
}

/**
 * determine_earliest_next_send_time - Determines the earliest next send time.
 * @priv: Pointer to the server's private data structure.
 *
 * This function determines the earliest next send time for all PGN handlers
 * based on their repetition rates, last send times and pending joystick
 * events.
 *
 * Return: the earliest time as a struct timespec.
 */
static struct timespec determine_earliest_next_send_time(struct j1939_bjm_srv_priv *priv)
{
	struct timespec earliest = { 0, 0 };

	for (size_t i = 0; i < priv->num_handlers; ++i) {
		struct timespec next;

		if (!priv->handlers[i].enabled)
			continue;

		next = handler_next_send_time(priv, &priv->handlers[i]);

		if (earliest.tv_sec == 0 && earliest.tv_nsec == 0)
			earliest = next;

		if ((next.tv_sec < earliest.tv_sec) || (next.tv_sec == earliest.tv_sec && next.tv_nsec < earliest.tv_nsec))
			earliest = next;
	}

	return earliest;
}

/**
 * send_message_for_handler - Sends a periodic message for a PGN handler.
 * @priv: Pointer to the server's private data structure.
 * @handler: Pointer to the PGN handler structure.
 *
 * This function sends a message for a specific PGN handler based on the
 * repetition rate and jitter, or earlier when new joystick events are
 * pending and the minimum event interval has elapsed since the last
 * transmission. Event-driven sends allow no jitter, which enforces the
 * minimum interval as a rate limit. It updates the last and next send
 * times for the handler and clears the pending event flag.
 *
 * Returns 0 on success or a negative error code if an issue occurs.
 */
static int send_message_for_handler(struct j1939_bjm_srv_priv *priv, struct j1939_pgn_handler *handler)
{
	struct timespec next;
	int64_t time_diff;
	bool updated;
	int ret;

	if (!handler->enabled)
		return 0;

	updated = priv->joy[handler->joy_index].updated;
	next = handler_next_send_time(priv, handler);

	time_diff = timespec_diff_ms(&next, &priv->cmn.last_time);
	if (time_diff > (updated ? 0 : handler->jitter_ms))
		return 0;

	ret = prepare_and_send_message(priv, handler);
	if (ret < 0)
		return ret;

	priv->joy[handler->joy_index].updated = false;
	handler->last_time = priv->cmn.last_time;
	handler->next_time = priv->cmn.last_time;
	timespec_add_ms(&handler->next_time, handler->repetition_rate_ms);

	return 0;
}

/**
 * send_periodic_messages - Sends periodic messages for all PGN handlers.
 * @priv: Pointer to the server's private data structure.
 *
 * This function sends periodic messages for all PGN handlers based on their
 * repetition rates and jitter. It iterates over all handlers and calls the
 * send_message_for_handler function to send the messages.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int send_periodic_messages(struct j1939_bjm_srv_priv *priv)
{
	int ret = 0;

	for (size_t i = 0; i < priv->num_handlers; ++i) {
		ret = send_message_for_handler(priv, &priv->handlers[i]);
		if (ret < 0) {
			pr_warn("Failed to send periodic message for handler %zu. Error: %d (%s)\n", i, ret, strerror(-ret));
		}
	}

	return ret;
}

/**
 * j1939_bjm_srv_process_events_and_tasks - Processes events and tasks for the
 *					    J1939 BJM server.
 * @priv: Pointer to the server's private data structure.
 *
 * This function processes events and tasks for the J1939 BJM server by
 * preparing for events, handling joystick and socket events, and sending
 * the periodic BJM messages.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int j1939_bjm_srv_process_events_and_tasks(struct j1939_bjm_srv_priv *priv)
{
	int64_t time_diff;
	int ret, nfds;

	priv->cmn.next_send_time = determine_earliest_next_send_time(priv);
	ret = libj1939_prepare_for_events(&priv->cmn, &nfds, false);
	if (ret)
		pr_err("failed to prepare for events: %i (%s)\n", ret, strerror(-ret));

	if (!ret && nfds > 0) {
		ret = j1939_bjm_srv_handle_events(priv, nfds);
		if (ret)
			pr_err("failed to handle events: %i (%s)\n", ret, strerror(-ret));
	}

	/* Test if it is proper time to send next status message. */
	time_diff = timespec_diff_ms(&priv->cmn.next_send_time, &priv->cmn.last_time);
	if (time_diff > 0) {
		/* Too early to send next message */
		return 0;
	}

	if (priv->sim_mode)
		simulate_joystick_data(priv);

	return send_periodic_messages(priv);
}

static struct j1939_pgn_handler pgn_handlers[] = {
	{
		.pgn = J1939_PGN_BJM1,
		.prepare_data = j1939_bjm1_prepare_data,
		.sock_priority = J1939_BJM_PRIO_DEFAULT,
		.repetition_rate_ms = J1939_BJM_REPETITION_RATE_MS,
		.jitter_ms = J1939_BJM_JITTER_MS,
		.data_size = sizeof(struct j1939_bjm_packet),
		.joy_index = 0,
	},
	{
		.pgn = J1939_PGN_BJM2,
		.prepare_data = j1939_bjm2_prepare_data,
		.sock_priority = J1939_BJM_PRIO_DEFAULT,
		.repetition_rate_ms = J1939_BJM_REPETITION_RATE_MS,
		.jitter_ms = J1939_BJM_JITTER_MS,
		.data_size = sizeof(struct j1939_bjm_packet),
		.joy_index = 1,
	},
};

/**
 * initialize_socket_for_handler - Initializes a socket for a PGN handler.
 * @priv: Pointer to the server's private data structure.
 * @handler: Pointer to the PGN handler structure.
 *
 * This function initializes a socket for a specific PGN handler by opening,
 * binding, and connecting the socket. It sets the socket priority, enables
 * broadcast, and adds the socket to the epoll instance for event handling.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int initialize_socket_for_handler(struct j1939_bjm_srv_priv *priv, struct j1939_pgn_handler *handler)
{
	struct sockaddr_can addr = priv->sockname;
	int ret;

	ret = libj1939_open_socket();
	if (ret < 0) {
		pr_err("Failed to open socket for PGN %u: %d\n", handler->pgn, ret);
		return ret;
	}

	handler->sock = ret;

	ret = libj1939_bind_socket(handler->sock, &addr);
	if (ret < 0) {
		pr_err("Failed to bind socket for PGN %u: %d\n", handler->pgn, ret);
		return ret;
	}

	ret = libj1939_socket_prio(handler->sock, handler->sock_priority);
	if (ret < 0) {
		pr_err("Failed to set socket priority for PGN %u: %d\n", handler->pgn, ret);
		return ret;
	}

	ret = libj1939_set_broadcast(handler->sock);
	if (ret < 0) {
		pr_err("Failed to set broadcast for PGN %u: %d\n", handler->pgn, ret);
		return ret;
	}

	addr.can_addr.j1939.name = J1939_NO_NAME;
	addr.can_addr.j1939.addr = J1939_NO_ADDR;
	addr.can_addr.j1939.pgn = handler->pgn;
	ret = libj1939_connect_socket(handler->sock, &addr);
	if (ret < 0) {
		pr_err("Failed to connect socket for PGN %u: %d\n", handler->pgn, ret);
		return ret;
	}

	ret = libj1939_add_socket_to_epoll(priv->cmn.epoll_fd, handler->sock, EPOLLIN);
	if (ret < 0) {
		pr_err("Failed to add socket to epoll for PGN %u: %d\n", handler->pgn, ret);
		return ret;
	}

	return 0;
}

/**
 * j1939_bjm_srv_init - Initializes the J1939 BJM server.
 * @priv: Pointer to the server's private data structure.
 *
 * This function sets up the necessary resources for the J1939 BJM server,
 * including creating an epoll instance, allocating memory for epoll events,
 * opening the joystick devices, and initializing sockets for each enabled
 * PGN handler. A handler is enabled if its joystick device opened
 * successfully, or unconditionally in simulation mode.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int j1939_bjm_srv_init(struct j1939_bjm_srv_priv *priv)
{
	bool any_joystick = false;
	struct timespec ts;
	int ret;

	ret = libj1939_create_epoll();
	if (ret < 0) {
		pr_err("Failed to create epoll: %d\n", ret);
		return ret;
	}

	priv->cmn.epoll_fd = ret;
	priv->cmn.epoll_events = calloc(J1939_BJM_SRV_MAX_EPOLL_EVENTS, sizeof(struct epoll_event));
	if (!priv->cmn.epoll_events) {
		pr_err("Failed to allocate memory for epoll events\n");
		return -ENOMEM;
	}
	priv->cmn.epoll_events_size = J1939_BJM_SRV_MAX_EPOLL_EVENTS;

	ret = clock_gettime(CLOCK_MONOTONIC, &ts);
	if (ret < 0) {
		ret = -errno;
		pr_err("Failed to get current time: %d (%s)\n", ret, strerror(-ret));
		return ret;
	}

	for (size_t i = 0; i < ARRAY_SIZE(priv->joy); ++i) {
		struct j1939_bjm_joystick *joy = &priv->joy[i];

		if (priv->sim_mode) {
			any_joystick = true;
			continue;
		}

		ret = j1939_bjm_joy_open(priv, joy);
		if (ret < 0) {
			if (joy->explicit_path) {
				pr_err("%s: failed to open joystick: %d (%s)\n", joy->path, ret, strerror(-ret));
				return ret;
			}
			pr_info("%s: no joystick attached, BJM%zu disabled\n", joy->path, i + 1);
			continue;
		}

		any_joystick = true;
	}

	if (!any_joystick) {
		pr_err("No joystick attached and simulation mode not enabled\n");
		return -ENODEV;
	}

	priv->handlers = pgn_handlers;
	priv->num_handlers = ARRAY_SIZE(pgn_handlers);

	for (size_t i = 0; i < priv->num_handlers; ++i) {
		struct j1939_pgn_handler *handler = &priv->handlers[i];

		handler->sock = -1;
		handler->enabled = priv->sim_mode || priv->joy[handler->joy_index].state == J1939_BJM_JOY_OK;
		if (!handler->enabled)
			continue;

		ret = initialize_socket_for_handler(priv, handler);
		if (ret < 0) {
			pr_err("Failed to initialize socket for handler %zu: %d\n", i, ret);
			return ret;
		}

		handler->next_time = ts;
	}

	return 0;
}

static void j1939_bjm_srv_print_help(void)
{
	printf("j1939-bjm-srv - J1939 Basic Joystick Message Server\n");
	printf("\n");
	printf("This program acts as a J1939 Basic Joystick Message Server, broadcasting\n");
	printf("Basic Joystick Message 1 (BJM1) and Basic Joystick Message 2 (BJM2) with\n");
	printf("lever positions and button statuses read from the attached Linux joystick\n");
	printf("devices. The first joystick is broadcast as BJM1 and the second as BJM2.\n");
	printf("\n");
	printf("Each message is broadcast every %d ms, or as soon as %d ms after the\n", J1939_BJM_REPETITION_RATE_MS, J1939_BJM_EVENT_RATE_MS);
	printf("previous transmission when new joystick events arrive.\n");
	printf("\n");
	printf("Supported PGNs:\n");
	printf("  - Basic Joystick Message 1 (PGN 64982)\n");
	printf("  - Basic Joystick Message 2 (PGN 64984)\n");
	printf("\n");
	printf("Usage: j1939-bjm-srv [options]\n");
	printf("Options:\n");
	printf("  --interface <interface_name> or -i <interface_name>\n");
	printf("      Specifies the CAN interface to use (mandatory).\n");
	printf("  --local-address <local_address_hex> or -a <local_address_hex>\n");
	printf("      Specifies the local address in hexadecimal (mandatory if\n");
	printf("      local name is not provided).\n");
	printf("  --local-name <local_name_hex> or -n <local_name_hex>\n");
	printf("      Specifies the local NAME in hexadecimal (mandatory if\n");
	printf("      local address is not provided).\n");
	printf("\n");
	printf("Note: Local address and local name are mutually exclusive and one\n");
	printf("      must be provided.\n");
	printf("\n");
	printf("  --joy1 <device_path> or -1 <device_path>\n");
	printf("      Joystick device broadcast as BJM1. Default: %s\n", J1939_BJM_SRV_JOY1_DEFAULT);
	printf("  --joy2 <device_path> or -2 <device_path>\n");
	printf("      Joystick device broadcast as BJM2. Default: %s\n", J1939_BJM_SRV_JOY2_DEFAULT);
	printf("\n");
	printf("Note: A default joystick device that is not attached merely disables\n");
	printf("      the corresponding BJM message; an explicitly given device that\n");
	printf("      cannot be opened is a fatal error.\n");
	printf("\n");
	printf("  --sim-mode or -s\n");
	printf("    Enables simulation mode to generate joystick data instead of reading\n");
	printf("    attached joystick devices.\n");
	printf("\n");
	printf("Usage Examples:\n");
	printf("  Using local address:\n");
	printf("    j1939-bjm-srv -i vcan0 -a 0x90\n");
	printf("\n");
	printf("  Using local NAME:\n");
	printf("    j1939acd -r 64-95 -c /tmp/1122334455667789.jacd 1122334455667789 vcan0 &\n");
	printf("    j1939-bjm-srv -i vcan0 -n 0x1122334455667789\n");
}

/**
 * j1939_bjm_srv_parse_args - Parses command line arguments for the J1939 BJM
 *			      server.
 * @priv: Pointer to the server's private data structure.
 * @argc: The number of command line arguments.
 * @argv: The array of command line arguments.
 *
 * This function parses the command line arguments for the J1939 BJM server,
 * including the interface, local address, local name, joystick device paths
 * and simulation mode. It sets the corresponding values in the private data
 * structure.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int j1939_bjm_srv_parse_args(struct j1939_bjm_srv_priv *priv, int argc, char *argv[])
{
	struct sockaddr_can *local = &priv->sockname;
	bool local_address_set = false;
	bool local_name_set = false;
	bool interface_set = false;
	int long_index = 0;
	int opt;

	static struct option long_options[] = { { "interface", required_argument, 0, 'i' },
						{ "local-address", required_argument, 0, 'a' },
						{ "local-name", required_argument, 0, 'n' },
						{ "joy1", required_argument, 0, '1' },
						{ "joy2", required_argument, 0, '2' },
						{ "sim-mode", no_argument, 0, 's' },
						{ 0, 0, 0, 0 } };

	while ((opt = getopt_long(argc, argv, "a:n:i:1:2:s", long_options, &long_index)) != -1) {
		switch (opt) {
		case 'a':
			local->can_addr.j1939.addr = strtoul(optarg, NULL, 16);
			local_address_set = true;
			break;
		case 'n':
			local->can_addr.j1939.name = strtoull(optarg, NULL, 16);
			local_name_set = true;
			break;
		case 'i':
			local->can_ifindex = if_nametoindex(optarg);
			if (!local->can_ifindex) {
				pr_err("Interface %s not found. Error: %d (%s)\n", optarg, -errno, strerror(errno));
				return -EINVAL;
			}
			interface_set = true;
			break;
		case '1':
			priv->joy[0].path = optarg;
			priv->joy[0].explicit_path = true;
			break;
		case '2':
			priv->joy[1].path = optarg;
			priv->joy[1].explicit_path = true;
			break;
		case 's':
			priv->sim_mode = true;
			break;
		default:
			j1939_bjm_srv_print_help();
			return -EINVAL;
		}
	}

	if (!interface_set) {
		pr_err("interface not specified\n");
		j1939_bjm_srv_print_help();
		return -EINVAL;
	}

	if (local_address_set && local_name_set) {
		pr_err("local address and local name are mutually exclusive\n");
		j1939_bjm_srv_print_help();
		return -EINVAL;
	}

	return 0;
}

/**
 * j1939_bjm_close_handler_sockets - Closes the sockets for all PGN handlers.
 * @priv: Pointer to the server's private data structure.
 *
 * This function closes the sockets for all PGN handlers by iterating over
 * the handlers and closing the socket if it is open. It resets the socket
 * descriptor to an invalid value after closing.
 */
static void j1939_bjm_close_handler_sockets(struct j1939_bjm_srv_priv *priv)
{
	for (size_t i = 0; i < priv->num_handlers; ++i) {
		struct j1939_pgn_handler *handler = &priv->handlers[i];

		if (handler->sock >= 0) {
			close(handler->sock);
			handler->sock = -1;
		}
	}
}

/**
 * j1939_bjm_close_joysticks - Closes all open joystick devices.
 * @priv: Pointer to the server's private data structure.
 *
 * This function closes the file descriptors of all joystick devices that
 * are still open and resets them to an invalid value.
 */
static void j1939_bjm_close_joysticks(struct j1939_bjm_srv_priv *priv)
{
	for (size_t i = 0; i < ARRAY_SIZE(priv->joy); ++i) {
		struct j1939_bjm_joystick *joy = &priv->joy[i];

		if (joy->fd >= 0) {
			close(joy->fd);
			joy->fd = -1;
		}
	}
}

static void j1939_bjm_srv_close(struct j1939_bjm_srv_priv *priv)
{
	j1939_bjm_close_handler_sockets(priv);
	j1939_bjm_close_joysticks(priv);

	close(priv->cmn.epoll_fd);
	free(priv->cmn.epoll_events);
}

int main(int argc, char *argv[])
{
	struct j1939_bjm_srv_priv *priv;
	int ret;

	priv = malloc(sizeof(*priv));
	if (!priv)
		err(EXIT_FAILURE, "can't allocate priv");

	bzero(priv, sizeof(*priv));

	priv->joy[0].path = J1939_BJM_SRV_JOY1_DEFAULT;
	priv->joy[1].path = J1939_BJM_SRV_JOY2_DEFAULT;
	priv->joy[0].fd = -1;
	priv->joy[1].fd = -1;

	libj1939_init_sockaddr_can(&priv->sockname, J1939_PGN_REQUEST_PGN);

	ret = j1939_bjm_srv_parse_args(priv, argc, argv);
	if (ret)
		return ret;

	ret = j1939_bjm_srv_init(priv);
	if (ret) {
		pr_err("failed to initialize: %i (%s)\n", ret, strerror(-ret));
		return ret;
	}

	while (1) {
		ret = j1939_bjm_srv_process_events_and_tasks(priv);
		/* Even if error we continue to do our best. But we need to
		 * slow down to avoid busy loop. So, we sleep for a while.
		 */
		if (ret) {
			pr_warn("failed to process events and tasks: %i (%s). Sleeping for a while\n", ret, strerror(-ret));
			sleep(1);
		}
	}

	j1939_bjm_srv_close(priv);
	free(priv);

	return ret;
}
