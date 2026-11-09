// SPDX-License-Identifier: LGPL-2.0-only
// SPDX-FileCopyrightText: 2026 Roy Ratcliffe

/*
 * isobus-hrtbtm-mon - monitor ISO 11783-7 Heartbeat messages (HRTBTM,
 * PGN 61668) on the bus, reporting sequence counter gaps and heartbeat
 * timeouts per source address.
 */

#include <err.h>
#include <errno.h>
#include <getopt.h>
#include <linux/kernel.h>
#include <net/if.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "isobus_hrtbtm_cmn.h"

#define ISOBUS_HRTBTM_MON_MAX_SRC	256

struct isobus_hrtbtm_mon_src {
	bool active;
	uint8_t last_seq;
	struct timespec last_time;
	unsigned long count;
};

struct isobus_hrtbtm_mon_priv {
	int sock;
	struct sockaddr_can sockname;
	struct isobus_hrtbtm_mon_src srcs[ISOBUS_HRTBTM_MON_MAX_SRC];
	int timeout_ms;
	bool quiet;
};

/**
 * isobus_hrtbtm_mon_process_one - Processes a single received heartbeat.
 * @priv: Pointer to the monitor's private data structure.
 * @peername: Address of the transmitting CF.
 * @hbp: Pointer to the received heartbeat packet.
 * @now: Monotonic reception time.
 *
 * This function tracks the heartbeat of the transmitting source address,
 * printing each message with its interval and warning when the sequence
 * counter skips, repeats or resumes after a timeout.
 */
static void isobus_hrtbtm_mon_process_one(struct isobus_hrtbtm_mon_priv *priv,
					  const struct sockaddr_can *peername,
					  const struct isobus_hrtbtm_packet *hbp,
					  struct timespec *now)
{
	uint8_t sa = peername->can_addr.j1939.addr;
	struct isobus_hrtbtm_mon_src *src = &priv->srcs[sa];
	uint8_t seq = isobus_hrtbtm_get_sequence_counter(hbp);

	if (src->active) {
		uint8_t expected;
		int64_t interval;

		interval = timespec_diff_ms(now, &src->last_time);

		if (!priv->quiet)
			pr_info("SA 0x%02X: heartbeat seq %3u, interval %4lli ms\n",
				sa, seq, (long long)interval);

		if (seq > ISOBUS_HRTBTM_SEQ_MAX) {
			pr_warn("SA 0x%02X: reserved sequence value %u\n",
				sa, seq);
		} else {
			expected = (src->last_seq + 1) %
				ISOBUS_HRTBTM_SEQ_MODULO;
			if (seq != expected) {
				unsigned int missed;

				missed = (seq + ISOBUS_HRTBTM_SEQ_MODULO -
					  src->last_seq - 1) %
					ISOBUS_HRTBTM_SEQ_MODULO;
				pr_warn("SA 0x%02X: sequence gap, expected %u, got %u (%u missed)\n",
					sa, expected, seq, missed);
			}
		}

		if (interval > priv->timeout_ms)
			pr_warn("SA 0x%02X: heartbeat interval %lli ms exceeds timeout of %i ms\n",
				sa, (long long)interval, priv->timeout_ms);
	} else {
		pr_info("SA 0x%02X: heartbeat detected, seq %u\n", sa, seq);
	}

	src->active = true;
	src->last_seq = seq;
	src->last_time = *now;
	src->count++;
}

/**
 * isobus_hrtbtm_mon_check_timeouts - Checks all tracked sources for loss.
 * @priv: Pointer to the monitor's private data structure.
 * @now: Current monotonic time.
 *
 * This function warns once for each tracked source address whose heartbeat
 * has not been seen within the configured timeout and stops tracking it
 * until it reappears.
 */
static void isobus_hrtbtm_mon_check_timeouts(struct isobus_hrtbtm_mon_priv *priv,
					     struct timespec *now)
{
	for (size_t sa = 0; sa < ARRAY_SIZE(priv->srcs); sa++) {
		struct isobus_hrtbtm_mon_src *src = &priv->srcs[sa];
		int64_t silence;

		if (!src->active)
			continue;

		silence = timespec_diff_ms(now, &src->last_time);
		if (silence > priv->timeout_ms) {
			pr_warn("SA 0x%02zX: heartbeat lost, no message for %lli ms (last seq %u)\n",
				sa, (long long)silence, src->last_seq);
			src->active = false;
		}
	}
}

/**
 * isobus_hrtbtm_mon_rx_one - Receives and processes one heartbeat message.
 * @priv: Pointer to the monitor's private data structure.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int isobus_hrtbtm_mon_rx_one(struct isobus_hrtbtm_mon_priv *priv)
{
	struct isobus_hrtbtm_packet hbp;
	struct sockaddr_can peername;
	socklen_t peer_addr_len = sizeof(peername);
	struct timespec now;
	int ret;

	ret = recvfrom(priv->sock, &hbp, sizeof(hbp), 0,
		       (struct sockaddr *)&peername, &peer_addr_len);
	if (ret < 0) {
		ret = -errno;
		pr_warn("recvfrom() failed: %i (%s)\n", ret, strerror(-ret));
		return ret;
	}

	if (ret < (int)sizeof(hbp)) {
		pr_warn("received too short heartbeat message: %i\n", ret);
		return 0;
	}

	ret = clock_gettime(CLOCK_MONOTONIC, &now);
	if (ret < 0) {
		ret = -errno;
		pr_err("failed to get time: %i (%s)\n", ret, strerror(-ret));
		return ret;
	}

	isobus_hrtbtm_mon_process_one(priv, &peername, &hbp, &now);

	return 0;
}

/**
 * isobus_hrtbtm_mon_run - Runs the heartbeat monitoring loop.
 * @priv: Pointer to the monitor's private data structure.
 *
 * This function polls the heartbeat socket, processing received messages
 * and periodically checking tracked source addresses for timeouts.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int isobus_hrtbtm_mon_run(struct isobus_hrtbtm_mon_priv *priv)
{
	struct pollfd pfd = {
		.fd = priv->sock,
		.events = POLLIN,
	};
	struct timespec now;
	int ret;

	while (1) {
		ret = poll(&pfd, 1, priv->timeout_ms);
		if (ret < 0) {
			ret = -errno;
			if (ret == -EINTR)
				continue;
			pr_err("poll() failed: %i (%s)\n", ret,
			       strerror(-ret));
			return ret;
		}

		if (ret > 0 && (pfd.revents & POLLIN)) {
			ret = isobus_hrtbtm_mon_rx_one(priv);
			if (ret < 0)
				return ret;
		}

		ret = clock_gettime(CLOCK_MONOTONIC, &now);
		if (ret < 0) {
			ret = -errno;
			pr_err("failed to get time: %i (%s)\n", ret,
			       strerror(-ret));
			return ret;
		}

		isobus_hrtbtm_mon_check_timeouts(priv, &now);
	}
}

/**
 * isobus_hrtbtm_mon_sock_prepare - Prepares the monitoring socket.
 * @priv: Pointer to the monitor's private data structure.
 *
 * This function opens a J1939 socket in promiscuous mode, filtered on the
 * heartbeat PGN, and binds it to the interface so heartbeats from all
 * source addresses are received.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int isobus_hrtbtm_mon_sock_prepare(struct isobus_hrtbtm_mon_priv *priv)
{
	struct sockaddr_can addr = priv->sockname;
	struct j1939_filter filt = {
		.pgn = ISOBUS_PGN_HRTBTM,
		.pgn_mask = ~0,
	};
	int ival_1 = 1;
	int ret;

	ret = libj1939_open_socket();
	if (ret < 0)
		return ret;

	priv->sock = ret;

	ret = setsockopt(priv->sock, SOL_CAN_J1939, SO_J1939_FILTER, &filt,
			 sizeof(filt));
	if (ret < 0) {
		ret = -errno;
		pr_err("failed to set filter: %i (%s)\n", ret, strerror(-ret));
		return ret;
	}

	ret = setsockopt(priv->sock, SOL_CAN_J1939, SO_J1939_PROMISC, &ival_1,
			 sizeof(ival_1));
	if (ret < 0) {
		ret = -errno;
		pr_err("failed to set promiscuous mode: %i (%s)\n", ret,
		       strerror(-ret));
		return ret;
	}

	return libj1939_bind_socket(priv->sock, &addr);
}

static void isobus_hrtbtm_mon_print_help(void)
{
	printf("Usage: isobus-hrtbtm-mon [options]\n");
	printf("Options:\n");
	printf("  --interface <interface_name> or -i <interface_name>\n");
	printf("      Specifies the CAN interface to use (mandatory).\n");
	printf("  --timeout <timeout_ms> or -t <timeout_ms>\n");
	printf("      Heartbeat timeout in milliseconds before a source is\n");
	printf("      declared lost (default: %u ms).\n",
	       ISOBUS_HRTBTM_TIMEOUT_MS);
	printf("  --quiet or -q\n");
	printf("      Only print warnings (sequence gaps, timeouts), not every\n");
	printf("      received heartbeat.\n");
	printf("\n");
	printf("Usage Example:\n");
	printf("    isobus-hrtbtm-mon -i vcan0\n");
}

static int isobus_hrtbtm_mon_parse_args(struct isobus_hrtbtm_mon_priv *priv,
					int argc, char *argv[])
{
	struct sockaddr_can *local = &priv->sockname;
	bool interface_set = false;
	int long_index = 0;
	int opt;

	static struct option long_options[] = {
		{"interface", required_argument, 0, 'i'},
		{"timeout", required_argument, 0, 't'},
		{"quiet", no_argument, 0, 'q'},
		{0, 0, 0, 0}
	};

	while ((opt = getopt_long(argc, argv, "i:t:q", long_options,
				  &long_index)) != -1) {
		switch (opt) {
		case 'i':
			local->can_ifindex = if_nametoindex(optarg);
			if (!local->can_ifindex) {
				pr_err("Interface %s not found. Error: %d (%s)\n",
				       optarg, errno, strerror(errno));
				return -EINVAL;
			}
			interface_set = true;
			break;
		case 't':
			priv->timeout_ms = atoi(optarg);
			if (priv->timeout_ms <= 0) {
				pr_err("invalid timeout: %s\n", optarg);
				return -EINVAL;
			}
			break;
		case 'q':
			priv->quiet = true;
			break;
		default:
			isobus_hrtbtm_mon_print_help();
			return -EINVAL;
		}
	}

	if (!interface_set) {
		pr_err("interface not specified\n");
		isobus_hrtbtm_mon_print_help();
		return -EINVAL;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct isobus_hrtbtm_mon_priv *priv;
	int ret;

	priv = malloc(sizeof(*priv));
	if (!priv)
		err(EXIT_FAILURE, "can't allocate priv");

	bzero(priv, sizeof(*priv));
	priv->timeout_ms = ISOBUS_HRTBTM_TIMEOUT_MS;

	/* Line-buffer stdout so piped/redirected output is timely */
	setvbuf(stdout, NULL, _IOLBF, 0);

	libj1939_init_sockaddr_can(&priv->sockname, J1939_NO_PGN);

	ret = isobus_hrtbtm_mon_parse_args(priv, argc, argv);
	if (ret)
		return EXIT_FAILURE;

	ret = isobus_hrtbtm_mon_sock_prepare(priv);
	if (ret)
		return EXIT_FAILURE;

	ret = isobus_hrtbtm_mon_run(priv);

	close(priv->sock);
	free(priv);

	return ret ? EXIT_FAILURE : EXIT_SUCCESS;
}
