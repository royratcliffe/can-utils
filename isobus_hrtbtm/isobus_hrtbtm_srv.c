// SPDX-License-Identifier: LGPL-2.0-only
// SPDX-FileCopyrightText: 2026 Roy Ratcliffe

/*
 * isobus-hrtbtm-srv - cyclically transmit the ISO 11783-7 Heartbeat
 * message (HRTBTM, PGN 61668) with an incrementing sequence counter.
 */

#include <err.h>
#include <errno.h>
#include <getopt.h>
#include <linux/kernel.h>
#include <net/if.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "isobus_hrtbtm_cmn.h"

struct isobus_hrtbtm_srv_priv {
	int sock;
	struct sockaddr_can sockname;
	uint8_t seq;
	int repetition_rate_ms;
	unsigned long count;
	bool verbose;
};

/**
 * isobus_hrtbtm_srv_send_one - Sends a single heartbeat message.
 * @priv: Pointer to the server's private data structure.
 *
 * This function populates a heartbeat packet with the current sequence
 * counter, transmits it as a broadcast and advances the counter, wrapping
 * from 250 back to 0 as required by ISO 11783-7.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int isobus_hrtbtm_srv_send_one(struct isobus_hrtbtm_srv_priv *priv)
{
	struct isobus_hrtbtm_packet hbp;
	int ret;

	isobus_hrtbtm_set_sequence_counter(&hbp, priv->seq);

	ret = send(priv->sock, &hbp, sizeof(hbp), 0);
	if (ret == -1) {
		ret = -errno;
		pr_warn("failed to send heartbeat: %i (%s)\n", ret, strerror(-ret));
		return ret;
	}

	if (priv->verbose)
		pr_info("sent heartbeat, sequence: %u\n", priv->seq);

	priv->seq = (priv->seq + 1) % ISOBUS_HRTBTM_SEQ_MODULO;

	return 0;
}

/**
 * isobus_hrtbtm_srv_run - Runs the cyclic heartbeat transmission loop.
 * @priv: Pointer to the server's private data structure.
 *
 * This function transmits heartbeat messages at the configured repetition
 * rate using an absolute monotonic deadline so the cadence does not drift.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int isobus_hrtbtm_srv_run(struct isobus_hrtbtm_srv_priv *priv)
{
	struct timespec next_time;
	unsigned long sent = 0;
	int ret;

	ret = clock_gettime(CLOCK_MONOTONIC, &next_time);
	if (ret < 0) {
		ret = -errno;
		pr_err("failed to get time: %i (%s)\n", ret, strerror(-ret));
		return ret;
	}

	while (1) {
		ret = isobus_hrtbtm_srv_send_one(priv);
		if (ret < 0)
			return ret;

		sent++;
		if (priv->count && sent >= priv->count)
			return 0;

		timespec_add_ms(&next_time, priv->repetition_rate_ms);

		do {
			ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
		} while (ret == EINTR);
		if (ret) {
			pr_err("failed to sleep: %i (%s)\n", -ret, strerror(ret));
			return -ret;
		}
	}
}

/**
 * isobus_hrtbtm_srv_sock_prepare - Prepares the heartbeat socket.
 * @priv: Pointer to the server's private data structure.
 *
 * This function opens a J1939 socket, binds it to the local address,
 * sets the default heartbeat priority and connects it to the broadcast
 * address with the heartbeat PGN.
 *
 * Return: 0 on success or a negative error code if an issue occurs.
 */
static int isobus_hrtbtm_srv_sock_prepare(struct isobus_hrtbtm_srv_priv *priv)
{
	struct sockaddr_can addr = priv->sockname;
	int ret;

	ret = libj1939_open_socket();
	if (ret < 0)
		return ret;

	priv->sock = ret;

	ret = libj1939_bind_socket(priv->sock, &addr);
	if (ret < 0)
		return ret;

	ret = libj1939_socket_prio(priv->sock, ISOBUS_HRTBTM_PRIO_DEFAULT);
	if (ret < 0)
		return ret;

	ret = libj1939_set_broadcast(priv->sock);
	if (ret < 0)
		return ret;

	addr.can_addr.j1939.name = J1939_NO_NAME;
	addr.can_addr.j1939.addr = J1939_NO_ADDR;
	addr.can_addr.j1939.pgn = ISOBUS_PGN_HRTBTM;

	return libj1939_connect_socket(priv->sock, &addr);
}

static void isobus_hrtbtm_srv_print_help(void)
{
	printf("Usage: isobus-hrtbtm-srv [options]\n");
	printf("Options:\n");
	printf("  --interface <interface_name> or -i <interface_name>\n");
	printf("      Specifies the CAN interface to use (mandatory).\n");
	printf("  --local-address <local_address_hex> or -a <local_address_hex>\n");
	printf("      Specifies the local address in hexadecimal (mandatory if\n");
	printf("      local name is not provided).\n");
	printf("  --local-name <local_name_hex> or -n <local_name_hex>\n");
	printf("      Specifies the local NAME in hexadecimal (mandatory if\n");
	printf("      local address is not provided).\n");
	printf("  --rate <rate_ms> or -r <rate_ms>\n");
	printf("      Specifies the repetition rate in milliseconds\n");
	printf("      (default: %u ms as required by ISO 11783-7).\n", ISOBUS_HRTBTM_REPETITION_RATE_MS);
	printf("  --count <count> or -c <count>\n");
	printf("      Number of heartbeat messages to send before exiting\n");
	printf("      (default: 0, send forever).\n");
	printf("  --verbose or -v\n");
	printf("      Print each transmitted heartbeat.\n");
	printf("\n");
	printf("Note: Local address and local name are mutually exclusive and one\n");
	printf("      must be provided.\n");
	printf("\n");
	printf("Usage Examples:\n");
	printf("  Using local address:\n");
	printf("    isobus-hrtbtm-srv -i vcan0 -a 0x90\n");
	printf("\n");
	printf("  Using local NAME:\n");
	printf("    j1939acd -r 64-95 -c /tmp/1122334455667789.jacd 1122334455667789 vcan0 &\n");
	printf("    isobus-hrtbtm-srv -i vcan0 -n 0x1122334455667789\n");
}

static int isobus_hrtbtm_srv_parse_args(struct isobus_hrtbtm_srv_priv *priv, int argc, char *argv[])
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
						{ "rate", required_argument, 0, 'r' },
						{ "count", required_argument, 0, 'c' },
						{ "verbose", no_argument, 0, 'v' },
						{ 0, 0, 0, 0 } };

	while ((opt = getopt_long(argc, argv, "a:n:i:r:c:v", long_options, &long_index)) != -1) {
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
				pr_err("Interface %s not found. Error: %d (%s)\n", optarg, errno, strerror(errno));
				return -EINVAL;
			}
			interface_set = true;
			break;
		case 'r':
			priv->repetition_rate_ms = atoi(optarg);
			if (priv->repetition_rate_ms <= 0) {
				pr_err("invalid repetition rate: %s\n", optarg);
				return -EINVAL;
			}
			break;
		case 'c':
			priv->count = strtoul(optarg, NULL, 0);
			break;
		case 'v':
			priv->verbose = true;
			break;
		default:
			isobus_hrtbtm_srv_print_help();
			return -EINVAL;
		}
	}

	if (!interface_set) {
		pr_err("interface not specified\n");
		isobus_hrtbtm_srv_print_help();
		return -EINVAL;
	}

	if (local_address_set && local_name_set) {
		pr_err("local address and local name are mutually exclusive\n");
		isobus_hrtbtm_srv_print_help();
		return -EINVAL;
	}

	if (!local_address_set && !local_name_set) {
		pr_err("local address or local name must be provided\n");
		isobus_hrtbtm_srv_print_help();
		return -EINVAL;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct isobus_hrtbtm_srv_priv *priv;
	int ret;

	priv = malloc(sizeof(*priv));
	if (!priv)
		err(EXIT_FAILURE, "can't allocate priv");

	bzero(priv, sizeof(*priv));
	priv->repetition_rate_ms = ISOBUS_HRTBTM_REPETITION_RATE_MS;

	libj1939_init_sockaddr_can(&priv->sockname, ISOBUS_PGN_HRTBTM);

	ret = isobus_hrtbtm_srv_parse_args(priv, argc, argv);
	if (ret)
		return EXIT_FAILURE;

	ret = isobus_hrtbtm_srv_sock_prepare(priv);
	if (ret)
		return EXIT_FAILURE;

	ret = isobus_hrtbtm_srv_run(priv);

	close(priv->sock);
	free(priv);

	return ret ? EXIT_FAILURE : EXIT_SUCCESS;
}
