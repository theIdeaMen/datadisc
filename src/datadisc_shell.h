/*
 * Copyright (c) 2022 Griffin Adams
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DATADISC_SHELL_H_
#define DATADISC_SHELL_H_

/** Calculate the estimated battery level based on a measured voltage.
 *
 * @param batt_mV a measured battery voltage level.
 *
 * @param curve the discharge curve for the type of battery installed
 * on the system.
 *
 * @return the estimated remaining capacity in parts per ten
 * thousand.
 * TODO: change comment header above and add one for all functions
 */
static int cmd_log_test_start(const struct shell *shell, size_t argc,
			      char **argv, uint32_t period);

static int cmd_log_test_start_demo(const struct shell *shell, size_t argc,
				   char **argv);

static int cmd_log_test_start_flood(const struct shell *shell, size_t argc,
				    char **argv);

static int cmd_log_test_stop(const struct shell *shell, size_t argc,
			     char **argv);
static int cmd_demo_ping(const struct shell *shell, size_t argc, char **argv);

#if defined CONFIG_SHELL_GETOPT
static int cmd_demo_getopt(const struct shell *shell, size_t argc, char **argv);
#endif

static int cmd_demo_params(const struct shell *shell, size_t argc, char **argv);

static int cmd_demo_hexdump(const struct shell *shell, size_t argc, char **argv);

static int cmd_version(const struct shell *shell, size_t argc, char **argv);
static int cmd_bypass(const struct shell *sh, size_t argc, char **argv);
static int cmd_dict(const struct shell *shell, size_t argc, char **argv,
		    void *data);

#endif /* DATADISC_SHELL_H_ */
