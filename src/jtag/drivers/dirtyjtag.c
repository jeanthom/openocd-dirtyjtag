/***************************************************************************
 *   Copyright (C) 2020 Jean THOMAS                                        *
 *   pub0@git.jeanthomas.me                                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if IS_CYGWIN == 1
#include "windows.h"
#undef LOG_ERROR
#endif

/* project specific includes */
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <helper/time_support.h>
#include "libusb_helper.h"

/* system includes */
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

/**
 * USB settings
 */
static const uint8_t ep_write = 0x01;
static const uint8_t ep_read = 0x82;
#define DIRTYJTAG_USB_TIMEOUT 100
static const uint16_t dirtyjtag_vid = 0x1209;
static const uint16_t dirtyjtag_pid = 0xC0CA;

/**
 * DirtyJTAG commands
 */
#define CMD_STOP 0x00
#define CMD_INFO 0x01
#define CMD_FREQ 0x02
#define CMD_XFER 0x03
#define CMD_SETSIG 0x04
#define CMD_GETSIG 0x05
#define CMD_CLK 0x06

/**
 * DirtyJTAG signal definitions
 */
#define SIG_TCK (1 << 1)
#define SIG_TDI (1 << 2)
#define SIG_TDO (1 << 3)
#define SIG_TMS (1 << 4)
#define SIG_TRST (1 << 5)
#define SIG_SRST (1 << 6)

static struct libusb_device_handle *usb_handle;

/**
 * Utils
 */
static int min(int a, int b)
{
	return (a < b) ? a : b;
}

static unsigned char swap_bits(unsigned char x)
{
	const unsigned char lut[16] = {
		0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf
	};

	return lut[x & 0xF] << 4 | lut[x >> 4];
}

/*
 * DirtyJTAG command buffer code
 */
#define DIRTYJTAG_BUFFER_SIZE 64
static const int dirtyjtag_buffer_size = DIRTYJTAG_BUFFER_SIZE;
static uint8_t dirtyjtag_buffer[DIRTYJTAG_BUFFER_SIZE];
static int dirtyjtag_buffer_use;

static void dirtyjtag_buffer_flush(void)
{
	int sent = 0, res;

	if (dirtyjtag_buffer_use == 0) {
		return;
	}

	dirtyjtag_buffer[dirtyjtag_buffer_use] = '\0';
	
	res = jtag_libusb_bulk_write(usb_handle, ep_write, (char*)dirtyjtag_buffer,
		dirtyjtag_buffer_use+1, DIRTYJTAG_USB_TIMEOUT, &sent);
	assert(res == ERROR_OK);
	assert(sent == dirtyjtag_buffer_use+1);

	dirtyjtag_buffer_use = 0;
}

static void dirtyjtag_buffer_append(const uint8_t* command, size_t length)
{
	assert(length < dirtyjtag_buffer_size);
	assert(command != NULL);

	if (dirtyjtag_buffer_use + length + 1 > dirtyjtag_buffer_size) {
		dirtyjtag_buffer_flush();
	}

	memcpy(&dirtyjtag_buffer[dirtyjtag_buffer_use], command, length);
	dirtyjtag_buffer_use += length;
}

/**
 * Add one TCK/TMS/TDI sample to send buffer.
 */
static void dirtyjtag_write(int tck, int tms, int tdi)
{
	uint8_t command[] = {
		CMD_SETSIG,
		SIG_TCK | SIG_TMS | SIG_TDI,
		(tck ? SIG_TCK : 0) | (tms ? SIG_TMS : 0) | (tdi ? SIG_TDI : 0)
	};
	dirtyjtag_buffer_append(command, sizeof(command)/sizeof(command[0]));
}

/**
 * Read TDO pin
 */
static bool dirtyjtag_get_tdo(void)
{
	int res, read = 0;
	char state;
	uint8_t command[] = {
		CMD_GETSIG
	};

	dirtyjtag_buffer_append(command, sizeof(command)/sizeof(command[0]));
	dirtyjtag_buffer_flush();

	res = jtag_libusb_bulk_read(usb_handle, ep_read,
		&state, 1, DIRTYJTAG_USB_TIMEOUT, &read);
	assert(res == ERROR_OK);
	assert(read == 1);

	return !!(state & SIG_TDO);
}

/**
 * Send bunch of clock pulses
 */
static void dirtyjtag_clk(int num_cycles, int tms, int tdi)
{
	uint8_t command[] = {
		CMD_CLK,
		(tms ? SIG_TMS : 0) | (tdi ? SIG_TDI : 0),
		0
	};

	/*
	 * We can only do 255 clock pulses in one command, so we need
	 * to send multiple clock commands.
	 */
	while (num_cycles > 0) {
		command[2] = min(255, num_cycles);
		num_cycles -= min(255, num_cycles);
		dirtyjtag_buffer_append(command, sizeof(command)/sizeof(command[0]));
	}
}

/**
 * Control /TRST and /SYSRST pins.
 * Perform immediate bitbang transaction.
 */
static void dirtyjtag_reset(int trst, int srst)
{
	uint8_t command[] = {
		CMD_SETSIG,
		SIG_TRST | SIG_SRST,
		(trst ? SIG_TRST : 0) | (srst ? SIG_SRST : 0)
	};

	LOG_DEBUG("dirtyjtag_reset(%d,%d)", trst, srst);
	dirtyjtag_buffer_append(command, sizeof(command)/sizeof(command[0]));
	dirtyjtag_buffer_flush();
}

static int dirtyjtag_speed(int divisor)
{
	uint8_t command[] = {
		CMD_FREQ,
		divisor >> 8,
		divisor
	};

	dirtyjtag_buffer_append(command, sizeof(command)/sizeof(command[0]));

	return ERROR_OK;
}

static int dirtyjtag_init(void)
{
	uint16_t avids[] = {dirtyjtag_vid, 0};
	uint16_t apids[] = {dirtyjtag_pid, 0};
	if (jtag_libusb_open(avids, apids, NULL, &usb_handle, NULL)) {
		LOG_ERROR("dirtyjtag not found: vid=%04x, pid=%04x\n",
			dirtyjtag_vid, dirtyjtag_pid);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (libusb_claim_interface(usb_handle, 0)) {
		LOG_ERROR("unable to claim interface");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int dirtyjtag_quit(void)
{
	if (libusb_release_interface(usb_handle, 0) != 0) {
		LOG_ERROR("usb release interface failed");
	}

	jtag_libusb_close(usb_handle);

	return ERROR_OK;
}

static int dirtyjtag_speed_div(int divisor, int *khz)
{
	*khz = divisor;
	return ERROR_OK;
}

static int dirtyjtag_khz(int khz, int *divisor)
{
	if (khz == 0) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	*divisor = (khz < 65535) ? khz : 65535;
	return ERROR_OK;
}

static const struct command_registration dirtyjtag_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static void syncbb_end_state(tap_state_t state)
{
	assert(tap_is_state_stable(state));
	tap_set_end_state(state);
}

static void syncbb_state_move(int skip)
{
	int i = 0, tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = skip; i < tms_count; i++) {
		tms = (tms_scan >> i) & 1;
		dirtyjtag_write(0, tms, 0);
		dirtyjtag_write(1, tms, 0);
	}
	dirtyjtag_write(0, tms, 0);

	tap_set_state(tap_get_end_state());
}

/**
 * Clock a bunch of TMS (or SWDIO) transitions, to change the JTAG
 * (or SWD) state machine.
 */
static int syncbb_execute_tms(struct jtag_command *cmd)
{
	unsigned num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;

	LOG_DEBUG_IO("TMS: %d bits", num_bits);

	int tms = 0;
	for (unsigned i = 0; i < num_bits; i++) {
		tms = ((bits[i/8] >> (i % 8)) & 1);
		dirtyjtag_write(0, tms, 0);
		dirtyjtag_write(1, tms, 0);
	}
	dirtyjtag_write(0, tms, 0);

	return ERROR_OK;
}

static void syncbb_path_move(struct pathmove_command *cmd)
{
	int num_states = cmd->num_states;
	int state_count;
	int tms = 0;

	state_count = 0;
	while (num_states) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count]) {
			tms = 0;
		} else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count]) {
			tms = 1;
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
				tap_state_name(tap_get_state()),
				tap_state_name(cmd->path[state_count]));
			exit(-1);
		}

		dirtyjtag_write(0, tms, 0);
		dirtyjtag_write(1, tms, 0);

		tap_set_state(cmd->path[state_count]);
		state_count++;
		num_states--;
	}

	dirtyjtag_write(0, tms, 0);

	tap_set_end_state(tap_get_state());
}

static void syncbb_runtest(int num_cycles)
{
	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		syncbb_end_state(TAP_IDLE);
		syncbb_state_move(0);
	}

	dirtyjtag_clk(num_cycles, 0, 0);

	/* finish in end_state */
	syncbb_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		syncbb_state_move(0);
}

/**
 * CMD_XFER:
 *   Read TDO
 *   Set TDI
 *   Set TCK high
 *	 Set TCK low
 *
 * Bitbang:
 *   Read TDO
 *   Set TDI, TMS, TCK low
 *   Set TCK high
 *
 */
static void syncbb_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size)
{
	tap_state_t saved_end_state = tap_get_end_state();
	int sent_bits, sent_bytes, read, res;
	size_t i, buffer_pos = 0;
	uint8_t xfer_rx[32], xfer_tx[32] = {
		CMD_XFER,
		0
	};
	int pos_last_byte = (scan_size-1)/8;
	int pos_last_bit = (scan_size-1)%8;
	bool last_bit = !!(buffer[pos_last_byte] & (1 << pos_last_bit));
	const size_t xfer_bytes = (scan_size+7)/8;

	printf("\n*** syncbb_scan ****************************************\n");
	printf("TX\t");
	for (i = 0; i < xfer_bytes; i++) {
		printf("%02x", buffer[i]);
	}
	printf("\n");

	assert(scan_size > 0);
	scan_size--;

	if (!((!ir_scan && (tap_get_state() == TAP_DRSHIFT)) || (ir_scan && (tap_get_state() == TAP_IRSHIFT)))) {
		if (ir_scan) {
			syncbb_end_state(TAP_IRSHIFT);
		} else {
			syncbb_end_state(TAP_DRSHIFT);
		}
		syncbb_state_move(0);
		syncbb_end_state(saved_end_state);
	}

	if (dirtyjtag_buffer_use+32+1 > dirtyjtag_buffer_size) {
		dirtyjtag_buffer_flush();
	}

	while (scan_size > 0) {
		sent_bits = min(240, scan_size);
		sent_bytes = (sent_bits+7)/8;

		if (type != SCAN_IN) {
			memcpy(&xfer_tx[2], &buffer[buffer_pos], sent_bytes);
			for (i = 2; i < 32; i++) {
				xfer_tx[i] = swap_bits(xfer_tx[i]);
			}
		} else {
			/* Set TDO to 0 */
			memset(&xfer_tx[2], 0, 30);
		}
		xfer_tx[1] = sent_bits;

		dirtyjtag_buffer_append(xfer_tx, 32);
		dirtyjtag_buffer_flush();

		read = 0;
		res = jtag_libusb_bulk_read(usb_handle, ep_read,
			(char*)xfer_rx, 32, DIRTYJTAG_USB_TIMEOUT, &read);
		assert(res == ERROR_OK);
		assert(read == 32);
		if (type != SCAN_OUT) {
			for (i = 0; i < 32; i++) {
				xfer_rx[i] = swap_bits(xfer_rx[i]);
			}
			memcpy(&buffer[buffer_pos], xfer_rx, sent_bytes);
		}

		scan_size -= sent_bits;
		buffer_pos += sent_bytes; // equivalent to CEIL(sent_bits/8)
	}

	dirtyjtag_write(0, 1, last_bit);
	buffer[pos_last_byte] = (buffer[pos_last_byte] & ~(1 << pos_last_bit)) | (dirtyjtag_get_tdo() << pos_last_bit);
	dirtyjtag_clk(1, 1, last_bit);

	printf("RX\t");
	for (i = 0; i < xfer_bytes; i++) {
		printf("%02x", buffer[i]);
	}
	printf("\n");

	if (tap_get_state() != tap_get_end_state()) {
		/* we *KNOW* the above loop transitioned out of
		 * the shift state, so we skip the first state
		 * and move directly to the end state.
		 */
		syncbb_state_move(1);
	}
}

static int syncbb_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue; /* currently processed command */
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;
	int retval;

	/* return ERROR_OK, unless a jtag_read_buffer returns a failed check
	 * that wasn't handled by a caller-provided error handler
	 */
	retval = ERROR_OK;

	while (cmd) {
		switch (cmd->type) {
			case JTAG_RESET:
				LOG_DEBUG_IO("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);

				if ((cmd->cmd.reset->trst == 1) ||
					(cmd->cmd.reset->srst &&
					(jtag_get_reset_config() & RESET_SRST_PULLS_TRST))) {
					tap_set_state(TAP_RESET);
				}
				dirtyjtag_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;

			case JTAG_RUNTEST:
				LOG_DEBUG_IO("runtest %i cycles, end in %s", cmd->cmd.runtest->num_cycles,
					tap_state_name(cmd->cmd.runtest->end_state));

				syncbb_end_state(cmd->cmd.runtest->end_state);
				syncbb_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_STABLECLOCKS:
				dirtyjtag_clk(cmd->cmd.stableclocks->num_cycles,
					(tap_get_state() == TAP_RESET ? SIG_TMS : 0), 0);
				break;

			case JTAG_TLR_RESET: /* renamed from JTAG_STATEMOVE */
				LOG_DEBUG_IO("statemove end in %s", tap_state_name(cmd->cmd.statemove->end_state));

				syncbb_end_state(cmd->cmd.statemove->end_state);
				syncbb_state_move(0);
				break;

			case JTAG_PATHMOVE:
				LOG_DEBUG_IO("pathmove: %i states, end in %s", cmd->cmd.pathmove->num_states,
					tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));

				syncbb_path_move(cmd->cmd.pathmove);
				break;

			case JTAG_SCAN:
				LOG_DEBUG_IO("%s scan end in %s",  (cmd->cmd.scan->ir_scan) ? "IR" : "DR",
					tap_state_name(cmd->cmd.scan->end_state));

				syncbb_end_state(cmd->cmd.scan->end_state);
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				type = jtag_scan_type(cmd->cmd.scan);
				syncbb_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
				if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				if (buffer)
					free(buffer);
				break;

			case JTAG_SLEEP:
				jtag_sleep(cmd->cmd.sleep->us);
				break;

			case JTAG_TMS:
				retval = syncbb_execute_tms(cmd);
				break;
			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}

	dirtyjtag_buffer_flush();

	return retval;
}

static struct jtag_interface dirtyjtag_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = syncbb_execute_queue,
};

struct adapter_driver dirtyjtag_adapter_driver = {
	.name = "dirtyjtag",
	.transports = jtag_only,
	.commands = dirtyjtag_command_handlers,

	.init = dirtyjtag_init,
	.quit = dirtyjtag_quit,
	.speed = dirtyjtag_speed,
	.khz = dirtyjtag_khz,
	.speed_div = dirtyjtag_speed_div,

	.jtag_ops = &dirtyjtag_interface,
};
