#include "smol_servo.h"
#include "smol_fast.h"

#include "fusb302.h"
#include "pico_debug.h"

#include <string.h>

/**
 * USB PD sink, USB PD Revision 3.2 Version 1.0 (SPR and EPR).
 *
 * The TMC6200 needs at least 8V to work with, so USP PD of 9V.
 *
 * We have the FUSB302 connected through a TPD4S480, which means it is getting a reduced voltage by a factor of 0.42 to account for ESR 48V supplies. This means that likely VBUSOK will never trigger, but that's ok because we can measure it.
 *
 * The default negotiation is requesting at least 9V at the highest power envelope.
 *
 * If we already have a VBUS we will not request higher voltage over USB.
 *
 * EPR: after an SPR Explicit Contract, when the Source sets the EPR Mode Capable bit in its 5V PDO, the sink sends EPR_Mode (Enter). In EPR Mode the Source sends EPR_Source_Capabilities, and the sink replies with EPR_Request. The sink sends EPR_KeepAlive when there is no other traffic.
 *
 * Extended Messages are always Chunked: the RDO does not set "Unchunked Extended Messages Supported".
 */

#define VBUS_LV_RATIO (0.42f)
#define PD_VBUS_MIN_THRESHOLD 4.5f

// USB PD message header Spec Revision field [7:6]: 00=Rev1.0, 01=Rev2.0, 10=Rev3.x
#define PD_SPEC_REV_20  1
#define PD_SPEC_REV_30  2

// Revision Message Data Object: Revision 3.2, Version 1.0
#define PD_RMDO ((3u << 28) | (2u << 24) | (1u << 20) | (0u << 16))

// set to 0 to stay in SPR Mode
#ifndef SMOL_PD_EPR_ENABLE
#define SMOL_PD_EPR_ENABLE 1
#endif

// highest VBUS voltage the board accepts. The VBUS capacitors and MOSFETs are 60V parts.
#define PD_MAX_VOLTAGE_MV (48000)

// Sink Power Delivery Power (PDP) values, in W. They go into Sink_Capabilities_Extended and EPR_Mode (Enter).
#define PD_SINK_MIN_PDP_W (10)
#define PD_SINK_OPERATIONAL_PDP_W (100)
#define PD_SINK_MAX_PDP_W (100)
#define PD_EPR_SINK_MIN_PDP_W (15)
#define PD_EPR_SINK_OPERATIONAL_PDP_W (140)
#define PD_EPR_SINK_MAX_PDP_W (240)

// Sink_Capabilities_Extended: VID/PID must be the same as the USB device descriptor (pico-sdk stdio USB)
#define PD_SKEDB_VID (0x2E8A)
#define PD_SKEDB_PID (0x0009)
#define PD_SKEDB_XID (0)
#define PD_SKEDB_FW_VERSION (0)
#define PD_SKEDB_HW_VERSION (0)

// tSenderResponse (max)
#define PD_SENDER_RESPONSE_US (30000)
// tTypeCSinkWaitCap (max)
#define PD_SINK_WAIT_CAP_US (620000)
// after a Hard Reset: tSafe0V + tSrcRecover + tSrcTurnOn + tTypeCSinkWaitCap (max)
#define PD_HARD_RESET_WAIT_CAP_US (2600000)
// nHardResetCount
#define PD_HARD_RESET_COUNT (2)
// tHardReset (max) is 5ms
#define PD_HARD_RESET_SEND_US (10000)
// tPSTransition (max), SPR and EPR
#define PD_PS_TRANSITION_US (550000)
#define PD_PS_TRANSITION_EPR_US (1020000)
// tSinkRequest: wait time before a new Request after a Wait message
#define PD_SINK_REQUEST_US (100000)
// tEnterEPR (max)
#define PD_ENTER_EPR_US (550000)
// tSinkEPRKeepAlive is 250..500ms
#define PD_EPR_KEEPALIVE_US (300000)
// wait after the Explicit Contract before EPR Mode entry, so that the Source can finish its own AMS (cable discovery, Get_Sink_Cap, ...)
#define PD_EPR_ENTRY_DELAY_US (100000)
// EPR Mode entry attempts after a failure that the Source did not report (timeout, interruption)
#define PD_EPR_ENTRY_ATTEMPTS (3)
// ChunkSenderRequestTimer: tChunkSenderRequest max 30ms
#define PD_CHUNK_SENDER_REQUEST_US (30000)
// SinkPPSPeriodicTimer: re-send the Request for a PPS contract before the source's PPSTimeout (tPPSTimeout min 12s, tPPSRequest max 10s)
#define PD_PPS_REQUEST_US (5000000)
// detach: Rp gone from the active CC pin for longer than tPDDebounce (max)
#define PD_DETACH_DEBOUNCE_US (20000)
#define PD_DETACH_POLL_US (2000)
// attach: Rp stable on one CC pin for tCCDebounce (min)
#define PD_ATTACH_DEBOUNCE_US (100000)
#define PD_ATTACH_POLL_US (20000)

// MaxExtendedMsgChunkLen
#define PD_CHUNK_LEN (26)

#define TXON_TOKEN 0xA1

// frequent FUSB302 interrupts: printing them delays the PD response too much
#ifdef SMOL_PD_DEBUG_IRQ
#define PD_IRQ_DBG(...) dbg_println(__VA_ARGS__)
#else
#define PD_IRQ_DBG(...) do {} while (0)
#endif

#define PD_TOKEN_SOP  0x7
#define PD_TOKEN_SOP1 0x6
#define PD_TOKEN_SOP2 0x5
#define PD_TOKEN_SOP1BD 0x4
#define PD_TOKEN_SOP2BD 0x3


typedef enum {
	SMOL_USB_PD_IDLE,
	// waiting for Source_Capabilities or EPR_Source_Capabilities
	SMOL_USB_PD_WAIT_CAPS,
	SMOL_USB_PD_REQUEST_SENT,
	SMOL_USB_PD_REQUEST_WAIT,
	// Accept received, waiting for PS_RDY
	SMOL_USB_PD_REQUEST_ACCEPTED,
	// Explicit Contract
	SMOL_USB_PD_READY,
	SMOL_USB_PD_SOFT_RESET_SENT,
	// EPR_Mode (Enter) sent, waiting for Enter Acknowledged
	SMOL_USB_PD_EPR_ENTER_SENT,
	// Enter Acknowledged received, waiting for Enter Succeeded
	SMOL_USB_PD_EPR_ENTER_WAIT,
	// EPR_KeepAlive sent, waiting for EPR_KeepAlive_Ack
	SMOL_USB_PD_EPR_KEEPALIVE_SENT,
} smol_usb_pd_state_t;

typedef enum {
	PD_CMD_RESERVED,
	PD_CMD_GOOD_CRC,
	PD_CMD_GOTO_MIN,
	PD_CMD_ACCEPT,
	PD_CMD_REJECT,
	PD_CMD_PING,
	PD_CMD_PS_READY,
	PD_CMD_GET_SOURCE_CAP,
	PD_CMD_GET_SINK_CAP,
	PD_CMD_DR_SWAP,
	PD_CMD_PR_SWAP,
	PD_CMD_VCONN_SWAP,
	PD_CMD_WAIT,
	PD_CMD_SOFT_RESET,
	PD_CMD_DATA_RESET,
	PD_CMD_DATA_RESET_COMPLETE,
	PD_CMD_NOT_SUPPORTED,
	PD_CMD_GET_SOURCE_CAP_EXTENDED,
	PD_CMD_GET_STATUS,
	PD_CMD_FR_SWAP,
	PD_CMD_GET_PPS_STATUS,
	PD_CMD_GET_COUNTRY_CODES,
	PD_CMD_GET_SINK_CAP_EXTENDED,
	PD_CMD_GET_SOURCE_INFO,
	PD_CMD_GET_REVISION,
} smol_pd_command_t;

typedef enum {
	PD_DATA_RESERVED,
	PD_DATA_SOURCE_CAPS,
	PD_DATA_REQUEST,
	PD_DATA_BIST,
	PD_DATA_SINK_CAPS,
	PD_DATA_BATTERY_STATUS,
	PD_DATA_ALERT,
	PD_DATA_GET_COUNTRY_INFO,
	PD_DATA_ENTER_USB,
	PD_DATA_EPR_REQUEST,
	PD_DATA_EPR_MODE,
	PD_DATA_SOURCE_INFO,
	PD_DATA_REVISION,
	PD_DATA_VENDOR_DEFINED = 15,
} smol_pd_data_t;

typedef enum {
	PD_EXT_RESERVED,
	PD_EXT_SOURCE_CAP_EXTENDED,
	PD_EXT_STATUS,
	PD_EXT_GET_BATTERY_CAP,
	PD_EXT_GET_BATTERY_STATUS,
	PD_EXT_BATTERY_CAPABILITIES,
	PD_EXT_GET_MFG_INFO,
	PD_EXT_MFG_INFO,
	PD_EXT_SECURITY_REQUEST,
	PD_EXT_SECURITY_RESPONSE,
	PD_EXT_FWU_REQUEST,
	PD_EXT_FWU_RESPONSE,
	PD_EXT_PPS_STATUS,
	PD_EXT_COUNTRY_INFO,
	PD_EXT_COUNTRY_CODES,
	PD_EXT_SINK_CAP_EXTENDED,
	PD_EXT_EXTENDED_CONTROL,
	PD_EXT_EPR_SOURCE_CAP,
	PD_EXT_EPR_SINK_CAP
} smol_pd_ext_t;

// Extended_Control Message types (ECDB Type)
typedef enum {
	PD_ECDB_EPR_GET_SOURCE_CAP = 1,
	PD_ECDB_EPR_GET_SINK_CAP = 2,
	PD_ECDB_EPR_KEEPALIVE = 3,
	PD_ECDB_EPR_KEEPALIVE_ACK = 4,
} smol_pd_ecdb_t;

// EPR_Mode Data Object Action field
typedef enum {
	PD_EPR_MODE_ENTER = 1,
	PD_EPR_MODE_ENTER_ACK = 2,
	PD_EPR_MODE_ENTER_SUCCEEDED = 3,
	PD_EPR_MODE_ENTER_FAILED = 4,
	PD_EPR_MODE_EXIT = 5,
} smol_pd_epr_action_t;

typedef enum {
	PD_PREFERENCE_MAX_POWER,
	PD_PREFERENCE_MAX_CURRENT,
	PD_PREFERENCE_MAX_VOLTAGE,
} pd_preference_t;

typedef enum {
	PDO_FIXED,
	PDO_BATTERY,
	PDO_VARIABLE,
	PDO_APDO,
} pdo_type_t;

typedef enum {
	APDO_SPR_PROGRAMMABLE,
	APDO_EPR_ADJUSTABLE,
	APDO_SPR_ADJUSTABLE,
} apdo_type_t;

typedef struct {
	uint32_t min_voltage;
	uint32_t max_voltage;
	uint32_t max_current;
	uint32_t max_power;
	bool is_battery;
	bool is_variable;
	bool is_apdo;
	bool is_pps;
	bool is_spr_avs;
	bool is_epr_avs;
} src_cap_t;

#define MAX_NUM_SPR_CAPS (7)
#define MAX_NUM_SRC_CAPS (7+4) // SPR + EPR

static smol_usb_pd_state_t m_usb_pd_state = SMOL_USB_PD_IDLE;
static uint64_t _state_since_us = 0;
static size_t _cc_index = 0;
static uint32_t _msg_counter = 0;
// Spec Revision for TX: the lower of our revision and the partner's revision
static uint32_t _spec_rev = PD_SPEC_REV_30;
// MessageID of the last received SOP message, -1 = none. Used to discard retransmitted messages.
static int _last_rx_msgid = -1;
static uint64_t _wait_until_us = 0;
// SinkWaitCapTimer value for the current WAIT_CAPS state. 0 = no timeout.
static uint32_t _wait_caps_timeout_us = 0;
static uint32_t _hard_reset_counter = 0;

// contract and EPR state
static bool _have_contract = false;
static bool _contract_is_epr = false;
static uint32_t _requested_pos = 0;
static bool _requested_is_pps = false;
static bool _contract_is_pps = false;
// time of the last Request / EPR_Request, for SinkPPSPeriodicTimer
static uint64_t _last_request_us = 0;
static bool _src_epr_capable = false;
static bool _epr_mode = false;
// EPR Mode entry attempts since attach. PD_EPR_ENTRY_ATTEMPTS = no more attempts.
static uint32_t _epr_entry_attempts = 0;
static uint64_t _epr_enter_sent_us = 0;
// last SOP message sent or received, for EPR_KeepAlive
static uint64_t _last_traffic_us = 0;

// detach / attach detection
static uint64_t _cc_poll_us = 0;
static uint64_t _cc_low_since_us = 0;
static size_t _attach_candidate = 0;
static uint64_t _attach_candidate_since_us = 0;
// BC_LVL of the active CC pin, from the last detach poll
static uint8_t _bc_lvl = 0;

static src_cap_t _src_capabilities[MAX_NUM_SRC_CAPS] = {};
// raw PDOs of the last Source_Capabilities / EPR_Source_Capabilities, EPR_Request sends a copy of the requested PDO
static uint32_t _src_pdos[MAX_NUM_SRC_CAPS] = {};
static pd_preference_t _pd_power_preference = PD_PREFERENCE_MAX_POWER;

// Chunked Extended Message receive
static struct {
	uint8_t type;
	size_t size; // 0 = no message in progress
	size_t received;
	uint8_t data[MAX_NUM_SRC_CAPS * 4];
} _ext_rx = {};

// Chunked Extended Message transmit
static struct {
	uint8_t type;
	size_t size; // 0 = no message in progress
	size_t next_chunk;
	// time the last Chunk was sent, for ChunkSenderRequestTimer
	uint64_t sent_us;
	uint8_t data[MAX_NUM_SRC_CAPS * 4];
} _ext_tx = {};

static void _pd_set_state(smol_usb_pd_state_t state) {
	m_usb_pd_state = state;
	_state_since_us = smol_time64(timer0_hw);
}

static void _pd_wait_for_caps(uint32_t timeout_us) {
	_wait_caps_timeout_us = timeout_us;
	_pd_set_state(SMOL_USB_PD_WAIT_CAPS);
}

// reset the contract and EPR state: after attach, detach and Hard Reset
static void _pd_reset_contract(void) {
	memset(_src_capabilities, 0, sizeof(_src_capabilities));
	memset(_src_pdos, 0, sizeof(_src_pdos));
	_have_contract = false;
	_contract_is_epr = false;
	_requested_pos = 0;
	_requested_is_pps = false;
	_contract_is_pps = false;
	_src_epr_capable = false;
	_epr_mode = false;
	_epr_entry_attempts = 0;
	_ext_rx.size = 0;
	_ext_tx.size = 0;
}

static void _pd_attach(uint32_t wait_caps_timeout_us) {
	// reset PD machinery, then connect the BMC transceiver with AUTO_CRC to the active CC pin
	fusb302_write_reg(FUSB_REG_RESET, FUSB_RESET_PD_MASK);
	fusb302_set_pd_cc(_cc_index);
	_msg_counter = 0;
	_last_rx_msgid = -1;
	_spec_rev = PD_SPEC_REV_30;
	_cc_low_since_us = 0;
	_pd_reset_contract();
	_pd_wait_for_caps(wait_caps_timeout_us);
}

void smol_usb_pd_init(void) {
	uint8_t bc_lvl1 = 0;
	fusb302_check_cc1(&bc_lvl1);

	uint8_t bc_lvl2 = 0;
	fusb302_check_cc2(&bc_lvl2);
	info_println("BC_LVL1 = %u, BC_LVL2 = %0u", bc_lvl1, bc_lvl2);

	size_t cc_index = 0;
	if ((bc_lvl1 != 0) && (bc_lvl2 == 0))
		cc_index = 1;
	else if ((bc_lvl1 == 0) && (bc_lvl2 != 0))
		cc_index = 2;

	if (cc_index == 0) {
		warn_println("No USB PD source detected!");
	}
	else
	{
		info_println("USB PD source detected on CC%u!", cc_index);
		_cc_index = cc_index;
		_hard_reset_counter = 0;
		_pd_attach(PD_SINK_WAIT_CAP_US);
	}
}

static void _pd_tx_header(uint32_t type, size_t num_objects, bool extended) {
	uint8_t sop[4] = {FUSB_FIFO_TX_SYNC1, FUSB_FIFO_TX_SYNC1, FUSB_FIFO_TX_SYNC1, FUSB_FIFO_TX_SYNC2};
	fusb302_tx(sop, sizeof(sop));
	uint8_t packsym = FUSB_FIFO_TX_PACKSYM | (2 + 4*num_objects);
	fusb302_tx(&packsym, 1);
	// Port Data Role (bit 5) = UFP and Port Power Role (bit 8) = Sink stay 0
	uint16_t header = 0;
	header |= (extended ? 1u : 0u) << 15;
	header |= (_spec_rev & 0x3) << 6;
	header |= ((_msg_counter++) & 0x7) << 9;
	header |= num_objects << 12;
	header |= type << 0;
	fusb302_tx(&header, 2);
	_last_traffic_us = smol_time64(timer0_hw);
}

static void _pd_tx_object(uint32_t object) {
	fusb302_tx(&object, sizeof(object));
}

static void _pd_tx_send(void) {
	uint8_t jam_crc = FUSB_FIFO_TX_JAM_CRC;
	fusb302_tx(&jam_crc, 1);
	uint8_t eop = FUSB_FIFO_TX_EOP;
	fusb302_tx(&eop, 1);
	uint8_t off = FUSB_FIFO_TX_OFF;
	fusb302_tx(&off, 1);
	uint8_t on = FUSB_FIFO_TX_ON;
	fusb302_tx(&on, 1);
}

static void _pd_send_control(uint32_t cmd) {
	_pd_tx_header(cmd, 0, false);
	_pd_tx_send();
}

static void _pd_send_data(uint32_t type, const uint32_t* objects, size_t num_objects) {
	_pd_tx_header(type, num_objects, false);
	for (size_t i = 0; i < num_objects; ++i)
		_pd_tx_object(objects[i]);
	_pd_tx_send();
}

// Reply to a message that this sink does not support. Not_Supported exists only in PD Rev 3.x, a Rev 2.0 port ignores the message.
static void _pd_reply_not_supported(void) {
	if (_spec_rev >= PD_SPEC_REV_30)
		_pd_send_control(PD_CMD_NOT_SUPPORTED);
}

// Rev 3.x: Not_Supported. Rev 2.0: Reject.
static void _pd_reply_not_supported_or_reject(void) {
	if (_spec_rev >= PD_SPEC_REV_30)
		_pd_send_control(PD_CMD_NOT_SUPPORTED);
	else
		_pd_send_control(PD_CMD_REJECT);
}

// Send the next Chunk of the Extended Message in _ext_tx. Every Extended Message is Chunked, also when it fits into one Chunk.
static void _pd_send_ext_chunk(void) {
	size_t offset = PD_CHUNK_LEN * _ext_tx.next_chunk;
	size_t n = _ext_tx.size - offset;
	if (n > PD_CHUNK_LEN)
		n = PD_CHUNK_LEN;
	size_t num_objects = (2 + n + 3) / 4;

	uint8_t buf[2 + PD_CHUNK_LEN + 2] = {};
	uint16_t ext_header = (1u << 15) | ((_ext_tx.next_chunk & 0xF) << 11) | (_ext_tx.size & 0x1FF);
	buf[0] = ext_header & 0xFF;
	buf[1] = ext_header >> 8;
	memcpy(buf + 2, _ext_tx.data + offset, n);

	_pd_tx_header(_ext_tx.type, num_objects, true);
	fusb302_tx(buf, 4*num_objects);
	_pd_tx_send();

	_ext_tx.next_chunk++;
	_ext_tx.sent_us = smol_time64(timer0_hw);
	if (offset + n >= _ext_tx.size)
		_ext_tx.size = 0; // last Chunk sent
}

static void _pd_send_extended(uint8_t type, const void* data, size_t size) {
	_ext_tx.type = type;
	_ext_tx.size = size;
	_ext_tx.next_chunk = 0;
	memcpy(_ext_tx.data, data, size);
	_pd_send_ext_chunk();
}

// Chunk Request: Request Chunk = 1, Data Size = 0, padded to one Data Object
static void _pd_send_chunk_request(uint8_t type, size_t chunk_number) {
	uint8_t buf[4] = {};
	uint16_t ext_header = (1u << 15) | ((chunk_number & 0xF) << 11) | (1u << 10);
	buf[0] = ext_header & 0xFF;
	buf[1] = ext_header >> 8;
	_pd_tx_header(type, 1, true);
	fusb302_tx(buf, sizeof(buf));
	_pd_tx_send();
}

static void _pd_send_extended_control(uint8_t ecdb_type) {
	uint8_t ecdb[2] = {ecdb_type, 0};
	_pd_send_extended(PD_EXT_EXTENDED_CONTROL, ecdb, sizeof(ecdb));
}

// Rev 3.x collision avoidance: the sink starts an AMS only when the Source sets Rp to SinkTxOk (3.0A). BC_LVL comes from the detach poll.
static bool _pd_sink_tx_ok(void) {
	return _bc_lvl == 3;
}

static bool _pd_check_vbus(void) {
	// if we VUSB is low and VBUS is higher, we have external power.
	float vusb_lv = 4.0f * VBUS_LV_RATIO;
	bool vusb_ok = false;
	fusb302_measure_vbus(vusb_lv, &vusb_ok);
	// info_println("PD check VUSB_OK = %u", vusb_ok);
	float vbus = smol_adc_vbus();
	// info_println("PD check VBUS = %.3f", vbus);

	return false;
}

static void _pd_hard_reset(void) {
	// reset PD machinery. The chip re-init clears SWITCHES0/1, so attach again to restore the CC setup.
	fusb302_hard_reset();
	if (_cc_index != 0) {
		_pd_attach(PD_HARD_RESET_WAIT_CAP_US);
	}
	else {
		_pd_reset_contract();
		m_usb_pd_state = SMOL_USB_PD_IDLE;
	}
}

// Send Hard Reset signaling, then reset the local PD state. The Source then cycles VBUS through vSafe0V.
static void _pd_send_hard_reset(const char* reason) {
	warn_println("PD sending hard reset: %s", reason);
	_hard_reset_counter++;
	fusb302_send_hard_reset();
	uint64_t start_us = smol_time64(timer0_hw);
	while (smol_time64(timer0_hw) < start_us + PD_HARD_RESET_SEND_US) {
		uint8_t interrupta = 0;
		fusb302_read_reg(FUSB_REG_INTERRUPTA, &interrupta);
		if (interrupta & FUSB_INTERRUPTA_HARDSENT_MASK)
			break;
	}
	_pd_hard_reset();
}

static void _pd_soft_reset(void) {
	// a Soft_Reset message always uses MessageID 0
	_msg_counter = 0;
	// the reply (Accept) also starts at MessageID 0
	_last_rx_msgid = -1;
	_ext_rx.size = 0;
	_ext_tx.size = 0;
	_pd_tx_header(PD_CMD_SOFT_RESET, 0, false);
	_pd_tx_send();
	_pd_set_state(SMOL_USB_PD_SOFT_RESET_SENT);
}

static src_cap_t _parse_pdo(uint32_t capability) {
	uint32_t pdo_type = (capability >> 30) & 0x3;

	src_cap_t src_cap = {};

	if (pdo_type == PDO_APDO)
	{
		src_cap.is_apdo = true;
		uint32_t apdo_type = (capability >> 28) & 0x3;

		if (apdo_type == APDO_SPR_PROGRAMMABLE)
		{
			uint32_t voltage_max_100m = (capability >> 17) & 0xFF;
			uint32_t voltage_min_100m = (capability >> 8) & 0xFF;
			uint32_t max_current_50m = (capability >> 0) & 0x7F;
			src_cap.min_voltage = 100*voltage_min_100m;
			src_cap.max_voltage = 100*voltage_max_100m;
			src_cap.max_current = 50*max_current_50m;
			src_cap.max_power = (src_cap.max_voltage * src_cap.max_current) / 1000;
			src_cap.is_pps = true;
		}
		else if (apdo_type == APDO_EPR_ADJUSTABLE)
		{
			uint32_t voltage_max_100m = (capability >> 17) & 0x1FF;
			uint32_t voltage_min_100m = (capability >> 8) & 0xFF;
			uint32_t pdp_1w = (capability >> 0) & 0xFF;
			src_cap.min_voltage = 100*voltage_min_100m;
			src_cap.max_voltage = 100*voltage_max_100m;
			src_cap.max_power = 1000*pdp_1w;
			src_cap.is_epr_avs = true;
		}
		else if (apdo_type == APDO_SPR_ADJUSTABLE)
		{
			uint32_t lo_range_current_10m = (capability >> 10) & 0x3FF;
			uint32_t hi_range_current_10m = (capability >> 0) & 0x3FF;
			src_cap.min_voltage = 9000;
			if (hi_range_current_10m == 0) {
				src_cap.max_voltage = 15000;
				src_cap.max_current = 10*lo_range_current_10m;
			}
			else
			{
				src_cap.max_voltage = 20000;
				src_cap.max_current = 10*hi_range_current_10m;
			}
			src_cap.max_power = (src_cap.max_voltage * src_cap.max_current) / 1000;
			src_cap.is_spr_avs = true;
		}
	}
	else if (pdo_type == PDO_FIXED)
	{
		uint32_t voltage_50m = (capability >> 10) & 0x3FF;
		uint32_t max_current_10m = (capability >> 0) & 0x3FF;
		src_cap.min_voltage = 50*voltage_50m;
		src_cap.max_voltage = 50*voltage_50m;
		src_cap.max_current = 10*max_current_10m;
		src_cap.max_power = (src_cap.max_voltage * src_cap.max_current) / 1000;
	}
	else if (pdo_type == PDO_VARIABLE)
	{
		uint32_t voltage_max_50m = (capability >> 20) & 0x3FF;
		uint32_t voltage_min_50m = (capability >> 10) & 0x3FF;
		uint32_t max_current_10m = (capability >> 0) & 0x3FF;
		src_cap.min_voltage = 50*voltage_min_50m;
		src_cap.max_voltage = 50*voltage_max_50m;
		src_cap.max_current = 10*max_current_10m;
		src_cap.max_power = (src_cap.min_voltage * src_cap.max_current) / 1000;
		src_cap.is_variable = true;
	}
	else if (pdo_type == PDO_BATTERY)
	{
		uint32_t voltage_max_50m = (capability >> 20) & 0x3FF;
		uint32_t voltage_min_50m = (capability >> 10) & 0x3FF;
		uint32_t max_power_250m = (capability >> 0) & 0x3FF;
		src_cap.min_voltage = 50*voltage_min_50m;
		src_cap.max_voltage = 50*voltage_max_50m;
		src_cap.max_power = 250*max_power_250m;
		src_cap.is_variable = true;
		src_cap.is_battery = true;
	}

	return src_cap;
}

// EPR (A)PDO: Fixed above 20V, or EPR AVS APDO
static bool _pdo_is_epr(uint32_t pdo) {
	src_cap_t cap = _parse_pdo(pdo);
	if (cap.is_epr_avs)
		return true;
	return (((pdo >> 30) & 0x3) == PDO_FIXED) && (cap.max_voltage > 20000);
}

static void _pd_respond_to_src_caps(void) {
	// we have to go through the source capabilities and pick one.
	// Since the TMC6200 needs at least 8V, we should try to go for 9V or more

	// bool already_have_vbus = _pd_check_vbus();
	bool already_have_vbus = false;

	int best_current_cap = 0;
	int best_voltage_cap = 0;
	int best_power_cap = 0;
	uint32_t best_voltage = 5000;
	for (size_t i = 1; i < MAX_NUM_SRC_CAPS; ++i) {
		const src_cap_t* c = &_src_capabilities[i];
		if (c->max_voltage == 0)
			continue; // empty position
		// EPR AVS only in EPR Mode. In EPR Mode, EPR_Request can also request an SPR (A)PDO.
		if (c->is_epr_avs && !_epr_mode)
			continue;
		uint32_t voltage = 0;
		if (c->is_apdo) {
			// APDO: request the highest voltage in the range, at most PD_MAX_VOLTAGE_MV
			voltage = c->max_voltage < PD_MAX_VOLTAGE_MV ? c->max_voltage : PD_MAX_VOLTAGE_MV;
			if (voltage < c->min_voltage)
				continue;
		}
		else {
			// need at least 9V, so variable supplies are not allowed to dip below.
			voltage = c->is_variable ? c->min_voltage : c->max_voltage;
			if (c->max_voltage > PD_MAX_VOLTAGE_MV)
				continue;
		}
		if (voltage < 9000)
			continue;

		if (voltage > best_voltage) {
			best_voltage_cap = i;
			best_voltage = voltage;
		}
		if (_src_capabilities[i].max_current > _src_capabilities[best_current_cap].max_current)
			best_current_cap = i;
		if (_src_capabilities[i].max_power > _src_capabilities[best_power_cap].max_power)
			best_power_cap = i;
	}

	int pdo_index = 0;
	switch(_pd_power_preference) {
		case PD_PREFERENCE_MAX_POWER:
			pdo_index = best_power_cap;
			break;
		case PD_PREFERENCE_MAX_CURRENT:
			pdo_index = best_current_cap;
			break;
		case PD_PREFERENCE_MAX_VOLTAGE:
			pdo_index = best_voltage_cap;
			break;
	}
	if (already_have_vbus)
		pdo_index = 0;

	const src_cap_t* cap = &_src_capabilities[pdo_index];
	uint32_t voltage = cap->is_variable ? cap->min_voltage : cap->max_voltage;
	if (cap->is_apdo && (voltage > PD_MAX_VOLTAGE_MV))
		voltage = PD_MAX_VOLTAGE_MV;
	uint32_t current = cap->max_current;

	uint32_t request_obj = 0;
	request_obj |= (pdo_index + 1) << 28;  // PD object positions are 1-based; array index 0 = PDO position 1
	request_obj |= ((voltage < 9000) && !already_have_vbus) << 26; // cap mismatch
	request_obj |= 1u << 25; // USB comms
	request_obj |= 1u << 24; // no SUSPEND
	request_obj |= 0u << 23; // no unchunked ext messages
	request_obj |= (SMOL_PD_EPR_ENABLE ? 1u : 0u) << 22; // EPR Mode capable
	if (cap->is_pps) {
		// PPS RDO: output voltage in 20mV units, operating current in 50mA units
		voltage = voltage / 20 * 20;
		current = current / 50 * 50;
		request_obj |= (voltage / 20) << 9;
		request_obj |= (current / 50) << 0;
	}
	else if (cap->is_spr_avs || cap->is_epr_avs) {
		// AVS RDO: output voltage in 25mV units with 100mV steps, operating current in 50mA units
		voltage = voltage / 100 * 100;
		if (cap->is_epr_avs) {
			// EPR AVS: current at most PDP / voltage
			current = cap->max_power * 1000 / voltage;
			if (current > 5000)
				current = 5000;
		}
		current = current / 50 * 50;
		request_obj |= ((voltage / 100) * 4) << 9;
		request_obj |= (current / 50) << 0;
	}
	else if (cap->is_battery) {
		request_obj |= (cap->max_power / 250) << 10;
		request_obj |= (cap->max_power / 250) << 0;
	}
	else
	{
		request_obj |= (current / 10) << 10;
		request_obj |= (current / 10) << 0;
	}

	_requested_pos = pdo_index + 1;
	_requested_is_pps = cap->is_pps;
	_last_request_us = smol_time64(timer0_hw);

	// in EPR Mode, every request is an EPR_Request with a copy of the requested PDO
	if (_epr_mode) {
		uint32_t objects[2] = {request_obj, _src_pdos[pdo_index]};
		_pd_send_data(PD_DATA_EPR_REQUEST, objects, 2);
	}
	else {
		_pd_send_data(PD_DATA_REQUEST, &request_obj, 1);
	}

	_pd_set_state(SMOL_USB_PD_REQUEST_SENT);

	dbg_println("PD %sRequest 0x%08x: PDO %u%s, %umV, %umA", _epr_mode ? "EPR_" : "", request_obj, pdo_index + 1,
		cap->is_pps ? " PPS" : (cap->is_apdo ? " AVS" : ""), voltage, current);
}

/**
 * SPR Sink PDOs. PDO order: vSafe5V Fixed first, then Fixed, Battery, Variable, APDO.
 * EPR PDOs are not allowed here (they go into EPR_Sink_Capabilities).
 */
static size_t _pd_spr_sink_pdos(uint32_t* pdos) {
	size_t n = 0;

	uint32_t safe5vcap = 0;
	safe5vcap |= PDO_FIXED << 30;
	safe5vcap |= 1 << 28; // higher capability
	safe5vcap |= 1 << 26; // USB comms capable
	safe5vcap |= (5000/50) << 10; // voltage
	safe5vcap |= (100/10) << 0; // operational current
	pdos[n++] = safe5vcap;

	uint32_t batcap = 0;
	batcap |= PDO_BATTERY << 30;
	batcap |= (21000/50) << 20; // max voltage
	batcap |= (9000/50) << 10; // min voltage
	batcap |= (100000/250) << 0; // operational power, SPR maximum is 100W
	pdos[n++] = batcap;

	uint32_t varcap = 0;
	varcap |= PDO_VARIABLE << 30;
	varcap |= (21000/50) << 20; // max voltage
	varcap |= (9000/50) << 10; // min voltage
	varcap |= (5000/10) << 0; // operational current
	pdos[n++] = varcap;

	// APDOs exist only in PD Rev 3.x
	if (_spec_rev >= PD_SPEC_REV_30) {
		uint32_t spr_prog_cap = 0;
		spr_prog_cap |= PDO_APDO << 30;
		spr_prog_cap |= APDO_SPR_PROGRAMMABLE << 28;
		spr_prog_cap |= (21000/100) << 17; // max voltage
		spr_prog_cap |= (9000/100) << 8; // min voltage
		spr_prog_cap |= (5000/50) << 0; // current
		pdos[n++] = spr_prog_cap;
	}
	return n;
}

static void _pd_send_sink_caps(void) {
	uint32_t pdos[MAX_NUM_SPR_CAPS];
	size_t n = _pd_spr_sink_pdos(pdos);
	_pd_send_data(PD_DATA_SINK_CAPS, pdos, n);
}

/**
 * EPR_Sink_Capabilities: the SPR PDOs in positions 1..7 (zero filled), then the EPR PDOs from position 8.
 * EPR PDO order: Fixed 28V, 36V, 48V (low to high), then one EPR AVS APDO.
 */
static void _pd_send_epr_sink_caps(void) {
	uint32_t pdos[MAX_NUM_SRC_CAPS] = {};
	_pd_spr_sink_pdos(pdos);
	size_t n = MAX_NUM_SPR_CAPS;

	const uint32_t epr_fixed_mv[] = {28000, 36000, 48000};
	for (size_t i = 0; i < sizeof(epr_fixed_mv)/sizeof(*epr_fixed_mv); ++i) {
		if (epr_fixed_mv[i] > PD_MAX_VOLTAGE_MV)
			continue;
		uint32_t fixed = 0;
		fixed |= PDO_FIXED << 30;
		fixed |= (epr_fixed_mv[i]/50) << 10; // voltage
		fixed |= (5000/10) << 0; // operational current
		pdos[n++] = fixed;
	}

	uint32_t avs = 0;
	avs |= PDO_APDO << 30;
	avs |= APDO_EPR_ADJUSTABLE << 28;
	avs |= (PD_MAX_VOLTAGE_MV/100) << 17; // max voltage
	avs |= (15000/100) << 8; // min voltage
	avs |= PD_EPR_SINK_MAX_PDP_W << 0; // PDP
	pdos[n++] = avs;

	_pd_send_extended(PD_EXT_EPR_SINK_CAP, pdos, 4*n);
}

// Sink_Capabilities_Extended: 24-byte SKEDB
static void _pd_send_sink_caps_extended(void) {
	uint8_t skedb[24] = {};
	skedb[0] = PD_SKEDB_VID & 0xFF;
	skedb[1] = PD_SKEDB_VID >> 8;
	skedb[2] = PD_SKEDB_PID & 0xFF;
	skedb[3] = PD_SKEDB_PID >> 8;
	uint32_t xid = PD_SKEDB_XID;
	memcpy(skedb + 4, &xid, 4);
	skedb[8] = PD_SKEDB_FW_VERSION;
	skedb[9] = PD_SKEDB_HW_VERSION;
	skedb[10] = 1; // SKEDB Version 1.0
	skedb[11] = 0; // Load Step: 150mA/us
	skedb[12] = 0; // Sink Load Characteristics: default
	skedb[13] = 0;
	skedb[14] = 0; // Compliance
	skedb[15] = 0; // Touch Temp: not applicable
	skedb[16] = 0; // Battery Info: no batteries
	skedb[17] = (1u << 1) | ((SMOL_PD_EPR_ENABLE ? 1u : 0u) << 5); // Sink Modes: VBUS powered, AVS supported
	skedb[18] = PD_SINK_MIN_PDP_W;
	skedb[19] = PD_SINK_OPERATIONAL_PDP_W;
	skedb[20] = PD_SINK_MAX_PDP_W;
	if (SMOL_PD_EPR_ENABLE) {
		skedb[21] = PD_EPR_SINK_MIN_PDP_W;
		skedb[22] = PD_EPR_SINK_OPERATIONAL_PDP_W;
		skedb[23] = PD_EPR_SINK_MAX_PDP_W;
	}
	_pd_send_extended(PD_EXT_SINK_CAP_EXTENDED, skedb, sizeof(skedb));
}

static void _pd_send_epr_enter(void) {
	uint32_t eprmdo = (PD_EPR_MODE_ENTER << 24) | (PD_EPR_SINK_OPERATIONAL_PDP_W << 16);
	_pd_send_data(PD_DATA_EPR_MODE, &eprmdo, 1);
	_epr_enter_sent_us = smol_time64(timer0_hw);
	_epr_entry_attempts++;
	_pd_set_state(SMOL_USB_PD_EPR_ENTER_SENT);
	dbg_println("PD EPR_Mode Enter, %uW", PD_EPR_SINK_OPERATIONAL_PDP_W);
}

// EPR Mode entry failed: Soft Reset. The sink tries again after the new contract, at most PD_EPR_ENTRY_ATTEMPTS times.
static void _pd_epr_entry_failed(const char* reason) {
	warn_println("PD EPR Mode entry failed (attempt %u): %s", _epr_entry_attempts, reason);
	_pd_soft_reset();
}

static void _parse_pd_command(uint32_t cmd) {
	switch(cmd) {
		// messages that need no reply
		case PD_CMD_GOOD_CRC:
		case PD_CMD_GOTO_MIN:
		case PD_CMD_PING:
		case PD_CMD_NOT_SUPPORTED:
			break;
		case PD_CMD_ACCEPT:
			if (m_usb_pd_state == SMOL_USB_PD_REQUEST_SENT) {
				_pd_set_state(SMOL_USB_PD_REQUEST_ACCEPTED);
				dbg_println("PD request accepted");
			}
			else if (m_usb_pd_state == SMOL_USB_PD_SOFT_RESET_SENT) {
				_pd_wait_for_caps(PD_SINK_WAIT_CAP_US);
				dbg_println("PD soft reset accepted");
			}
			break;
		case PD_CMD_REJECT:
			if (m_usb_pd_state == SMOL_USB_PD_REQUEST_SENT) {
				warn_println("PD request rejected");
				// keep the existing Explicit Contract, else wait for new capabilities
				if (_have_contract)
					_pd_set_state(SMOL_USB_PD_READY);
				else
					_pd_wait_for_caps(PD_SINK_WAIT_CAP_US);
			}
			break;
		case PD_CMD_WAIT:
			if (m_usb_pd_state == SMOL_USB_PD_REQUEST_SENT) {
				if (_epr_mode) {
					// Wait is not a valid response to EPR_Request
					_pd_send_hard_reset("Wait response to EPR_Request");
				}
				else {
					// send the Request again after tSinkRequest
					_pd_set_state(SMOL_USB_PD_REQUEST_WAIT);
					_wait_until_us = smol_time64(timer0_hw) + PD_SINK_REQUEST_US;
					dbg_println("PD request: wait");
				}
			}
			break;
		case PD_CMD_PS_READY:
			if (m_usb_pd_state == SMOL_USB_PD_REQUEST_ACCEPTED) {
				_have_contract = true;
				_contract_is_epr = _requested_pos > MAX_NUM_SPR_CAPS;
				_contract_is_pps = _requested_is_pps;
				_hard_reset_counter = 0;
				_pd_set_state(SMOL_USB_PD_READY);
				dbg_println("PD PS_RDY, %s%s contract established", _contract_is_epr ? "EPR" : "SPR", _contract_is_pps ? " PPS" : "");
			}
			break;
		case PD_CMD_SOFT_RESET:
			// the source resets the protocol layer: accept, then wait for new (EPR_)Source_Capabilities. EPR Mode stays.
			_msg_counter = 0;
			_last_rx_msgid = -1;
			_ext_rx.size = 0;
			_ext_tx.size = 0;
			_pd_send_control(PD_CMD_ACCEPT);
			_pd_wait_for_caps(PD_SINK_WAIT_CAP_US);
			dbg_println("PD soft reset received, accepted");
			break;
		case PD_CMD_GET_SINK_CAP:
			_pd_send_sink_caps();
			break;
		case PD_CMD_GET_SINK_CAP_EXTENDED:
			if (_spec_rev >= PD_SPEC_REV_30)
				_pd_send_sink_caps_extended();
			break;
		case PD_CMD_GET_REVISION:
			if (_spec_rev >= PD_SPEC_REV_30) {
				uint32_t rmdo = PD_RMDO;
				_pd_send_data(PD_DATA_REVISION, &rmdo, 1);
			}
			break;
		// sink-only UFP without VCONN: no swaps, no source capabilities
		case PD_CMD_DR_SWAP:
		case PD_CMD_PR_SWAP:
		case PD_CMD_VCONN_SWAP:
		case PD_CMD_GET_SOURCE_CAP:
			_pd_reply_not_supported_or_reject();
			break;
		case PD_CMD_DATA_RESET:
		case PD_CMD_GET_SOURCE_CAP_EXTENDED:
		case PD_CMD_GET_STATUS:
		case PD_CMD_FR_SWAP:
		case PD_CMD_GET_PPS_STATUS:
		case PD_CMD_GET_COUNTRY_CODES:
		case PD_CMD_GET_SOURCE_INFO:
			_pd_reply_not_supported();
			break;
		default:
			// reserved message types are ignored
			break;
	}
}

static void _pd_handle_src_caps(uint16_t header, const uint8_t* objects, size_t num_objects) {
	if (_epr_mode) {
		// in EPR Mode the Source must use EPR_Source_Capabilities
		_pd_send_hard_reset("Source_Capabilities in EPR Mode");
		return;
	}

	memset(_src_capabilities, 0, sizeof(_src_capabilities));
	memset(_src_pdos, 0, sizeof(_src_pdos));
	// use the lower of the two Spec Revisions from now on
	uint32_t partner_rev = (header >> 6) & 0x3;
	_spec_rev = partner_rev < PD_SPEC_REV_30 ? partner_rev : PD_SPEC_REV_30;

	for (size_t i = 0; (i < num_objects) && (i < MAX_NUM_SPR_CAPS); ++i) {
		memcpy(&_src_pdos[i], objects + 4*i, 4);
		_src_capabilities[i] = _parse_pdo(_src_pdos[i]);
	}
	// EPR Mode Capable: bit 23 of the 5V Fixed PDO
	_src_epr_capable = (_spec_rev >= PD_SPEC_REV_30) && (0 != (_src_pdos[0] & (1u << 23)));

	_pd_respond_to_src_caps();
	// print after the Request, so the print does not delay it
	dbg_println("PD Source_Capabilities: %08x %08x %08x %08x %08x %08x %08x",
		_src_pdos[0], _src_pdos[1], _src_pdos[2], _src_pdos[3], _src_pdos[4], _src_pdos[5], _src_pdos[6]);
}

static void _pd_handle_epr_src_caps(const uint8_t* data, size_t size) {
	if (!_epr_mode) {
		// in SPR Mode, EPR_Source_Capabilities is informational only
		return;
	}
	size_t num_objects = size / 4;
	if (num_objects > MAX_NUM_SRC_CAPS)
		num_objects = MAX_NUM_SRC_CAPS;

	memset(_src_capabilities, 0, sizeof(_src_capabilities));
	memset(_src_pdos, 0, sizeof(_src_pdos));
	for (size_t i = 0; i < num_objects; ++i) {
		uint32_t pdo;
		memcpy(&pdo, data + 4*i, 4);
		if ((i < MAX_NUM_SPR_CAPS) && _pdo_is_epr(pdo)) {
			_pd_send_hard_reset("EPR PDO in SPR position");
			return;
		}
		_src_pdos[i] = pdo;
		_src_capabilities[i] = _parse_pdo(pdo);
	}
	_pd_respond_to_src_caps();
}

static void _pd_handle_epr_mode(uint32_t eprmdo) {
	uint32_t action = eprmdo >> 24;
	uint32_t data = (eprmdo >> 16) & 0xFF;

	switch (action) {
		case PD_EPR_MODE_ENTER_ACK:
			// ignored outside the entry process
			if (m_usb_pd_state == SMOL_USB_PD_EPR_ENTER_SENT)
				_pd_set_state(SMOL_USB_PD_EPR_ENTER_WAIT);
			break;
		case PD_EPR_MODE_ENTER_SUCCEEDED:
			if ((m_usb_pd_state == SMOL_USB_PD_EPR_ENTER_SENT) || (m_usb_pd_state == SMOL_USB_PD_EPR_ENTER_WAIT)) {
				_epr_mode = true;
				_epr_entry_attempts = 0;
				dbg_println("PD EPR Mode entered");
				// the Source now sends EPR_Source_Capabilities
				_pd_wait_for_caps(PD_SINK_WAIT_CAP_US);
			}
			break;
		case PD_EPR_MODE_ENTER_FAILED:
			if ((m_usb_pd_state == SMOL_USB_PD_EPR_ENTER_SENT) || (m_usb_pd_state == SMOL_USB_PD_EPR_ENTER_WAIT)) {
				warn_println("PD EPR_Mode Enter Failed, cause %u", data);
				// the Source refused: do not try again until the next attach
				_epr_entry_attempts = PD_EPR_ENTRY_ATTEMPTS;
				_pd_epr_entry_failed("Enter Failed");
			}
			break;
		case PD_EPR_MODE_EXIT:
			if (!_epr_mode)
				break;
			if (_contract_is_epr) {
				_pd_send_hard_reset("EPR Mode exit with EPR contract");
			}
			else {
				dbg_println("PD EPR Mode exit");
				_epr_mode = false;
				// the Source ended EPR Mode: do not enter again until the next attach
				_epr_entry_attempts = PD_EPR_ENTRY_ATTEMPTS;
				// the Source now sends Source_Capabilities
				_pd_wait_for_caps(PD_SINK_WAIT_CAP_US);
			}
			break;
		default:
			// Enter is sent by the sink only
			_pd_reply_not_supported();
			break;
	}
}

static void _parse_pd_data_message(uint16_t header, uint32_t cmd, const uint8_t* objects, size_t num_objects) {
	switch(cmd) {
		case PD_DATA_SOURCE_CAPS:
			_pd_handle_src_caps(header, objects, num_objects);
			break;
		case PD_DATA_EPR_MODE:
			if (SMOL_PD_EPR_ENABLE) {
				uint32_t eprmdo;
				memcpy(&eprmdo, objects, 4);
				_pd_handle_epr_mode(eprmdo);
			}
			else {
				_pd_reply_not_supported();
			}
			break;
		// messages that need no reply, or are only valid as a reply to a request of ours
		case PD_DATA_BIST:
		case PD_DATA_SOURCE_INFO:
		case PD_DATA_REVISION:
			break;
		case PD_DATA_REQUEST:
		case PD_DATA_SINK_CAPS:
		case PD_DATA_BATTERY_STATUS:
		case PD_DATA_ALERT:
		case PD_DATA_GET_COUNTRY_INFO:
		case PD_DATA_ENTER_USB:
		case PD_DATA_EPR_REQUEST:
		case PD_DATA_VENDOR_DEFINED: // includes Discover Identity: no Structured VDM support
			_pd_reply_not_supported();
			break;
		default:
			break;
	}
}

static void _pd_handle_extended_control(const uint8_t* ecdb) {
	switch (ecdb[0]) {
		case PD_ECDB_EPR_GET_SINK_CAP:
			if (SMOL_PD_EPR_ENABLE)
				_pd_send_epr_sink_caps();
			else
				_pd_reply_not_supported();
			break;
		case PD_ECDB_EPR_KEEPALIVE_ACK:
			if (m_usb_pd_state == SMOL_USB_PD_EPR_KEEPALIVE_SENT)
				_pd_set_state(SMOL_USB_PD_READY);
			break;
		case PD_ECDB_EPR_GET_SOURCE_CAP:
		case PD_ECDB_EPR_KEEPALIVE:
		default:
			_pd_reply_not_supported();
			break;
	}
}

static void _pd_handle_extended_message(uint8_t type, const uint8_t* data, size_t size) {
	switch (type) {
		case PD_EXT_EPR_SOURCE_CAP:
			_pd_handle_epr_src_caps(data, size);
			break;
		case PD_EXT_EXTENDED_CONTROL:
			if (size >= 2)
				_pd_handle_extended_control(data);
			break;
	}
}

/**
 * Extended Message receive, with the Chunking state machine reduced to what the sink needs.
 * payload points to the data after the Extended Message Header, payload_len is the padded length.
 */
static void _pd_rx_extended(uint8_t type, uint16_t ext_header, const uint8_t* payload, size_t payload_len) {
	bool is_chunked = (ext_header >> 15) & 0x1;
	size_t chunk_number = (ext_header >> 11) & 0xF;
	bool request_chunk = (ext_header >> 10) & 0x1;
	size_t data_size = ext_header & 0x1FF;

	if (request_chunk) {
		// the partner requests the next Chunk of a message we send
		if ((_ext_tx.size != 0) && (type == _ext_tx.type) && (chunk_number == _ext_tx.next_chunk))
			_pd_send_ext_chunk();
		else
			_ext_tx.size = 0;
		return;
	}
	// any other message ends a Chunked transmit
	_ext_tx.size = 0;

	bool supported = (type == PD_EXT_EPR_SOURCE_CAP) || (type == PD_EXT_EXTENDED_CONTROL);
	if (!supported || (data_size > sizeof(_ext_rx.data))) {
		_ext_rx.size = 0;
		if (chunk_number == 0)
			_pd_reply_not_supported();
		return;
	}

	if (chunk_number == 0) {
		_ext_rx.type = type;
		_ext_rx.size = data_size;
		_ext_rx.received = 0;
	}
	else if ((_ext_rx.size == 0) || (type != _ext_rx.type) || (chunk_number * PD_CHUNK_LEN != _ext_rx.received)) {
		// Chunk out of sequence
		_ext_rx.size = 0;
		return;
	}

	size_t n = _ext_rx.size - _ext_rx.received;
	if (is_chunked && (n > PD_CHUNK_LEN))
		n = PD_CHUNK_LEN;
	if (n > payload_len)
		n = payload_len;
	memcpy(_ext_rx.data + _ext_rx.received, payload, n);
	_ext_rx.received += n;

	if (_ext_rx.received < _ext_rx.size) {
		if (is_chunked)
			_pd_send_chunk_request(type, chunk_number + 1);
		else
			_ext_rx.size = 0;
		return;
	}

	size_t size = _ext_rx.size;
	_ext_rx.size = 0;
	_pd_handle_extended_message(type, _ext_rx.data, size);
}

/**
 * The EPR Mode entry process is a non-interruptible AMS. A message other than EPR_Mode during entry is an error, and the sink sends Soft_Reset.
 * Exceptions: Ping, Soft_Reset, and VCONN_Swap while waiting for Enter Succeeded (the Source can do a VCONN Swap for cable discovery).
 */
static bool _pd_epr_entry_interrupted(bool is_extended, size_t num_data_objects, uint32_t type) {
	if ((m_usb_pd_state != SMOL_USB_PD_EPR_ENTER_SENT) && (m_usb_pd_state != SMOL_USB_PD_EPR_ENTER_WAIT))
		return false;
	if (is_extended)
		return true;
	if (num_data_objects > 0)
		return type != PD_DATA_EPR_MODE;
	if ((type == PD_CMD_PING) || (type == PD_CMD_SOFT_RESET) || (type == PD_CMD_GOOD_CRC))
		return false;
	if ((type == PD_CMD_VCONN_SWAP) && (m_usb_pd_state == SMOL_USB_PD_EPR_ENTER_WAIT))
		return false;
	return true;
}

/**
 * Detach and attach detection on the CC pins.
 *
 * A sink detects detach when the source removes Rp: BC_LVL on the active CC pin reads 0 for longer than tPDDebounce. BMC traffic and Hard Reset do not remove Rp. VBUS is not usable here: the FUSB302 sees VBUS through the TPD4S480 VBUS_LV divider, so VBUSOK does not trigger.
 * This is only relevant when the board has a supply other than USB. Without one, a detach removes power.
 */
static void _pd_check_attach_detach(uint64_t now_us) {
	if (m_usb_pd_state != SMOL_USB_PD_IDLE) {
		if (now_us < _cc_poll_us + PD_DETACH_POLL_US)
			return;
		_cc_poll_us = now_us;

		uint8_t status0 = 0;
		if (0 != fusb302_read_reg(FUSB_REG_STATUS0, &status0))
			return;
		_bc_lvl = status0 & FUSB_STATUS0_BC_LVL_MASK;
		bool rp_present = 0 != _bc_lvl;
		if (rp_present) {
			_cc_low_since_us = 0;
		}
		else if (_cc_low_since_us == 0) {
			_cc_low_since_us = now_us;
		}
		else if (now_us > _cc_low_since_us + PD_DETACH_DEBOUNCE_US) {
			info_println("USB PD detach on CC%u", _cc_index);
			// chip re-init: AUTO_CRC off, so a new source sees no GoodCRC before attach
			fusb302_hard_reset();
			_pd_reset_contract();
			_cc_index = 0;
			_attach_candidate = 0;
			_hard_reset_counter = 0;
			m_usb_pd_state = SMOL_USB_PD_IDLE;
		}
		return;
	}

	// IDLE: poll both CC pins for a source
	if (now_us < _cc_poll_us + PD_ATTACH_POLL_US)
		return;
	_cc_poll_us = now_us;

	uint8_t bc_lvl1 = 0;
	uint8_t bc_lvl2 = 0;
	fusb302_check_cc1(&bc_lvl1);
	fusb302_check_cc2(&bc_lvl2);
	size_t candidate = 0;
	if ((bc_lvl1 != 0) && (bc_lvl2 == 0))
		candidate = 1;
	else if ((bc_lvl1 == 0) && (bc_lvl2 != 0))
		candidate = 2;

	if (candidate != _attach_candidate) {
		_attach_candidate = candidate;
		_attach_candidate_since_us = now_us;
	}
	else if ((candidate != 0) && (now_us > _attach_candidate_since_us + PD_ATTACH_DEBOUNCE_US)) {
		info_println("USB PD attach on CC%u", candidate);
		_cc_index = candidate;
		_attach_candidate = 0;
		_pd_attach(PD_SINK_WAIT_CAP_US);
	}
}

// protocol timers and sink-initiated messages
static void _pd_run_timers(uint64_t now_us) {
	uint64_t in_state_us = now_us - _state_since_us;

	// ChunkSenderRequestTimer: no Chunk Request for the next Chunk, end the Chunked transmit
	if ((_ext_tx.size != 0) && (now_us > _ext_tx.sent_us + PD_CHUNK_SENDER_REQUEST_US)) {
		warn_println("PD no Chunk Request for chunk %u of extended message %u", _ext_tx.next_chunk, _ext_tx.type);
		_ext_tx.size = 0;
	}

	switch (m_usb_pd_state) {
		case SMOL_USB_PD_WAIT_CAPS:
			// SinkWaitCapTimer: Hard Reset, at most nHardResetCount times. After that, stay at USB Type-C current.
			if ((_wait_caps_timeout_us != 0) && (in_state_us > _wait_caps_timeout_us)) {
				if (_hard_reset_counter < PD_HARD_RESET_COUNT) {
					_pd_send_hard_reset("no Source_Capabilities");
				}
				else {
					warn_println("PD no Source_Capabilities, stay at USB Type-C current");
					_wait_caps_timeout_us = 0;
				}
			}
			break;
		case SMOL_USB_PD_REQUEST_SENT:
			// SenderResponseTimer
			if (in_state_us > PD_SENDER_RESPONSE_US)
				_pd_send_hard_reset("no response to Request");
			break;
		case SMOL_USB_PD_REQUEST_WAIT:
			if (now_us > _wait_until_us) {
				dbg_println("PD request again after wait");
				_pd_respond_to_src_caps();
			}
			break;
		case SMOL_USB_PD_REQUEST_ACCEPTED:
			// PSTransitionTimer
			if (in_state_us > (_epr_mode ? PD_PS_TRANSITION_EPR_US : PD_PS_TRANSITION_US))
				_pd_send_hard_reset("no PS_RDY");
			break;
		case SMOL_USB_PD_SOFT_RESET_SENT:
			if (in_state_us > PD_SENDER_RESPONSE_US)
				_pd_send_hard_reset("no response to Soft_Reset");
			break;
		case SMOL_USB_PD_EPR_ENTER_SENT:
			if (in_state_us > PD_SENDER_RESPONSE_US)
				_pd_epr_entry_failed("no Enter Acknowledged");
			break;
		case SMOL_USB_PD_EPR_ENTER_WAIT:
			// SinkEPREnterTimer
			if (now_us > _epr_enter_sent_us + PD_ENTER_EPR_US)
				_pd_epr_entry_failed("no Enter Succeeded");
			break;
		case SMOL_USB_PD_READY:
			// SinkPPSPeriodicTimer: re-send the Request while the contract is PPS
			if (_contract_is_pps && (now_us > _last_request_us + PD_PPS_REQUEST_US) && (_ext_tx.size == 0) && _pd_sink_tx_ok()) {
				_pd_respond_to_src_caps();
				break;
			}
			if (!SMOL_PD_EPR_ENABLE)
				break;
			if (_epr_mode) {
				// SinkEPRKeepAliveTimer
				if ((now_us > _last_traffic_us + PD_EPR_KEEPALIVE_US) && (_ext_tx.size == 0) && _pd_sink_tx_ok()) {
					_pd_send_extended_control(PD_ECDB_EPR_KEEPALIVE);
					_pd_set_state(SMOL_USB_PD_EPR_KEEPALIVE_SENT);
				}
			}
			else if (_src_epr_capable && (_epr_entry_attempts < PD_EPR_ENTRY_ATTEMPTS) && !_contract_is_epr
				&& (in_state_us > PD_EPR_ENTRY_DELAY_US) && (now_us > _last_traffic_us + PD_EPR_ENTRY_DELAY_US)
				&& _pd_sink_tx_ok()) {
				_pd_send_epr_enter();
			}
			break;
		case SMOL_USB_PD_EPR_KEEPALIVE_SENT:
			// SenderResponseTimer
			if (in_state_us > PD_SENDER_RESPONSE_US)
				_pd_send_hard_reset("no EPR_KeepAlive_Ack");
			break;
		default:
			break;
	}
}

/**
 * Idle function for PD negotiations. Statemachine and RX checks.
 *
 * Since we're not running the FUSB302 in "autonomous" mode, and with the VUSB sense pin connected to the TPD4S480's VBUS_LV pin, we have to manually read out VUSB to detect attach/detach.
 */
void smol_usb_pd_run(void) {
	bool have_irq = fusb302_have_irq();

	if (have_irq) {
		uint8_t interrupta = 0;
		fusb302_read_reg(FUSB_REG_INTERRUPTA, &interrupta);
		uint8_t interruptb = 0;
		fusb302_read_reg(FUSB_REG_INTERRUPTB, &interruptb);
		uint8_t interrupt = 0;
		fusb302_read_reg(FUSB_REG_INTERRUPT, &interrupt);

		bool vbus_ok = (interrupt >> 7) & 0x01;
		bool activity = (interrupt >> 6) & 0x01;
		bool comp_chng = (interrupt >> 5) & 0x01;
		bool crc_chk = (interrupt >> 4) & 0x01;
		bool alert = (interrupt >> 3) & 0x01;
		bool wake = (interrupt >> 2) & 0x01;
		bool collision = (interrupt >> 1) & 0x01;
		bool bc_lvl = (interrupt >> 0) & 0x01;
		bool overtemp = (interrupta >> 7) & 0x01;
		bool togdone = (interrupta >> 6) & 0x01;
		bool soft_fail = (interrupta >> 5) & 0x01;
		bool retry_fail = (interrupta >> 4) & 0x01;
		bool hard_sent = (interrupta >> 3) & 0x01;
		bool tx_sent = (interrupta >> 2) & 0x01;
		bool soft_rst = (interrupta >> 1) & 0x01;
		bool hard_rst = (interrupta >> 0) & 0x01;
		bool gcrc_sent = (interruptb >> 0) & 0x01;

		if (vbus_ok)
			PD_IRQ_DBG("USB PD I_VBUSOK");
		if (activity)
			PD_IRQ_DBG("USB PD I_ACTIVITY");
		if (comp_chng)
			PD_IRQ_DBG("USB PD I_COMP_CHNG");
		if (crc_chk)
			PD_IRQ_DBG("USB PD I_CRC_CHK");
		if (alert)
			PD_IRQ_DBG("USB PD I_ALERT");
		if (wake)
			PD_IRQ_DBG("USB PD I_WAKE");
		if (collision)
			dbg_println("USB PD I_COLLISION");
		if (bc_lvl)
			PD_IRQ_DBG("USB PD I_BC_LVL");
		if (overtemp)
			dbg_println("USB PD I_OVERTEMP");
		if (togdone)
			PD_IRQ_DBG("USB PD I_TOGDONE");
		if (soft_fail)
			dbg_println("USB PD I_SOFTFAIL");
		if (retry_fail)
			dbg_println("USB PD I_RETRYFAIL");
		if (hard_sent)
			dbg_println("USB PD I_HARDSENT");
		if (tx_sent)
			PD_IRQ_DBG("USB PD I_TXSENT");
		if (soft_rst)
			dbg_println("USB PD I_SOFTRST");
		if (hard_rst) {
			dbg_println("USB PD I_HARDRST");
			_pd_hard_reset();
		}
		if (gcrc_sent)
			PD_IRQ_DBG("USB PD I_GCRCSENT");
	}

    uint8_t status1 = 0;
    assert(0 == fusb302_read_reg(FUSB_REG_STATUS1, &status1));

    bool have_rx = (0 == (status1 & FUSB_STATUS1_RX_EMPTY_MASK));

	if (have_rx) {
		// start reading token
		uint64_t rx_start_us = smol_time64(timer0_hw);
		uint8_t rx_token = 0;
		assert(0 == fusb302_rx(&rx_token, 1));
		uint32_t token = rx_token >> 5;
		bool is_sop = token == PD_TOKEN_SOP;

		/**
		 * The RX FIFO holds: token, 2 header bytes, 4 bytes per data object, 4 CRC bytes.
		 * Read the full message in one burst, so the next message starts at a token byte.
		 * Do not print before the response is sent: the sink has only tSenderResponse (~24ms) to send a Request, and tChunkReceiverRequest (15ms) to send a Chunk Request.
		 */
		uint8_t msg[2 + 7*4 + 4];
		fusb302_rx(msg, 2);
		uint16_t header = msg[0] | ((uint16_t)msg[1] << 8);
		bool is_extended_message = 0 != (header & 0x8000);
		size_t command = (header >> 0) & 0x1f;
		size_t num_data_objects = (header >> 12) & 0x07;
		fusb302_rx(msg + 2, 4*num_data_objects + 4);

		bool is_control = (num_data_objects == 0) && !is_extended_message;
		bool is_good_crc = is_control && (command == PD_CMD_GOOD_CRC);
		bool is_soft_reset = is_control && (command == PD_CMD_SOFT_RESET);
		int msgid = (header >> 9) & 0x7;
		bool is_duplicate = false;
		if (is_sop && !is_good_crc) {
			// A retransmitted message has the same MessageID: GoodCRC is sent again by AUTO_CRC, but the message is not processed again. Soft_Reset is always processed.
			is_duplicate = !is_soft_reset && (msgid == _last_rx_msgid);
			_last_rx_msgid = msgid;
			_last_traffic_us = rx_start_us;
		}
		uint16_t ext_header = 0;
		if (is_extended_message && (num_data_objects > 0))
			ext_header = msg[2] | ((uint16_t)msg[3] << 8);

		// a control or data message ends a Chunked transmit. _pd_rx_extended() handles the extended messages.
		if (is_sop && !is_duplicate && !is_good_crc && !is_extended_message)
			_ext_tx.size = 0;

		if (!is_sop || is_duplicate || is_good_crc) {
			// SOP' / SOP'' messages go to the cable, ignore them
		}
		else if (_pd_epr_entry_interrupted(is_extended_message, num_data_objects, command)) {
			_pd_epr_entry_failed("other message during entry");
		}
		else if (is_extended_message) {
			if (num_data_objects > 0)
				_pd_rx_extended(command, ext_header, msg + 4, 4*num_data_objects - 2);
		}
		else if (num_data_objects == 0) {
			_parse_pd_command(command);
		}
		else
		{
			_parse_pd_data_message(header, command, msg + 2, num_data_objects);
		}

		uint32_t rx_handling_us = (uint32_t)(smol_time64(timer0_hw) - rx_start_us);
		if (!is_good_crc)
			dbg_println("PD RX token 0x%x header 0x%04x cmd %u n %u ext %u (0x%04x) id %d%s, handled in %u us", token, header, command, num_data_objects, is_extended_message, ext_header, msgid, is_duplicate ? " (duplicate)" : "", rx_handling_us);
	}

	/**
	 * NOTE: Do not call fusb302_measure_vbus() here. It disconnects the CC measure switch for each call, and the BMC receiver then misses messages.
	 */

	// read the time after the RX handling: a response in the RX FIFO is processed before its timer expires
	uint64_t now_us = smol_time64(timer0_hw);
	_pd_run_timers(now_us);
	_pd_check_attach_detach(now_us);
}
