#include "smol_servo.h"
#include "smol_fast.h"

#include "fusb302.h"
#include "pico_debug.h"

#include <string.h>

/**
 * The TMC6200 needs at least 8V to work with, so USP PD of 9V.
 * 
 * We have the FUSB302 connected through a TPD4S480, which means it is getting a reduced voltage by a factor of 0.42 to account for ESR 48V supplies. This means that likely VBUSOK will never trigger, but that's ok because we can measure it.
 * 
 * The default negotiation is requesting at least 9V at the highest power envelope.
 * 
 * If we already have a VBUS we will not request higher voltage over USB. 
 * 
 */

#define VBUS_LV_RATIO (0.42f)
#define PD_VBUS_MIN_THRESHOLD 4.5f

// timeout for initial power negotation
#define PD_ATTACH_TIMEOUT_US (5000000)
// USB PD message header Spec Revision field [6:5]: 00=Rev1.0, 01=Rev2.0, 10=Rev3.0/3.1
#define PD_SPEC_REV_30  2

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
	SMOL_USB_PD_CABLE_ATTACHED,
	SMOL_USB_PD_REQUEST_SENDING,
	SMOL_USB_PD_REQUEST_SENT,
	SMOL_USB_PD_REQUEST_ACCEPTED,
	SMOL_USB_PD_REQUEST_REJECTED,

	SMOL_USB_PD_SEND_SOFT_RESET,
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



static smol_usb_pd_state_t m_usb_pd_state = SMOL_USB_PD_IDLE;
static uint64_t _attach_time_us = 0;
static size_t _cc_index = 0;
static uint32_t _msg_counter = 0;

static void _pd_attach(void) {
	// reset PD machinery, then connect the BMC transceiver with AUTO_CRC to the active CC pin
	fusb302_write_reg(FUSB_REG_RESET, FUSB_RESET_PD_MASK);
	fusb302_set_pd_cc(_cc_index);
	_msg_counter = 0;
	_attach_time_us = smol_time64(timer0_hw);
	m_usb_pd_state = SMOL_USB_PD_CABLE_ATTACHED;
}

void smol_usb_pd_init(void) {
	// float vbus_lv = 0.0f;
	// fusb302_measure_vbus(&vbus_lv);
	// float vbus = vbus_lv * VBUS_LV_RATIO;
	// info_println("VUSB = %.3f", vbus);

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
		_pd_attach();
	}

	// float cc1, cc2;
	// fusb302_measure_cc(&cc1, &cc2);
	// info_println("CC1 = %.3f, CC2 = %0.3f", cc1, cc2);
}

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
} src_cap_t;

#define MAX_NUM_SRC_CAPS (7+4) // SPR + EPR

static src_cap_t _src_capabilities[MAX_NUM_SRC_CAPS] = {};
static pd_preference_t _pd_power_preference = PD_PREFERENCE_MAX_POWER;

static void _pd_tx_header(uint32_t cmd, size_t num_objects) {
	uint8_t sop[4] = {FUSB_FIFO_TX_SYNC1, FUSB_FIFO_TX_SYNC1, FUSB_FIFO_TX_SYNC1, FUSB_FIFO_TX_SYNC2};
	fusb302_tx(sop, sizeof(sop));
	uint8_t packsym = FUSB_FIFO_TX_PACKSYM | (2 + 4*num_objects);
	fusb302_tx(&packsym, 1);
	uint16_t header = 0;
	header |= PD_SPEC_REV_30 << 5;
	header |= ((_msg_counter++) & 0x7) << 9;
	header |= num_objects << 12;
	header |= cmd << 0;
	fusb302_tx(&header, 2);
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
		// APDOs (PPS, AVS) need a different RDO format, which is not implemented yet.
		if (_src_capabilities[i].is_apdo)
			continue;
		// need at least 9V, so variable supplies are not allowed to dip below.
		uint32_t voltage = _src_capabilities[i].is_variable ? _src_capabilities[i].min_voltage : _src_capabilities[i].max_voltage;
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

	best_voltage = _src_capabilities[pdo_index].is_variable ? _src_capabilities[pdo_index].min_voltage : _src_capabilities[pdo_index].max_voltage;

	uint32_t request_obj = 0;
	request_obj |= (pdo_index + 1) << 28;  // PD object positions are 1-based; array index 0 = PDO position 1
	request_obj |= ((best_voltage < 9000) && !already_have_vbus) << 26; // cap mismatch
	request_obj |= 1u << 25; // USB comms
	request_obj |= 1u << 24; // no SUSPEND
	request_obj |= 0u << 23; // no unchunked ext messages
	request_obj |= 1u << 22; // EPR capable
	if (_src_capabilities[pdo_index].is_battery) {
		request_obj |= (_src_capabilities[pdo_index].max_power / 250) << 10;
		request_obj |= (_src_capabilities[pdo_index].max_power / 250) << 0;
	}
	else
	{
		request_obj |= (_src_capabilities[pdo_index].max_current / 10) << 10;
		request_obj |= (_src_capabilities[pdo_index].max_current / 10) << 0;
	}


	// craft message
	_pd_tx_header(PD_DATA_REQUEST, 1);
	_pd_tx_object(request_obj);
	_pd_tx_send();

	m_usb_pd_state = SMOL_USB_PD_REQUEST_SENT;
	_attach_time_us = smol_time64(timer0_hw);

	dbg_println("PD Request 0x%08x: PDO %u, %umV, %umA", request_obj, pdo_index + 1, best_voltage, _src_capabilities[pdo_index].max_current);
}

static void _pd_hard_reset(void) {
	// reset source caps
	memset(_src_capabilities, 0, sizeof(_src_capabilities));

	// reset PD machinery. The chip re-init clears SWITCHES0/1, so attach again to restore the CC setup.
	fusb302_hard_reset();
	if (_cc_index != 0)
		_pd_attach();
}

static void _parse_pd_command(bool is_sop, uint32_t cmd) {
	switch(cmd) {
		case PD_CMD_GOOD_CRC:
			break;
		case PD_CMD_ACCEPT:
			if (m_usb_pd_state == SMOL_USB_PD_REQUEST_SENT) {
				m_usb_pd_state = SMOL_USB_PD_REQUEST_ACCEPTED;
				dbg_println("PD request accepted");
			}
			break;
		case PD_CMD_REJECT:
		case PD_CMD_WAIT:
			if (m_usb_pd_state == SMOL_USB_PD_REQUEST_SENT) {
				m_usb_pd_state = SMOL_USB_PD_REQUEST_REJECTED;
				warn_println("PD request rejected (cmd %u)", cmd);
			}
			break;
		case PD_CMD_PS_READY:
			dbg_println("PD PS_RDY, contract established");
			break;
		case PD_CMD_SOFT_RESET:
			// the source resets the protocol layer: accept, then wait for new Source_Capabilities
			_msg_counter = 0;
			_pd_tx_header(PD_CMD_ACCEPT, 0);
			_pd_tx_send();
			m_usb_pd_state = SMOL_USB_PD_CABLE_ATTACHED;
			_attach_time_us = smol_time64(timer0_hw);
			dbg_println("PD soft reset received, accepted");
			break;
		case PD_CMD_GET_SINK_CAP:
		{
			uint32_t safe5vcap = 0;
			safe5vcap |= PDO_FIXED << 30;
			safe5vcap |= 1 << 28; // higher capability
			safe5vcap |= 1 << 26; // USB comms capable
			safe5vcap |= (5000/50) << 10; // voltage
			safe5vcap |= (100/10) << 0; // current

			uint32_t varcap = 0;
			varcap |= PDO_VARIABLE << 30;
			varcap |= (21000/50) << 20; // max voltage
			varcap |= (9000/50) << 10; // min voltage
			varcap |= (5000/10) << 0; // current

			uint32_t batcap = 0;
			batcap |= PDO_BATTERY << 30;
			batcap |= (21000/50) << 20; // max voltage
			batcap |= (9000/50) << 10; // min voltage
			batcap |= (240000/250) << 0; // power

			uint32_t spr_prog_cap = 0;
			spr_prog_cap |= PDO_APDO << 30;
			spr_prog_cap |= APDO_SPR_PROGRAMMABLE << 28;
			spr_prog_cap |= (21000/100) << 17; // max voltage
			spr_prog_cap |= (9000/100) << 8; // min voltage
			spr_prog_cap |= (5000/50) << 0; // current

			uint32_t epr_adj_cap = 0;
			epr_adj_cap |= PDO_APDO << 30;
			epr_adj_cap |= APDO_EPR_ADJUSTABLE << 28;
			epr_adj_cap |= (48000/100) << 17; // max voltage
			epr_adj_cap |= (9000/100) << 8; // min voltage
			epr_adj_cap |= (240000/1000) << 0; // power

			_pd_tx_header(PD_DATA_SINK_CAPS, 5);
			_pd_tx_object(safe5vcap);
			_pd_tx_object(varcap);
			_pd_tx_object(batcap);
			_pd_tx_object(spr_prog_cap);
			_pd_tx_object(epr_adj_cap);
			_pd_tx_send();

			break;
		}
	}
}

static void _pd_respond_to_chunk_request(uint32_t cmd, size_t n) {
	assert(0);
}

static void _pd_soft_reset(void) {
	// a Soft_Reset message always uses MessageID 0
	_msg_counter = 0;
	_pd_tx_header(PD_CMD_SOFT_RESET, 0);
	_pd_tx_send();
	_attach_time_us = smol_time64(timer0_hw);
	m_usb_pd_state = SMOL_USB_PD_CABLE_ATTACHED;
}

static void _parse_pd_data_object(bool is_sop, uint32_t cmd, uint8_t obj[4], size_t i) {
	switch(cmd) {
		case PD_DATA_SOURCE_CAPS:
		{
			// we can receive up to 7 capabilities
			uint32_t capability;
			memcpy(&capability, obj, sizeof(capability));
			uint32_t pdo_type = (capability >> 30) & 0x3;

			src_cap_t src_cap = {};

			// info_println("PD RX cap [%u]", i);

			// info_println("  PDO type %u", pdo_type);

			if (pdo_type == PDO_APDO) // APDO
			{
				src_cap.is_apdo = true;
				uint32_t apdo_type = (capability >> 28) & 0x3;
				// info_println("  APDO type %u", apdo_type);

				if (apdo_type == APDO_SPR_PROGRAMMABLE)
				{
					uint32_t voltage_max_100m = (capability >> 17) & 0xFF;
					uint32_t voltage_min_100m = (capability >> 8) & 0xFF;
					uint32_t max_current_50m = (capability >> 0) & 0x7F;
					src_cap.min_voltage = 100*voltage_min_100m;
					src_cap.max_voltage = 100*voltage_max_100m;
					src_cap.max_current = 50*max_current_50m;
					src_cap.max_power = (src_cap.max_voltage * src_cap.max_current) / 1000;
				}
				else if (apdo_type == APDO_EPR_ADJUSTABLE)
				{
					// uint32_t overload = (capability >> 26) & 0x3;
					uint32_t voltage_max_100m = (capability >> 17) & 0x1FF;
					uint32_t voltage_min_100m = (capability >> 8) & 0xFF;
					uint32_t max_power_1000m = (capability >> 0) & 0xFF;
					src_cap.min_voltage = 100*voltage_min_100m;
					src_cap.max_voltage = 100*voltage_max_100m;
					src_cap.max_power = 1000*max_power_1000m;
				}
				else if (apdo_type == APDO_SPR_ADJUSTABLE)
				{
					// uint32_t overload = (capability >> 26) & 0x3;
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
				}

			}
			else if (pdo_type == PDO_FIXED)
			{
				uint32_t epr_capable = (capability >> 23) & 0x1;
				// uint32_t overload = (capability >> 20) & 0x3;
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

			// info_println("  VMIN %umV", src_cap.min_voltage);
			// info_println("  VMAX %umV", src_cap.max_voltage);
			// info_println("  IMAX %umA", src_cap.max_current);
			// info_println("  PMAX %umW", src_cap.max_power);

			_src_capabilities[i] = src_cap;


			break;
		}
	}

}

/**
 * Idle function for PD negotiations. Statemachine and RX checks.
 * 
 * Since we're not running the FUSB302 in "autonomous" mode, and with the VUSB sense pin connected to the TPD4S480's VBUS_LV pin, we have to manually read out VUSB to detect attach/detach.
 */
void smol_usb_pd_run(void) {
	uint64_t now_us = smol_time64(timer0_hw);

	bool have_irq = fusb302_have_irq();

	if (have_irq) {
		// dbg_println("USB PD have_irq");
		uint8_t interrupta = 0;
		fusb302_read_reg(FUSB_REG_INTERRUPTA, &interrupta);
		uint8_t interruptb = 0;
		fusb302_read_reg(FUSB_REG_INTERRUPTB, &interruptb);
		uint8_t interrupt = 0;
		fusb302_read_reg(FUSB_REG_INTERRUPT, &interrupt);

		// dbg_println("USB PD INTERRUPT  0x%02x", interrupt);
		// dbg_println("USB PD INTERRUPTA 0x%02x", interrupta);
		// dbg_println("USB PD INTERRUPTB 0x%02x", interruptb);

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

		if (vbus_ok) {
			PD_IRQ_DBG("USB PD I_VBUSOK");
			// fusb302_write_reg(FUSB_REG_INTERRUPT, 1u << 7);
		}
		if (activity) {
			PD_IRQ_DBG("USB PD I_ACTIVITY");
			// fusb302_write_reg(FUSB_REG_INTERRUPT, 1u << 6);
		}
		if (comp_chng) {
			PD_IRQ_DBG("USB PD I_COMP_CHNG");
			// fusb302_write_reg(FUSB_REG_INTERRUPT, 1u << 5);
		}
		if (crc_chk) {
			PD_IRQ_DBG("USB PD I_CRC_CHK");
			// fusb302_write_reg(FUSB_REG_INTERRUPT, 1u << 4);
		}
		if (alert) {
			PD_IRQ_DBG("USB PD I_ALERT");
			// fusb302_write_reg(FUSB_REG_INTERRUPT, 1u << 3);
		}
		if (wake) {
			PD_IRQ_DBG("USB PD I_WAKE");
			// fusb302_write_reg(FUSB_REG_INTERRUPT, 1u << 2);
		}
		if (collision) {
			dbg_println("USB PD I_COLLISION");
			// fusb302_write_reg(FUSB_REG_INTERRUPT, 1u << 1);
		}
		if (bc_lvl) {
			PD_IRQ_DBG("USB PD I_BC_LVL");
			// fusb302_write_reg(FUSB_REG_INTERRUPT, 1u << 0);
		}
		if (overtemp) {
			dbg_println("USB PD I_OVERTEMP");
			// fusb302_write_reg(FUSB_REG_INTERRUPTA, 1u << 7);
		}
		if (togdone) {
			PD_IRQ_DBG("USB PD I_TOGDONE");
			// fusb302_write_reg(FUSB_REG_INTERRUPTA, 1u << 6);
		}
		if (soft_fail) {
			dbg_println("USB PD I_SOFTFAIL");
			// fusb302_write_reg(FUSB_REG_INTERRUPTA, 1u << 5);
		}
		if (retry_fail) {
			dbg_println("USB PD I_RETRYFAIL");
			// _pd_soft_reset();
			// fusb302_write_reg(FUSB_REG_INTERRUPTA, 1u << 4);
		}
		if (hard_sent) {
			dbg_println("USB PD I_HARDSENT");
			// fusb302_write_reg(FUSB_REG_INTERRUPTA, 1u << 3);
		}
		if (tx_sent) {
			PD_IRQ_DBG("USB PD I_TXSENT");
			// fusb302_write_reg(FUSB_REG_INTERRUPTA, 1u << 2);
		}
		if (soft_rst) {
			dbg_println("USB PD I_SOFTRST");
			// fusb302_write_reg(FUSB_REG_INTERRUPTA, 1u << 1);
		}
		if (hard_rst) {
			dbg_println("USB PD I_HARDRST");
			// fusb302_write_reg(FUSB_REG_INTERRUPTA, 1u << 0);

			_pd_hard_reset();

		}
		if (gcrc_sent) {
			PD_IRQ_DBG("USB PD I_GCRCSENT");
			// fusb302_write_reg(FUSB_REG_INTERRUPTB, 1u << 0);
		}
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
		 * Do not print before the response is sent: the sink has only tSenderResponse (~24ms) to send a Request.
		 */
		uint8_t msg[2 + 7*4 + 4];
		fusb302_rx(msg, 2);
		uint16_t header = msg[0] | ((uint16_t)msg[1] << 8);
		bool is_extended_message = 0 != (header & 0x8000);
		size_t command = (header >> 0) & 0x1f;
		size_t num_data_objects = (header >> 12) & 0x07;
		fusb302_rx(msg + 2, 4*num_data_objects + 4);

		if (!is_sop) {
			// SOP' / SOP'' messages go to the cable, ignore them
		}
		else if (is_extended_message) {
			// TODO: support extended messages. The data is drained above.
			uint16_t ext_header = msg[2] | ((uint16_t)msg[3] << 8);
			bool is_chunked = (ext_header >> 15) & 0x1;
			bool request_chunk = is_chunked && ((ext_header >> 10) & 0x1);
			if (request_chunk) {
				_pd_respond_to_chunk_request(command, (ext_header >> 11) & 0xF);
			}
		}
		else if (num_data_objects == 0) {
			_parse_pd_command(is_sop, command);
		}
		else
		{
			if (command == PD_DATA_SOURCE_CAPS)
				memset(_src_capabilities, 0, sizeof(_src_capabilities));

			for (size_t i = 0; i < num_data_objects; ++i) {
				_parse_pd_data_object(is_sop, command, msg + 2 + 4*i, i);
			}

			switch(command) {
				case PD_DATA_SOURCE_CAPS:
					_pd_respond_to_src_caps();
					break;
			}

		}

		uint32_t rx_handling_us = (uint32_t)(smol_time64(timer0_hw) - rx_start_us);
		dbg_println("PD RX token 0x%x header 0x%04x cmd %u n %u ext %u, handled in %u us", token, header, command, num_data_objects, is_extended_message, rx_handling_us);
	}

	/**
	 * NOTE: Do not call fusb302_measure_vbus() here. It disconnects the CC measure switch for each call, and the BMC receiver then misses messages. VUSB detach detection needs a different approach (eg. the VBUSOK interrupt or the ADC).
	 */

	bool waiting = (m_usb_pd_state == SMOL_USB_PD_CABLE_ATTACHED) || (m_usb_pd_state == SMOL_USB_PD_REQUEST_SENT);
	if (waiting && (now_us > _attach_time_us + PD_ATTACH_TIMEOUT_US)) {
		// if there is no comms after cable was attached, or no reply to the request, issue a soft reset.
		dbg_println("PD issuing soft reset due to timeout (state %u)...", m_usb_pd_state);
		_pd_soft_reset();
	}

}