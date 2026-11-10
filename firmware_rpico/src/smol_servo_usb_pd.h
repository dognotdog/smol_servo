#pragma once

#include <stdint.h>

/**
 * USB PD sink: run-time contract voltage. Call these functions on core 0 only (the core that runs smol_usb_pd_run()).
 * smol_servo.h declares smol_usb_pd_init(), smol_usb_pd_run(), smol_usb_pd_run_startup() and smol_usb_pd_contract_mv().
 */

// lowest voltage that smol_usb_pd_request_mv() accepts. The TMC6200 needs at least 8V.
#define SMOL_USB_PD_REQUEST_MIN_MV (9000)

typedef enum {
	SMOL_USB_PD_REQ_OK = 0,
	// the voltage is below SMOL_USB_PD_REQUEST_MIN_MV or above the board maximum
	SMOL_USB_PD_REQ_BAD_VOLTAGE = -1,
	// there is no Explicit Contract, or a contract negotiation runs
	SMOL_USB_PD_REQ_NO_CONTRACT = -2,
	// no Fixed PDO has this voltage, and no PPS / AVS APDO has this voltage in its range
	SMOL_USB_PD_REQ_NO_PDO = -3,
} smol_usb_pd_req_result_t;

/**
 * Request a new Explicit Contract at target_mv, from the present (EPR_)Source_Capabilities.
 * A Fixed PDO with this voltage has priority. Then a PPS or AVS APDO with this voltage in its range.
 * target_mv = 0: the boot default (the highest voltage).
 * The sink sends the Request from smol_usb_pd_run(). smol_usb_pd_contract_mv() gives the new voltage after PS_RDY.
 * The target stays in effect for all later Source_Capabilities, until the next call. When the Source rejects the Request, the sink keeps the present contract and the target before this call.
 * If the function does not return SMOL_USB_PD_REQ_OK, the contract and the target do not change.
 */
smol_usb_pd_req_result_t smol_usb_pd_request_mv(uint32_t target_mv);

// target voltage of smol_usb_pd_request_mv(), 0 = boot default
uint32_t smol_usb_pd_target_mv(void);

// Print the source capabilities, the present contract and the target to the log.
void smol_usb_pd_print_status(void);
