
#include "ssf_mctrl_private.h"
#include "ssf_mctrl.h"

#include "ssf_spi.h"
#include "ssf_main.h"

#include "debug.h"

#include <string.h>
#include <assert.h>

static sspi_tmc_state_t _tmcState = {
	.GCONF = {.reg = 0,},
};

void mctrl_diag_tmc6200_gstat_rx_callback(uint8_t rx[5])
{
	_tmcState.GSTAT.reg = ((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 8) | ((uint32_t)rx[4] << 0);
}

void mctrl_diag_tmc6200_ioin_rx_callback(uint8_t rx[5])
{
	_tmcState.IOIN.reg = ((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 8) | ((uint32_t)rx[4] << 0);	
}

void mctrl_diag_tmc6200_gconf_rx_callback(uint8_t rx[5])
{
	_tmcState.GCONF.reg = ((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 8) | ((uint32_t)rx[4] << 0);	
}


static void mctrl_diag_idle(uint32_t now_us)
{
	switch (sspi_drvType())
	{
		case SSPI_DEVICE_TMC6200:
		{
			// this assumes we've got the fastloop readings going
			if (_tmcState.GSTAT.reset)
				dbg_println("TMC6200 GSTAT.reset = 1");
			if (_tmcState.GSTAT.drv_optw)
				warn_println("TMC6200 GSTAT.drv_optw = 1");
			if (_tmcState.GSTAT.drv_ot)
				err_println("TMC6200 GSTAT.drv_ot = 1");
			if (_tmcState.GSTAT.uv_cp)
				err_println("TMC6200 GSTAT.uv_cp = 1");
			if (_tmcState.GSTAT.shortdet_u)
				err_println("TMC6200 GSTAT.shortdet_u = 1");
			if (_tmcState.GSTAT.shortdet_v)
				err_println("TMC6200 GSTAT.shortdet_v = 1");
			if (_tmcState.GSTAT.shortdet_w)
				err_println("TMC6200 GSTAT.shortdet_w = 1");

			// if (!_tmcState.IOIN.DRV_EN)
			// 	warn_println("TMC6200 IOIN.DRV_EN = 0");
			if (_tmcState.IOIN.OTPW)
				warn_println("TMC6200 IOIN.OTPW = 1");

			if (_tmcState.GCONF.disable)
				warn_println("TMC6200 GCONF.disable = 1");
			// if (!_tmcState.GCONF.singleline)
			// 	warn_println("TMC6200 GCONF.singleline = 0");

			break;
		}
		default:
		{
			break;
		}
	}
}

static const char* _bridgeStateString(int state) 
{
	switch (state)
	{
		case MCTRL_BRIDGE_LO:
			return "LO";
		case MCTRL_BRIDGE_HI:
			return "HI";
		case MCTRL_BRIDGE_ZZ:
			return "ZZ";
		default:
			return "??";
	}
}

static void _printBridgeState(int id)
{
	dbg_println(
		"A: %s, B: %s, C: %s", 
		_bridgeStateString(mctrl_params.sysId.idSequence[id][0]), 
		_bridgeStateString(mctrl_params.sysId.idSequence[id][1]), 
		_bridgeStateString(mctrl_params.sysId.idSequence[id][2])
	);

}

extern uint16_t an1_buf[6];


void mctrl_idle(uint32_t now_us)
{
	static uint32_t waitStart = 0;

	static float RestSum = 0.0f;
	static float RestSqrSum = 0.0f;
	static size_t k = 0;

	static float inductanceEstimate = 0.0f;
	static float inductanceVariance = 0.0f;

	uint32_t waitElapsed = now_us - waitStart;


	mctrl_diag_idle(now_us);

	// assert((mctrl.debug.lastEventCount == 0) || (mctrl.debug.lastEventCountDelta == 6));

	switch(mctrl_state)
	{
		case MCTRL_INIT:
		{
			// initial state, do nothing
			break;
		}
		case MCTRL_DRIVER_RESET_START:
		{
			sspi_enableFastloopReads(false);

			spwm_enableHalfBridges(0x0);
			spwm_setDrvChannel(HTIM_DRV_CH_R, 0.0f);
			spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0f);
			spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0f);
			spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0f);

			HAL_GPIO_WritePin(PIN_DRVEN, GPIO_PIN_RESET);
			waitStart = now_us;
			mctrl_state = MCTRL_DRIVER_RESET_WAIT;
			break;
		}
		case MCTRL_DRIVER_RESET_WAIT:
		{
			if (waitElapsed >= 2000)
			{
				HAL_GPIO_WritePin(PIN_DRVEN, GPIO_PIN_SET);
				waitStart = now_us;
				mctrl_state = MCTRL_DRIVER_DETECT_START;
			}
			break;
		}
		case MCTRL_DRIVER_RESET_COOLDOWN:
		{
			if (waitElapsed >= 3000000)
			{
				waitStart = now_us;
				mctrl_state = MCTRL_DRIVER_RESET_START;
			}
			break;
		}
		case MCTRL_DRIVER_DETECT_START:
		{

			// auto-detect available hardware

			if (sspi_detectAs5047d()) 
			{
				dbg_println("AS5047D detected!");
			}
			else
			{
				warn_println("AS5047D not detected!");
			}

			if (sspi_detectTmc6200()) 
			{
				dbg_println("TMC6200 detected!");
			}
			else if (sspi_detectDrv83xx()) 
			{
				dbg_println("DRV83xx detected!");
			}
			else
			{
				warn_println("No gate driver detected!");
			}

			// init analog stuff after driver is known for proper configuration
			mctrl.debug.lastEventCount = 0;
			ssfa_isenseAnalogInit();
			spwm_init();



			mctrl_state = MCTRL_DRIVER_INIT_START;
			break;
		}
		case MCTRL_DRIVER_TEST_START:
		{
			// this is being skipped right now
			// sspi_enableFastloopReads(true);
			break;
		}
		case MCTRL_DRIVER_INIT_START:
		{
			if (waitElapsed >= 2000)
			{
				// dbg_println("MCTRL_DRIVER_INIT_START");
				ssf_setMotorDriver3PwmMode();
				// sspi_drv_state_t drvState = ssf_readMotorDriver();
				// dbg_println("DRV8323 DRV_CTRL set to 0x%04X", drvState.DRV_CTRL);

				memset(mctrl.adcZeroCalibs, 0, sizeof(mctrl.adcZeroCalibs));
				mctrl.calibCounter = 0;

				// sspi_enableFastloopReads(true);

				waitStart = now_us;
				mctrl_state = MCTRL_DRIVER_INIT_WAIT;
			}
			break;
		}
		case MCTRL_DRIVER_INIT_WAIT:
		{
			if (waitElapsed >= 20000)
				mctrl_state = MCTRL_DRIVER_CALIB_ENTER;
			break;
		}
		case MCTRL_DRIVER_CALIB_ENTER:
		{
			if (sspi_drvType() == SSPI_DEVICE_DRV83XX)
			{
				// dbg_println("MCTRL_DRIVER_CALIB_ENTER");

				// first is analog recalibration
				// put the DRV8323 into calibration mode
				// switchover in DRV8323 takes 100us
				// do calibration twice, as otherwise we seem to get flip-flopping between ~2047 and ~2070 ADC counts, it is more stable this way, but not perfect, for some reason SOC still seems to be unstable
				ssf_enterMotorDriverCalibrationMode();

				waitStart = now_us;
				mctrl_state = MCTRL_DRIVER_CALIB_WAIT;
			}
			else
			{		
				// otherwise, skip ahead to ADC zero calibration
				mctrl_state = MCTRL_ANALOG_CALIBRATION_START;
			}
			break;
		}
		case MCTRL_DRIVER_CALIB_WAIT:
		{
			if (waitElapsed >= 2000)
				mctrl_state = MCTRL_DRIVER_CALIB_EXIT;
			break;
		}
		case MCTRL_DRIVER_CALIB_EXIT:
		{
			// dbg_println("MCTRL_DRIVER_CALIB_EXIT");
			ssf_exitMotorDriverCalibrationMode();
			waitStart = now_us;
			mctrl_state = MCTRL_DRIVER_CALIB_EXIT_WAIT;
			break;
		}
		case MCTRL_DRIVER_CALIB_EXIT_WAIT:
		{
			// dbg_println("MCTRL_DRIVER_CALIB_EXIT_WAIT...");
			// after driver chip done with calibration, go on to calibrate our ADC inputs
			if (waitElapsed >= 2000)
			{
				waitStart = now_us;
				dbg_println("MCTRL_DRIVER_CALIB_EXIT_WAIT done.");
				mctrl.calibCounter = 0;
				mctrl_state = MCTRL_ANALOG_CALIBRATION_RUN;
			}
			break;			
		}
		case MCTRL_ANALOG_CALIBRATION_START:
		{
			ssfa_isenseResetAdcZeroCalibration();
			
			mctrl_state = MCTRL_ANALOG_CALIBRATION_START_LO;
			break;
		}
		case MCTRL_ANALOG_CALIBRATION_START_LO:
		{
			waitStart = now_us;

			// set all bridges to low
			spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0f);
			spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0f);
			spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0f);

			spwm_enableHalfBridges(0x7);

			if (waitElapsed >= 10000)
			{
				mctrl_state = MCTRL_ANALOG_CALIBRATION_RUN_LO;
			}

			break;
		}
		case MCTRL_ANALOG_CALIBRATION_RUN_LO:
		{
			// dbg_println("MCTRL_ANALOG_CALIBRATION_RUN...");
			if (waitElapsed >= 1000000)
			{
				warn_println("MCTRL_ANALOG_CALIBRATION_RUN_LO more than a second passed!");
				// debugging output to figure out why ADC DMA is not run
				// dbg_println("HTIM_DRV->CNT = %u", HTIM_DRV->Instance->CNT);
				// dbg_println("TIM15->CNT = %u", TIM15->CNT);
				// dbg_println("TIM15->CC1 = %u", TIM15->CCR1);
				// dbg_println("TIM15->CC2 = %u", TIM15->CCR1);
				// dbg_println("TIM1->CNT = %u", TIM1->CNT);
				// dbg_println("ADC2->DR = %u", ADC2->DR);
				// dbg_println("an1_buf[0] = %u", an1_buf[0]);
				waitStart = now_us;
			};
			// do nothing, wait for fastLoop
			break;
		}
		case MCTRL_ANALOG_CALIBRATION_FINISH:
		{
			dbg_println("MCTRL_ANALOG_CALIBRATION_FINISH");
			// average out the reads for an adc count calibration
			bool calibOk = true;
			for (size_t i = 0; i < 3; ++i)
			{
				float calib = (mctrl.adcZeroCalibs[2*i]+mctrl.adcZeroCalibs[2*i+1])*(0.5f/mctrl.calibCounter);
				float vdda = ssf_getVdda();
				float maxc = ssfa_isenseMaxCount();
				float calibv = calib/maxc*vdda;

				// dbg_println("    calib0 = %.3f, calib1 = %.3f", (double)(_adcZeroCalibs[2*i]*(2.0f/CALIBREADS)), (double)(_adcZeroCalibs[2*i+1]*(2.0f/CALIBREADS)));
				
				mctrl.adcZeroCalibs[2*i] = calib;
				mctrl.adcZeroCalibs[2*i+1] = calib;

				dbg_println("SO[%u] calibrated to 0A = %.3f counts (%.3f V)", i, (double)calib, (double)calibv);

				switch (sspi_drvType())
				{
					case SSPI_DEVICE_DRV83XX:
					{
						// restart calibration if we're more than 5% off center
						float centerv = 0.5f*vdda;
						if (fabsf(calibv - centerv) > centerv*0.05f)
						{
							err_println("SO[%u] is too far off center at %.3f V (center = %.3f V)!", i, (double)calibv, (double)centerv);
							calibOk = false;
						}
						break;
					}
					case SSPI_DEVICE_TMC6200:
					{
						// analog center is at 5.0/3V
						// (500k - 250k resistor divider)
						float refv = 5.0f/3.0f;

						if (fabsf(calibv - refv) > 0.2f)
						{
							err_println("SO[%u] is too far off center at %.3f V (center = %.3f V)!", i, (double)calibv, (double)refv);
							calibOk = false;
						}

						break;
					}
					default:
					{
						err_println("Unknown gate driver type, don't know how to calibrate analog inputs.");
						calibOk = false;
						break;
					}
				}

			}

			if (!calibOk)
			{
				err_println("Restarting motor control init...");
				waitStart = now_us;
				mctrl_state = MCTRL_DRIVER_RESET_COOLDOWN;
				break;
			}

			// done with no-current stuff

			// enable driver chip
			if (sspi_drvType() == SSPI_DEVICE_TMC6200)
			{

			}

			// enable realtime readouts
			sspi_enableFastloopReads(true);

			mctrl.idRunCounter = 0;
			mctrl.calibCounter = 0;
			mctrl_state = MCTRL_ID_ALIGN_START;
			break;
		}
		case MCTRL_ID_ALIGN_START:
		case MCTRL_ID_ALIGN_WAIT:
		case MCTRL_ID_ALIGN_SETTLE:
		{
			// do nothing, wait for fastloop
			break;
		}
		case MCTRL_ID_ALIGN_FINISH:
		{
			mctrl_enableBridges(mctrl_params.sysId.idSequence[mctrl.idRunCounter]);

			// ballpark measure how fast current ramps up
			memset((void*)mctrl.lastMeasurement, 0, sizeof(mctrl.lastMeasurement));
			memset((void*)mctrl.lastVbus, 0, sizeof(mctrl.lastVbus));
			mctrl.calibCounter = 0;
			mctrl_state = MCTRL_RAMPTIME_ID_START;
			break;
		}
		case MCTRL_RAMPTIME_ID_START:
		case MCTRL_RAMPTIME_ID_WAIT:
		{
			// do nothing, wait for fastloop
			break;
		}
		case MCTRL_RAMPTIME_ID_FINISH:
		{
			size_t numSampleExceedingCurrentLimit = mctrl.calibCounter;

			const mctrl_bridge_activation_t* bridges = mctrl_params.sysId.idSequence[mctrl.idRunCounter];
			size_t loBridgeId = mctrl_loBridge(bridges);


			if (numSampleExceedingCurrentLimit >= NUM_STATIC_MEASUREMENTS)
			{
				{
					float i0 = mctrl.lastMeasurement[NUM_STATIC_MEASUREMENTS-1][(2*loBridgeId+0)];
					float i1 = mctrl.lastMeasurement[NUM_STATIC_MEASUREMENTS-1][(2*loBridgeId+1)];

					float i = 0.5*(i0+i1);

					float time = (NUM_STATIC_MEASUREMENTS-1)*(float)MEAS_FULL_PERIOD + (mctrl.idRunCounter % MCTRL_DRIVER_PHASES)*2*MEAS_SINGLE_PERIOD;
					dbg_println("could not reach %.3f, max current reached is %.3f after %8.0f us", (double)mctrl_params.sysId.maxCurrent, (double)i, (double)(time*1e6f));

				}
			}
			else
			{
				float i0 = mctrl.lastMeasurement[numSampleExceedingCurrentLimit][(2*loBridgeId+0)];
				float i1 = mctrl.lastMeasurement[numSampleExceedingCurrentLimit][(2*loBridgeId+1)];

				float i = 0.5*(i0+i1);
				float time = numSampleExceedingCurrentLimit*(float)MEAS_FULL_PERIOD + mctrl.idRunCounter*2*MEAS_SINGLE_PERIOD;

				dbg_println("reached %.3f after %8.0f us", (double)i, (double)(time*1e6f));
			}

			// prep next state
			RestSum = 0.0f;
			RestSqrSum = 0.0f;
			k = 0;
			mctrl_state = MCTRL_RESISTANCE_ID_PREPARE;
			break;
		}
		case MCTRL_RESISTANCE_ID_PREPARE:
		{
			// dbg_println("MCTRL_RESISTANCE_ID_PREPARE mctrl.idRunCounter %u, _calibCounter %u, k %u", mctrl.idRunCounter, _calibCounter, k);
			mctrl_params.sysId.staticIdentificationDutyCycle = 1.0f/ssf_getVbus();
			memset((void*)mctrl.lastMeasurement, 0, sizeof(mctrl.lastMeasurement));
			memset((void*)mctrl.lastVbus, 0, sizeof(mctrl.lastVbus));
			mctrl.calibCounter = 0;
			mctrl_state = MCTRL_RESISTANCE_ID_START;
			break;
		}
		case MCTRL_RESISTANCE_ID_START:
		case MCTRL_RESISTANCE_ID_RUN:
		{
			// do nothing, wait for fastloop
			break;
		}
		case MCTRL_RESISTANCE_ID_FINISH:
		{
			float i = 0.0f;
			float vbus = 0.0f;

			const mctrl_bridge_activation_t* bridges = mctrl_params.sysId.idSequence[mctrl.idRunCounter];
			size_t loBridgeId = mctrl_loBridge(bridges);

			for (size_t j = NUM_STATIC_MEASUREMENTS/2; j < NUM_STATIC_MEASUREMENTS; ++j)
			{
				float i0 = mctrl.lastMeasurement[j][(2*loBridgeId+0)];
				float i1 = mctrl.lastMeasurement[j][(2*loBridgeId+1)];

				i += 0.5f*(i0+i1);
				// i += i1;

				vbus += mctrl.lastVbus[j];

			}

			i *= 2.0f/NUM_STATIC_MEASUREMENTS;
			vbus *= 2.0f/NUM_STATIC_MEASUREMENTS;

			float dc = mctrl_params.sysId.staticIdentificationDutyCycle;
			float u = vbus;
			float R = u/i*dc;

			// dbg_println("steady state current is %1.3f A @ %3.3f %% of %7.3f V for %.3f R", (double)i, (double)(dc*100.0f), (double)u, (double)(R));

			RestSum += R;
			RestSqrSum += R*R;

			++k;

			if (k < MAX_IDENTIFICATION_REPEATS)
			{
				mctrl_state = MCTRL_RESISTANCE_ID_PREPARE;
			}
			else
			{
				float Rest = RestSum*(1.0f/k);
				float Rvar = RestSqrSum*(1.0f/k) - Rest*Rest;

				_printBridgeState(mctrl.idRunCounter);
				dbg_println("  RestSum %.3f , RestSqrSum %.3f", (double)RestSum, (double)(RestSqrSum));
				dbg_println("  steady state estimate %.3f R, sigma = %.3f", (double)Rest, (double)(sqrtf(Rvar)));

				mctrl.sysParamEstimates.phases.Rest[mctrl.idRunCounter] = Rest;
				mctrl.sysParamEstimates.phases.Rvar[mctrl.idRunCounter] = Rvar;

				k = 0;
				mctrl_state = MCTRL_IMPEDANCE_ID_PREPARE;
			}

			break;
		}
		case MCTRL_INDUCTANCE_ID_PREPARE:
		{
			// dbg_println("MCTRL_INDUCTANCE_ID_PREPARE mctrl.idRunCounter %u, _calibCounter %u, k %u", mctrl.idRunCounter, _calibCounter, k);
			mctrl_params.sysId.staticIdentificationDutyCycle = 1.0f/ssf_getVbus();
			memset((void*)mctrl.lastMeasurement, 0, sizeof(mctrl.lastMeasurement));
			memset((void*)mctrl.lastVbus, 0, sizeof(mctrl.lastVbus));
			mctrl.calibCounter = 0;
			mctrl_state = MCTRL_INDUCTANCE_ID_START;
			break;
		}
		case MCTRL_INDUCTANCE_ID_START:
		case MCTRL_INDUCTANCE_ID_RUN:
		{
			// wait for fastloop
			break;
		}
		case MCTRL_INDUCTANCE_ID_FINISH:
		{
			float Lest[NUM_ID_ALGO_ESTIMATES] = {};
			size_t numValidLMeasurements = 0;

			float R = mctrl.sysParamEstimates.phases.Rest[mctrl.idRunCounter];

			const mctrl_bridge_activation_t* bridges = mctrl_params.sysId.idSequence[mctrl.idRunCounter];
			size_t loBridgeId = mctrl_loBridge(bridges);

			for (size_t j = 1; j+NUM_ID_ALGO_SAMPLES-1 < NUM_STATIC_MEASUREMENTS; ++j)
			{
#if NUM_ID_ALGO_SAMPLES == 2
				// inductance estimation only based on previous R estimate
				const float h = MEAS_FULL_PERIOD;
				float vbus0 = mctrl.lastVbus[j+0];
				float vbus1 = mctrl.lastVbus[j+1];
				float vbus = 0.5*(vbus0 + vbus1);
				float i0 = mctrl.lastMeasurement[j+0][(2*loBridgeId)];
				float i0a = mctrl.lastMeasurement[j+0][(2*loBridgeId+1)];
				float i1 = mctrl.lastMeasurement[j+1][(2*loBridgeId)];
				float i1a = mctrl.lastMeasurement[j+1][(2*loBridgeId+1)];
				float di = (i1-i0)/h;
				float dia = (i1a-i0a)/h;


				float L0 = (vbus - 0.5f*(i0 + i1)*R) / di;
				float L1 = (vbus - 0.5f*(i0a+i1a)*R) / dia;

				if (fabsf(di) > FLT_EPSILON)
					Lest[numValidLMeasurements++] = L0;
				if (fabsf(dia) > FLT_EPSILON)
					Lest[numValidLMeasurements++] = L1;
#else
#error only 2 algo samples supported
#endif

			}

			float Lavg = 0.0;
			float Lvar = 0.0;

			for (size_t j = 0; j < numValidLMeasurements; ++j)
			{
				Lavg += Lest[j];
				Lvar += Lest[j]*Lest[j];
			}

			Lavg *= 1.0f/numValidLMeasurements;
			Lvar *= 1.0f/numValidLMeasurements;

			Lvar -= Lavg*Lavg;

			if (k == 0)
			{
				inductanceEstimate = Lavg;
				inductanceVariance = Lvar;	
			}
			else
			{
				// update estimate via Kalman Filter
				/*
				K = sigma1^2 / (sigma1^2 + sigma2^2)
				x = x1 + K (x2 - x1)
				sigma^2 = (1 - K) sigma1^2
				*/
				if (numValidLMeasurements > 1)
				{
					float KL = inductanceVariance / (inductanceVariance +  Lvar);
					inductanceEstimate += KL * (Lavg - inductanceEstimate);
					inductanceVariance *= 1.0f - KL;	
				}

				// dbg_println("  KL = %8.3f, KR = %8.3f", (double)(KL), (double)(KR));

			}

			++k;

			if (k < MAX_IDENTIFICATION_REPEATS)
			{
				mctrl_state = MCTRL_INDUCTANCE_ID_PREPARE;
			}
			else
			{
				mctrl.sysParamEstimates.phases.Lest[mctrl.idRunCounter] = inductanceEstimate;
				mctrl.sysParamEstimates.phases.Lvar[mctrl.idRunCounter] = inductanceVariance;

				dbg_println("  Lest[%u] is %8.3f mH, sigma = %8.3f mH", mctrl.idRunCounter, (double)(inductanceEstimate*1e3), (double)(sqrtf(inductanceVariance)*1e3));
				dbg_println("  Rest[%u] is %8.3f R,  sigma = %8.3f R", mctrl.idRunCounter, (double)(mctrl.sysParamEstimates.phases.Rest[mctrl.idRunCounter]*1e0), (double)(sqrtf(mctrl.sysParamEstimates.phases.Rvar[mctrl.idRunCounter])*1e0));
				dbg_println("   Tau[%u] is %8.3f ms", mctrl.idRunCounter, (double)(mctrl.sysParamEstimates.phases.Lest[mctrl.idRunCounter]/mctrl.sysParamEstimates.phases.Rest[mctrl.idRunCounter]*1e3));

				k = 0;
				mctrl_state = MCTRL_IMPEDANCE_ID_PREPARE;
			}

			break;
		}
		case MCTRL_IMPEDANCE_ID_PREPARE:
		{
			// dbg_println("MCTRL_INDUCTANCE_ID_PREPARE mctrl.idRunCounter %u, _calibCounter %u, k %u", mctrl.idRunCounter, _calibCounter, k);
			float pwmScale = mctrl_params.sysId.staticIdentificationDutyCycle;
			for (size_t i = 0; i < NUM_PHASE_MEASUREMENTS; ++i)
			{
				float x = 2.0f*M_PI*(i+0.5f)/NUM_PHASE_MEASUREMENTS;
				float y = sintab(x);
				mctrl.phasePwm[i] = y*pwmScale;
			}

			memset((void*)mctrl.phaseCurrents, 0, sizeof(mctrl.phaseCurrents));

			mctrl.calibCounter = 0;
			mctrl_state = MCTRL_IMPEDANCE_ID_START;
			break;
		}
		case MCTRL_IMPEDANCE_ID_START:
		case MCTRL_IMPEDANCE_ID_RUN:
		{
			// do nothing, wait for fastloop
			break;
		}
		case MCTRL_IMPEDANCE_ID_FINISH:
		{
			// TODO: looking at the data, it seems like the 2nd reading of iSense cycle is good, the first is garbage, why is that?
			// Interestingly, even the garbage looking readings average to zero, so at least that's good. It might be switching interference, as the test pwm duty cycle was very low. Might have to re-think the ADC-sampling strategy, and do more frequent samples with less over-sampling, and cull the ones potentially overlapping the bridge switching


			const mctrl_bridge_activation_t* bridges = mctrl_params.sysId.idSequence[mctrl.idRunCounter];
			size_t loBridgeId = mctrl_loBridge(bridges);
			size_t hiBridgeId = mctrl_hiBridge(bridges);

			// scale currents by number of readouts
			size_t n = NUM_INDUCTANCE_ID_CYCLES/2;
			float n1 = 1.0f/n; // have CALIBREADS/2 number of samples added up
			float U = ssf_getVbus()*mctrl_params.sysId.staticIdentificationDutyCycle;

			float currentMeans[ISENSE_COUNT] = {0.0f};
			float currentSqr[ISENSE_COUNT] = {0.0f};
			float currentRms[ISENSE_COUNT] = {0.0f};

			for (size_t i = 0; i < NUM_PHASE_MEASUREMENTS; ++i)
			{
				// dbg_println("  sin[%u] = %6.3f", i, (double)(mctrl.phasePwm[i]/mctrl_params.sysId.staticIdentificationDutyCycle));
				for (size_t j = 0; j < ISENSE_COUNT; ++j)
				{
					mctrl.phaseCurrents[i][j] *= n1;
					currentMeans[j] += mctrl.phaseCurrents[i][j]*(1.0f/NUM_PHASE_MEASUREMENTS);
					// dbg_println("    %6.3f", (double)(mctrl.phaseCurrents[i][j]));
				}
			}

			if (0 && (k == 0))
			{
				dbg_printf("  pwm = [");
				for (size_t i = 0; i < NUM_PHASE_MEASUREMENTS; ++i)
				{
					dbg_printf(" %.6f,", (double)(U*mctrl.phasePwm[i]/mctrl_params.sysId.staticIdentificationDutyCycle));
				}
				dbg_printf("]\r\n");
				for (size_t j = 0; j < ISENSE_COUNT; ++j)
				{
					dbg_printf("isense%u = [", j);
					for (size_t i = 0; i < NUM_PHASE_MEASUREMENTS; ++i)
					{
						dbg_printf(" %.6f,", (double)(mctrl.phaseCurrents[i][j]));
					}
					dbg_printf("]\r\n");
				}
			}

			// subtract out DC component and find AC RMS
			for (size_t i = 0; i < NUM_PHASE_MEASUREMENTS; ++i)
			{
				for (size_t j = 0; j < ISENSE_COUNT; ++j)
				{
					mctrl.phaseCurrents[i][j] -= currentMeans[j];
					currentSqr[j] += mctrl.phaseCurrents[i][j]*mctrl.phaseCurrents[i][j]*(1.0f/NUM_PHASE_MEASUREMENTS);
				}
			}
			for (size_t j = 0; j < ISENSE_COUNT; ++j)
			{
				currentRms[j] = sqrtf(currentSqr[j]);
			}


			// find impedance
			// we know the signal is 16 samples for a full period, so this is easy enough
			// find the low isense for both observed phases
			size_t j0 = (1 + 2*loBridgeId);
			size_t j1 = (1 + 2*hiBridgeId);

			// RMS applied voltage
			float Urms = U/sqrtf(2.0);
			float R = mctrl.sysParamEstimates.phases.Rest[mctrl.idRunCounter];

			float Z0 = Urms/currentRms[j0];
			float Z1 = Urms/currentRms[j1];
			float X0 = sqrtf(Z0*Z0 - R*R);
			float X1 = sqrtf(Z1*Z1 - R*R);
			float omega = (2.0f*M_PI)/(MEAS_FULL_PERIOD*NUM_PHASE_MEASUREMENTS);
			float Lest0 = X0/omega;
			float Lest1 = X1/omega;

			// phase
			float uiMean0 = 0.0f, uiMean1 = 0.0f;
			float iiMean = 0.0;
			for (size_t i = 0; i < NUM_PHASE_MEASUREMENTS; ++i)
			{
				// reconstruct sinusoid down to the iSense phase shift
				float x0 = 2.0f*M_PI*(i + (float)(j0-0.0f)/ISENSE_COUNT)/NUM_PHASE_MEASUREMENTS;
				float y0 = sintab(x0);
				float U0 = U*y0;

				float x1 = 2.0f*M_PI*(i + (float)(j1-0.0f)/ISENSE_COUNT)/NUM_PHASE_MEASUREMENTS;
				float y1 = sintab(x1);
				float U1 = U*y1;

				// negative sign for j0, positive for j1
				uiMean0 -= U0*mctrl.phaseCurrents[i][j0]*(1.0f/NUM_PHASE_MEASUREMENTS);
				uiMean1 += U1*mctrl.phaseCurrents[i][j1]*(1.0f/NUM_PHASE_MEASUREMENTS);
				iiMean -= mctrl.phaseCurrents[i][j0]*mctrl.phaseCurrents[i][j1]*(1.0f/NUM_PHASE_MEASUREMENTS);
			}
			// float uiRms0 = Urms*currentRms[j0];

			float Rphase0 = uiMean0/currentSqr[j0];
			float Rphase1 = uiMean1/currentSqr[j1];
			float iphase = acos(iiMean/(currentRms[j0]*currentRms[j1]));
			if (1 && (k == 0))
			{
				dbg_println("  Rphase0 = %6.3f R, Z0 = %6.3f, X0 = %6.3f", (double)(Rphase0*1.0e0), (double)Z0, (double)X0);
				dbg_println("  Rphase1 = %6.3f R, Z1 = %6.3f, X1 = %6.3f", (double)(Rphase1*1.0e0), (double)Z1, (double)X1);				
			}
			// dbg_println("  iphase = %6.3f deg, %6.3f isense, j0 %d, j1 %d", (double)(iphase*180.0f/M_PI), (double)(iphase/(2.0f*M_PI)*16.0f*6.0f), j0, j1);

			
			// dbg_println("  Lest0 = %6.3f mH, Z0 = %6.3f, X0 = %6.3f", (double)(Lest0*1.0e3), (double)Z0, (double)X0);
			// dbg_println("  Lest1 = %6.3f mH, Z1 = %6.3f, X1 = %6.3f", (double)(Lest1*1.0e3), (double)Z1, (double)X1);

			float Lavg = 0.5f*(Lest0+Lest1);
			float Lvar = 0.5f*(Lest0*Lest0 + Lest1*Lest1) - Lavg*Lavg;


			// float ph0 = 0.0f, ph1 = 0.0f;
			// for (size_t i = 0; i < NUM_PHASE_MEASUREMENTS; ++i)
			// {
			// 	// find phases,
			// 	size_t iy = (i + NUM_PHASE_MEASUREMENTS/4) % NUM_PHASE_MEASUREMENTS;
			// 	float x0 = mctrl.phaseCurrents[i][j0];
			// 	float y0 = mctrl.phaseCurrents[iy][j0];
			// 	float x1 = mctrl.phaseCurrents[i][j1];
			// 	float y1 = mctrl.phaseCurrents[iy][j1];

			// 	ph0 += atan2f(y0, x0)*(1.0f/NUM_PHASE_MEASUREMENTS);
			// 	ph1 += atan2f(y1, x1)*(1.0f/NUM_PHASE_MEASUREMENTS);
			// }
			// 	dbg_println("  phase[%u] = %6.3f", mctrl.idRunCounter, (double)(ph0*180.0f/M_PI));
			// 	dbg_println("  phase[%u] = %6.3f", (mctrl.idRunCounter + 1) % 3, (double)(ph1*180.0f/M_PI));

			if (k == 0)
			{
				mctrl.debug.iiPhaseEstimate = iphase*(1.0f/MAX_IDENTIFICATION_REPEATS);

				inductanceEstimate = Lavg;
				inductanceVariance = Lvar;	
			}
			else
			{
				mctrl.debug.iiPhaseEstimate += iphase*(1.0f/MAX_IDENTIFICATION_REPEATS);

				// update estimate via Kalman Filter
				/*
				K = sigma1^2 / (sigma1^2 + sigma2^2)
				x = x1 + K (x2 - x1)
				sigma^2 = (1 - K) sigma1^2
				*/
				// if (numValidLMeasurements > 1)
				{
					float KL = inductanceVariance / (inductanceVariance +  Lvar);
					inductanceEstimate += KL * (Lavg - inductanceEstimate);
					inductanceVariance *= 1.0f - KL;	
				}

				// dbg_println("  KL = %8.3f, KR = %8.3f", (double)(KL), (double)(KR));

			}


			++k;

			if (k < MAX_IDENTIFICATION_REPEATS)
			{
				mctrl_state = MCTRL_IMPEDANCE_ID_PREPARE;
			}
			else
			{
				mctrl.sysParamEstimates.phases.Lest[mctrl.idRunCounter] = inductanceEstimate;
				mctrl.sysParamEstimates.phases.Lvar[mctrl.idRunCounter] = inductanceVariance;

				// dbg_println("  iphase = %6.3f deg, %6.3f isense, j0 %d, j1 %d", (double)(mctrl.debug.iiPhaseEstimate*180.0f/M_PI), (double)(mctrl.debug.iiPhaseEstimate/(2.0f*M_PI)*16.0f*6.0f), j0, j1);
				// dbg_println("  Lest[%u] is %8.3f mH, sigma = %8.3f mH", mctrl.idRunCounter, (double)(inductanceEstimate*1e3), (double)(sqrtf(inductanceVariance)*1e3));
				// dbg_println("  Rest[%u] is %8.3f R,  sigma = %8.3f R", mctrl.idRunCounter, (double)(mctrl.sysParamEstimates.phases.Rest[mctrl.idRunCounter]*1e0), (double)(sqrtf(mctrl.sysParamEstimates.phases.Rvar[mctrl.idRunCounter])*1e0));
				// dbg_println("   Tau[%u] is %8.3f ms", mctrl.idRunCounter, (double)(mctrl.sysParamEstimates.phases.Lest[mctrl.idRunCounter]/mctrl.sysParamEstimates.phases.Rest[mctrl.idRunCounter]*1e3));

				k = 0;
				mctrl_state = MCTRL_SYSID_FINISH;
			}
			break;
		}
		case MCTRL_SYSID_FINISH:
		{
			if (mctrl.idRunCounter + 1 < NUM_IDENTIFICATION_RUNS)
			{
				++mctrl.idRunCounter;
				mctrl_state = MCTRL_ID_ALIGN_START;
			}
			else
			{
				mctrl_state = MCTRL_SYSID_DONE;
			}
			break;
		}
		case MCTRL_SYSID_DONE:
		{
			for (size_t i = 0; i < NUM_IDENTIFICATION_RUNS; ++i)
			{
				_printBridgeState(i);
				dbg_println("  Lest[%u] is %8.3f mH, sigma = %8.3f mH", i, (double)(mctrl.sysParamEstimates.phases.Lest[i]*1e3), (double)(sqrtf(mctrl.sysParamEstimates.phases.Lvar[i])*1e3));
				dbg_println("  Rest[%u] is %8.3f R,  sigma = %8.3f R", i, (double)(mctrl.sysParamEstimates.phases.Rest[i]*1e0), (double)(sqrtf(mctrl.sysParamEstimates.phases.Rvar[i])*1e0));
				dbg_println("   Tau[%u] is %8.3f ms", i, (double)(mctrl.sysParamEstimates.phases.Lest[i]/mctrl.sysParamEstimates.phases.Rest[i]*1e3));
			}

			// having estimated per phases parameters, find out if we have a 2 phase or 3 phase motor
			// 3 phase all 3 resistances and windings should be close
			// 2 phase C should have roughly double the R and L of A,B
			// calculate weights based on how far away we are from ideal ratios, lowest weight wins
			float ph2Weight = fabsf(mctrl.sysParamEstimates.phases.Rest[0]/mctrl.sysParamEstimates.phases.Rest[2] - 0.5f) + fabsf(mctrl.sysParamEstimates.phases.Rest[1]/mctrl.sysParamEstimates.phases.Rest[2] - 0.5f);
			float ph3Weight = fabsf(mctrl.sysParamEstimates.phases.Rest[0]/mctrl.sysParamEstimates.phases.Rest[2] - 1.0f) + fabsf(mctrl.sysParamEstimates.phases.Rest[1]/mctrl.sysParamEstimates.phases.Rest[2] - 1.0f);

			dbg_println("  ph2Weight is %6.3f, ph3Weight is %6.3f", (double)(ph2Weight), (double)(ph3Weight));

			float weightRatio = fminf(ph2Weight,ph3Weight)/fmaxf(ph2Weight,ph3Weight);

			if ((weightRatio > 0.5f) && (weightRatio < 2.0f))
			{
				err_println("Can't quite tell motor config!");		
			}
			else if (ph3Weight > ph2Weight)
			{
				dbg_println("Looks like we have a 2-Phase motor.");
				mctrl.sysParamEstimates.motorType = MCTRL_MOT_2PH;
			}
			else
			{
				dbg_println("Looks like we have a 3-Phase motor.");		
				mctrl.sysParamEstimates.motorType = MCTRL_MOT_3PH_TRAP;
			}

			spwm_enableHalfBridges(0x7);
			mctrl_state = MCTRL_PPID_PREPARE;
			break;
		}
		case MCTRL_PPID_PREPARE:
		{
			mctrl.calibCounter = 0;
			mctrl.counter = 0;
			mctrl.phase = 0;

			mctrl_state = MCTRL_PPID_START;
			break;
		}
		case MCTRL_PPID_START:
		case MCTRL_PPID_RUN:
		{
			// wait on fastloop
			break;
		}
		case MCTRL_PPID_FINISH:
		{
			float angle = ssf_getEncoderAngle();

			dbg_println("Encoder angle = %.3f deg", (double)(angle*180.0f/M_PI));

			if (mctrl.counter == 0)
			{
				mctrl.phase = angle;
				mctrl.angleSum = 0.0f;
				++mctrl.counter;
			}
			else
			{
				float delta = mctrl_modAngle(angle - mctrl.phase);
				mctrl.phase = angle;
				mctrl.angleSum += fabsf(delta);

				++mctrl.counter;
			}

			if (mctrl.counter < NUM_ANGLE_MEASUREMENTS)
			{
				mctrl_state = MCTRL_PPID_START;
			}
			else
			{

				float stepSize = mctrl.angleSum*(1.0f/(NUM_ANGLE_MEASUREMENTS-1));
				dbg_println("Encoder step angle = %.3f deg", (double)(stepSize*180.0f/M_PI));

				mctrl.sysParamEstimates.stepsPerRev = 2.0f*M_PI*NUM_ANGLE_MEASUREMENTS/fabsf(mctrl.angleSum) + 0.5f;

				switch(mctrl.sysParamEstimates.motorType)
				{
					case MCTRL_MOT_2PH:
					{
						// round to nearest 4
						mctrl.sysParamEstimates.stepsPerRev = ((mctrl.sysParamEstimates.stepsPerRev + 2)/4)*4;

						break;
					}
					case MCTRL_MOT_3PH_SIN:
					case MCTRL_MOT_3PH_TRAP:
					{
						// round to nearest 6
						mctrl.sysParamEstimates.stepsPerRev = ((mctrl.sysParamEstimates.stepsPerRev + 3)/6)*6;
						
						break;
					}
					default:
					{
						break;
					}
				}

				dbg_println("Steps per rev = %u", mctrl.sysParamEstimates.stepsPerRev);

				mctrl_state = MCTRL_MAPSTEP_PREPARE;
			}
			break;
		}
		case MCTRL_COGID_PREPARE:
		{
			mctrl.calibCounter = 0;
			mctrl.counter = 0;
			mctrl.phase = 0;

			// Cog identification. We need energize around the neutral points, where one phase is fully energized and the other two are halfway energized.
			// Then, we wiggle around a bit. How much? we want to stay as close to the neutral as possible, and certainly not move the rotor past max cogging torque (1/4 step).

			// How to deal with hysterisis / static friction?


			break;
		}
		case MCTRL_COGID_START: 
		{
			int k = (mctrl.sysParamEstimates.motorType == MCTRL_MOT_2PH) ? 6 : 4;

			float lambda = mctrl.counter*2.0f*M_PI/(float)k;


			break;
		}
		case MCTRL_COGID_RUN:
		{
			break;
		}
		case MCTRL_COGID_FINISH:
		{
			mctrl.counter++;

			if (mctrl.counter == mctrl.sysParamEstimates.stepsPerRev) {
				// we are done
			}
		}
		case MCTRL_MAPSTEP_PREPARE:
		{
			mctrl.calibCounter = 0;
			mctrl.counter = 0;
			mctrl.phase = 0;
			memset(mctrl.stepmap_i, 0, sizeof(mctrl.stepmap_i));
			memset(mctrl.stepmap_a, 0, sizeof(mctrl.stepmap_a));

			float dc = 2.0f*mctrl_params.sysId.staticIdentificationDutyCycle;

			mctrl_setPhasorPwmSin(0.0f, dc, mctrl.sysParamEstimates.motorType);

			mctrl_state = MCTRL_MAPSTEP_START;
			break;
		}
		case MCTRL_MAPSTEP_START:
		case MCTRL_MAPSTEP_RUN:
		{
			// wait on fastloop
			break;
		}
		case MCTRL_MAPSTEP_FINISH:
		{
			// add an extra electrical cycle at the beginning to start measurement at steady state
			int substepsPerStep = mctrl.sysParamEstimates.motorType == MCTRL_MOT_3PH_TRAP ? 6 : 4;
			int electricalRotations = mctrl.sysParamEstimates.stepsPerRev/substepsPerStep;
			const size_t limit = (electricalRotations+1)*NUM_MAPSTEP_MEASUREMENTS;
			float angle = ssf_getEncoderAngle();


			if (mctrl.counter == 0)
			{
				mctrl.phase = angle;
				mctrl.angleSum = 0.0f;

				dbg_println("Mapstep with %.1f*%d steps for %d steps/rev", (double)(electricalRotations), NUM_MAPSTEP_MEASUREMENTS, mctrl.sysParamEstimates.stepsPerRev);

				++mctrl.counter;
			}
			else
			{
				float delta = mctrl_modAngle(angle - mctrl.phase);
				mctrl.phase = angle;
				mctrl.angleSum += fabsf(delta);


				// the first cycle is just to get things going, we don't want to measure, yet.
				if (mctrl.counter > NUM_MAPSTEP_MEASUREMENTS)
				{
					size_t i = mctrl.counter % NUM_MAPSTEP_MEASUREMENTS;
					mctrl.stepmap_a[i] += delta;
					mctrl.stepmap_i[i] += 0.0f;					
				}

				++mctrl.counter;
			}


			dbg_println("Mapstep encoder angle = %.3f deg", (double)(angle*180.0f/M_PI));
			// dbg_println("   ph = %.3f", (double)mctrl.phase);
			// dbg_println("  u   = %.3f,%.3f,%.3f", (double)mctrl.pwm.u[0], (double)mctrl.pwm.u[1], (double)mctrl.pwm.u[2]);
			// dbg_println("  pwm = %.3f,%.3f,%.3f", (double)mctrl.pwm.pwm[0], (double)mctrl.pwm.pwm[1], (double)mctrl.pwm.pwm[2]);

			if (mctrl.counter < limit)
			{
				mctrl_state = MCTRL_MAPSTEP_START;				
			}
			else
			{
				spwm_setDrvChannel(HTIM_DRV_CH_A, 0.0f);
				spwm_setDrvChannel(HTIM_DRV_CH_B, 0.0f);
				spwm_setDrvChannel(HTIM_DRV_CH_C, 0.0f);

				dbg_println("Mapstep angleSum = %.3f deg after %d counts", (double)(mctrl.angleSum*180.0f/M_PI), mctrl.counter);

				dbg_println("Step Mapping is [phasor_angle, encoder_delta, current]:");
				for (size_t i = 0; i < NUM_MAPSTEP_MEASUREMENTS; ++i)
				{
					float scaleAngle = 1.0f/NUM_MAPSTEP_MEASUREMENTS;
					float scaleSum = 1.0f/electricalRotations;
					dbg_printf("[%.4f, %.4f, %.4f], \r\n", 
						(double)(i*(2.0f*M_PI)*scaleAngle), 
						(double)(mctrl.stepmap_a[i]), // angles have to add up to 2pi
						(double)(mctrl.stepmap_i[i]*scaleSum)
					);
				}

				mctrl_state = MCTRL_EMF_PREPARE;
			}
			break;
		}

		case MCTRL_EMF_PREPARE:
		{
			mctrl.idRunCounter = 0; // up to NUM_STATIC_MEASUREMENTS
			mctrl.calibCounter = now_us; // time keeping 
			mctrl.counter = 0; // per-run sample counting
			mctrl.phase = 0.0f;
			mctrl.stallSpeed = 0.1f*2.0f*M_PI;

			mctrl_state = MCTRL_EMF_STALL_RAMP;
			break;
		}
		case MCTRL_EMF_STALL_RAMP:
		{
			// wait on fastloop
			break;
		}
		case MCTRL_EMF_STALL_EVAL:
		{
			mctrl.idRunCounter = 0; // up to NUM_STATIC_MEASUREMENTS
			mctrl.calibCounter = 0; // time keeping 
			mctrl.counter = 0; // per-run sample counting
			mctrl.phase = 0.0f;
			memset(mctrl.currentSqrSum, 0, sizeof(mctrl.currentSqrSum));
			memset(mctrl.emfRegressions, 0, sizeof(mctrl.emfRegressions));

			dbg_println(" EMF stall speed is = %6.3f RPM", (double)(mctrl.stallSpeed*30.0f/M_PI));


			mctrl_state = MCTRL_EMF_START;
			break;
		}
		case MCTRL_EMF_START:
		case MCTRL_EMF_RAMP:
		case MCTRL_EMF_RUN:
		case MCTRL_EMF_DECELERATE:
		{
			// wait on fastloop
			break;
		}
		case MCTRL_EMF_FINISH:
		{
			dbg_println("MCTRL_EMF_FINISH");

			for (size_t i = 0; i < ISENSE_COUNT; ++i)
			{
				float y = sqrtf(mctrl.currentSqrSum[i]/(mctrl.counter));
				// mctrl.lastMeasurement[mctrl.idRunCounter][i] = y;
				float x = 0.9f*mctrl.stallSpeed*(1.0f+mctrl.idRunCounter)/NUM_STATIC_MEASUREMENTS;
				mctrl.emfRegressions[i] = linreg_addSample(mctrl.emfRegressions[i], x, y);
				dbg_println(" RMS current[%u] = %6.3f @ %6.3f rpm", i, (double)y, (double)(x*30.0f/M_PI));
			}

			++mctrl.idRunCounter;

			if (mctrl.idRunCounter < NUM_STATIC_MEASUREMENTS)
			{
				memset(mctrl.currentSqrSum, 0, sizeof(mctrl.currentSqrSum));
				mctrl_state = MCTRL_EMF_START;
			}
			else
			{
				float vbus = ssf_getVbus();
				float u = 1.0f/M_SQRT2*vbus*mctrl_params.sysId.staticIdentificationDutyCycle;
				// evaluate regressions
				for (size_t i = 1; i < ISENSE_COUNT; i += 2)
				{
					linregResult_t res = linreg_solve(mctrl.emfRegressions[i]);
					float amp_per_rev = res.slope;
					float zamp = res.intercept;

					dbg_println(" EMF[%u] = %8.3f mA/rpm", i, (double)(amp_per_rev*1000.0f/(30.0f/M_PI)));
					dbg_println("           %8.3f mA @ 0 rpm", (double)(zamp*1000.0f));
					dbg_println("           %8.3f R  @ 0 rpm", (double)(u/zamp));
				}

				spwm_enableHalfBridges(0x7);
				mctrl_state = MCTRL_DEMO;
			}
			break;
		}
		case MCTRL_DEMO:
		{
			// do nothing, fastloop does work
			break;
		}
	}
}
