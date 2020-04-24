
#include "ssf_mctrl_private.h"

#include "ssf_spi.h"
#include "ssf_main.h"

#include "debug.h"

#include <string.h>

void mctrl_idle(uint32_t now_us)
{
	static uint32_t waitStart = 0;

	static float RestSum = 0.0f;
	static float RestSqrSum = 0.0f;
	static size_t k = 0;

	static float inductanceEstimate = 0.0f;
	static float inductanceVariance = 0.0f;

	uint32_t waitElapsed = now_us - waitStart;

	switch(mctrl_state)
	{
		case MCTRL_INIT:
		{
			// initial state, do nothing
			break;
		}
		case MCTRL_DRIVER_RESET_START:
		{
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
				mctrl_state = MCTRL_DRIVER_INIT_START;
			}
			break;
		}
		case MCTRL_DRIVER_INIT_START:
		{
			if (waitElapsed >= 2000)
			{
				dbg_println("MCTRL_DRIVER_INIT_START");
				ssf_setMotorDriver3PwmMode();
				memset(mctrl.adcZeroCalibs, 0, sizeof(mctrl.adcZeroCalibs));
				mctrl.calibCounter = 0;

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
			dbg_println("MCTRL_DRIVER_CALIB_ENTER");

			// first is analog recalibration
			// put the DRV8323 into calibration mode
			// switchover in DRV8323 takes 100us
			// do calibration twice, as otherwise we seem to get flip-flopping between ~2047 and ~2070 ADC counts, it is more stable this way, but not perfect, for some reason SOC still seems to be unstable
			ssf_enterMotorDriverCalibrationMode();

			waitStart = now_us;
			mctrl_state = MCTRL_DRIVER_CALIB_WAIT;
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
			dbg_println("MCTRL_DRIVER_CALIB_EXIT");
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
		case MCTRL_ANALOG_CALIBRATION_RUN:
		{
			// dbg_println("MCTRL_ANALOG_CALIBRATION_RUN...");
			if (waitElapsed >= 1000000)
			{
				warn_println("MCTRL_ANALOG_CALIBRATION_RUN more than a second passed!");
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
				
				// dbg_println("    calib0 = %.3f, calib1 = %.3f", (double)(_adcZeroCalibs[2*i]*(2.0f/CALIBREADS)), (double)(_adcZeroCalibs[2*i+1]*(2.0f/CALIBREADS)));
				
				mctrl.adcZeroCalibs[2*i] = calib;
				mctrl.adcZeroCalibs[2*i+1] = calib;

				dbg_println("SO[%u] calibrated to 0A = %.3f counts", i, (double)calib);

				// restart calibration if we're more than 300 counts off center
				if (fabsf(calib - 8192.0f) > 300.0f)
				{
					err_println("SO[%u] is too far off center at %.3f counts!", i, (double)calib);
					calibOk = false;
				}
			}

			if (!calibOk)
			{
				err_println("Restarting motor control init...");
				mctrl_state = MCTRL_DRIVER_RESET_START;
				break;
			}

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
			int lowBridge = mctrl.idRunCounter;
			int highBridge = (mctrl.idRunCounter + 1) % 3;

			spwm_enableHalfBridges((1 << lowBridge) | (1 << highBridge));

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

			if (numSampleExceedingCurrentLimit >= NUM_STATIC_MEASUREMENTS)
			{
				{
					float i0 = mctrl.lastMeasurement[NUM_STATIC_MEASUREMENTS-1][2*mctrl.idRunCounter+0];
					float i1 = mctrl.lastMeasurement[NUM_STATIC_MEASUREMENTS-1][2*mctrl.idRunCounter+1];

					float i = 0.5*(i0+i1);

					float time = (NUM_STATIC_MEASUREMENTS-1)*(float)MEAS_FULL_PERIOD + mctrl.idRunCounter*2*MEAS_SINGLE_PERIOD;
					dbg_println("could not reach %.3f, max current reached is %.3f after %8.0f us", (double)mctrl_params.sysId.maxCurrent, (double)i, (double)(time*1e6f));

				}
			}
			else
			{
				float i0 = mctrl.lastMeasurement[numSampleExceedingCurrentLimit][2*mctrl.idRunCounter+0];
				float i1 = mctrl.lastMeasurement[numSampleExceedingCurrentLimit][2*mctrl.idRunCounter+1];

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

			for (size_t j = NUM_STATIC_MEASUREMENTS/2; j < NUM_STATIC_MEASUREMENTS; ++j)
			{
				float i0 = mctrl.lastMeasurement[j][2*mctrl.idRunCounter+0];
				float i1 = mctrl.lastMeasurement[j][2*mctrl.idRunCounter+1];

				i += (i0+i1);

				vbus += mctrl.lastVbus[j];

			}

			i *= 1.0f/NUM_STATIC_MEASUREMENTS;
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

				dbg_println("RestSum %.3f , RestSqrSum %.3f", (double)RestSum, (double)(RestSqrSum));
				dbg_println("steady state estimate %.3f R, sigma = %.3f", (double)Rest, (double)(sqrtf(Rvar)));

				mctrl.sysParamEstimates.phases.Rest[mctrl.idRunCounter] = Rest;
				mctrl.sysParamEstimates.phases.Rvar[mctrl.idRunCounter] = Rvar;

				k = 0;
				mctrl_state = MCTRL_INDUCTANCE_ID_PREPARE;
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

			for (size_t j = 1; j+NUM_ID_ALGO_SAMPLES-1 < NUM_STATIC_MEASUREMENTS; ++j)
			{
#if NUM_ID_ALGO_SAMPLES == 2
				// inductance estimation only based on previous R estimate
				const float h = MEAS_FULL_PERIOD;
				float vbus0 = mctrl.lastVbus[j+0];
				float vbus1 = mctrl.lastVbus[j+1];
				float vbus = 0.5*(vbus0 + vbus1);
				float i0 = mctrl.lastMeasurement[j+0][2*mctrl.idRunCounter];
				float i0a = mctrl.lastMeasurement[j+0][2*mctrl.idRunCounter+1];
				float i1 = mctrl.lastMeasurement[j+1][2*mctrl.idRunCounter];
				float i1a = mctrl.lastMeasurement[j+1][2*mctrl.idRunCounter+1];
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

				if (mctrl.idRunCounter < 2)
				{
					++mctrl.idRunCounter;
					mctrl_state = MCTRL_ID_ALIGN_START;
				}
				else
				{
					mctrl_state = MCTRL_SYSID_DONE;
				}
			}

			break;
		}
		case MCTRL_SYSID_DONE:
		{
			spwm_enableHalfBridges(0x7);
			mctrl_state = MCTRL_DEMO;
			break;
		}
		case MCTRL_DEMO:
		{
			// do nothing, fastloop does work
			break;
		}
	}
}
