
#include "ssf_mctrl_private.h"
#include "ssf_mctrl.h"

#include "ssf_spi.h"
#include "ssf_main.h"

#include "debug.h"

#include <string.h>
#include <assert.h>

void mctrl_idle(uint32_t now_us)
{
	static uint32_t waitStart = 0;

	static float RestSum = 0.0f;
	static float RestSqrSum = 0.0f;
	static size_t k = 0;

	static float inductanceEstimate = 0.0f;
	static float inductanceVariance = 0.0f;

	uint32_t waitElapsed = now_us - waitStart;

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
				// dbg_println("MCTRL_DRIVER_INIT_START");
				ssf_setMotorDriver3PwmMode();
				// sspi_drv_state_t drvState = ssf_readMotorDriver();
				// dbg_println("DRV8323 DRV_CTRL set to 0x%04X", drvState.DRV_CTRL);

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
			// dbg_println("MCTRL_DRIVER_CALIB_ENTER");

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

				// restart calibration if we're more than 5% off center
				float center = 0.5f*((ADC2_NOMINAL_MAXCOUNT*ADC2_OVERSAMPLING_COUNT) >> ADC2_SHIFT);
				if (fabsf(calib - center) > center*0.05f)
				{
					err_println("SO[%u] is too far off center at %.3f counts (center = %.3f)!", i, (double)calib, (double)center);
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

				dbg_println("RestSum %.3f , RestSqrSum %.3f", (double)RestSum, (double)(RestSqrSum));
				dbg_println("steady state estimate %.3f R, sigma = %.3f", (double)Rest, (double)(sqrtf(Rvar)));

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
			if (0 && (k == 0))
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
				mctrl.sysParamEstimates.motorType = MCTRL_MOT_3PH;
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
			}

			if (mctrl.counter < NUM_ANGLE_MEASUREMENTS)
			{
				++mctrl.counter;
				mctrl_state = MCTRL_PPID_START;
			}
			else
			{
				float delta = mctrl_modAngle(angle - mctrl.phase);
				mctrl.phase = angle;
				mctrl.angleSum += fabsf(delta);

				float stepSize = mctrl.angleSum*(1.0f/NUM_ANGLE_MEASUREMENTS);
				dbg_println("Encoder step angle = %.3f deg", (double)(stepSize*180.0f/M_PI));

				mctrl.sysParamEstimates.ph2.stepsPerRev = 2.0f*M_PI*NUM_ANGLE_MEASUREMENTS/fabsf(mctrl.angleSum) + 0.5f;
				// round to nearest 4
				mctrl.sysParamEstimates.ph2.stepsPerRev = ((mctrl.sysParamEstimates.ph2.stepsPerRev + 2)/4)*4;

				dbg_println("Steps per rev = %u", mctrl.sysParamEstimates.ph2.stepsPerRev);

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
			mctrl.stallSpeed = 0.1*2.0f*M_PI;

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
