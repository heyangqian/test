/*******************************************************************************
Copyright © 2016, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
_2(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
_2(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef _VL53L0X_API_CORE_2_H_
#define _VL53L0X_API_CORE_2_H_

#include "vl53l0x_def_2.h"
#include "vl53l0x_platform_2.h"


#ifdef __cplusplus
extern "C" {
#endif


VL53L0X_Error_2 VL53L0X_reverse_bytes_2(uint8_t *data, uint32_t size);

VL53L0X_Error_2 VL53L0X_measurement_poll_for_completion_2(VL53L0X_DEV_2 Dev);

uint8_t VL53L0X_encode_vcsel_period_2(uint8_t vcsel_period_pclks);

uint8_t VL53L0X_decode_vcsel_period_2(uint8_t vcsel_period_reg);

uint32_t VL53L0X_isqrt_2(uint32_t num);

uint32_t VL53L0X_quadrature_sum_2(uint32_t a, uint32_t b);

VL53L0X_Error_2 VL53L0X_get_info_from_device_2(VL53L0X_DEV_2 Dev, uint8_t option);

VL53L0X_Error_2 VL53L0X_set_vcsel_pulse_period_2(VL53L0X_DEV_2 Dev,
	VL53L0X_VcselPeriod_2 VcselPeriodType, uint8_t VCSELPulsePeriodPCLK);

VL53L0X_Error_2 VL53L0X_get_vcsel_pulse_period_2(VL53L0X_DEV_2 Dev,
	VL53L0X_VcselPeriod_2 VcselPeriodType, uint8_t *pVCSELPulsePeriodPCLK);

uint32_t VL53L0X_decode_timeout_2(uint16_t encoded_timeout);

VL53L0X_Error_2 get_sequence_step_timeout_2(VL53L0X_DEV_2 Dev,
			VL53L0X_SequenceStepId_2 SequenceStepId,
			uint32_t *pTimeOutMicroSecs);

VL53L0X_Error_2 set_sequence_step_timeout_2(VL53L0X_DEV_2 Dev,
			VL53L0X_SequenceStepId_2 SequenceStepId,
			uint32_t TimeOutMicroSecs);

VL53L0X_Error_2 VL53L0X_set_measurement_timing_budget_micro_seconds_2(VL53L0X_DEV_2 Dev,
	uint32_t MeasurementTimingBudgetMicroSeconds);

VL53L0X_Error_2 VL53L0X_get_measurement_timing_budget_micro_seconds_2(VL53L0X_DEV_2 Dev,
		uint32_t *pMeasurementTimingBudgetMicroSeconds);

VL53L0X_Error_2 VL53L0X_load_tuning_settings_2(VL53L0X_DEV_2 Dev,
		uint8_t *pTuningSettingBuffer);

VL53L0X_Error_2 VL53L0X_calc_sigma_estimate_2(VL53L0X_DEV_2 Dev,
		VL53L0X_RangingMeasurementData_t_2 *pRangingMeasurementData,
		FixPoint1616_t_2 *pSigmaEstimate, uint32_t *pDmax_mm);

VL53L0X_Error_2 VL53L0X_get_total_xtalk_rate_2(VL53L0X_DEV_2 Dev,
	VL53L0X_RangingMeasurementData_t_2 *pRangingMeasurementData,
	FixPoint1616_t_2 *ptotal_xtalk_rate_mcps);

VL53L0X_Error_2 VL53L0X_get_total_signal_rate_2(VL53L0X_DEV_2 Dev,
	VL53L0X_RangingMeasurementData_t_2 *pRangingMeasurementData,
	FixPoint1616_t_2 *ptotal_signal_rate_mcps);

VL53L0X_Error_2 VL53L0X_get_pal_range_status_2(VL53L0X_DEV_2 Dev,
		 uint8_t DeviceRangeStatus,
		 FixPoint1616_t_2 SignalRate,
		 uint16_t EffectiveSpadRtnCount,
		 VL53L0X_RangingMeasurementData_t_2 *pRangingMeasurementData,
		 uint8_t *pPalRangeStatus);

uint32_t VL53L0X_calc_timeout_mclks_2(VL53L0X_DEV_2 Dev,
	uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

uint16_t VL53L0X_encode_timeout_2(uint32_t timeout_macro_clks);

#ifdef __cplusplus
}
#endif

#endif /* _VL53L0X_API_CORE_H_ */
