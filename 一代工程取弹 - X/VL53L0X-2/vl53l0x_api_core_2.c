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
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "vl53l0x_api_2.h"
#include "vl53l0x_api_core_2.h"
#include "vl53l0x_api_calibration_2.h"


#ifndef __KERNEL__
#include <stdlib.h>
#endif
#define LOG_FUNCTION_START_2(fmt, ...) \
	_LOG_FUNCTION_START_2(TRACE_MODULE_API, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END_2(status, ...) \
	_LOG_FUNCTION_END_2(TRACE_MODULE_API, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_2_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_2_FMT(TRACE_MODULE_API, status, fmt, ##__VA_ARGS__)

VL53L0X_Error_2 VL53L0X_reverse_bytes_2(uint8_t *data, uint32_t size)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t tempData;
	uint32_t mirrorIndex;
	uint32_t middle = size/2;
	uint32_t index;

	for (index = 0; index < middle; index++) {
		mirrorIndex		 = size - index - 1;
		tempData		 = data[index];
		data[index]		 = data[mirrorIndex];
		data[mirrorIndex] = tempData;
	}
	return Status;
}

VL53L0X_Error_2 VL53L0X_measurement_poll_for_completion_2(VL53L0X_DEV_2 Dev)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t NewDataReady = 0;
	uint32_t LoopNb;

	LOG_FUNCTION_START_2("");

	LoopNb = 0;

	do {
		Status = VL53L0X_GetMeasurementDataReady_2(Dev, &NewDataReady);
		if (Status != 0)
			break; /* the error is set */

		if (NewDataReady == 1)
			break; /* done note that status == 0 */

		LoopNb++;
		if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP_2) {
			Status = VL53L0X_ERROR_TIME_OUT_2;
			break;
		}

		VL53L0X_PollingDelay_2(Dev);
	} while (1);

	LOG_FUNCTION_END_2(Status);

	return Status;
}


uint8_t VL53L0X_decode_vcsel_period_2(uint8_t vcsel_period_reg)
{
	/*!
	 * Converts the encoded VCSEL period register value into the real
	 * period in PLL clocks
	 */

	uint8_t vcsel_period_pclks = 0;

	vcsel_period_pclks = (vcsel_period_reg + 1) << 1;

	return vcsel_period_pclks;
}

uint8_t VL53L0X_encode_vcsel_period_2(uint8_t vcsel_period_pclks)
{
	/*!
	 * Converts the encoded VCSEL period register value into the real period
	 * in PLL clocks
	 */

	uint8_t vcsel_period_reg = 0;

	vcsel_period_reg = (vcsel_period_pclks >> 1) - 1;

	return vcsel_period_reg;
}


uint32_t VL53L0X_isqrt_2(uint32_t num)
{
	/*
	 * Implements an integer square root
	 *
	 * From: http://en.wikipedia.org/wiki/Methods_of_computing_square_roots
	 */

	uint32_t  res = 0;
	uint32_t  bit = 1 << 30;
	/* The second-to-top bit is set:
	 *	1 << 14 for 16-bits, 1 << 30 for 32 bits */

	 /* "bit" starts at the highest power of four <= the argument. */
	while (bit > num)
		bit >>= 2;


	while (bit != 0) {
		if (num >= res + bit) {
			num -= res + bit;
			res = (res >> 1) + bit;
		} else
			res >>= 1;

		bit >>= 2;
	}

	return res;
}


uint32_t VL53L0X_quadrature_sum_2(uint32_t a, uint32_t b)
{
	/*
	 * Implements a quadrature sum
	 *
	 * rea = sqrt(a^2 + b^2)
	 *
	 * Trap overflow case max input value is 65535 (16-bit value)
	 * as internal calc are 32-bit wide
	 *
	 * If overflow then seta output to maximum
	 */
	uint32_t  res = 0;

	if (a > 65535 || b > 65535)
		res = 65535;
	else
		res = VL53L0X_isqrt_2(a * a + b * b);

	return res;
}


VL53L0X_Error_2 VL53L0X_device_read_strobe_2(VL53L0X_DEV_2 Dev)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t strobe;
	uint32_t LoopNb;
	LOG_FUNCTION_START_2("");

	Status |= VL53L0X_WrByte_2(Dev, 0x83, 0x00);

	/* polling
	 * use timeout to avoid deadlock*/
	if (Status == VL53L0X_ERROR_NONE_2) {
		LoopNb = 0;
		do {
			Status = VL53L0X_RdByte_2(Dev, 0x83, &strobe);
			if ((strobe != 0x00) || Status != VL53L0X_ERROR_NONE_2)
					break;

			LoopNb = LoopNb + 1;
		} while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP_2);

		if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP_2)
			Status = VL53L0X_ERROR_TIME_OUT_2;

	}

	Status |= VL53L0X_WrByte_2(Dev, 0x83, 0x01);

	LOG_FUNCTION_END_2(Status);
	return Status;

}

VL53L0X_Error_2 VL53L0X_get_info_from_device_2(VL53L0X_DEV_2 Dev, uint8_t option)
{

	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t byte;
	uint32_t TmpDWord;
	uint8_t ModuleId;
	uint8_t Revision;
	uint8_t ReferenceSpadCount = 0;
	uint8_t ReferenceSpadType = 0;
	uint32_t PartUIDUpper = 0;
	uint32_t PartUIDLower = 0;
	uint32_t OffsetFixed1104_mm = 0;
	int16_t OffsetMicroMeters = 0;
	uint32_t DistMeasTgtFixed1104_mm = 400 << 4;
	uint32_t DistMeasFixed1104_400_mm = 0;
	uint32_t SignalRateMeasFixed1104_400_mm = 0;
	char ProductId[19];
	char *ProductId_tmp;
	uint8_t ReadDataFromDeviceDone;
	FixPoint1616_t_2 SignalRateMeasFixed400mmFix_2 = 0;
	uint8_t NvmRefGoodSpadMap[VL53L0X_REF_SPAD_BUFFER_SIZE_2];
	int i;


	LOG_FUNCTION_START_2("");

	ReadDataFromDeviceDone = VL53L0X_GETDEVICESPECIFICPARAMETER_2(Dev,
			ReadDataFromDeviceDone);

	/* This access is done only once after that a GetDeviceInfo or
	 * datainit is done*/
	if (ReadDataFromDeviceDone != 7) {

		Status |= VL53L0X_WrByte_2(Dev, 0x80, 0x01);
		Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x01);
		Status |= VL53L0X_WrByte_2(Dev, 0x00, 0x00);

		Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x06);
		Status |= VL53L0X_RdByte_2(Dev, 0x83, &byte);
		Status |= VL53L0X_WrByte_2(Dev, 0x83, byte|4);
		Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x07);
		Status |= VL53L0X_WrByte_2(Dev, 0x81, 0x01);

		Status |= VL53L0X_PollingDelay_2(Dev);

		Status |= VL53L0X_WrByte_2(Dev, 0x80, 0x01);

		if (((option & 1) == 1) &&
			((ReadDataFromDeviceDone & 1) == 0)) {
			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x6b);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &TmpDWord);

			ReferenceSpadCount = (uint8_t)((TmpDWord >> 8) & 0x07f);
			ReferenceSpadType  = (uint8_t)((TmpDWord >> 15) & 0x01);

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x24);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &TmpDWord);


			NvmRefGoodSpadMap[0] = (uint8_t)((TmpDWord >> 24)
				& 0xff);
			NvmRefGoodSpadMap[1] = (uint8_t)((TmpDWord >> 16)
				& 0xff);
			NvmRefGoodSpadMap[2] = (uint8_t)((TmpDWord >> 8)
				& 0xff);
			NvmRefGoodSpadMap[3] = (uint8_t)(TmpDWord & 0xff);

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x25);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &TmpDWord);

			NvmRefGoodSpadMap[4] = (uint8_t)((TmpDWord >> 24)
				& 0xff);
			NvmRefGoodSpadMap[5] = (uint8_t)((TmpDWord >> 16)
				& 0xff);
		}

		if (((option & 2) == 2) &&
			((ReadDataFromDeviceDone & 2) == 0)) {

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x02);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdByte_2(Dev, 0x90, &ModuleId);

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x7B);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdByte_2(Dev, 0x90, &Revision);

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x77);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &TmpDWord);

			ProductId[0] = (char)((TmpDWord >> 25) & 0x07f);
			ProductId[1] = (char)((TmpDWord >> 18) & 0x07f);
			ProductId[2] = (char)((TmpDWord >> 11) & 0x07f);
			ProductId[3] = (char)((TmpDWord >> 4) & 0x07f);

			byte = (uint8_t)((TmpDWord & 0x00f) << 3);

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x78);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &TmpDWord);

			ProductId[4] = (char)(byte +
					((TmpDWord >> 29) & 0x07f));
			ProductId[5] = (char)((TmpDWord >> 22) & 0x07f);
			ProductId[6] = (char)((TmpDWord >> 15) & 0x07f);
			ProductId[7] = (char)((TmpDWord >> 8) & 0x07f);
			ProductId[8] = (char)((TmpDWord >> 1) & 0x07f);

			byte = (uint8_t)((TmpDWord & 0x001) << 6);

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x79);

			Status |= VL53L0X_device_read_strobe_2(Dev);

			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &TmpDWord);

			ProductId[9] = (char)(byte +
					((TmpDWord >> 26) & 0x07f));
			ProductId[10] = (char)((TmpDWord >> 19) & 0x07f);
			ProductId[11] = (char)((TmpDWord >> 12) & 0x07f);
			ProductId[12] = (char)((TmpDWord >> 5) & 0x07f);

			byte = (uint8_t)((TmpDWord & 0x01f) << 2);

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x7A);

			Status |= VL53L0X_device_read_strobe_2(Dev);

			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &TmpDWord);

			ProductId[13] = (char)(byte +
					((TmpDWord >> 30) & 0x07f));
			ProductId[14] = (char)((TmpDWord >> 23) & 0x07f);
			ProductId[15] = (char)((TmpDWord >> 16) & 0x07f);
			ProductId[16] = (char)((TmpDWord >> 9) & 0x07f);
			ProductId[17] = (char)((TmpDWord >> 2) & 0x07f);
			ProductId[18] = '\0';

		}

		if (((option & 4) == 4) &&
			((ReadDataFromDeviceDone & 4) == 0)) {

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x7B);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &PartUIDUpper);

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x7C);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &PartUIDLower);

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x73);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &TmpDWord);

			SignalRateMeasFixed1104_400_mm = (TmpDWord &
				0x0000000ff) << 8;

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x74);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &TmpDWord);

			SignalRateMeasFixed1104_400_mm |= ((TmpDWord &
				0xff000000) >> 24);

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x75);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &TmpDWord);

			DistMeasFixed1104_400_mm = (TmpDWord & 0x0000000ff)
							<< 8;

			Status |= VL53L0X_WrByte_2(Dev, 0x94, 0x76);
			Status |= VL53L0X_device_read_strobe_2(Dev);
			Status |= VL53L0X_RdDWord_2(Dev, 0x90, &TmpDWord);

			DistMeasFixed1104_400_mm |= ((TmpDWord & 0xff000000)
							>> 24);
		}

		Status |= VL53L0X_WrByte_2(Dev, 0x81, 0x00);
		Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x06);
		Status |= VL53L0X_RdByte_2(Dev, 0x83, &byte);
		Status |= VL53L0X_WrByte_2(Dev, 0x83, byte&0xfb);
		Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x01);
		Status |= VL53L0X_WrByte_2(Dev, 0x00, 0x01);

		Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x00);
		Status |= VL53L0X_WrByte_2(Dev, 0x80, 0x00);
	}

	if ((Status == VL53L0X_ERROR_NONE_2) &&
		(ReadDataFromDeviceDone != 7)) {
		/* Assign to variable if status is ok */
		if (((option & 1) == 1) &&
			((ReadDataFromDeviceDone & 1) == 0)) {
			VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev,
				ReferenceSpadCount, ReferenceSpadCount);

			VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev,
				ReferenceSpadType, ReferenceSpadType);

			for (i = 0; i < VL53L0X_REF_SPAD_BUFFER_SIZE_2; i++) {
				Dev->Data.SpadData.RefGoodSpadMap[i] =
					NvmRefGoodSpadMap[i];
			}
		}

		if (((option & 2) == 2) &&
			((ReadDataFromDeviceDone & 2) == 0)) {
			VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev,
					ModuleId, ModuleId);

			VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev,
					Revision, Revision);

			ProductId_tmp = VL53L0X_GETDEVICESPECIFICPARAMETER_2(Dev,
					ProductId);
			VL53L0X_COPYSTRING_2(ProductId_tmp, ProductId);

		}

		if (((option & 4) == 4) &&
			((ReadDataFromDeviceDone & 4) == 0)) {
			VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev,
						PartUIDUpper, PartUIDUpper);

			VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev,
						PartUIDLower, PartUIDLower);

			SignalRateMeasFixed400mmFix_2 =
				VL53L0X_FIXPOINT97TOFIXPOINT1616_2(
					SignalRateMeasFixed1104_400_mm);

			VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev,
				SignalRateMeasFixed400mm,
				SignalRateMeasFixed400mmFix_2);

			OffsetMicroMeters = 0;
			if (DistMeasFixed1104_400_mm != 0) {
					OffsetFixed1104_mm =
						DistMeasFixed1104_400_mm -
						DistMeasTgtFixed1104_mm;
					OffsetMicroMeters = (OffsetFixed1104_mm
						* 1000) >> 4;
					OffsetMicroMeters *= -1;
			}

			PALDevDataSet_2(Dev,
				Part2PartOffsetAdjustmentNVMMicroMeter,
				OffsetMicroMeters);
		}
		byte = (uint8_t)(ReadDataFromDeviceDone|option);
		VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev, ReadDataFromDeviceDone,
				byte);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}


uint32_t VL53L0X_calc_macro_period_ps_2(VL53L0X_DEV_2 Dev, uint8_t vcsel_period_pclks)
{
	uint64_t PLL_period_ps;
	uint32_t macro_period_vclks;
	uint32_t macro_period_ps;

	LOG_FUNCTION_START_2("");

	/* The above calculation will produce rounding errors,
	   therefore set fixed value
	*/
	PLL_period_ps = 1655;

	macro_period_vclks = 2304;
	macro_period_ps = (uint32_t)(macro_period_vclks
			* vcsel_period_pclks * PLL_period_ps);

	LOG_FUNCTION_END_2("");
	return macro_period_ps;
}

uint16_t VL53L0X_encode_timeout_2(uint32_t timeout_macro_clks)
{
	/*!
	 * Encode timeout in macro periods in (LSByte * 2^MSByte) + 1 format
	 */

	uint16_t encoded_timeout = 0;
	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (timeout_macro_clks > 0) {
		ls_byte = timeout_macro_clks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0) {
			ls_byte = ls_byte >> 1;
			ms_byte++;
		}

		encoded_timeout = (ms_byte << 8)
				+ (uint16_t) (ls_byte & 0x000000FF);
	}

	return encoded_timeout;

}

uint32_t VL53L0X_decode_timeout_2(uint16_t encoded_timeout)
{
	/*!
	 * Decode 16-bit timeout register value - format (LSByte * 2^MSByte) + 1
	 */

	uint32_t timeout_macro_clks = 0;

	timeout_macro_clks = ((uint32_t) (encoded_timeout & 0x00FF)
			<< (uint32_t) ((encoded_timeout & 0xFF00) >> 8)) + 1;

	return timeout_macro_clks;
}


/* To convert ms into register value */
uint32_t VL53L0X_calc_timeout_mclks_2(VL53L0X_DEV_2 Dev,
		uint32_t timeout_period_us,
		uint8_t vcsel_period_pclks)
{
	uint32_t macro_period_ps;
	uint32_t macro_period_ns;
	uint32_t timeout_period_mclks = 0;

	macro_period_ps = VL53L0X_calc_macro_period_ps_2(Dev, vcsel_period_pclks);
	macro_period_ns = (macro_period_ps + 500) / 1000;

	timeout_period_mclks =
		(uint32_t) (((timeout_period_us * 1000)
		+ (macro_period_ns / 2)) / macro_period_ns);

    return timeout_period_mclks;
}

/* To convert register value into us */
uint32_t VL53L0X_calc_timeout_us_2(VL53L0X_DEV_2 Dev,
		uint16_t timeout_period_mclks,
		uint8_t vcsel_period_pclks)
{
	uint32_t macro_period_ps;
	uint32_t macro_period_ns;
	uint32_t actual_timeout_period_us = 0;

	macro_period_ps = VL53L0X_calc_macro_period_ps_2(Dev, vcsel_period_pclks);
	macro_period_ns = (macro_period_ps + 500) / 1000;

	actual_timeout_period_us =
		((timeout_period_mclks * macro_period_ns) + 500) / 1000;

	return actual_timeout_period_us;
}


VL53L0X_Error_2 get_sequence_step_timeout_2(VL53L0X_DEV_2 Dev,
				VL53L0X_SequenceStepId_2 SequenceStepId,
				uint32_t *pTimeOutMicroSecs)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t CurrentVCSELPulsePeriodPClk;
	uint8_t EncodedTimeOutByte = 0;
	uint32_t TimeoutMicroSeconds = 0;
	uint16_t PreRangeEncodedTimeOut = 0;
	uint16_t MsrcTimeOutMClks;
	uint16_t PreRangeTimeOutMClks;
	uint16_t FinalRangeTimeOutMClks = 0;
	uint16_t FinalRangeEncodedTimeOut;
	VL53L0X_SchedulerSequenceSteps_t_2 SchedulerSequenceSteps;

	if ((SequenceStepId == VL53L0X_SEQUENCESTEP_TCC_2)	 ||
		(SequenceStepId == VL53L0X_SEQUENCESTEP_DSS_2)	 ||
		(SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC_2)) {

		Status = VL53L0X_GetVcselPulsePeriod_2(Dev,
					VL53L0X_VCSEL_PERIOD_PRE_RANGE_2,
					&CurrentVCSELPulsePeriodPClk);
		if (Status == VL53L0X_ERROR_NONE_2) {
			Status = VL53L0X_RdByte_2(Dev,
					VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP_2,
					&EncodedTimeOutByte);
		}
		MsrcTimeOutMClks = VL53L0X_decode_timeout_2(EncodedTimeOutByte);

		TimeoutMicroSeconds = VL53L0X_calc_timeout_us_2(Dev,
						MsrcTimeOutMClks,
						CurrentVCSELPulsePeriodPClk);
	} else if (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE_2) {
		/* Retrieve PRE-RANGE VCSEL Period */
		Status = VL53L0X_GetVcselPulsePeriod_2(Dev,
						VL53L0X_VCSEL_PERIOD_PRE_RANGE_2,
						&CurrentVCSELPulsePeriodPClk);

		/* Retrieve PRE-RANGE Timeout in Macro periods (MCLKS) */
		if (Status == VL53L0X_ERROR_NONE_2) {

			/* Retrieve PRE-RANGE VCSEL Period */
			Status = VL53L0X_GetVcselPulsePeriod_2(Dev,
					VL53L0X_VCSEL_PERIOD_PRE_RANGE_2,
					&CurrentVCSELPulsePeriodPClk);

			if (Status == VL53L0X_ERROR_NONE_2) {
				Status = VL53L0X_RdWord_2(Dev,
				VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI_2,
				&PreRangeEncodedTimeOut);
			}

			PreRangeTimeOutMClks = VL53L0X_decode_timeout_2(
					PreRangeEncodedTimeOut);

			TimeoutMicroSeconds = VL53L0X_calc_timeout_us_2(Dev,
					PreRangeTimeOutMClks,
					CurrentVCSELPulsePeriodPClk);
		}
	} else if (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE_2) {

		VL53L0X_GetSequenceStepEnables_2(Dev, &SchedulerSequenceSteps);
		PreRangeTimeOutMClks = 0;

		if (SchedulerSequenceSteps.PreRangeOn) {
			/* Retrieve PRE-RANGE VCSEL Period */
			Status = VL53L0X_GetVcselPulsePeriod_2(Dev,
				VL53L0X_VCSEL_PERIOD_PRE_RANGE_2,
				&CurrentVCSELPulsePeriodPClk);

			/* Retrieve PRE-RANGE Timeout in Macro periods
			 * (MCLKS) */
			if (Status == VL53L0X_ERROR_NONE_2) {
				Status = VL53L0X_RdWord_2(Dev,
				VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI_2,
				&PreRangeEncodedTimeOut);
				PreRangeTimeOutMClks = VL53L0X_decode_timeout_2(
						PreRangeEncodedTimeOut);
			}
		}

		if (Status == VL53L0X_ERROR_NONE_2) {
			/* Retrieve FINAL-RANGE VCSEL Period */
			Status = VL53L0X_GetVcselPulsePeriod_2(Dev,
					VL53L0X_VCSEL_PERIOD_FINAL_RANGE_2,
					&CurrentVCSELPulsePeriodPClk);
		}

		/* Retrieve FINAL-RANGE Timeout in Macro periods (MCLKS) */
		if (Status == VL53L0X_ERROR_NONE_2) {
			Status = VL53L0X_RdWord_2(Dev,
				VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI_2,
				&FinalRangeEncodedTimeOut);
			FinalRangeTimeOutMClks = VL53L0X_decode_timeout_2(
					FinalRangeEncodedTimeOut);
		}

		FinalRangeTimeOutMClks -= PreRangeTimeOutMClks;
		TimeoutMicroSeconds = VL53L0X_calc_timeout_us_2(Dev,
						FinalRangeTimeOutMClks,
						CurrentVCSELPulsePeriodPClk);
	}

	*pTimeOutMicroSecs = TimeoutMicroSeconds;

	return Status;
}


VL53L0X_Error_2 set_sequence_step_timeout_2(VL53L0X_DEV_2 Dev,
					VL53L0X_SequenceStepId_2 SequenceStepId,
					uint32_t TimeOutMicroSecs)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t CurrentVCSELPulsePeriodPClk;
	uint8_t MsrcEncodedTimeOut;
	uint16_t PreRangeEncodedTimeOut;
	uint16_t PreRangeTimeOutMClks;
	uint16_t MsrcRangeTimeOutMClks;
	uint32_t FinalRangeTimeOutMClks;
	uint16_t FinalRangeEncodedTimeOut;
	VL53L0X_SchedulerSequenceSteps_t_2 SchedulerSequenceSteps;

	if ((SequenceStepId == VL53L0X_SEQUENCESTEP_TCC_2)	 ||
		(SequenceStepId == VL53L0X_SEQUENCESTEP_DSS_2)	 ||
		(SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC_2)) {

		Status = VL53L0X_GetVcselPulsePeriod_2(Dev,
					VL53L0X_VCSEL_PERIOD_PRE_RANGE_2,
					&CurrentVCSELPulsePeriodPClk);

		if (Status == VL53L0X_ERROR_NONE_2) {
			MsrcRangeTimeOutMClks = VL53L0X_calc_timeout_mclks_2(Dev,
					TimeOutMicroSecs,
					(uint8_t)CurrentVCSELPulsePeriodPClk);

			if (MsrcRangeTimeOutMClks > 256)
				MsrcEncodedTimeOut = 255;
			else
				MsrcEncodedTimeOut =
					(uint8_t)MsrcRangeTimeOutMClks - 1;

			VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev,
				LastEncodedTimeout,
				MsrcEncodedTimeOut);
		}

		if (Status == VL53L0X_ERROR_NONE_2) {
			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP_2,
				MsrcEncodedTimeOut);
		}
	} else {

		if (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE_2) {

			if (Status == VL53L0X_ERROR_NONE_2) {
				Status = VL53L0X_GetVcselPulsePeriod_2(Dev,
						VL53L0X_VCSEL_PERIOD_PRE_RANGE_2,
						&CurrentVCSELPulsePeriodPClk);
				PreRangeTimeOutMClks =
					VL53L0X_calc_timeout_mclks_2(Dev,
					TimeOutMicroSecs,
					(uint8_t)CurrentVCSELPulsePeriodPClk);
				PreRangeEncodedTimeOut = VL53L0X_encode_timeout_2(
					PreRangeTimeOutMClks);

				VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev,
					LastEncodedTimeout,
					PreRangeEncodedTimeOut);
			}

			if (Status == VL53L0X_ERROR_NONE_2) {
				Status = VL53L0X_WrWord_2(Dev,
				VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI_2,
				PreRangeEncodedTimeOut);
			}

			if (Status == VL53L0X_ERROR_NONE_2) {
				VL53L0X_SETDEVICESPECIFICPARAMETER_2(
					Dev,
					PreRangeTimeoutMicroSecs,
					TimeOutMicroSecs);
			}
		} else if (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE_2) {

			/* For the final range timeout, the pre-range timeout
			 * must be added. To do this both final and pre-range
			 * timeouts must be expressed in macro periods MClks
			 * because they have different vcsel periods.
			 */

			VL53L0X_GetSequenceStepEnables_2(Dev,
					&SchedulerSequenceSteps);
			PreRangeTimeOutMClks = 0;
			if (SchedulerSequenceSteps.PreRangeOn) {

				/* Retrieve PRE-RANGE VCSEL Period */
				Status = VL53L0X_GetVcselPulsePeriod_2(Dev,
					VL53L0X_VCSEL_PERIOD_PRE_RANGE_2,
					&CurrentVCSELPulsePeriodPClk);

				/* Retrieve PRE-RANGE Timeout in Macro periods
				 * (MCLKS) */
				if (Status == VL53L0X_ERROR_NONE_2) {
					Status = VL53L0X_RdWord_2(Dev, 0x51,
						&PreRangeEncodedTimeOut);
					PreRangeTimeOutMClks =
						VL53L0X_decode_timeout_2(
							PreRangeEncodedTimeOut);
				}
			}

			/* Calculate FINAL RANGE Timeout in Macro Periods
			 * (MCLKS) and add PRE-RANGE value
			 */
			if (Status == VL53L0X_ERROR_NONE_2) {

				Status = VL53L0X_GetVcselPulsePeriod_2(Dev,
						VL53L0X_VCSEL_PERIOD_FINAL_RANGE_2,
						&CurrentVCSELPulsePeriodPClk);
			}
			if (Status == VL53L0X_ERROR_NONE_2) {

				FinalRangeTimeOutMClks =
					VL53L0X_calc_timeout_mclks_2(Dev,
					TimeOutMicroSecs,
					(uint8_t) CurrentVCSELPulsePeriodPClk);

				FinalRangeTimeOutMClks += PreRangeTimeOutMClks;

				FinalRangeEncodedTimeOut =
				VL53L0X_encode_timeout_2(FinalRangeTimeOutMClks);

				if (Status == VL53L0X_ERROR_NONE_2) {
					Status = VL53L0X_WrWord_2(Dev, 0x71,
					FinalRangeEncodedTimeOut);
				}

				if (Status == VL53L0X_ERROR_NONE_2) {
					VL53L0X_SETDEVICESPECIFICPARAMETER_2(
						Dev,
						FinalRangeTimeoutMicroSecs,
						TimeOutMicroSecs);
				}
			}
		} else
			Status = VL53L0X_ERROR_INVALID_PARAMS_2;

	}
	return Status;
}

VL53L0X_Error_2 VL53L0X_set_vcsel_pulse_period_2(VL53L0X_DEV_2 Dev,
	VL53L0X_VcselPeriod_2 VcselPeriodType, uint8_t VCSELPulsePeriodPCLK)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t vcsel_period_reg;
	uint8_t MinPreVcselPeriodPCLK = 12;
	uint8_t MaxPreVcselPeriodPCLK = 18;
	uint8_t MinFinalVcselPeriodPCLK = 8;
	uint8_t MaxFinalVcselPeriodPCLK = 14;
	uint32_t MeasurementTimingBudgetMicroSeconds_2;
	uint32_t FinalRangeTimeoutMicroSeconds;
	uint32_t PreRangeTimeoutMicroSeconds;
	uint32_t MsrcTimeoutMicroSeconds;
	uint8_t PhaseCalInt = 0;

	/* Check if valid clock period requested */

	if ((VCSELPulsePeriodPCLK % 2) != 0) {
		/* Value must be an even number */
		Status = VL53L0X_ERROR_INVALID_PARAMS_2;
	} else if (VcselPeriodType == VL53L0X_VCSEL_PERIOD_PRE_RANGE_2 &&
		(VCSELPulsePeriodPCLK < MinPreVcselPeriodPCLK ||
		VCSELPulsePeriodPCLK > MaxPreVcselPeriodPCLK)) {
		Status = VL53L0X_ERROR_INVALID_PARAMS_2;
	} else if (VcselPeriodType == VL53L0X_VCSEL_PERIOD_FINAL_RANGE_2 &&
		(VCSELPulsePeriodPCLK < MinFinalVcselPeriodPCLK ||
		 VCSELPulsePeriodPCLK > MaxFinalVcselPeriodPCLK)) {

		Status = VL53L0X_ERROR_INVALID_PARAMS_2;
	}

	/* Apply specific settings for the requested clock period */

	if (Status != VL53L0X_ERROR_NONE_2)
		return Status;


	if (VcselPeriodType == VL53L0X_VCSEL_PERIOD_PRE_RANGE_2) {

		/* Set phase check limits */
		if (VCSELPulsePeriodPCLK == 12) {

			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH_2,
				0x18);
			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW_2,
				0x08);
		} else if (VCSELPulsePeriodPCLK == 14) {

			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH_2,
				0x30);
			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW_2,
				0x08);
		} else if (VCSELPulsePeriodPCLK == 16) {

			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH_2,
				0x40);
			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW_2,
				0x08);
		} else if (VCSELPulsePeriodPCLK == 18) {

			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH_2,
				0x50);
			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW_2,
				0x08);
		}
	} else if (VcselPeriodType == VL53L0X_VCSEL_PERIOD_FINAL_RANGE_2) {

		if (VCSELPulsePeriodPCLK == 8) {

			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH_2,
				0x10);
			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW_2,
				0x08);

			Status |= VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH_2, 0x02);
			Status |= VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT_2, 0x0C);

			Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x01);
			Status |= VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_ALGO_PHASECAL_LIM_2,
				0x30);
			Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x00);
		} else if (VCSELPulsePeriodPCLK == 10) {

			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH_2,
				0x28);
			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW_2,
				0x08);

			Status |= VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH_2, 0x03);
			Status |= VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT_2, 0x09);

			Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x01);
			Status |= VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_ALGO_PHASECAL_LIM_2,
				0x20);
			Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x00);
		} else if (VCSELPulsePeriodPCLK == 12) {

			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH_2,
				0x38);
			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW_2,
				0x08);

			Status |= VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH_2, 0x03);
			Status |= VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT_2, 0x08);

			Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x01);
			Status |= VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_ALGO_PHASECAL_LIM_2,
				0x20);
			Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x00);
		} else if (VCSELPulsePeriodPCLK == 14) {

			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH_2,
				0x048);
			Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW_2,
				0x08);

			Status |= VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH_2, 0x03);
			Status |= VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT_2, 0x07);

			Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x01);
			Status |= VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_ALGO_PHASECAL_LIM_2,
				0x20);
			Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x00);
		}
	}


	/* Re-calculate and apply timeouts, in macro periods */

	if (Status == VL53L0X_ERROR_NONE_2) {
		vcsel_period_reg = VL53L0X_encode_vcsel_period_2((uint8_t)
			VCSELPulsePeriodPCLK);

		/* When the VCSEL period for the pre or final range is changed,
		* the corresponding timeout must be read from the device using
		* the current VCSEL period, then the new VCSEL period can be
		* applied. The timeout then must be written back to the device
		* using the new VCSEL period.
		*
		* For the MSRC timeout, the same applies - this timeout being
		* dependant on the pre-range vcsel period.
		*/
		switch (VcselPeriodType) {
		case VL53L0X_VCSEL_PERIOD_PRE_RANGE_2:
			Status = get_sequence_step_timeout_2(Dev,
				VL53L0X_SEQUENCESTEP_PRE_RANGE_2,
				&PreRangeTimeoutMicroSeconds);

			if (Status == VL53L0X_ERROR_NONE_2)
				Status = get_sequence_step_timeout_2(Dev,
					VL53L0X_SEQUENCESTEP_MSRC_2,
					&MsrcTimeoutMicroSeconds);

			if (Status == VL53L0X_ERROR_NONE_2)
				Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD_2,
					vcsel_period_reg);


			if (Status == VL53L0X_ERROR_NONE_2)
				Status = set_sequence_step_timeout_2(Dev,
					VL53L0X_SEQUENCESTEP_PRE_RANGE_2,
					PreRangeTimeoutMicroSeconds);


			if (Status == VL53L0X_ERROR_NONE_2)
				Status = set_sequence_step_timeout_2(Dev,
					VL53L0X_SEQUENCESTEP_MSRC_2,
					MsrcTimeoutMicroSeconds);

			VL53L0X_SETDEVICESPECIFICPARAMETER_2(
				Dev,
				PreRangeVcselPulsePeriod,
				VCSELPulsePeriodPCLK);
			break;
		case VL53L0X_VCSEL_PERIOD_FINAL_RANGE_2:
			Status = get_sequence_step_timeout_2(Dev,
				VL53L0X_SEQUENCESTEP_FINAL_RANGE_2,
				&FinalRangeTimeoutMicroSeconds);

			if (Status == VL53L0X_ERROR_NONE_2)
				Status = VL53L0X_WrByte_2(Dev,
				VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD_2,
					vcsel_period_reg);


			if (Status == VL53L0X_ERROR_NONE_2)
				Status = set_sequence_step_timeout_2(Dev,
					VL53L0X_SEQUENCESTEP_FINAL_RANGE_2,
					FinalRangeTimeoutMicroSeconds);

			VL53L0X_SETDEVICESPECIFICPARAMETER_2(
				Dev,
				FinalRangeVcselPulsePeriod,
				VCSELPulsePeriodPCLK);
			break;
		default:
			Status = VL53L0X_ERROR_INVALID_PARAMS_2;
		}
	}

	/* Finally, the timing budget must be re-applied */
	if (Status == VL53L0X_ERROR_NONE_2) {
		VL53L0X_GETPARAMETERFIELD_2(Dev,
			MeasurementTimingBudgetMicroSeconds_2,
			MeasurementTimingBudgetMicroSeconds_2);

		Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds_2(Dev,
				MeasurementTimingBudgetMicroSeconds_2);
	}

	/* Perform the phase calibration. This is needed after changing on
	 * vcsel period.
	 * get_data_enable = 0, restore_config = 1 */
	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_perform_phase_calibration_2(
			Dev, &PhaseCalInt, 0, 1);

	return Status;
}

VL53L0X_Error_2 VL53L0X_get_vcsel_pulse_period_2(VL53L0X_DEV_2 Dev,
	VL53L0X_VcselPeriod_2 VcselPeriodType, uint8_t *pVCSELPulsePeriodPCLK)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t vcsel_period_reg;

	switch (VcselPeriodType) {
	case VL53L0X_VCSEL_PERIOD_PRE_RANGE_2:
		Status = VL53L0X_RdByte_2(Dev,
			VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD_2,
			&vcsel_period_reg);
	break;
	case VL53L0X_VCSEL_PERIOD_FINAL_RANGE_2:
		Status = VL53L0X_RdByte_2(Dev,
			VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD_2,
			&vcsel_period_reg);
	break;
	default:
		Status = VL53L0X_ERROR_INVALID_PARAMS_2;
	}

	if (Status == VL53L0X_ERROR_NONE_2)
		*pVCSELPulsePeriodPCLK =
			VL53L0X_decode_vcsel_period_2(vcsel_period_reg);

	return Status;
}



VL53L0X_Error_2 VL53L0X_set_measurement_timing_budget_micro_seconds_2(VL53L0X_DEV_2 Dev,
		uint32_t MeasurementTimingBudgetMicroSeconds_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint32_t FinalRangeTimingBudgetMicroSeconds;
	VL53L0X_SchedulerSequenceSteps_t_2 SchedulerSequenceSteps;
	uint32_t MsrcDccTccTimeoutMicroSeconds	= 2000;
	uint32_t StartOverheadMicroSeconds		= 1910;
	uint32_t EndOverheadMicroSeconds		= 960;
	uint32_t MsrcOverheadMicroSeconds		= 660;
	uint32_t TccOverheadMicroSeconds		= 590;
	uint32_t DssOverheadMicroSeconds		= 690;
	uint32_t PreRangeOverheadMicroSeconds	= 660;
	uint32_t FinalRangeOverheadMicroSeconds = 550;
	uint32_t PreRangeTimeoutMicroSeconds	= 0;
	uint32_t cMinTimingBudgetMicroSeconds	= 20000;
	uint32_t SubTimeout = 0;

	LOG_FUNCTION_START_2("");

	if (MeasurementTimingBudgetMicroSeconds_2
			< cMinTimingBudgetMicroSeconds) {
		Status = VL53L0X_ERROR_INVALID_PARAMS_2;
		return Status;
	}

	FinalRangeTimingBudgetMicroSeconds =
		MeasurementTimingBudgetMicroSeconds_2 -
		(StartOverheadMicroSeconds + EndOverheadMicroSeconds);

	Status = VL53L0X_GetSequenceStepEnables_2(Dev, &SchedulerSequenceSteps);

	if (Status == VL53L0X_ERROR_NONE_2 &&
		(SchedulerSequenceSteps.TccOn  ||
		SchedulerSequenceSteps.MsrcOn ||
		SchedulerSequenceSteps.DssOn)) {

		/* TCC, MSRC and DSS all share the same timeout */
		Status = get_sequence_step_timeout_2(Dev,
					VL53L0X_SEQUENCESTEP_MSRC_2,
					&MsrcDccTccTimeoutMicroSeconds);

		/* Subtract the TCC, MSRC and DSS timeouts if they are
		 * enabled. */

		if (Status != VL53L0X_ERROR_NONE_2)
			return Status;

		/* TCC */
		if (SchedulerSequenceSteps.TccOn) {

			SubTimeout = MsrcDccTccTimeoutMicroSeconds
				+ TccOverheadMicroSeconds;

			if (SubTimeout <
				FinalRangeTimingBudgetMicroSeconds) {
				FinalRangeTimingBudgetMicroSeconds -=
							SubTimeout;
			} else {
				/* Requested timeout too big. */
				Status = VL53L0X_ERROR_INVALID_PARAMS_2;
			}
		}

		if (Status != VL53L0X_ERROR_NONE_2) {
			LOG_FUNCTION_END_2(Status);
			return Status;
		}

		/* DSS */
		if (SchedulerSequenceSteps.DssOn) {

			SubTimeout = 2 * (MsrcDccTccTimeoutMicroSeconds +
				DssOverheadMicroSeconds);

			if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
				FinalRangeTimingBudgetMicroSeconds
							-= SubTimeout;
			} else {
				/* Requested timeout too big. */
				Status = VL53L0X_ERROR_INVALID_PARAMS_2;
			}
		} else if (SchedulerSequenceSteps.MsrcOn) {
			/* MSRC */
			SubTimeout = MsrcDccTccTimeoutMicroSeconds +
						MsrcOverheadMicroSeconds;

			if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
				FinalRangeTimingBudgetMicroSeconds
							-= SubTimeout;
			} else {
				/* Requested timeout too big. */
				Status = VL53L0X_ERROR_INVALID_PARAMS_2;
			}
		}

	}

	if (Status != VL53L0X_ERROR_NONE_2) {
		LOG_FUNCTION_END_2(Status);
		return Status;
	}

	if (SchedulerSequenceSteps.PreRangeOn) {

		/* Subtract the Pre-range timeout if enabled. */

		Status = get_sequence_step_timeout_2(Dev,
				VL53L0X_SEQUENCESTEP_PRE_RANGE_2,
				&PreRangeTimeoutMicroSeconds);

		SubTimeout = PreRangeTimeoutMicroSeconds +
				PreRangeOverheadMicroSeconds;

		if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
			FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
		} else {
			/* Requested timeout too big. */
			Status = VL53L0X_ERROR_INVALID_PARAMS_2;
		}
	}


	if (Status == VL53L0X_ERROR_NONE_2 &&
		SchedulerSequenceSteps.FinalRangeOn) {

		FinalRangeTimingBudgetMicroSeconds -=
				FinalRangeOverheadMicroSeconds;

		/* Final Range Timeout
		 * Note that the final range timeout is determined by the timing
		 * budget and the sum of all other timeouts within the sequence.
		 * If there is no room for the final range timeout, then an error
		 * will be set. Otherwise the remaining time will be applied to
		 * the final range.
		 */
		Status = set_sequence_step_timeout_2(Dev,
			VL53L0X_SEQUENCESTEP_FINAL_RANGE_2,
			FinalRangeTimingBudgetMicroSeconds);

		VL53L0X_SETPARAMETERFIELD_2(Dev,
			MeasurementTimingBudgetMicroSeconds_2,
			MeasurementTimingBudgetMicroSeconds_2);
	}

	LOG_FUNCTION_END_2(Status);

	return Status;
}

VL53L0X_Error_2 VL53L0X_get_measurement_timing_budget_micro_seconds_2(VL53L0X_DEV_2 Dev,
		uint32_t *pMeasurementTimingBudgetMicroSeconds_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	VL53L0X_SchedulerSequenceSteps_t_2 SchedulerSequenceSteps;
	uint32_t FinalRangeTimeoutMicroSeconds;
	uint32_t MsrcDccTccTimeoutMicroSeconds	= 2000;
	uint32_t StartOverheadMicroSeconds		= 1910;
	uint32_t EndOverheadMicroSeconds		= 960;
	uint32_t MsrcOverheadMicroSeconds		= 660;
	uint32_t TccOverheadMicroSeconds		= 590;
	uint32_t DssOverheadMicroSeconds		= 690;
	uint32_t PreRangeOverheadMicroSeconds	= 660;
	uint32_t FinalRangeOverheadMicroSeconds = 550;
	uint32_t PreRangeTimeoutMicroSeconds	= 0;

	LOG_FUNCTION_START_2("");

	/* Start and end overhead times always present */
	*pMeasurementTimingBudgetMicroSeconds_2
		= StartOverheadMicroSeconds + EndOverheadMicroSeconds;

	Status = VL53L0X_GetSequenceStepEnables_2(Dev, &SchedulerSequenceSteps);

	if (Status != VL53L0X_ERROR_NONE_2) {
		LOG_FUNCTION_END_2(Status);
		return Status;
	}


	if (SchedulerSequenceSteps.TccOn  ||
		SchedulerSequenceSteps.MsrcOn ||
		SchedulerSequenceSteps.DssOn) {

		Status = get_sequence_step_timeout_2(Dev,
				VL53L0X_SEQUENCESTEP_MSRC_2,
				&MsrcDccTccTimeoutMicroSeconds);

		if (Status == VL53L0X_ERROR_NONE_2) {
			if (SchedulerSequenceSteps.TccOn) {
				*pMeasurementTimingBudgetMicroSeconds_2 +=
					MsrcDccTccTimeoutMicroSeconds +
					TccOverheadMicroSeconds;
			}

			if (SchedulerSequenceSteps.DssOn) {
				*pMeasurementTimingBudgetMicroSeconds_2 +=
				2 * (MsrcDccTccTimeoutMicroSeconds +
					DssOverheadMicroSeconds);
			} else if (SchedulerSequenceSteps.MsrcOn) {
				*pMeasurementTimingBudgetMicroSeconds_2 +=
					MsrcDccTccTimeoutMicroSeconds +
					MsrcOverheadMicroSeconds;
			}
		}
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		if (SchedulerSequenceSteps.PreRangeOn) {
			Status = get_sequence_step_timeout_2(Dev,
				VL53L0X_SEQUENCESTEP_PRE_RANGE_2,
				&PreRangeTimeoutMicroSeconds);
			*pMeasurementTimingBudgetMicroSeconds_2 +=
				PreRangeTimeoutMicroSeconds +
				PreRangeOverheadMicroSeconds;
		}
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		if (SchedulerSequenceSteps.FinalRangeOn) {
			Status = get_sequence_step_timeout_2(Dev,
					VL53L0X_SEQUENCESTEP_FINAL_RANGE_2,
					&FinalRangeTimeoutMicroSeconds);
			*pMeasurementTimingBudgetMicroSeconds_2 +=
				(FinalRangeTimeoutMicroSeconds +
				FinalRangeOverheadMicroSeconds);
		}
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		VL53L0X_SETPARAMETERFIELD_2(Dev,
			MeasurementTimingBudgetMicroSeconds_2,
			*pMeasurementTimingBudgetMicroSeconds_2);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}



VL53L0X_Error_2 VL53L0X_load_tuning_settings_2(VL53L0X_DEV_2 Dev,
		uint8_t *pTuningSettingBuffer)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	int i;
	int Index;
	uint8_t msb;
	uint8_t lsb;
	uint8_t SelectParam;
	uint8_t NumberOfWrites;
	uint8_t Address;
	uint8_t localBuffer[4]; /* max */
	uint16_t Temp16;

	LOG_FUNCTION_START_2("");

	Index = 0;

	while ((*(pTuningSettingBuffer + Index) != 0) &&
			(Status == VL53L0X_ERROR_NONE_2)) {
		NumberOfWrites = *(pTuningSettingBuffer + Index);
		Index++;
		if (NumberOfWrites == 0xFF) {
			/* internal parameters */
			SelectParam = *(pTuningSettingBuffer + Index);
			Index++;
			switch (SelectParam) {
			case 0: /* uint16_t SigmaEstRefArray -> 2 bytes */
				msb = *(pTuningSettingBuffer + Index);
				Index++;
				lsb = *(pTuningSettingBuffer + Index);
				Index++;
				Temp16 = VL53L0X_MAKEUINT16_2(lsb, msb);
				PALDevDataSet_2(Dev, SigmaEstRefArray, Temp16);
				break;
			case 1: /* uint16_t SigmaEstEffPulseWidth -> 2 bytes */
				msb = *(pTuningSettingBuffer + Index);
				Index++;
				lsb = *(pTuningSettingBuffer + Index);
				Index++;
				Temp16 = VL53L0X_MAKEUINT16_2(lsb, msb);
				PALDevDataSet_2(Dev, SigmaEstEffPulseWidth,
					Temp16);
				break;
			case 2: /* uint16_t SigmaEstEffAmbWidth -> 2 bytes */
				msb = *(pTuningSettingBuffer + Index);
				Index++;
				lsb = *(pTuningSettingBuffer + Index);
				Index++;
				Temp16 = VL53L0X_MAKEUINT16_2(lsb, msb);
				PALDevDataSet_2(Dev, SigmaEstEffAmbWidth, Temp16);
				break;
			case 3: /* uint16_t targetRefRate -> 2 bytes */
				msb = *(pTuningSettingBuffer + Index);
				Index++;
				lsb = *(pTuningSettingBuffer + Index);
				Index++;
				Temp16 = VL53L0X_MAKEUINT16_2(lsb, msb);
				PALDevDataSet_2(Dev, targetRefRate, Temp16);
				break;
			default: /* invalid parameter */
				Status = VL53L0X_ERROR_INVALID_PARAMS_2;
			}

		} else if (NumberOfWrites <= 4) {
			Address = *(pTuningSettingBuffer + Index);
			Index++;

			for (i = 0; i < NumberOfWrites; i++) {
				localBuffer[i] = *(pTuningSettingBuffer +
							Index);
				Index++;
			}

			Status = VL53L0X_WriteMulti_2(Dev, Address, localBuffer,
					NumberOfWrites);

		} else {
			Status = VL53L0X_ERROR_INVALID_PARAMS_2;
		}
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_get_total_xtalk_rate_2(VL53L0X_DEV_2 Dev,
	VL53L0X_RangingMeasurementData_t_2 *pRangingMeasurementData,
	FixPoint1616_t_2 *ptotal_xtalk_rate_mcps)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	uint8_t xtalkCompEnable;
	FixPoint1616_t_2 totalXtalkMegaCps;
	FixPoint1616_t_2 xtalkPerSpadMegaCps;

	*ptotal_xtalk_rate_mcps = 0;

	Status = VL53L0X_GetXTalkCompensationEnable_2(Dev, &xtalkCompEnable);
	if (Status == VL53L0X_ERROR_NONE_2) {

		if (xtalkCompEnable) {

			VL53L0X_GETPARAMETERFIELD_2(
				Dev,
				XTalkCompensationRateMegaCps_2,
				xtalkPerSpadMegaCps);

			/* FixPoint1616 * FixPoint 8:8 = FixPoint0824 */
			totalXtalkMegaCps =
				pRangingMeasurementData->EffectiveSpadRtnCount_2 *
				xtalkPerSpadMegaCps;

			/* FixPoint0824 >> 8 = FixPoint1616 */
			*ptotal_xtalk_rate_mcps =
				(totalXtalkMegaCps + 0x80) >> 8;
		}
	}

	return Status;
}

VL53L0X_Error_2 VL53L0X_get_total_signal_rate_2(VL53L0X_DEV_2 Dev,
	VL53L0X_RangingMeasurementData_t_2 *pRangingMeasurementData,
	FixPoint1616_t_2 *ptotal_signal_rate_mcps)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	FixPoint1616_t_2 totalXtalkMegaCps;

	LOG_FUNCTION_START_2("");

	*ptotal_signal_rate_mcps =
		pRangingMeasurementData->SignalRateRtnMegaCps_2;

	Status = VL53L0X_get_total_xtalk_rate_2(
		Dev, pRangingMeasurementData, &totalXtalkMegaCps);

	if (Status == VL53L0X_ERROR_NONE_2)
		*ptotal_signal_rate_mcps += totalXtalkMegaCps;

	return Status;
}

VL53L0X_Error_2 VL53L0X_calc_dmax_2(
	VL53L0X_DEV_2 Dev,
	FixPoint1616_t_2 totalSignalRate_mcps,
	FixPoint1616_t_2 totalCorrSignalRate_mcps,
	FixPoint1616_t_2 pwMult,
	uint32_t sigmaEstimateP1,
	FixPoint1616_t_2 sigmaEstimateP2,
	uint32_t peakVcselDuration_us,
	uint32_t *pdmax_mm)
{
	const uint32_t cSigmaLimit		= 18;
	const FixPoint1616_t_2 cSignalLimit	= 0x4000; /* 0.25 */
	const FixPoint1616_t_2 cSigmaEstRef	= 0x00000042; /* 0.001 */
	const uint32_t cAmbEffWidthSigmaEst_ns = 6;
	const uint32_t cAmbEffWidthDMax_ns	   = 7;
	uint32_t dmaxCalRange_mm;
	FixPoint1616_t_2 dmaxCalSignalRateRtn_mcps;
	FixPoint1616_t_2 minSignalNeeded;
	FixPoint1616_t_2 minSignalNeeded_p1;
	FixPoint1616_t_2 minSignalNeeded_p2;
	FixPoint1616_t_2 minSignalNeeded_p3;
	FixPoint1616_t_2 minSignalNeeded_p4;
	FixPoint1616_t_2 sigmaLimitTmp;
	FixPoint1616_t_2 sigmaEstSqTmp;
	FixPoint1616_t_2 signalLimitTmp;
	FixPoint1616_t_2 SignalAt0mm;
	FixPoint1616_t_2 dmaxDark;
	FixPoint1616_t_2 dmaxAmbient;
	FixPoint1616_t_2 dmaxDarkTmp;
	FixPoint1616_t_2 sigmaEstP2Tmp;
	uint32_t signalRateTemp_mcps;

	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("");

	dmaxCalRange_mm =
		PALDevDataGet_2(Dev, DmaxCalRangeMilliMeter);

	dmaxCalSignalRateRtn_mcps =
		PALDevDataGet_2(Dev, DmaxCalSignalRateRtnMegaCps);

	/* uint32 * FixPoint1616 = FixPoint1616 */
	SignalAt0mm = dmaxCalRange_mm * dmaxCalSignalRateRtn_mcps;

	/* FixPoint1616 >> 8 = FixPoint2408 */
	SignalAt0mm = (SignalAt0mm + 0x80) >> 8;
	SignalAt0mm *= dmaxCalRange_mm;

	minSignalNeeded_p1 = 0;
	if (totalCorrSignalRate_mcps > 0) {

		/* Shift by 10 bits to increase resolution prior to the
		 * division */
		signalRateTemp_mcps = totalSignalRate_mcps << 10;

		/* Add rounding value prior to division */
		minSignalNeeded_p1 = signalRateTemp_mcps +
			(totalCorrSignalRate_mcps/2);

		/* FixPoint0626/FixPoint1616 = FixPoint2210 */
		minSignalNeeded_p1 /= totalCorrSignalRate_mcps;

		/* Apply a factored version of the speed of light.
		 Correction to be applied at the end */
		minSignalNeeded_p1 *= 3;

		/* FixPoint2210 * FixPoint2210 = FixPoint1220 */
		minSignalNeeded_p1 *= minSignalNeeded_p1;

		/* FixPoint1220 >> 16 = FixPoint2804 */
		minSignalNeeded_p1 = (minSignalNeeded_p1 + 0x8000) >> 16;
	}

	minSignalNeeded_p2 = pwMult * sigmaEstimateP1;

	/* FixPoint1616 >> 16 =	 uint32 */
	minSignalNeeded_p2 = (minSignalNeeded_p2 + 0x8000) >> 16;

	/* uint32 * uint32	=  uint32 */
	minSignalNeeded_p2 *= minSignalNeeded_p2;

	/* Check sigmaEstimateP2
	 * If this value is too high there is not enough signal rate
	 * to calculate dmax value so set a suitable value to ensure
	 * a very small dmax.
	 */
	sigmaEstP2Tmp = (sigmaEstimateP2 + 0x8000) >> 16;
	sigmaEstP2Tmp = (sigmaEstP2Tmp + cAmbEffWidthSigmaEst_ns/2)/
		cAmbEffWidthSigmaEst_ns;
	sigmaEstP2Tmp *= cAmbEffWidthDMax_ns;

	if (sigmaEstP2Tmp > 0xffff) {
		minSignalNeeded_p3 = 0xfff00000;
	} else {

		/* DMAX uses a different ambient width from sigma, so apply
		 * correction.
		 * Perform division before multiplication to prevent overflow.
		 */
		sigmaEstimateP2 = (sigmaEstimateP2 + cAmbEffWidthSigmaEst_ns/2)/
			cAmbEffWidthSigmaEst_ns;
		sigmaEstimateP2 *= cAmbEffWidthDMax_ns;

		/* FixPoint1616 >> 16 = uint32 */
		minSignalNeeded_p3 = (sigmaEstimateP2 + 0x8000) >> 16;

		minSignalNeeded_p3 *= minSignalNeeded_p3;

	}

	/* FixPoint1814 / uint32 = FixPoint1814 */
	sigmaLimitTmp = ((cSigmaLimit << 14) + 500) / 1000;

	/* FixPoint1814 * FixPoint1814 = FixPoint3628 := FixPoint0428 */
	sigmaLimitTmp *= sigmaLimitTmp;

	/* FixPoint1616 * FixPoint1616 = FixPoint3232 */
	sigmaEstSqTmp = cSigmaEstRef * cSigmaEstRef;

	/* FixPoint3232 >> 4 = FixPoint0428 */
	sigmaEstSqTmp = (sigmaEstSqTmp + 0x08) >> 4;

	/* FixPoint0428 - FixPoint0428	= FixPoint0428 */
	sigmaLimitTmp -=  sigmaEstSqTmp;

	/* uint32_t * FixPoint0428 = FixPoint0428 */
	minSignalNeeded_p4 = 4 * 12 * sigmaLimitTmp;

	/* FixPoint0428 >> 14 = FixPoint1814 */
	minSignalNeeded_p4 = (minSignalNeeded_p4 + 0x2000) >> 14;

	/* uint32 + uint32 = uint32 */
	minSignalNeeded = (minSignalNeeded_p2 + minSignalNeeded_p3);

	/* uint32 / uint32 = uint32 */
	minSignalNeeded += (peakVcselDuration_us/2);
	minSignalNeeded /= peakVcselDuration_us;

	/* uint32 << 14 = FixPoint1814 */
	minSignalNeeded <<= 14;

	/* FixPoint1814 / FixPoint1814 = uint32 */
	minSignalNeeded += (minSignalNeeded_p4/2);
	minSignalNeeded /= minSignalNeeded_p4;

	/* FixPoint3200 * FixPoint2804 := FixPoint2804*/
	minSignalNeeded *= minSignalNeeded_p1;

	/* Apply correction by dividing by 1000000.
	 * This assumes 10E16 on the numerator of the equation
	 * and 10E-22 on the denominator.
	 * We do this because 32bit fix point calculation can't
	 * handle the larger and smaller elements of this equation,
	 * i.e. speed of light and pulse widths.
	 */
	minSignalNeeded = (minSignalNeeded + 500) / 1000;
	minSignalNeeded <<= 4;

	minSignalNeeded = (minSignalNeeded + 500) / 1000;

	/* FixPoint1616 >> 8 = FixPoint2408 */
	signalLimitTmp = (cSignalLimit + 0x80) >> 8;

	/* FixPoint2408/FixPoint2408 = uint32 */
	if (signalLimitTmp != 0)
		dmaxDarkTmp = (SignalAt0mm + (signalLimitTmp / 2))
			/ signalLimitTmp;
	else
		dmaxDarkTmp = 0;

	dmaxDark = VL53L0X_isqrt_2(dmaxDarkTmp);

	/* FixPoint2408/FixPoint2408 = uint32 */
	if (minSignalNeeded != 0)
		dmaxAmbient = (SignalAt0mm + minSignalNeeded/2)
			/ minSignalNeeded;
	else
		dmaxAmbient = 0;

	dmaxAmbient = VL53L0X_isqrt_2(dmaxAmbient);

	*pdmax_mm = dmaxDark;
	if (dmaxDark > dmaxAmbient)
		*pdmax_mm = dmaxAmbient;

	LOG_FUNCTION_END_2(Status);

	return Status;
}


VL53L0X_Error_2 VL53L0X_calc_sigma_estimate_2(VL53L0X_DEV_2 Dev,
	VL53L0X_RangingMeasurementData_t_2 *pRangingMeasurementData,
	FixPoint1616_t_2 *pSigmaEstimate,
	uint32_t *pDmax_mm)
{
	/* Expressed in 100ths of a ns, i.e. centi-ns */
	const uint32_t cPulseEffectiveWidth_centi_ns   = 800;
	/* Expressed in 100ths of a ns, i.e. centi-ns */
	const uint32_t cAmbientEffectiveWidth_centi_ns = 600;
	const FixPoint1616_t_2 cDfltFinalRangeIntegrationTimeMilliSecs	= 0x00190000; /* 25ms */
	const uint32_t cVcselPulseWidth_ps	= 4700; /* pico secs */
	const FixPoint1616_t_2 cSigmaEstMax	= 0x028F87AE;
	const FixPoint1616_t_2 cSigmaEstRtnMax	= 0xF000;
	const FixPoint1616_t_2 cAmbToSignalRatioMax = 0xF0000000/
		cAmbientEffectiveWidth_centi_ns;
	/* Time Of Flight per mm (6.6 pico secs) */
	const FixPoint1616_t_2 cTOF_per_mm_ps		= 0x0006999A;
	const uint32_t c16BitRoundingParam		= 0x00008000;
	const FixPoint1616_t_2 cMaxXTalk_kcps		= 0x00320000;
	const uint32_t cPllPeriod_ps			= 1655;

	uint32_t vcselTotalEventsRtn;
	uint32_t finalRangeTimeoutMicroSecs;
	uint32_t preRangeTimeoutMicroSecs;
	uint32_t finalRangeIntegrationTimeMilliSecs;
	FixPoint1616_t_2 sigmaEstimateP1;
	FixPoint1616_t_2 sigmaEstimateP2;
	FixPoint1616_t_2 sigmaEstimateP3;
	FixPoint1616_t_2 deltaT_ps;
	FixPoint1616_t_2 pwMult;
	FixPoint1616_t_2 sigmaEstRtn;
	FixPoint1616_t_2 sigmaEstimate;
	FixPoint1616_t_2 xTalkCorrection;
	FixPoint1616_t_2 ambientRate_kcps;
	FixPoint1616_t_2 peakSignalRate_kcps;
	FixPoint1616_t_2 xTalkCompRate_mcps;
	uint32_t xTalkCompRate_kcps;
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	FixPoint1616_t_2 diff1_mcps;
//	FixPoint1616_t_2 diff2_mcps;
	FixPoint1616_t_2 sqr1;
	FixPoint1616_t_2 sqr2;
	FixPoint1616_t_2 sqrSum;
	FixPoint1616_t_2 sqrtResult_centi_ns;
	FixPoint1616_t_2 sqrtResult;
	FixPoint1616_t_2 totalSignalRate_mcps;
	FixPoint1616_t_2 correctedSignalRate_mcps;
	FixPoint1616_t_2 sigmaEstRef;
	uint32_t vcselWidth;
	uint32_t finalRangeMacroPCLKS;
	uint32_t preRangeMacroPCLKS;
	uint32_t peakVcselDuration_us;
	uint8_t finalRangeVcselPCLKS;
	uint8_t preRangeVcselPCLKS;
	/*! \addtogroup calc_sigma_estimate
	 * @{
	 *
	 * Estimates the range sigma
	 */

	LOG_FUNCTION_START_2("");

	VL53L0X_GETPARAMETERFIELD_2(Dev, XTalkCompensationRateMegaCps_2,
			xTalkCompRate_mcps);

	/*
	 * We work in kcps rather than mcps as this helps keep within the
	 * confines of the 32 Fix1616 type.
	 */

	ambientRate_kcps =
		(pRangingMeasurementData->AmbientRateRtnMegaCps_2 * 1000) >> 16;

	correctedSignalRate_mcps =
		pRangingMeasurementData->SignalRateRtnMegaCps_2;


	Status = VL53L0X_get_total_signal_rate_2(
		Dev, pRangingMeasurementData, &totalSignalRate_mcps);
	Status = VL53L0X_get_total_xtalk_rate_2(
		Dev, pRangingMeasurementData, &xTalkCompRate_mcps);


	/* Signal rate measurement provided by device is the
	 * peak signal rate, not average.
	 */
	peakSignalRate_kcps = (totalSignalRate_mcps * 1000);
	peakSignalRate_kcps = (peakSignalRate_kcps + 0x8000) >> 16;

	xTalkCompRate_kcps = xTalkCompRate_mcps * 1000;

	if (xTalkCompRate_kcps > cMaxXTalk_kcps)
		xTalkCompRate_kcps = cMaxXTalk_kcps;

	if (Status == VL53L0X_ERROR_NONE_2) {

		/* Calculate final range macro periods */
		finalRangeTimeoutMicroSecs = VL53L0X_GETDEVICESPECIFICPARAMETER_2(
			Dev, FinalRangeTimeoutMicroSecs);

		finalRangeVcselPCLKS = VL53L0X_GETDEVICESPECIFICPARAMETER_2(
			Dev, FinalRangeVcselPulsePeriod);

		finalRangeMacroPCLKS = VL53L0X_calc_timeout_mclks_2(
			Dev, finalRangeTimeoutMicroSecs, finalRangeVcselPCLKS);

		/* Calculate pre-range macro periods */
		preRangeTimeoutMicroSecs = VL53L0X_GETDEVICESPECIFICPARAMETER_2(
			Dev, PreRangeTimeoutMicroSecs);

		preRangeVcselPCLKS = VL53L0X_GETDEVICESPECIFICPARAMETER_2(
			Dev, PreRangeVcselPulsePeriod);

		preRangeMacroPCLKS = VL53L0X_calc_timeout_mclks_2(
			Dev, preRangeTimeoutMicroSecs, preRangeVcselPCLKS);

		vcselWidth = 3;
		if (finalRangeVcselPCLKS == 8)
			vcselWidth = 2;


		peakVcselDuration_us = vcselWidth * 2048 *
			(preRangeMacroPCLKS + finalRangeMacroPCLKS);
		peakVcselDuration_us = (peakVcselDuration_us + 500)/1000;
		peakVcselDuration_us *= cPllPeriod_ps;
		peakVcselDuration_us = (peakVcselDuration_us + 500)/1000;

		/* Fix1616 >> 8 = Fix2408 */
		totalSignalRate_mcps = (totalSignalRate_mcps + 0x80) >> 8;

		/* Fix2408 * uint32 = Fix2408 */
		vcselTotalEventsRtn = totalSignalRate_mcps *
			peakVcselDuration_us;

		/* Fix2408 >> 8 = uint32 */
		vcselTotalEventsRtn = (vcselTotalEventsRtn + 0x80) >> 8;

		/* Fix2408 << 8 = Fix1616 = */
		totalSignalRate_mcps <<= 8;
	}

	if (Status != VL53L0X_ERROR_NONE_2) {
		LOG_FUNCTION_END_2(Status);
		return Status;
	}

	if (peakSignalRate_kcps == 0) {
		*pSigmaEstimate = cSigmaEstMax;
		PALDevDataSet_2(Dev, SigmaEstimate, cSigmaEstMax);
		*pDmax_mm = 0;
	} else {
		if (vcselTotalEventsRtn < 1)
			vcselTotalEventsRtn = 1;

		sigmaEstimateP1 = cPulseEffectiveWidth_centi_ns;

		/* ((FixPoint1616 << 16)* uint32)/uint32 = FixPoint1616 */
		sigmaEstimateP2 = (ambientRate_kcps << 16)/peakSignalRate_kcps;
		if (sigmaEstimateP2 > cAmbToSignalRatioMax) {
			/* Clip to prevent overflow. Will ensure safe
			 * max result. */
			sigmaEstimateP2 = cAmbToSignalRatioMax;
		}
		sigmaEstimateP2 *= cAmbientEffectiveWidth_centi_ns;

		sigmaEstimateP3 = 2 * VL53L0X_isqrt_2(vcselTotalEventsRtn * 12);

		/* uint32 * FixPoint1616 = FixPoint1616 */
		deltaT_ps = pRangingMeasurementData->RangeMilliMeter_2 *
					cTOF_per_mm_ps;

		/*
		 * vcselRate - xtalkCompRate
		 * (uint32 << 16) - FixPoint1616 = FixPoint1616.
		 * Divide result by 1000 to convert to mcps.
		 * 500 is added to ensure rounding when integer division
		 * truncates.
		 */
		diff1_mcps = (((peakSignalRate_kcps << 16) -
			2 * xTalkCompRate_kcps) + 500)/1000;

		/* vcselRate + xtalkCompRate */
//		diff2_mcps = ((peakSignalRate_kcps << 16) + 500)/1000;

		/* Shift by 8 bits to increase resolution prior to the
		 * division */
		diff1_mcps <<= 8;

		/* FixPoint0824/FixPoint1616 = FixPoint2408 */
//		xTalkCorrection	 = abs_2(diff1_mcps/diff2_mcps);

		/* FixPoint2408 << 8 = FixPoint1616 */
		xTalkCorrection <<= 8;

		if(pRangingMeasurementData->RangeStatus_2 != 0){
			pwMult = 1 << 16;
		} else {
			/* FixPoint1616/uint32 = FixPoint1616 */
			pwMult = deltaT_ps/cVcselPulseWidth_ps; /* smaller than 1.0f */

			/*
			 * FixPoint1616 * FixPoint1616 = FixPoint3232, however both
			 * values are small enough such that32 bits will not be
			 * exceeded.
			 */
			pwMult *= ((1 << 16) - xTalkCorrection);

			/* (FixPoint3232 >> 16) = FixPoint1616 */
			pwMult =  (pwMult + c16BitRoundingParam) >> 16;

			/* FixPoint1616 + FixPoint1616 = FixPoint1616 */
			pwMult += (1 << 16);

			/*
			 * At this point the value will be 1.xx, therefore if we square
			 * the value this will exceed 32 bits. To address this perform
			 * a single shift to the right before the multiplication.
			 */
			pwMult >>= 1;
			/* FixPoint1715 * FixPoint1715 = FixPoint3430 */
			pwMult = pwMult * pwMult;

			/* (FixPoint3430 >> 14) = Fix1616 */
			pwMult >>= 14;
		}

		/* FixPoint1616 * uint32 = FixPoint1616 */
		sqr1 = pwMult * sigmaEstimateP1;

		/* (FixPoint1616 >> 16) = FixPoint3200 */
		sqr1 = (sqr1 + 0x8000) >> 16;

		/* FixPoint3200 * FixPoint3200 = FixPoint6400 */
		sqr1 *= sqr1;

		sqr2 = sigmaEstimateP2;

		/* (FixPoint1616 >> 16) = FixPoint3200 */
		sqr2 = (sqr2 + 0x8000) >> 16;

		/* FixPoint3200 * FixPoint3200 = FixPoint6400 */
		sqr2 *= sqr2;

		/* FixPoint64000 + FixPoint6400 = FixPoint6400 */
		sqrSum = sqr1 + sqr2;

		/* SQRT(FixPoin6400) = FixPoint3200 */
		sqrtResult_centi_ns = VL53L0X_isqrt_2(sqrSum);

		/* (FixPoint3200 << 16) = FixPoint1616 */
		sqrtResult_centi_ns <<= 16;

		/*
		 * Note that the Speed Of Light is expressed in um per 1E-10
		 * seconds (2997) Therefore to get mm/ns we have to divide by
		 * 10000
		 */
		sigmaEstRtn = (((sqrtResult_centi_ns+50)/100) /
				sigmaEstimateP3);
		sigmaEstRtn		 *= VL53L0X_SPEED_OF_LIGHT_IN_AIR_2;

		/* Add 5000 before dividing by 10000 to ensure rounding. */
		sigmaEstRtn		 += 5000;
		sigmaEstRtn		 /= 10000;

		if (sigmaEstRtn > cSigmaEstRtnMax) {
			/* Clip to prevent overflow. Will ensure safe
			 * max result. */
			sigmaEstRtn = cSigmaEstRtnMax;
		}
		finalRangeIntegrationTimeMilliSecs =
			(finalRangeTimeoutMicroSecs + preRangeTimeoutMicroSecs + 500)/1000;

		/* sigmaEstRef = 1mm * 25ms/final range integration time (inc pre-range)
		 * sqrt(FixPoint1616/int) = FixPoint2408)
		 */
		sigmaEstRef =
			VL53L0X_isqrt_2((cDfltFinalRangeIntegrationTimeMilliSecs +
				finalRangeIntegrationTimeMilliSecs/2)/
				finalRangeIntegrationTimeMilliSecs);

		/* FixPoint2408 << 8 = FixPoint1616 */
		sigmaEstRef <<= 8;
		sigmaEstRef = (sigmaEstRef + 500)/1000;

		/* FixPoint1616 * FixPoint1616 = FixPoint3232 */
		sqr1 = sigmaEstRtn * sigmaEstRtn;
		/* FixPoint1616 * FixPoint1616 = FixPoint3232 */
		sqr2 = sigmaEstRef * sigmaEstRef;

		/* sqrt(FixPoint3232) = FixPoint1616 */
		sqrtResult = VL53L0X_isqrt_2((sqr1 + sqr2));
		/*
		 * Note that the Shift by 4 bits increases resolution prior to
		 * the sqrt, therefore the result must be shifted by 2 bits to
		 * the right to revert back to the FixPoint1616 format.
		 */

		sigmaEstimate	 = 1000 * sqrtResult;

		if ((peakSignalRate_kcps < 1) || (vcselTotalEventsRtn < 1) ||
				(sigmaEstimate > cSigmaEstMax)) {
				sigmaEstimate = cSigmaEstMax;
		}

		*pSigmaEstimate = (uint32_t)(sigmaEstimate);
		PALDevDataSet_2(Dev, SigmaEstimate, *pSigmaEstimate);
		Status = VL53L0X_calc_dmax_2(
			Dev,
			totalSignalRate_mcps,
			correctedSignalRate_mcps,
			pwMult,
			sigmaEstimateP1,
			sigmaEstimateP2,
			peakVcselDuration_us,
			pDmax_mm);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_get_pal_range_status_2(VL53L0X_DEV_2 Dev,
		uint8_t DeviceRangeStatus_2,
		FixPoint1616_t_2 SignalRate,
		uint16_t EffectiveSpadRtnCount_2,
		VL53L0X_RangingMeasurementData_t_2 *pRangingMeasurementData,
		uint8_t *pPalRangeStatus_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t NoneFlag;
	uint8_t SigmaLimitflag = 0;
	uint8_t SignalRefClipflag = 0;
	uint8_t RangeIgnoreThresholdflag = 0;
	uint8_t SigmaLimitCheckEnable = 0;
	uint8_t SignalRateFinalRangeLimitCheckEnable = 0;
	uint8_t SignalRefClipLimitCheckEnable = 0;
	uint8_t RangeIgnoreThresholdLimitCheckEnable = 0;
	FixPoint1616_t_2 SigmaEstimate;
	FixPoint1616_t_2 SigmaLimitValue;
	FixPoint1616_t_2 SignalRefClipValue;
	FixPoint1616_t_2 RangeIgnoreThresholdValue;
	FixPoint1616_t_2 SignalRatePerSpad;
	uint8_t DeviceRangeStatus_2Internal = 0;
	uint16_t tmpWord = 0;
	uint8_t Temp8;
	uint32_t Dmax_mm = 0;
	FixPoint1616_t_2 LastSignalRefMcps;

	LOG_FUNCTION_START_2("");


	/*
	 * VL53L0X has a good ranging when the value of the
	 * DeviceRangeStatus_2 = 11. This function will replace the value 0 with
	 * the value 11 in the DeviceRangeStatus_2.
	 * In addition, the SigmaEstimator is not included in the VL53L0X
	 * DeviceRangeStatus_2, this will be added in the PalRangeStatus_2.
	 */

	DeviceRangeStatus_2Internal = ((DeviceRangeStatus_2 & 0x78) >> 3);

	if (DeviceRangeStatus_2Internal == 0 ||
		DeviceRangeStatus_2Internal == 5 ||
		DeviceRangeStatus_2Internal == 7 ||
		DeviceRangeStatus_2Internal == 12 ||
		DeviceRangeStatus_2Internal == 13 ||
		DeviceRangeStatus_2Internal == 14 ||
		DeviceRangeStatus_2Internal == 15
			) {
		NoneFlag = 1;
	} else {
		NoneFlag = 0;
	}

	/*
	 * Check if Sigma limit is enabled, if yes then do comparison with limit
	 * value and put the result back into pPalRangeStatus_2.
	 */
	if (Status == VL53L0X_ERROR_NONE_2)
		Status =  VL53L0X_GetLimitCheckEnable_2(Dev,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2,
			&SigmaLimitCheckEnable);

	if ((SigmaLimitCheckEnable != 0) && (Status == VL53L0X_ERROR_NONE_2)) {
		/*
		* compute the Sigma and check with limit
		*/
		Status = VL53L0X_calc_sigma_estimate_2(
			Dev,
			pRangingMeasurementData,
			&SigmaEstimate,
			&Dmax_mm);
		if (Status == VL53L0X_ERROR_NONE_2)
			pRangingMeasurementData->RangeDMaxMilliMeter_2 = Dmax_mm;

		if (Status == VL53L0X_ERROR_NONE_2) {
			Status = VL53L0X_GetLimitCheckValue_2(Dev,
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2,
				&SigmaLimitValue);

			if ((SigmaLimitValue > 0) &&
				(SigmaEstimate > SigmaLimitValue))
					/* Limit Fail */
					SigmaLimitflag = 1;
		}
	}

	/*
	 * Check if Signal ref clip limit is enabled, if yes then do comparison
	 * with limit value and put the result back into pPalRangeStatus_2.
	 */
	if (Status == VL53L0X_ERROR_NONE_2)
		Status =  VL53L0X_GetLimitCheckEnable_2(Dev,
				VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2,
				&SignalRefClipLimitCheckEnable);

	if ((SignalRefClipLimitCheckEnable != 0) &&
			(Status == VL53L0X_ERROR_NONE_2)) {

		Status = VL53L0X_GetLimitCheckValue_2(Dev,
				VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2,
				&SignalRefClipValue);

		/* Read LastSignalRefMcps from device */
		if (Status == VL53L0X_ERROR_NONE_2)
			Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x01);

		if (Status == VL53L0X_ERROR_NONE_2)
			Status = VL53L0X_RdWord_2(Dev,
				VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF_2,
				&tmpWord);

		if (Status == VL53L0X_ERROR_NONE_2)
			Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x00);

		LastSignalRefMcps = VL53L0X_FIXPOINT97TOFIXPOINT1616_2(tmpWord);
		PALDevDataSet_2(Dev, LastSignalRefMcps, LastSignalRefMcps);

		if ((SignalRefClipValue > 0) &&
				(LastSignalRefMcps > SignalRefClipValue)) {
			/* Limit Fail */
			SignalRefClipflag = 1;
		}
	}

	/*
	 * Check if Signal ref clip limit is enabled, if yes then do comparison
	 * with limit value and put the result back into pPalRangeStatus_2.
	 * EffectiveSpadRtnCount_2 has a format 8.8
	 * If (Return signal rate < (1.5 x Xtalk x number of Spads)) : FAIL
	 */
	if (Status == VL53L0X_ERROR_NONE_2)
		Status =  VL53L0X_GetLimitCheckEnable_2(Dev,
				VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD_2,
				&RangeIgnoreThresholdLimitCheckEnable);

	if ((RangeIgnoreThresholdLimitCheckEnable != 0) &&
			(Status == VL53L0X_ERROR_NONE_2)) {

		/* Compute the signal rate per spad */
		if (EffectiveSpadRtnCount_2 == 0) {
			SignalRatePerSpad = 0;
		} else {
			SignalRatePerSpad = (FixPoint1616_t_2)((256 * SignalRate)
				/ EffectiveSpadRtnCount_2);
		}

		Status = VL53L0X_GetLimitCheckValue_2(Dev,
				VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD_2,
				&RangeIgnoreThresholdValue);

		if ((RangeIgnoreThresholdValue > 0) &&
			(SignalRatePerSpad < RangeIgnoreThresholdValue)) {
			/* Limit Fail add 2^6 to range status */
			RangeIgnoreThresholdflag = 1;
		}
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		if (NoneFlag == 1) {
			*pPalRangeStatus_2 = 255;	 /* NONE */
		} else if (DeviceRangeStatus_2Internal == 1 ||
					DeviceRangeStatus_2Internal == 2 ||
					DeviceRangeStatus_2Internal == 3) {
			*pPalRangeStatus_2 = 5; /* HW fail */
		} else if (DeviceRangeStatus_2Internal == 6 ||
					DeviceRangeStatus_2Internal == 9) {
			*pPalRangeStatus_2 = 4;  /* Phase fail */
		} else if (DeviceRangeStatus_2Internal == 8 ||
					DeviceRangeStatus_2Internal == 10 ||
					SignalRefClipflag == 1) {
			*pPalRangeStatus_2 = 3;  /* Min range */
		} else if (DeviceRangeStatus_2Internal == 4 ||
					RangeIgnoreThresholdflag == 1) {
			*pPalRangeStatus_2 = 2;  /* Signal Fail */
		} else if (SigmaLimitflag == 1) {
			*pPalRangeStatus_2 = 1;  /* Sigma	 Fail */
		} else {
			*pPalRangeStatus_2 = 0; /* Range Valid */
		}
	}

	/* DMAX only relevant during range error */
	if (*pPalRangeStatus_2 == 0)
		pRangingMeasurementData->RangeDMaxMilliMeter_2 = 0;

	/* fill the Limit Check Status */

	Status =  VL53L0X_GetLimitCheckEnable_2(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE_2,
			&SignalRateFinalRangeLimitCheckEnable);

	if (Status == VL53L0X_ERROR_NONE_2) {
		if ((SigmaLimitCheckEnable == 0) || (SigmaLimitflag == 1))
			Temp8 = 1;
		else
			Temp8 = 0;

		VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksStatus_2,
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2, Temp8);

		if ((DeviceRangeStatus_2Internal == 4) ||
				(SignalRateFinalRangeLimitCheckEnable == 0))
			Temp8 = 1;
		else
			Temp8 = 0;
		VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksStatus_2,
				VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE_2,
				Temp8);

		if ((SignalRefClipLimitCheckEnable == 0) ||
					(SignalRefClipflag == 1))
			Temp8 = 1;
		else
			Temp8 = 0;

		VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksStatus_2,
				VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2, Temp8);

		if ((RangeIgnoreThresholdLimitCheckEnable == 0) ||
				(RangeIgnoreThresholdflag == 1))
			Temp8 = 1;
		else
			Temp8 = 0;

		VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksStatus_2,
				VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD_2,
				Temp8);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;

}


