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
#include "vl53l0x_tuning_2.h"
#include "vl53l0x_interrupt_threshold_settings_2.h"
#include "vl53l0x_api_core_2.h"
#include "vl53l0x_api_calibration_2.h"
#include "vl53l0x_api_strings_2.h"

#ifndef __KERNEL__
#include <stdlib.h>
#endif
#define LOG_FUNCTION_START_2(fmt, ...) \
	_LOG_FUNCTION_START_2(TRACE_MODULE_API, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END_2(status, ...) \
	_LOG_FUNCTION_END_2(TRACE_MODULE_API, status, ##__VA_ARGS__)
#define LOG_FUNCTION_ENDFMT_2(status, fmt, ...) \
	_LOG_FUNCTION_ENDFMT_2(TRACE_MODULE_API, status, fmt, ##__VA_ARGS__)

#ifdef VL53L0X_LOG_ENABLE_2
#define trace_print_2(level, ...) trace_print_module_function_2(TRACE_MODULE_API, \
	level, TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#endif

/* Group PAL General Functions */

VL53L0X_Error_2 VL53L0X_GetVersion_2(VL53L0X_Version_t_2 *pVersion)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	pVersion->major = VL53L0X_IMPLEMENTATION_VER_MAJOR_2;
	pVersion->minor = VL53L0X_IMPLEMENTATION_VER_MINOR_2;
	pVersion->build = VL53L0X_IMPLEMENTATION_VER_SUB_2;

	pVersion->revision = VL53L0X_IMPLEMENTATION_VER_REVISION_2;

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetPalSpecVersion_2(VL53L0X_Version_t_2 *pPalSpecVersion)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("");

	pPalSpecVersion->major = VL53L0X_SPECIFICATION_VER_MAJOR_2;
	pPalSpecVersion->minor = VL53L0X_SPECIFICATION_VER_MINOR_2;
	pPalSpecVersion->build = VL53L0X_SPECIFICATION_VER_SUB_2;

	pPalSpecVersion->revision = VL53L0X_SPECIFICATION_VER_REVISION_2;

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetProductRevision_2(VL53L0X_DEV_2 Dev,
	uint8_t *pProductRevisionMajor, uint8_t *pProductRevisionMinor)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t revision_id;

	LOG_FUNCTION_START_2("");

	Status = VL53L0X_RdByte_2(Dev, VL53L0X_REG_IDENTIFICATION_REVISION_ID_2,
		&revision_id);
	*pProductRevisionMajor = 1;
	*pProductRevisionMinor = (revision_id & 0xF0) >> 4;

	LOG_FUNCTION_END_2(Status);
	return Status;

}

VL53L0X_Error_2 VL53L0X_GetDeviceInfo_2(VL53L0X_DEV_2 Dev,
	VL53L0X_DeviceInfo_t_2 *pVL53L0X_DeviceInfo)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_device_info_2(Dev, pVL53L0X_DeviceInfo);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetDeviceErrorStatus_2(VL53L0X_DEV_2 Dev,
	VL53L0X_DeviceError_2 *pDeviceErrorStatus)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t RangeStatus_2;

	LOG_FUNCTION_START_2("");

	Status = VL53L0X_RdByte_2(Dev, VL53L0X_REG_RESULT_RANGE_STATUS_2,
		&RangeStatus_2);

	*pDeviceErrorStatus = (VL53L0X_DeviceError_2)((RangeStatus_2 & 0x78) >> 3);

	LOG_FUNCTION_END_2(Status);
	return Status;
}


VL53L0X_Error_2 VL53L0X_GetDeviceErrorString_2(VL53L0X_DeviceError_2 ErrorCode,
	char *pDeviceErrorString)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_device_error_string_2(ErrorCode, pDeviceErrorString);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetRangeStatusString_2(uint8_t RangeStatus_2,
	char *pRangeStatus_2String)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_range_status_string_2(RangeStatus_2,
		pRangeStatus_2String);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetPalErrorString_2(VL53L0X_Error_2 PalErrorCode,
	char *pPalErrorString)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_pal_error_string_2(PalErrorCode, pPalErrorString);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetPalStateString_2(VL53L0X_State_2 PalStateCode,
	char *pPalStateString)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_pal_state_string_2(PalStateCode, pPalStateString);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetPalState_2(VL53L0X_DEV_2 Dev, VL53L0X_State_2 *pPalState)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	*pPalState = PALDevDataGet_2(Dev, PalState);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetPowerMode_2(VL53L0X_DEV_2 Dev, VL53L0X_PowerModes_2 PowerMode)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	/* Only level1 of Power mode exists */
	if ((PowerMode != VL53L0X_POWERMODE_STANDBY_LEVEL1_2)
		&& (PowerMode != VL53L0X_POWERMODE_IDLE_LEVEL1_2)) {
		Status = VL53L0X_ERROR_MODE_NOT_SUPPORTED_2;
	} else if (PowerMode == VL53L0X_POWERMODE_STANDBY_LEVEL1_2) {
		/* set the standby level1 of power mode */
		Status = VL53L0X_WrByte_2(Dev, 0x80, 0x00);
		if (Status == VL53L0X_ERROR_NONE_2) {
			/* Set PAL State to standby */
			PALDevDataSet_2(Dev, PalState, VL53L0X_STATE_STANDBY_2);
			PALDevDataSet_2(Dev, PowerMode,
				VL53L0X_POWERMODE_STANDBY_LEVEL1_2);
		}

	} else {
		/* VL53L0X_POWERMODE_IDLE_LEVEL1_2 */
		Status = VL53L0X_WrByte_2(Dev, 0x80, 0x00);
		if (Status == VL53L0X_ERROR_NONE_2)
			Status = VL53L0X_StaticInit_2(Dev);

		if (Status == VL53L0X_ERROR_NONE_2)
			PALDevDataSet_2(Dev, PowerMode,
				VL53L0X_POWERMODE_IDLE_LEVEL1_2);

	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetPowerMode_2(VL53L0X_DEV_2 Dev, VL53L0X_PowerModes_2 *pPowerMode)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t Byte;
	LOG_FUNCTION_START_2("");

	/* Only level1 of Power mode exists */
	Status = VL53L0X_RdByte_2(Dev, 0x80, &Byte);

	if (Status == VL53L0X_ERROR_NONE_2) {
		if (Byte == 1) {
			PALDevDataSet_2(Dev, PowerMode,
				VL53L0X_POWERMODE_IDLE_LEVEL1_2);
		} else {
			PALDevDataSet_2(Dev, PowerMode,
				VL53L0X_POWERMODE_STANDBY_LEVEL1_2);
		}
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetOffsetCalibrationDataMicroMeter_2(VL53L0X_DEV_2 Dev,
	int32_t OffsetCalibrationDataMicroMeter)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_set_offset_calibration_data_micro_meter_2(Dev,
		OffsetCalibrationDataMicroMeter);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetOffsetCalibrationDataMicroMeter_2(VL53L0X_DEV_2 Dev,
	int32_t *pOffsetCalibrationDataMicroMeter)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_offset_calibration_data_micro_meter_2(Dev,
		pOffsetCalibrationDataMicroMeter);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetLinearityCorrectiveGain_2(VL53L0X_DEV_2 Dev,
	int16_t LinearityCorrectiveGain)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	if ((LinearityCorrectiveGain < 0) || (LinearityCorrectiveGain > 1000))
		Status = VL53L0X_ERROR_INVALID_PARAMS_2;
	else {
		PALDevDataSet_2(Dev, LinearityCorrectiveGain,
			LinearityCorrectiveGain);

		if (LinearityCorrectiveGain != 1000) {
			/* Disable FW Xtalk */
			Status = VL53L0X_WrWord_2(Dev,
			VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS_2, 0);
		}
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetLinearityCorrectiveGain_2(VL53L0X_DEV_2 Dev,
	uint16_t *pLinearityCorrectiveGain)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	*pLinearityCorrectiveGain = PALDevDataGet_2(Dev, LinearityCorrectiveGain);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetGroupParamHold_2(VL53L0X_DEV_2 Dev, uint8_t GroupParamHold)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NOT_IMPLEMENTED_2;
	LOG_FUNCTION_START_2("");

	/* not implemented on VL53L0X */

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetUpperLimitMilliMeter_2(VL53L0X_DEV_2 Dev,
	uint16_t *pUpperLimitMilliMeter)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NOT_IMPLEMENTED_2;
	LOG_FUNCTION_START_2("");

	/* not implemented on VL53L0X */

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetTotalSignalRate_2(VL53L0X_DEV_2 Dev,
	FixPoint1616_t_2 *pTotalSignalRate)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	VL53L0X_RangingMeasurementData_t_2 LastRangeDataBuffer;

	LOG_FUNCTION_START_2("");

	LastRangeDataBuffer = PALDevDataGet_2(Dev, LastRangeMeasure);

	Status = VL53L0X_get_total_signal_rate_2(
		Dev, &LastRangeDataBuffer, pTotalSignalRate);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

/* End Group PAL General Functions */

/* Group PAL Init Functions */
VL53L0X_Error_2 VL53L0X_SetDeviceAddress_2(VL53L0X_DEV_2 Dev, uint8_t DeviceAddress)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_WrByte_2(Dev, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS_2,
		DeviceAddress / 2);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_DataInit_2(VL53L0X_DEV_2 Dev)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	VL53L0X_DeviceParameters_t_2 CurrentParameters;
	int i;
	uint8_t StopVariable;

	LOG_FUNCTION_START_2("");

	/* by default the I2C is running at 1V8 if you want to change it you
	 * need to include this define at compilation level.
	 Ä¬ÈÏÇé¿öÏÂ£¬I2CÔËÐÐÔÚ1V8£¬Èç¹ûÄãÏë¸Ä±äËü*ÐèÒªÔÚ±àÒë¼¶±ð°üº¬´Ë¶¨Òå*/
	
#ifdef USE_I2C_2V8
	Status = VL53L0X_UpdateByte_2(Dev,
		VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
		0xFE,
		0x01);
#endif

	/* Set I2C standard mode ÉèÖÃI2C±ê×¼Ä£Ê½*/
	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_WrByte_2(Dev, 0x88, 0x00);

	VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev, ReadDataFromDeviceDone, 0);

#ifdef USE_IQC_STATION
	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_apply_offset_adjustment(Dev);
#endif

	/* Default value is 1000 for Linearity Corrective Gain ÏßÐÔÐ£ÕýÔöÒæµÄÄ¬ÈÏÖµÊÇ1000*/
	PALDevDataSet_2(Dev, LinearityCorrectiveGain, 1000);

	/* Dmax default Parameter DmaxÄ¬ÈÏÖµ²ÎÊý*/
	PALDevDataSet_2(Dev, DmaxCalRangeMilliMeter, 400);
	PALDevDataSet_2(Dev, DmaxCalSignalRateRtnMegaCps,
		(FixPoint1616_t_2)((0x00016B85))); /* 1.42 No Cover Glass*/

	/* Set Default static parameters
	 *set first temporary values 9.44MHz * 65536 = 618660 ÉèÖÃÄ¬ÈÏ¾²Ì¬²ÎÊý
	 *ÉèÖÃµÚÒ»¸öÁÙÊ±Öµ9.44MHz * 65536 = 618660*/
	VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev, OscFrequencyMHz, 618660);

	/* Set Default XTalkCompensationRateMegaCps_2 to 0  */
	VL53L0X_SETPARAMETERFIELD_2(Dev, XTalkCompensationRateMegaCps_2, 0);

	/* Get default parameters */
	Status = VL53L0X_GetDeviceParameters_2(Dev, &CurrentParameters);
	if (Status == VL53L0X_ERROR_NONE_2) {
		/* initialize PAL values */
		CurrentParameters.DeviceMode = VL53L0X_DEVICEMODE_SINGLE_RANGING_2;
		CurrentParameters.HistogramMode = VL53L0X_HISTOGRAMMODE_DISABLED_2;
		PALDevDataSet_2(Dev, CurrentParameters, CurrentParameters);
	}

	/* Sigma estimator variable */
	PALDevDataSet_2(Dev, SigmaEstRefArray, 100);
	PALDevDataSet_2(Dev, SigmaEstEffPulseWidth, 900);
	PALDevDataSet_2(Dev, SigmaEstEffAmbWidth, 500);
	PALDevDataSet_2(Dev, targetRefRate, 0x0A00); /* 20 MCPS in 9:7 format */

	/* Use internal default settings */
	PALDevDataSet_2(Dev, UseInternalTuningSettings, 1);

	Status |= VL53L0X_WrByte_2(Dev, 0x80, 0x01);
	Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x01);
	Status |= VL53L0X_WrByte_2(Dev, 0x00, 0x00);
	Status |= VL53L0X_RdByte_2(Dev, 0x91, &StopVariable);
	PALDevDataSet_2(Dev, StopVariable, StopVariable);
	Status |= VL53L0X_WrByte_2(Dev, 0x00, 0x01);
	Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x00);
	Status |= VL53L0X_WrByte_2(Dev, 0x80, 0x00);

	/* Enable all check */
	for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS_2; i++) {
		if (Status == VL53L0X_ERROR_NONE_2)
			Status |= VL53L0X_SetLimitCheckEnable_2(Dev, i, 1);
		else
			break;

	}

	/* Disable the following checks */
	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_SetLimitCheckEnable_2(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2, 0);

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_SetLimitCheckEnable_2(Dev,
			VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD_2, 0);

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_SetLimitCheckEnable_2(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC_2, 0);

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_SetLimitCheckEnable_2(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE_2, 0);

	/* Limit default values */
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_SetLimitCheckValue_2(Dev,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2,
				(FixPoint1616_t_2)(18 * 65536));
	}
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_SetLimitCheckValue_2(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE_2,
				(FixPoint1616_t_2)(25 * 65536 / 100));
				/* 0.25 * 65536 */
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_SetLimitCheckValue_2(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2,
				(FixPoint1616_t_2)(35 * 65536));
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_SetLimitCheckValue_2(Dev,
			VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD_2,
				(FixPoint1616_t_2)(0 * 65536));
	}

	if (Status == VL53L0X_ERROR_NONE_2) {

		PALDevDataSet_2(Dev, SequenceConfig, 0xFF);
		Status = VL53L0X_WrByte_2(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG_2,
			0xFF);

		/* Set PAL state to tell that we are waiting for call to
		 * VL53L0X_StaticInit_2 */
		PALDevDataSet_2(Dev, PalState, VL53L0X_STATE_WAIT_STATICINIT_2);
	}

	if (Status == VL53L0X_ERROR_NONE_2)
		VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev, RefSpadsInitialised, 0);


	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetTuningSettingBuffer_2(VL53L0X_DEV_2 Dev,
	uint8_t *pTuningSettingBuffer, uint8_t UseInternalTuningSettings)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("");

	if (UseInternalTuningSettings == 1) {
		/* Force use internal settings */
		PALDevDataSet_2(Dev, UseInternalTuningSettings, 1);
	} else {

		/* check that the first byte is not 0 */
		if (*pTuningSettingBuffer != 0) {
			PALDevDataSet_2(Dev, pTuningSettingsPointer,
				pTuningSettingBuffer);
			PALDevDataSet_2(Dev, UseInternalTuningSettings, 0);

		} else {
			Status = VL53L0X_ERROR_INVALID_PARAMS_2;
		}
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetTuningSettingBuffer_2(VL53L0X_DEV_2 Dev,
	uint8_t **ppTuningSettingBuffer, uint8_t *pUseInternalTuningSettings)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("");

	*ppTuningSettingBuffer = PALDevDataGet_2(Dev, pTuningSettingsPointer);
	*pUseInternalTuningSettings = PALDevDataGet_2(Dev,
		UseInternalTuningSettings);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_StaticInit_2(VL53L0X_DEV_2 Dev)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	VL53L0X_DeviceParameters_t_2 CurrentParameters = {0};
	uint8_t *pTuningSettingBuffer;
	uint16_t tempword = 0;
	uint8_t tempbyte = 0;
	uint8_t UseInternalTuningSettings = 0;
	uint32_t count = 0;
	uint8_t isApertureSpads = 0;
	uint32_t refSpadCount = 0;
	uint8_t ApertureSpads = 0;
	uint8_t vcselPulsePeriodPCLK;
	uint32_t seqTimeoutMicroSecs;

	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_info_from_device_2(Dev, 1);

	/* set the ref spad from NVM */
	count	= (uint32_t)VL53L0X_GETDEVICESPECIFICPARAMETER_2(Dev,
		ReferenceSpadCount);
	ApertureSpads = VL53L0X_GETDEVICESPECIFICPARAMETER_2(Dev,
		ReferenceSpadType);

	/* NVM value invalid */
	if ((ApertureSpads > 1) ||
		((ApertureSpads == 1) && (count > 32)) ||
		((ApertureSpads == 0) && (count > 12)))
		Status = VL53L0X_perform_ref_spad_management_2(Dev, &refSpadCount,
			&isApertureSpads);
	else
		Status = VL53L0X_set_reference_spads_2(Dev, count, ApertureSpads);


	/* Initialize tuning settings buffer to prevent compiler warning. */
	pTuningSettingBuffer = DefaultTuningSettings_2;

	if (Status == VL53L0X_ERROR_NONE_2) {
		UseInternalTuningSettings = PALDevDataGet_2(Dev,
			UseInternalTuningSettings);

		if (UseInternalTuningSettings == 0)
			pTuningSettingBuffer = PALDevDataGet_2(Dev,
				pTuningSettingsPointer);
		else
			pTuningSettingBuffer = DefaultTuningSettings_2;

	}

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_load_tuning_settings_2(Dev, pTuningSettingBuffer);


	/* Set interrupt config to new sample ready */
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_SetGpioConfig_2(Dev, 0, 0,
		VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY_2,
		VL53L0X_INTERRUPTPOLARITY_LOW_2);
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x01);
		Status |= VL53L0X_RdWord_2(Dev, 0x84, &tempword);
		Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x00);
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev, OscFrequencyMHz,
			VL53L0X_FIXPOINT412TOFIXPOINT1616_2(tempword));
	}

	/* After static init, some device parameters may be changed,
	 * so update them */
	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_GetDeviceParameters_2(Dev, &CurrentParameters);


	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_GetFractionEnable_2(Dev, &tempbyte);
		if (Status == VL53L0X_ERROR_NONE_2)
			PALDevDataSet_2(Dev, RangeFractionalEnable, tempbyte);

	}

	if (Status == VL53L0X_ERROR_NONE_2)
		PALDevDataSet_2(Dev, CurrentParameters, CurrentParameters);


	/* read the sequence config and save it */
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_RdByte_2(Dev,
		VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG_2, &tempbyte);
		if (Status == VL53L0X_ERROR_NONE_2)
			PALDevDataSet_2(Dev, SequenceConfig, tempbyte);

	}

	/* Disable MSRC and TCC by default */
	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_SetSequenceStepEnable_2(Dev,
					VL53L0X_SEQUENCESTEP_TCC_2, 0);


	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_SetSequenceStepEnable_2(Dev,
		VL53L0X_SEQUENCESTEP_MSRC_2, 0);


	/* Set PAL State to standby */
	if (Status == VL53L0X_ERROR_NONE_2)
		PALDevDataSet_2(Dev, PalState, VL53L0X_STATE_IDLE_2);



	/* Store pre-range vcsel period */
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_GetVcselPulsePeriod_2(
			Dev,
			VL53L0X_VCSEL_PERIOD_PRE_RANGE_2,
			&vcselPulsePeriodPCLK);
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
			VL53L0X_SETDEVICESPECIFICPARAMETER_2(
				Dev,
				PreRangeVcselPulsePeriod,
				vcselPulsePeriodPCLK);
	}

	/* Store final-range vcsel period */
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_GetVcselPulsePeriod_2(
			Dev,
			VL53L0X_VCSEL_PERIOD_FINAL_RANGE_2,
			&vcselPulsePeriodPCLK);
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
			VL53L0X_SETDEVICESPECIFICPARAMETER_2(
				Dev,
				FinalRangeVcselPulsePeriod,
				vcselPulsePeriodPCLK);
	}

	/* Store pre-range timeout */
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = get_sequence_step_timeout_2(
			Dev,
			VL53L0X_SEQUENCESTEP_PRE_RANGE_2,
			&seqTimeoutMicroSecs);
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		VL53L0X_SETDEVICESPECIFICPARAMETER_2(
			Dev,
			PreRangeTimeoutMicroSecs,
			seqTimeoutMicroSecs);
	}

	/* Store final-range timeout */
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = get_sequence_step_timeout_2(
			Dev,
			VL53L0X_SEQUENCESTEP_FINAL_RANGE_2,
			&seqTimeoutMicroSecs);
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		VL53L0X_SETDEVICESPECIFICPARAMETER_2(
			Dev,
			FinalRangeTimeoutMicroSecs,
			seqTimeoutMicroSecs);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_WaitDeviceBooted_2(VL53L0X_DEV_2 Dev)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NOT_IMPLEMENTED_2;
	LOG_FUNCTION_START_2("");

	/* not implemented on VL53L0X */

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_ResetDevice_2(VL53L0X_DEV_2 Dev)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t Byte;
	LOG_FUNCTION_START_2("");

	/* Set reset bit */
	Status = VL53L0X_WrByte_2(Dev, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N_2,
		0x00);

	/* Wait for some time */
	if (Status == VL53L0X_ERROR_NONE_2) {
		do {
			Status = VL53L0X_RdByte_2(Dev,
			VL53L0X_REG_IDENTIFICATION_MODEL_ID_2, &Byte);
		} while (Byte != 0x00);
	}

	VL53L0X_PollingDelay_2(Dev);

	/* Release reset */
	Status = VL53L0X_WrByte_2(Dev, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N_2,
		0x01);

	/* Wait until correct boot-up of the device */
	if (Status == VL53L0X_ERROR_NONE_2) {
		do {
			Status = VL53L0X_RdByte_2(Dev,
			VL53L0X_REG_IDENTIFICATION_MODEL_ID_2, &Byte);
		} while (Byte == 0x00);
	}

	VL53L0X_PollingDelay_2(Dev);

	/* Set PAL State to VL53L0X_STATE_POWERDOWN_2 */
	if (Status == VL53L0X_ERROR_NONE_2)
		PALDevDataSet_2(Dev, PalState, VL53L0X_STATE_POWERDOWN_2);


	LOG_FUNCTION_END_2(Status);
	return Status;
}
/* End Group PAL Init Functions */

/* Group PAL Parameters Functions */
VL53L0X_Error_2 VL53L0X_SetDeviceParameters_2(VL53L0X_DEV_2 Dev,
	const VL53L0X_DeviceParameters_t_2 *pDeviceParameters)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	int i;
	LOG_FUNCTION_START_2("");
	Status = VL53L0X_SetDeviceMode_2(Dev, pDeviceParameters->DeviceMode);

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_SetInterMeasurementPeriodMilliSeconds_2(Dev,
			pDeviceParameters->InterMeasurementPeriodMilliSeconds_2);


	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_SetXTalkCompensationRateMegaCps_2(Dev,
			pDeviceParameters->XTalkCompensationRateMegaCps_2);


	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_SetOffsetCalibrationDataMicroMeter_2(Dev,
			pDeviceParameters->RangeOffsetMicroMeters_2);


	for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS_2; i++) {
		if (Status == VL53L0X_ERROR_NONE_2)
			Status |= VL53L0X_SetLimitCheckEnable_2(Dev, i,
				pDeviceParameters->LimitChecksEnable_2[i]);
		else
			break;

		if (Status == VL53L0X_ERROR_NONE_2)
			Status |= VL53L0X_SetLimitCheckValue_2(Dev, i,
				pDeviceParameters->LimitChecksValue_2[i]);
		else
			break;

	}

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_SetWrapAroundCheckEnable_2(Dev,
			pDeviceParameters->WrapAroundCheckEnable_2);

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds_2(Dev,
			pDeviceParameters->MeasurementTimingBudgetMicroSeconds_2);


	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetDeviceParameters_2(VL53L0X_DEV_2 Dev,
	VL53L0X_DeviceParameters_t_2 *pDeviceParameters)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	int i;

	LOG_FUNCTION_START_2("");

	Status = VL53L0X_GetDeviceMode_2(Dev, &(pDeviceParameters->DeviceMode));

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_GetInterMeasurementPeriodMilliSeconds_2(Dev,
		&(pDeviceParameters->InterMeasurementPeriodMilliSeconds_2));


	if (Status == VL53L0X_ERROR_NONE_2)
		pDeviceParameters->XTalkCompensationEnable_2 = 0;

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_GetXTalkCompensationRateMegaCps_2(Dev,
			&(pDeviceParameters->XTalkCompensationRateMegaCps_2));


	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_GetOffsetCalibrationDataMicroMeter_2(Dev,
			&(pDeviceParameters->RangeOffsetMicroMeters_2));


	if (Status == VL53L0X_ERROR_NONE_2) {
		for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS_2; i++) {
			/* get first the values, then the enables.
			 * VL53L0X_GetLimitCheckValue_2 will modify the enable
			 * flags
			 */
			if (Status == VL53L0X_ERROR_NONE_2) {
				Status |= VL53L0X_GetLimitCheckValue_2(Dev, i,
				&(pDeviceParameters->LimitChecksValue_2[i]));
			} else {
				break;
			}
			if (Status == VL53L0X_ERROR_NONE_2) {
				Status |= VL53L0X_GetLimitCheckEnable_2(Dev, i,
				&(pDeviceParameters->LimitChecksEnable_2[i]));
			} else {
				break;
			}
		}
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_GetWrapAroundCheckEnable_2(Dev,
			&(pDeviceParameters->WrapAroundCheckEnable_2));
	}

	/* Need to be done at the end as it uses VCSELPulsePeriod */
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_GetMeasurementTimingBudgetMicroSeconds_2(Dev,
		&(pDeviceParameters->MeasurementTimingBudgetMicroSeconds_2));
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetDeviceMode_2(VL53L0X_DEV_2 Dev, VL53L0X_DeviceModes_2 DeviceMode_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("%d", (int)DeviceMode_2);

	switch (DeviceMode_2) {
	case VL53L0X_DEVICEMODE_SINGLE_RANGING_2:
	case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING_2:
	case VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING_2:
	case VL53L0X_DEVICEMODE_GPIO_DRIVE_2:
	case VL53L0X_DEVICEMODE_GPIO_OSC_2:
		/* Supported modes */
		VL53L0X_SETPARAMETERFIELD_2(Dev, DeviceMode, DeviceMode_2);
		break;
	default:
		/* Unsupported mode */
		Status = VL53L0X_ERROR_MODE_NOT_SUPPORTED_2;
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetDeviceMode_2(VL53L0X_DEV_2 Dev,
	VL53L0X_DeviceModes_2 *pDeviceMode_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	VL53L0X_GETPARAMETERFIELD_2(Dev, DeviceMode, *pDeviceMode_2);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetRangeFractionEnable_2(VL53L0X_DEV_2 Dev,	uint8_t Enable)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("%d", (int)Enable);

	Status = VL53L0X_WrByte_2(Dev, VL53L0X_REG_SYSTEM_RANGE_CONFIG_2, Enable);

	if (Status == VL53L0X_ERROR_NONE_2)
		PALDevDataSet_2(Dev, RangeFractionalEnable, Enable);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetFractionEnable_2(VL53L0X_DEV_2 Dev, uint8_t *pEnabled)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_RdByte_2(Dev, VL53L0X_REG_SYSTEM_RANGE_CONFIG_2, pEnabled);

	if (Status == VL53L0X_ERROR_NONE_2)
		*pEnabled = (*pEnabled & 1);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetHistogramMode_2(VL53L0X_DEV_2 Dev,
	VL53L0X_HistogramModes_2 HistogramMode)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NOT_IMPLEMENTED_2;
	LOG_FUNCTION_START_2("");

	/* not implemented on VL53L0X */

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetHistogramMode_2(VL53L0X_DEV_2 Dev,
	VL53L0X_HistogramModes_2 *pHistogramMode)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NOT_IMPLEMENTED_2;
	LOG_FUNCTION_START_2("");

	/* not implemented on VL53L0X */

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetMeasurementTimingBudgetMicroSeconds_2(VL53L0X_DEV_2 Dev,
	uint32_t MeasurementTimingBudgetMicroSeconds_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_set_measurement_timing_budget_micro_seconds_2(Dev,
		MeasurementTimingBudgetMicroSeconds_2);

	LOG_FUNCTION_END_2(Status);

	return Status;
}

VL53L0X_Error_2 VL53L0X_GetMeasurementTimingBudgetMicroSeconds_2(VL53L0X_DEV_2 Dev,
	uint32_t *pMeasurementTimingBudgetMicroSeconds_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_measurement_timing_budget_micro_seconds_2(Dev,
		pMeasurementTimingBudgetMicroSeconds_2);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetVcselPulsePeriod_2(VL53L0X_DEV_2 Dev,
	VL53L0X_VcselPeriod_2 VcselPeriodType, uint8_t VCSELPulsePeriodPCLK)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_set_vcsel_pulse_period_2(Dev, VcselPeriodType,
		VCSELPulsePeriodPCLK);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetVcselPulsePeriod_2(VL53L0X_DEV_2 Dev,
	VL53L0X_VcselPeriod_2 VcselPeriodType, uint8_t *pVCSELPulsePeriodPCLK)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_vcsel_pulse_period_2(Dev, VcselPeriodType,
		pVCSELPulsePeriodPCLK);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetSequenceStepEnable_2(VL53L0X_DEV_2 Dev,
	VL53L0X_SequenceStepId_2 SequenceStepId, uint8_t SequenceStepEnabled)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t SequenceConfig = 0;
	uint8_t SequenceConfigNew = 0;
	uint32_t MeasurementTimingBudgetMicroSeconds_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_RdByte_2(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG_2,
		&SequenceConfig);

	SequenceConfigNew = SequenceConfig;

	if (Status == VL53L0X_ERROR_NONE_2) {
		if (SequenceStepEnabled == 1) {

			/* Enable requested sequence step
			 */
			switch (SequenceStepId) {
			case VL53L0X_SEQUENCESTEP_TCC_2:
				SequenceConfigNew |= 0x10;
				break;
			case VL53L0X_SEQUENCESTEP_DSS_2:
				SequenceConfigNew |= 0x28;
				break;
			case VL53L0X_SEQUENCESTEP_MSRC_2:
				SequenceConfigNew |= 0x04;
				break;
			case VL53L0X_SEQUENCESTEP_PRE_RANGE_2:
				SequenceConfigNew |= 0x40;
				break;
			case VL53L0X_SEQUENCESTEP_FINAL_RANGE_2:
				SequenceConfigNew |= 0x80;
				break;
			default:
				Status = VL53L0X_ERROR_INVALID_PARAMS_2;
			}
		} else {
			/* Disable requested sequence step
			 */
			switch (SequenceStepId) {
			case VL53L0X_SEQUENCESTEP_TCC_2:
				SequenceConfigNew &= 0xef;
				break;
			case VL53L0X_SEQUENCESTEP_DSS_2:
				SequenceConfigNew &= 0xd7;
				break;
			case VL53L0X_SEQUENCESTEP_MSRC_2:
				SequenceConfigNew &= 0xfb;
				break;
			case VL53L0X_SEQUENCESTEP_PRE_RANGE_2:
				SequenceConfigNew &= 0xbf;
				break;
			case VL53L0X_SEQUENCESTEP_FINAL_RANGE_2:
				SequenceConfigNew &= 0x7f;
				break;
			default:
				Status = VL53L0X_ERROR_INVALID_PARAMS_2;
			}
		}
	}

	if (SequenceConfigNew != SequenceConfig) {
		/* Apply New Setting */
		if (Status == VL53L0X_ERROR_NONE_2) {
			Status = VL53L0X_WrByte_2(Dev,
			VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG_2, SequenceConfigNew);
		}
		if (Status == VL53L0X_ERROR_NONE_2)
			PALDevDataSet_2(Dev, SequenceConfig, SequenceConfigNew);


		/* Recalculate timing budget */
		if (Status == VL53L0X_ERROR_NONE_2) {
			VL53L0X_GETPARAMETERFIELD_2(Dev,
				MeasurementTimingBudgetMicroSeconds_2,
				MeasurementTimingBudgetMicroSeconds_2);

			VL53L0X_SetMeasurementTimingBudgetMicroSeconds_2(Dev,
				MeasurementTimingBudgetMicroSeconds_2);
		}
	}

	LOG_FUNCTION_END_2(Status);

	return Status;
}

VL53L0X_Error_2 sequence_step_enabled_2(VL53L0X_DEV_2 Dev,
	VL53L0X_SequenceStepId_2 SequenceStepId, uint8_t SequenceConfig,
	uint8_t *pSequenceStepEnabled)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	*pSequenceStepEnabled = 0;
	LOG_FUNCTION_START_2("");

	switch (SequenceStepId) {
	case VL53L0X_SEQUENCESTEP_TCC_2:
		*pSequenceStepEnabled = (SequenceConfig & 0x10) >> 4;
		break;
	case VL53L0X_SEQUENCESTEP_DSS_2:
		*pSequenceStepEnabled = (SequenceConfig & 0x08) >> 3;
		break;
	case VL53L0X_SEQUENCESTEP_MSRC_2:
		*pSequenceStepEnabled = (SequenceConfig & 0x04) >> 2;
		break;
	case VL53L0X_SEQUENCESTEP_PRE_RANGE_2:
		*pSequenceStepEnabled = (SequenceConfig & 0x40) >> 6;
		break;
	case VL53L0X_SEQUENCESTEP_FINAL_RANGE_2:
		*pSequenceStepEnabled = (SequenceConfig & 0x80) >> 7;
		break;
	default:
		Status = VL53L0X_ERROR_INVALID_PARAMS_2;
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetSequenceStepEnable_2(VL53L0X_DEV_2 Dev,
	VL53L0X_SequenceStepId_2 SequenceStepId, uint8_t *pSequenceStepEnabled)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t SequenceConfig = 0;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_RdByte_2(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG_2,
		&SequenceConfig);

	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = sequence_step_enabled_2(Dev, SequenceStepId,
			SequenceConfig, pSequenceStepEnabled);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetSequenceStepEnables_2(VL53L0X_DEV_2 Dev,
	VL53L0X_SchedulerSequenceSteps_t_2 *pSchedulerSequenceSteps)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t SequenceConfig = 0;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_RdByte_2(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG_2,
		&SequenceConfig);

	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = sequence_step_enabled_2(Dev,
		VL53L0X_SEQUENCESTEP_TCC_2, SequenceConfig,
			&pSchedulerSequenceSteps->TccOn);
	}
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = sequence_step_enabled_2(Dev,
		VL53L0X_SEQUENCESTEP_DSS_2, SequenceConfig,
			&pSchedulerSequenceSteps->DssOn);
	}
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = sequence_step_enabled_2(Dev,
		VL53L0X_SEQUENCESTEP_MSRC_2, SequenceConfig,
			&pSchedulerSequenceSteps->MsrcOn);
	}
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = sequence_step_enabled_2(Dev,
		VL53L0X_SEQUENCESTEP_PRE_RANGE_2, SequenceConfig,
			&pSchedulerSequenceSteps->PreRangeOn);
	}
	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = sequence_step_enabled_2(Dev,
		VL53L0X_SEQUENCESTEP_FINAL_RANGE_2, SequenceConfig,
			&pSchedulerSequenceSteps->FinalRangeOn);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetNumberOfSequenceSteps_2(VL53L0X_DEV_2 Dev,
	uint8_t *pNumberOfSequenceSteps)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	*pNumberOfSequenceSteps = VL53L0X_SEQUENCESTEP_NUMBER_OF_CHECKS_2;

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetSequenceStepsInfo_2(VL53L0X_SequenceStepId_2 SequenceStepId,
	char *pSequenceStepsString)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_sequence_steps_info_2(
			SequenceStepId,
			pSequenceStepsString);

	LOG_FUNCTION_END_2(Status);

	return Status;
}

VL53L0X_Error_2 VL53L0X_SetSequenceStepTimeout_2(VL53L0X_DEV_2 Dev,
	VL53L0X_SequenceStepId_2 SequenceStepId, FixPoint1616_t_2 TimeOutMilliSecs)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	VL53L0X_Error_2 Status1 = VL53L0X_ERROR_NONE_2;
	uint32_t TimeoutMicroSeconds = ((TimeOutMilliSecs * 1000) + 0x8000)
		>> 16;
	uint32_t MeasurementTimingBudgetMicroSeconds_2;
	FixPoint1616_t_2 OldTimeOutMicroSeconds;

	LOG_FUNCTION_START_2("");

	/* Read back the current value in case we need to revert back to this.
	 */
	Status = get_sequence_step_timeout_2(Dev, SequenceStepId,
		&OldTimeOutMicroSeconds);

	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = set_sequence_step_timeout_2(Dev, SequenceStepId,
			TimeoutMicroSeconds);
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		VL53L0X_GETPARAMETERFIELD_2(Dev,
			MeasurementTimingBudgetMicroSeconds_2,
			MeasurementTimingBudgetMicroSeconds_2);

		/* At this point we don't know if the requested value is valid,
		 therefore proceed to update the entire timing budget and
		 if this fails, revert back to the previous value.
		 */
		Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds_2(Dev,
			MeasurementTimingBudgetMicroSeconds_2);

		if (Status != VL53L0X_ERROR_NONE_2) {
			Status1 = set_sequence_step_timeout_2(Dev, SequenceStepId,
				OldTimeOutMicroSeconds);

			if (Status1 == VL53L0X_ERROR_NONE_2) {
				Status1 =
				VL53L0X_SetMeasurementTimingBudgetMicroSeconds_2(
					Dev,
					MeasurementTimingBudgetMicroSeconds_2);
			}

			Status = Status1;
		}
	}

	LOG_FUNCTION_END_2(Status);

	return Status;
}

VL53L0X_Error_2 VL53L0X_GetSequenceStepTimeout_2(VL53L0X_DEV_2 Dev,
	VL53L0X_SequenceStepId_2 SequenceStepId, FixPoint1616_t_2 *pTimeOutMilliSecs)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint32_t TimeoutMicroSeconds;
	LOG_FUNCTION_START_2("");

	Status = get_sequence_step_timeout_2(Dev, SequenceStepId,
		&TimeoutMicroSeconds);
	if (Status == VL53L0X_ERROR_NONE_2) {
		TimeoutMicroSeconds <<= 8;
		*pTimeOutMilliSecs = (TimeoutMicroSeconds + 500)/1000;
		*pTimeOutMilliSecs <<= 8;
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetInterMeasurementPeriodMilliSeconds_2(VL53L0X_DEV_2 Dev,
	uint32_t InterMeasurementPeriodMilliSeconds_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint16_t osc_calibrate_val;
	uint32_t IMPeriodMilliSeconds;

	LOG_FUNCTION_START_2("");

	Status = VL53L0X_RdWord_2(Dev, VL53L0X_REG_OSC_CALIBRATE_VAL_2,
		&osc_calibrate_val);

	if (Status == VL53L0X_ERROR_NONE_2) {
		if (osc_calibrate_val != 0) {
			IMPeriodMilliSeconds =
				InterMeasurementPeriodMilliSeconds_2
					* osc_calibrate_val;
		} else {
			IMPeriodMilliSeconds =
				InterMeasurementPeriodMilliSeconds_2;
		}
		Status = VL53L0X_WrDWord_2(Dev,
		VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD_2,
			IMPeriodMilliSeconds);
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		VL53L0X_SETPARAMETERFIELD_2(Dev,
			InterMeasurementPeriodMilliSeconds_2,
			InterMeasurementPeriodMilliSeconds_2);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetInterMeasurementPeriodMilliSeconds_2(VL53L0X_DEV_2 Dev,
	uint32_t *pInterMeasurementPeriodMilliSeconds_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint16_t osc_calibrate_val;
	uint32_t IMPeriodMilliSeconds;

	LOG_FUNCTION_START_2("");

	Status = VL53L0X_RdWord_2(Dev, VL53L0X_REG_OSC_CALIBRATE_VAL_2,
		&osc_calibrate_val);

	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_RdDWord_2(Dev,
		VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD_2,
			&IMPeriodMilliSeconds);
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		if (osc_calibrate_val != 0) {
			*pInterMeasurementPeriodMilliSeconds_2 =
				IMPeriodMilliSeconds / osc_calibrate_val;
		}
		VL53L0X_SETPARAMETERFIELD_2(Dev,
			InterMeasurementPeriodMilliSeconds_2,
			*pInterMeasurementPeriodMilliSeconds_2);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetXTalkCompensationEnable_2(VL53L0X_DEV_2 Dev,
	uint8_t XTalkCompensationEnable_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	FixPoint1616_t_2 TempFix1616;
	uint16_t LinearityCorrectiveGain;

	LOG_FUNCTION_START_2("");

	LinearityCorrectiveGain = PALDevDataGet_2(Dev, LinearityCorrectiveGain);

	if ((XTalkCompensationEnable_2 == 0)
		|| (LinearityCorrectiveGain != 1000)) {
		TempFix1616 = 0;
	} else {
		VL53L0X_GETPARAMETERFIELD_2(Dev, XTalkCompensationRateMegaCps_2,
			TempFix1616);
	}

	/* the following register has a format 3.13 */
	Status = VL53L0X_WrWord_2(Dev,
	VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS_2,
		VL53L0X_FIXPOINT1616TOFIXPOINT313_2(TempFix1616));

	if (Status == VL53L0X_ERROR_NONE_2) {
		if (XTalkCompensationEnable_2 == 0) {
			VL53L0X_SETPARAMETERFIELD_2(Dev, XTalkCompensationEnable_2,
				0);
		} else {
			VL53L0X_SETPARAMETERFIELD_2(Dev, XTalkCompensationEnable_2,
				1);
		}
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetXTalkCompensationEnable_2(VL53L0X_DEV_2 Dev,
	uint8_t *pXTalkCompensationEnable_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t Temp8;
	LOG_FUNCTION_START_2("");

	VL53L0X_GETPARAMETERFIELD_2(Dev, XTalkCompensationEnable_2, Temp8);
	*pXTalkCompensationEnable_2 = Temp8;

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetXTalkCompensationRateMegaCps_2(VL53L0X_DEV_2 Dev,
	FixPoint1616_t_2 XTalkCompensationRateMegaCps_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t Temp8;
	uint16_t LinearityCorrectiveGain;
	uint16_t data;
	LOG_FUNCTION_START_2("");

	VL53L0X_GETPARAMETERFIELD_2(Dev, XTalkCompensationEnable_2, Temp8);
	LinearityCorrectiveGain = PALDevDataGet_2(Dev, LinearityCorrectiveGain);

	if (Temp8 == 0) { /* disabled write only internal value */
		VL53L0X_SETPARAMETERFIELD_2(Dev, XTalkCompensationRateMegaCps_2,
			XTalkCompensationRateMegaCps_2);
	} else {
		/* the following register has a format 3.13 */
		if (LinearityCorrectiveGain == 1000) {
			data = VL53L0X_FIXPOINT1616TOFIXPOINT313_2(
				XTalkCompensationRateMegaCps_2);
		} else {
			data = 0;
		}

		Status = VL53L0X_WrWord_2(Dev,
		VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS_2, data);

		if (Status == VL53L0X_ERROR_NONE_2) {
			VL53L0X_SETPARAMETERFIELD_2(Dev,
				XTalkCompensationRateMegaCps_2,
				XTalkCompensationRateMegaCps_2);
		}
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetXTalkCompensationRateMegaCps_2(VL53L0X_DEV_2 Dev,
	FixPoint1616_t_2 *pXTalkCompensationRateMegaCps_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint16_t Value;
	FixPoint1616_t_2 TempFix1616;

	LOG_FUNCTION_START_2("");

	Status = VL53L0X_RdWord_2(Dev,
	VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS_2, (uint16_t *)&Value);
	if (Status == VL53L0X_ERROR_NONE_2) {
		if (Value == 0) {
			/* the Xtalk is disabled return value from memory */
			VL53L0X_GETPARAMETERFIELD_2(Dev,
				XTalkCompensationRateMegaCps_2, TempFix1616);
			*pXTalkCompensationRateMegaCps_2 = TempFix1616;
			VL53L0X_SETPARAMETERFIELD_2(Dev, XTalkCompensationEnable_2,
				0);
		} else {
			TempFix1616 = VL53L0X_FIXPOINT313TOFIXPOINT1616_2(Value);
			*pXTalkCompensationRateMegaCps_2 = TempFix1616;
			VL53L0X_SETPARAMETERFIELD_2(Dev,
				XTalkCompensationRateMegaCps_2, TempFix1616);
			VL53L0X_SETPARAMETERFIELD_2(Dev, XTalkCompensationEnable_2,
				1);
		}
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetRefCalibration_2(VL53L0X_DEV_2 Dev, uint8_t VhvSettings,
	uint8_t PhaseCal)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_set_ref_calibration_2(Dev, VhvSettings, PhaseCal);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetRefCalibration_2(VL53L0X_DEV_2 Dev, uint8_t *pVhvSettings,
	uint8_t *pPhaseCal)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_ref_calibration_2(Dev, pVhvSettings, pPhaseCal);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

/*
 * CHECK LIMIT FUNCTIONS
 */

VL53L0X_Error_2 VL53L0X_GetNumberOfLimitCheck_2(uint16_t *pNumberOfLimitCheck)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	*pNumberOfLimitCheck = VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS_2;

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetLimitCheckInfo_2(VL53L0X_DEV_2 Dev, uint16_t LimitCheckId,
	char *pLimitCheckString)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_limit_check_info_2(Dev, LimitCheckId,
		pLimitCheckString);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetLimitCheckStatus_2(VL53L0X_DEV_2 Dev, uint16_t LimitCheckId,
	uint8_t *pLimitCheckStatus)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t Temp8;

	LOG_FUNCTION_START_2("");

	if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS_2) {
		Status = VL53L0X_ERROR_INVALID_PARAMS_2;
	} else {

		VL53L0X_GETARRAYPARAMETERFIELD_2(Dev, LimitChecksStatus_2,
			LimitCheckId, Temp8);

		*pLimitCheckStatus = Temp8;

	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetLimitCheckEnable_2(VL53L0X_DEV_2 Dev, uint16_t LimitCheckId,
	uint8_t LimitCheckEnable)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	FixPoint1616_t_2 TempFix1616 = 0;
	uint8_t LimitCheckEnableInt = 0;
	uint8_t LimitCheckDisable = 0;
	uint8_t Temp8;

	LOG_FUNCTION_START_2("");

	if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS_2) {
		Status = VL53L0X_ERROR_INVALID_PARAMS_2;
	} else {
		if (LimitCheckEnable == 0) {
			TempFix1616 = 0;
			LimitCheckEnableInt = 0;
			LimitCheckDisable = 1;

		} else {
			VL53L0X_GETARRAYPARAMETERFIELD_2(Dev, LimitChecksValue_2,
				LimitCheckId, TempFix1616);
			LimitCheckDisable = 0;
			/* this to be sure to have either 0 or 1 */
			LimitCheckEnableInt = 1;
		}

		switch (LimitCheckId) {

		case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2:
			/* internal computation: */
			VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksEnable_2,
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2,
				LimitCheckEnableInt);

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE_2:

			Status = VL53L0X_WrWord_2(Dev,
			VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_2,
				VL53L0X_FIXPOINT1616TOFIXPOINT97_2(TempFix1616));

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2:

			/* internal computation: */
			VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksEnable_2,
				VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2,
				LimitCheckEnableInt);

			break;

		case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD_2:

			/* internal computation: */
			VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksEnable_2,
				VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD_2,
				LimitCheckEnableInt);

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC_2:

			Temp8 = (uint8_t)(LimitCheckDisable << 1);
			Status = VL53L0X_UpdateByte_2(Dev,
				VL53L0X_REG_MSRC_CONFIG_CONTROL_2,
				0xFE, Temp8);

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE_2:

			Temp8 = (uint8_t)(LimitCheckDisable << 4);
			Status = VL53L0X_UpdateByte_2(Dev,
				VL53L0X_REG_MSRC_CONFIG_CONTROL_2,
				0xEF, Temp8);

			break;


		default:
			Status = VL53L0X_ERROR_INVALID_PARAMS_2;

		}

	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		if (LimitCheckEnable == 0) {
			VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksEnable_2,
				LimitCheckId, 0);
		} else {
			VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksEnable_2,
				LimitCheckId, 1);
		}
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetLimitCheckEnable_2(VL53L0X_DEV_2 Dev, uint16_t LimitCheckId,
	uint8_t *pLimitCheckEnable)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t Temp8;

	LOG_FUNCTION_START_2("");

	if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS_2) {
		Status = VL53L0X_ERROR_INVALID_PARAMS_2;
		*pLimitCheckEnable = 0;
	} else {
		VL53L0X_GETARRAYPARAMETERFIELD_2(Dev, LimitChecksEnable_2,
			LimitCheckId, Temp8);
		*pLimitCheckEnable = Temp8;
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetLimitCheckValue_2(VL53L0X_DEV_2 Dev, uint16_t LimitCheckId,
	FixPoint1616_t_2 LimitCheckValue)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t Temp8;

	LOG_FUNCTION_START_2("");

	VL53L0X_GETARRAYPARAMETERFIELD_2(Dev, LimitChecksEnable_2, LimitCheckId,
		Temp8);

	if (Temp8 == 0) { /* disabled write only internal value */
		VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksValue_2,
			LimitCheckId, LimitCheckValue);
	} else {

		switch (LimitCheckId) {

		case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2:
			/* internal computation: */
			VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksValue_2,
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2,
				LimitCheckValue);
			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE_2:

			Status = VL53L0X_WrWord_2(Dev,
			VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_2,
				VL53L0X_FIXPOINT1616TOFIXPOINT97_2(
					LimitCheckValue));

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2:

			/* internal computation: */
			VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksValue_2,
				VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2,
				LimitCheckValue);

			break;

		case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD_2:

			/* internal computation: */
			VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksValue_2,
				VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD_2,
				LimitCheckValue);

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC_2:
		case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE_2:

			Status = VL53L0X_WrWord_2(Dev,
				VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT_2,
				VL53L0X_FIXPOINT1616TOFIXPOINT97_2(
					LimitCheckValue));

			break;

		default:
			Status = VL53L0X_ERROR_INVALID_PARAMS_2;

		}

		if (Status == VL53L0X_ERROR_NONE_2) {
			VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, LimitChecksValue_2,
				LimitCheckId, LimitCheckValue);
		}
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetLimitCheckValue_2(VL53L0X_DEV_2 Dev, uint16_t LimitCheckId,
	FixPoint1616_t_2 *pLimitCheckValue)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t EnableZeroValue = 0;
	uint16_t Temp16;
	FixPoint1616_t_2 TempFix1616;

	LOG_FUNCTION_START_2("");

	switch (LimitCheckId) {

	case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2:
		/* internal computation: */
		VL53L0X_GETARRAYPARAMETERFIELD_2(Dev, LimitChecksValue_2,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2, TempFix1616);
		EnableZeroValue = 0;
		break;

	case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE_2:
		Status = VL53L0X_RdWord_2(Dev,
		VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_2,
			&Temp16);
		if (Status == VL53L0X_ERROR_NONE_2)
			TempFix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616_2(Temp16);


		EnableZeroValue = 1;
		break;

	case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2:
		/* internal computation: */
		VL53L0X_GETARRAYPARAMETERFIELD_2(Dev, LimitChecksValue_2,
			VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2, TempFix1616);
		EnableZeroValue = 0;
		break;

	case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD_2:
		/* internal computation: */
		VL53L0X_GETARRAYPARAMETERFIELD_2(Dev, LimitChecksValue_2,
			VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD_2, TempFix1616);
		EnableZeroValue = 0;
		break;

	case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC_2:
	case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE_2:
		Status = VL53L0X_RdWord_2(Dev,
			VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT_2,
			&Temp16);
		if (Status == VL53L0X_ERROR_NONE_2)
			TempFix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616_2(Temp16);


		EnableZeroValue = 0;
		break;

	default:
		Status = VL53L0X_ERROR_INVALID_PARAMS_2;

	}

	if (Status == VL53L0X_ERROR_NONE_2) {

		if (EnableZeroValue == 1) {

			if (TempFix1616 == 0) {
				/* disabled: return value from memory */
				VL53L0X_GETARRAYPARAMETERFIELD_2(Dev,
					LimitChecksValue_2, LimitCheckId,
					TempFix1616);
				*pLimitCheckValue = TempFix1616;
				VL53L0X_SETARRAYPARAMETERFIELD_2(Dev,
					LimitChecksEnable_2, LimitCheckId, 0);
			} else {
				*pLimitCheckValue = TempFix1616;
				VL53L0X_SETARRAYPARAMETERFIELD_2(Dev,
					LimitChecksValue_2, LimitCheckId,
					TempFix1616);
				VL53L0X_SETARRAYPARAMETERFIELD_2(Dev,
					LimitChecksEnable_2, LimitCheckId, 1);
			}
		} else {
			*pLimitCheckValue = TempFix1616;
		}
	}

	LOG_FUNCTION_END_2(Status);
	return Status;

}

VL53L0X_Error_2 VL53L0X_GetLimitCheckCurrent_2(VL53L0X_DEV_2 Dev, uint16_t LimitCheckId,
	FixPoint1616_t_2 *pLimitCheckCurrent)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	VL53L0X_RangingMeasurementData_t_2 LastRangeDataBuffer;

	LOG_FUNCTION_START_2("");

	if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS_2) {
		Status = VL53L0X_ERROR_INVALID_PARAMS_2;
	} else {
		switch (LimitCheckId) {
		case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2:
			/* Need to run a ranging to have the latest values */
			*pLimitCheckCurrent = PALDevDataGet_2(Dev, SigmaEstimate);

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE_2:
			/* Need to run a ranging to have the latest values */
			LastRangeDataBuffer = PALDevDataGet_2(Dev,
				LastRangeMeasure);
			*pLimitCheckCurrent =
				LastRangeDataBuffer.SignalRateRtnMegaCps_2;

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2:
			/* Need to run a ranging to have the latest values */
			*pLimitCheckCurrent = PALDevDataGet_2(Dev,
				LastSignalRefMcps);

			break;

		case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD_2:
			/* Need to run a ranging to have the latest values */
			LastRangeDataBuffer = PALDevDataGet_2(Dev,
				LastRangeMeasure);
			*pLimitCheckCurrent =
				LastRangeDataBuffer.SignalRateRtnMegaCps_2;

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC_2:
			/* Need to run a ranging to have the latest values */
			LastRangeDataBuffer = PALDevDataGet_2(Dev,
				LastRangeMeasure);
			*pLimitCheckCurrent =
				LastRangeDataBuffer.SignalRateRtnMegaCps_2;

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE_2:
			/* Need to run a ranging to have the latest values */
			LastRangeDataBuffer = PALDevDataGet_2(Dev,
				LastRangeMeasure);
			*pLimitCheckCurrent =
				LastRangeDataBuffer.SignalRateRtnMegaCps_2;

			break;

		default:
			Status = VL53L0X_ERROR_INVALID_PARAMS_2;
		}
	}

	LOG_FUNCTION_END_2(Status);
	return Status;

}

/*
 * WRAPAROUND Check
 */
VL53L0X_Error_2 VL53L0X_SetWrapAroundCheckEnable_2(VL53L0X_DEV_2 Dev,
	uint8_t WrapAroundCheckEnable_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t Byte;
	uint8_t WrapAroundCheckEnable_2Int;

	LOG_FUNCTION_START_2("");

	Status = VL53L0X_RdByte_2(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG_2, &Byte);
	if (WrapAroundCheckEnable_2 == 0) {
		/* Disable wraparound */
		Byte = Byte & 0x7F;
		WrapAroundCheckEnable_2Int = 0;
	} else {
		/*Enable wraparound */
		Byte = Byte | 0x80;
		WrapAroundCheckEnable_2Int = 1;
	}

	Status = VL53L0X_WrByte_2(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG_2, Byte);

	if (Status == VL53L0X_ERROR_NONE_2) {
		PALDevDataSet_2(Dev, SequenceConfig, Byte);
		VL53L0X_SETPARAMETERFIELD_2(Dev, WrapAroundCheckEnable_2,
			WrapAroundCheckEnable_2Int);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetWrapAroundCheckEnable_2(VL53L0X_DEV_2 Dev,
	uint8_t *pWrapAroundCheckEnable_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t data;

	LOG_FUNCTION_START_2("");

	Status = VL53L0X_RdByte_2(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG_2, &data);
	if (Status == VL53L0X_ERROR_NONE_2) {
		PALDevDataSet_2(Dev, SequenceConfig, data);
		if (data & (0x01 << 7))
			*pWrapAroundCheckEnable_2 = 0x01;
		else
			*pWrapAroundCheckEnable_2 = 0x00;
	}
	if (Status == VL53L0X_ERROR_NONE_2) {
		VL53L0X_SETPARAMETERFIELD_2(Dev, WrapAroundCheckEnable_2,
			*pWrapAroundCheckEnable_2);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetDmaxCalParameters_2(VL53L0X_DEV_2 Dev,
	uint16_t RangeMilliMeter_2, FixPoint1616_t_2 SignalRateRtnMegaCps_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	FixPoint1616_t_2 SignalRateRtnMegaCps_2Temp = 0;

	LOG_FUNCTION_START_2("");

	/* Check if one of input parameter is zero, in that case the
	 * value are get from NVM */
	if ((RangeMilliMeter_2 == 0) || (SignalRateRtnMegaCps_2 == 0)) {
		/* NVM parameters */
		/* Run VL53L0X_get_info_from_device_2 wit option 4 to get
		 * signal rate at 400 mm if the value have been already
		 * get this function will return with no access to device */
		VL53L0X_get_info_from_device_2(Dev, 4);

		SignalRateRtnMegaCps_2Temp = VL53L0X_GETDEVICESPECIFICPARAMETER_2(
			Dev, SignalRateMeasFixed400mm);

		PALDevDataSet_2(Dev, DmaxCalRangeMilliMeter, 400);
		PALDevDataSet_2(Dev, DmaxCalSignalRateRtnMegaCps,
			SignalRateRtnMegaCps_2Temp);
	} else {
		/* User parameters */
		PALDevDataSet_2(Dev, DmaxCalRangeMilliMeter, RangeMilliMeter_2);
		PALDevDataSet_2(Dev, DmaxCalSignalRateRtnMegaCps,
			SignalRateRtnMegaCps_2);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetDmaxCalParameters_2(VL53L0X_DEV_2 Dev,
	uint16_t *pRangeMilliMeter_2, FixPoint1616_t_2 *pSignalRateRtnMegaCps_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("");

	*pRangeMilliMeter_2 = PALDevDataGet_2(Dev, DmaxCalRangeMilliMeter);
	*pSignalRateRtnMegaCps_2 = PALDevDataGet_2(Dev,
		DmaxCalSignalRateRtnMegaCps);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

/* End Group PAL Parameters Functions */

/* Group PAL Measurement Functions */
VL53L0X_Error_2 VL53L0X_PerformSingleMeasurement_2(VL53L0X_DEV_2 Dev)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	VL53L0X_DeviceModes_2 DeviceMode_2;

	LOG_FUNCTION_START_2("");

	/* Get Current DeviceMode_2 */
	Status = VL53L0X_GetDeviceMode_2(Dev, &DeviceMode_2);

	/* Start immediately to run a single ranging measurement in case of
	 * single ranging or single histogram */
	if (Status == VL53L0X_ERROR_NONE_2
		&& DeviceMode_2 == VL53L0X_DEVICEMODE_SINGLE_RANGING_2)
		Status = VL53L0X_StartMeasurement_2(Dev);


	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_measurement_poll_for_completion_2(Dev);


	/* Change PAL State in case of single ranging or single histogram */
	if (Status == VL53L0X_ERROR_NONE_2
		&& DeviceMode_2 == VL53L0X_DEVICEMODE_SINGLE_RANGING_2)
		PALDevDataSet_2(Dev, PalState, VL53L0X_STATE_IDLE_2);


	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_PerformSingleHistogramMeasurement_2(VL53L0X_DEV_2 Dev,
	VL53L0X_HistogramMeasurementData_t_2 *pHistogramMeasurementData)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NOT_IMPLEMENTED_2;
	LOG_FUNCTION_START_2("");

	/* not implemented on VL53L0X */

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_PerformRefCalibration_2(VL53L0X_DEV_2 Dev, uint8_t *pVhvSettings,
	uint8_t *pPhaseCal)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_perform_ref_calibration_2(Dev, pVhvSettings,
		pPhaseCal, 1);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_PerformXTalkMeasurement_2(VL53L0X_DEV_2 Dev,
	uint32_t TimeoutMs, FixPoint1616_t_2 *pXtalkPerSpad,
	uint8_t *pAmbientTooHigh)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NOT_IMPLEMENTED_2;
	LOG_FUNCTION_START_2("");

	/* not implemented on VL53L0X */

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_PerformXTalkCalibration_2(VL53L0X_DEV_2 Dev,
	FixPoint1616_t_2 XTalkCalDistance,
	FixPoint1616_t_2 *pXTalkCompensationRateMegaCps_2)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_perform_xtalk_calibration_2(Dev, XTalkCalDistance,
		pXTalkCompensationRateMegaCps_2);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_PerformOffsetCalibration_2(VL53L0X_DEV_2 Dev,
	FixPoint1616_t_2 CalDistanceMilliMeter, int32_t *pOffsetMicroMeter)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_perform_offset_calibration_2(Dev, CalDistanceMilliMeter,
		pOffsetMicroMeter);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_CheckAndLoadInterruptSettings_2(VL53L0X_DEV_2 Dev,
	uint8_t StartNotStopFlag)
{
	uint8_t InterruptConfig;
	FixPoint1616_t_2 ThresholdLow;
	FixPoint1616_t_2 ThresholdHigh;
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	InterruptConfig = VL53L0X_GETDEVICESPECIFICPARAMETER_2(Dev,
		Pin0GpioFunctionality);

	if ((InterruptConfig ==
		VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW_2) ||
		(InterruptConfig ==
		VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH_2) ||
		(InterruptConfig ==
		VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT_2)) {

		Status = VL53L0X_GetInterruptThresholds_2(Dev,
			VL53L0X_DEVICEMODE_CONTINUOUS_RANGING_2,
			&ThresholdLow, &ThresholdHigh);

		if (((ThresholdLow > 255*65536) ||
			(ThresholdHigh > 255*65536)) &&
			(Status == VL53L0X_ERROR_NONE_2)) {

			if (StartNotStopFlag != 0) {
				Status = VL53L0X_load_tuning_settings_2(Dev,
					InterruptThresholdSettings_2);
			} else {
				Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x04);
				Status |= VL53L0X_WrByte_2(Dev, 0x70, 0x00);
				Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x00);
				Status |= VL53L0X_WrByte_2(Dev, 0x80, 0x00);
			}

		}


	}

	return Status;

}


VL53L0X_Error_2 VL53L0X_StartMeasurement_2(VL53L0X_DEV_2 Dev)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	VL53L0X_DeviceModes_2 DeviceMode_2;
	uint8_t Byte;
	uint8_t StartStopByte = VL53L0X_REG_SYSRANGE_MODE_START_STOP_2;
	uint32_t LoopNb;
	LOG_FUNCTION_START_2("");

	/* Get Current DeviceMode_2 */
	VL53L0X_GetDeviceMode_2(Dev, &DeviceMode_2);

	Status = VL53L0X_WrByte_2(Dev, 0x80, 0x01);
	Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x01);
	Status = VL53L0X_WrByte_2(Dev, 0x00, 0x00);
	Status = VL53L0X_WrByte_2(Dev, 0x91, PALDevDataGet_2(Dev, StopVariable));
	Status = VL53L0X_WrByte_2(Dev, 0x00, 0x01);
	Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x00);
	Status = VL53L0X_WrByte_2(Dev, 0x80, 0x00);

	switch (DeviceMode_2) {
	case VL53L0X_DEVICEMODE_SINGLE_RANGING_2:
		Status = VL53L0X_WrByte_2(Dev, VL53L0X_REG_SYSRANGE_START_2, 0x01);

		Byte = StartStopByte;
		if (Status == VL53L0X_ERROR_NONE_2) {
			/* Wait until start bit has been cleared */
			LoopNb = 0;
			do {
				if (LoopNb > 0)
					Status = VL53L0X_RdByte_2(Dev,
					VL53L0X_REG_SYSRANGE_START_2, &Byte);
				LoopNb = LoopNb + 1;
			} while (((Byte & StartStopByte) == StartStopByte)
				&& (Status == VL53L0X_ERROR_NONE_2)
				&& (LoopNb < VL53L0X_DEFAULT_MAX_LOOP_2));

			if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP_2)
				Status = VL53L0X_ERROR_TIME_OUT_2;

		}

		break;
	case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING_2:
		/* Back-to-back mode */

		/* Check if need to apply interrupt settings */
		if (Status == VL53L0X_ERROR_NONE_2)
			Status = VL53L0X_CheckAndLoadInterruptSettings_2(Dev, 1);

		Status = VL53L0X_WrByte_2(Dev,
		VL53L0X_REG_SYSRANGE_START_2,
		VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK_2);
		if (Status == VL53L0X_ERROR_NONE_2) {
			/* Set PAL State to Running */
			PALDevDataSet_2(Dev, PalState, VL53L0X_STATE_RUNNING_2);
		}
		break;
	case VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING_2:
		/* Continuous mode */
		/* Check if need to apply interrupt settings */
		if (Status == VL53L0X_ERROR_NONE_2)
			Status = VL53L0X_CheckAndLoadInterruptSettings_2(Dev, 1);

		Status = VL53L0X_WrByte_2(Dev,
		VL53L0X_REG_SYSRANGE_START_2,
		VL53L0X_REG_SYSRANGE_MODE_TIMED_2);

		if (Status == VL53L0X_ERROR_NONE_2) {
			/* Set PAL State to Running */
			PALDevDataSet_2(Dev, PalState, VL53L0X_STATE_RUNNING_2);
		}
		break;
	default:
		/* Selected mode not supported */
		Status = VL53L0X_ERROR_MODE_NOT_SUPPORTED_2;
	}


	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_StopMeasurement_2(VL53L0X_DEV_2 Dev)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_WrByte_2(Dev, VL53L0X_REG_SYSRANGE_START_2,
	VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT_2);

	Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x01);
	Status = VL53L0X_WrByte_2(Dev, 0x00, 0x00);
	Status = VL53L0X_WrByte_2(Dev, 0x91, 0x00);
	Status = VL53L0X_WrByte_2(Dev, 0x00, 0x01);
	Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x00);

	if (Status == VL53L0X_ERROR_NONE_2) {
		/* Set PAL State to Idle */
		PALDevDataSet_2(Dev, PalState, VL53L0X_STATE_IDLE_2);
	}

	/* Check if need to apply interrupt settings */
	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_CheckAndLoadInterruptSettings_2(Dev, 0);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetMeasurementDataReady_2(VL53L0X_DEV_2 Dev,
	uint8_t *pMeasurementDataReady)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t SysRangeStatus_2Register;
	uint8_t InterruptConfig;
	uint32_t InterruptMask;
	LOG_FUNCTION_START_2("");

	InterruptConfig = VL53L0X_GETDEVICESPECIFICPARAMETER_2(Dev,
		Pin0GpioFunctionality);

	if (InterruptConfig ==
		VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY_2) {
		Status = VL53L0X_GetInterruptMaskStatus_2(Dev, &InterruptMask);
		if (InterruptMask ==
		VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY_2)
			*pMeasurementDataReady = 1;
		else
			*pMeasurementDataReady = 0;
	} else {
		Status = VL53L0X_RdByte_2(Dev, VL53L0X_REG_RESULT_RANGE_STATUS_2,
			&SysRangeStatus_2Register);
		if (Status == VL53L0X_ERROR_NONE_2) {
			if (SysRangeStatus_2Register & 0x01)
				*pMeasurementDataReady = 1;
			else
				*pMeasurementDataReady = 0;
		}
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_WaitDeviceReadyForNewMeasurement_2(VL53L0X_DEV_2 Dev,
	uint32_t MaxLoop)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NOT_IMPLEMENTED_2;
	LOG_FUNCTION_START_2("");

	/* not implemented for VL53L0X */

	LOG_FUNCTION_END_2(Status);
	return Status;
}


VL53L0X_Error_2 VL53L0X_GetRangingMeasurementData_2(VL53L0X_DEV_2 Dev,
	VL53L0X_RangingMeasurementData_t_2 *pRangingMeasurementData)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t DeviceRangeStatus_2;
	uint8_t RangeFractionalEnable;
	uint8_t PalRangeStatus_2;
	uint8_t XTalkCompensationEnable_2;
	uint16_t AmbientRate;
	FixPoint1616_t_2 SignalRate;
	uint16_t XTalkCompensationRateMegaCps_2;
	uint16_t EffectiveSpadRtnCount_2;
	uint16_t tmpuint16;
	uint16_t XtalkRangeMilliMeter_2;
	uint16_t LinearityCorrectiveGain;
	uint8_t localBuffer[12];
	VL53L0X_RangingMeasurementData_t_2 LastRangeDataBuffer;

	LOG_FUNCTION_START_2("");

	/*
	 * use multi read even if some registers are not useful, result will
	 * be more efficient
	 * start reading at 0x14 dec20
	 * end reading at 0x21 dec33 total 14 bytes to read
	 */
	Status = VL53L0X_ReadMulti_2(Dev, 0x14, localBuffer, 12);

	if (Status == VL53L0X_ERROR_NONE_2) {

		pRangingMeasurementData->ZoneId_2 = 0; /* Only one zone */
		pRangingMeasurementData->TimeStamp_2 = 0; /* Not Implemented */

		tmpuint16 = VL53L0X_MAKEUINT16_2(localBuffer[11], localBuffer[10]);
		/* cut1.1 if SYSTEM__RANGE_CONFIG if 1 range is 2bits fractional
		 *(format 11.2) else no fractional
		 */

		pRangingMeasurementData->MeasurementTimeUsec_2 = 0;

		SignalRate = VL53L0X_FIXPOINT97TOFIXPOINT1616_2(
			VL53L0X_MAKEUINT16_2(localBuffer[7], localBuffer[6]));
		/* peak_signal_count_rate_rtn_mcps */
		pRangingMeasurementData->SignalRateRtnMegaCps_2 = SignalRate;

		AmbientRate = VL53L0X_MAKEUINT16_2(localBuffer[9], localBuffer[8]);
		pRangingMeasurementData->AmbientRateRtnMegaCps_2 =
			VL53L0X_FIXPOINT97TOFIXPOINT1616_2(AmbientRate);

		EffectiveSpadRtnCount_2 = VL53L0X_MAKEUINT16_2(localBuffer[3],
			localBuffer[2]);
		/* EffectiveSpadRtnCount_2 is 8.8 format */
		pRangingMeasurementData->EffectiveSpadRtnCount_2 =
			EffectiveSpadRtnCount_2;

		DeviceRangeStatus_2 = localBuffer[0];

		/* Get Linearity Corrective Gain */
		LinearityCorrectiveGain = PALDevDataGet_2(Dev,
			LinearityCorrectiveGain);

		/* Get ranging configuration */
		RangeFractionalEnable = PALDevDataGet_2(Dev,
			RangeFractionalEnable);

		if (LinearityCorrectiveGain != 1000) {

			tmpuint16 = (uint16_t)((LinearityCorrectiveGain
				* tmpuint16 + 500) / 1000);

			/* Implement Xtalk */
			VL53L0X_GETPARAMETERFIELD_2(Dev,
				XTalkCompensationRateMegaCps_2,
				XTalkCompensationRateMegaCps_2);
			VL53L0X_GETPARAMETERFIELD_2(Dev, XTalkCompensationEnable_2,
				XTalkCompensationEnable_2);

			if (XTalkCompensationEnable_2) {

				if ((SignalRate
					- ((XTalkCompensationRateMegaCps_2
					* EffectiveSpadRtnCount_2) >> 8))
					<= 0) {
					if (RangeFractionalEnable)
						XtalkRangeMilliMeter_2 = 8888;
					else
						XtalkRangeMilliMeter_2 = 8888
							<< 2;
				} else {
					XtalkRangeMilliMeter_2 =
					(tmpuint16 * SignalRate)
						/ (SignalRate
						- ((XTalkCompensationRateMegaCps_2
						* EffectiveSpadRtnCount_2)
						>> 8));
				}

				tmpuint16 = XtalkRangeMilliMeter_2;
			}

		}

		if (RangeFractionalEnable) {
			pRangingMeasurementData->RangeMilliMeter_2 =
				(uint16_t)((tmpuint16) >> 2);
			pRangingMeasurementData->RangeFractionalPart_2 =
				(uint8_t)((tmpuint16 & 0x03) << 6);
		} else {
			pRangingMeasurementData->RangeMilliMeter_2 = tmpuint16;
			pRangingMeasurementData->RangeFractionalPart_2 = 0;
		}

		/*
		 * For a standard definition of RangeStatus_2, this should
		 * return 0 in case of good result after a ranging
		 * The range status depends on the device so call a device
		 * specific function to obtain the right Status.
		 */
		Status |= VL53L0X_get_pal_range_status_2(Dev, DeviceRangeStatus_2,
			SignalRate, EffectiveSpadRtnCount_2,
			pRangingMeasurementData, &PalRangeStatus_2);

		if (Status == VL53L0X_ERROR_NONE_2)
			pRangingMeasurementData->RangeStatus_2 = PalRangeStatus_2;

	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		/* Copy last read data into Dev buffer */
		LastRangeDataBuffer = PALDevDataGet_2(Dev, LastRangeMeasure);

		LastRangeDataBuffer.RangeMilliMeter_2 =
			pRangingMeasurementData->RangeMilliMeter_2;
		LastRangeDataBuffer.RangeFractionalPart_2 =
			pRangingMeasurementData->RangeFractionalPart_2;
		LastRangeDataBuffer.RangeDMaxMilliMeter_2 =
			pRangingMeasurementData->RangeDMaxMilliMeter_2;
		LastRangeDataBuffer.MeasurementTimeUsec_2 =
			pRangingMeasurementData->MeasurementTimeUsec_2;
		LastRangeDataBuffer.SignalRateRtnMegaCps_2 =
			pRangingMeasurementData->SignalRateRtnMegaCps_2;
		LastRangeDataBuffer.AmbientRateRtnMegaCps_2 =
			pRangingMeasurementData->AmbientRateRtnMegaCps_2;
		LastRangeDataBuffer.EffectiveSpadRtnCount_2 =
			pRangingMeasurementData->EffectiveSpadRtnCount_2;
		LastRangeDataBuffer.RangeStatus_2 =
			pRangingMeasurementData->RangeStatus_2;

		PALDevDataSet_2(Dev, LastRangeMeasure, LastRangeDataBuffer);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetMeasurementRefSignal_2(VL53L0X_DEV_2 Dev,
	FixPoint1616_t_2 *pMeasurementRefSignal)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t SignalRefClipLimitCheckEnable = 0;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_GetLimitCheckEnable_2(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP_2,
			&SignalRefClipLimitCheckEnable);
	if (SignalRefClipLimitCheckEnable != 0) {
		*pMeasurementRefSignal = PALDevDataGet_2(Dev, LastSignalRefMcps);
	} else {
		Status = VL53L0X_ERROR_INVALID_COMMAND_2;
	}
	LOG_FUNCTION_END_2(Status);

	return Status;
}

VL53L0X_Error_2 VL53L0X_GetHistogramMeasurementData_2(VL53L0X_DEV_2 Dev,
	VL53L0X_HistogramMeasurementData_t_2 *pHistogramMeasurementData)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NOT_IMPLEMENTED_2;
	LOG_FUNCTION_START_2("");

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_PerformSingleRangingMeasurement_2(VL53L0X_DEV_2 Dev,
	VL53L0X_RangingMeasurementData_t_2 *pRangingMeasurementData)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("");

	/* This function will do a complete single ranging
	 * Here we fix the mode! */
	Status = VL53L0X_SetDeviceMode_2(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING_2);

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_PerformSingleMeasurement_2(Dev);


	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_GetRangingMeasurementData_2(Dev,
			pRangingMeasurementData);


	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_ClearInterruptMask_2(Dev, 0);


	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetNumberOfROIZones_2(VL53L0X_DEV_2 Dev,
	uint8_t NumberOfROIZones)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("");

	if (NumberOfROIZones != 1)
		Status = VL53L0X_ERROR_INVALID_PARAMS_2;


	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetNumberOfROIZones_2(VL53L0X_DEV_2 Dev,
	uint8_t *pNumberOfROIZones)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("");

	*pNumberOfROIZones = 1;

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetMaxNumberOfROIZones_2(VL53L0X_DEV_2 Dev,
	uint8_t *pMaxNumberOfROIZones)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;

	LOG_FUNCTION_START_2("");

	*pMaxNumberOfROIZones = 1;

	LOG_FUNCTION_END_2(Status);
	return Status;
}

/* End Group PAL Measurement Functions */

VL53L0X_Error_2 VL53L0X_SetGpioConfig_2(VL53L0X_DEV_2 Dev, uint8_t Pin,
	VL53L0X_DeviceModes_2 DeviceMode_2, VL53L0X_GpioFunctionality_2 Functionality,
	VL53L0X_InterruptPolarity_2 Polarity)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t data;

	LOG_FUNCTION_START_2("");

	if (Pin != 0) {
		Status = VL53L0X_ERROR_GPIO_NOT_EXISTING_2;
	} else if (DeviceMode_2 == VL53L0X_DEVICEMODE_GPIO_DRIVE_2) {
		if (Polarity == VL53L0X_INTERRUPTPOLARITY_LOW_2)
			data = 0x10;
		else
			data = 1;

		Status = VL53L0X_WrByte_2(Dev,
		VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH_2, data);

	} else if (DeviceMode_2 == VL53L0X_DEVICEMODE_GPIO_OSC_2) {

		Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x01);
		Status |= VL53L0X_WrByte_2(Dev, 0x00, 0x00);

		Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x00);
		Status |= VL53L0X_WrByte_2(Dev, 0x80, 0x01);
		Status |= VL53L0X_WrByte_2(Dev, 0x85, 0x02);

		Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x04);
		Status |= VL53L0X_WrByte_2(Dev, 0xcd, 0x00);
		Status |= VL53L0X_WrByte_2(Dev, 0xcc, 0x11);

		Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x07);
		Status |= VL53L0X_WrByte_2(Dev, 0xbe, 0x00);

		Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x06);
		Status |= VL53L0X_WrByte_2(Dev, 0xcc, 0x09);

		Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x00);
		Status |= VL53L0X_WrByte_2(Dev, 0xff, 0x01);
		Status |= VL53L0X_WrByte_2(Dev, 0x00, 0x00);

	} else {

		if (Status == VL53L0X_ERROR_NONE_2) {
			switch (Functionality) {
			case VL53L0X_GPIOFUNCTIONALITY_OFF_2:
				data = 0x00;
				break;
			case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW_2:
				data = 0x01;
				break;
			case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH_2:
				data = 0x02;
				break;
			case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT_2:
				data = 0x03;
				break;
			case VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY_2:
				data = 0x04;
				break;
			default:
				Status =
				VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED_2;
			}
		}

		if (Status == VL53L0X_ERROR_NONE_2)
			Status = VL53L0X_WrByte_2(Dev,
			VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO_2, data);

		if (Status == VL53L0X_ERROR_NONE_2) {
			if (Polarity == VL53L0X_INTERRUPTPOLARITY_LOW_2)
				data = 0;
			else
				data = (uint8_t)(1 << 4);

			Status = VL53L0X_UpdateByte_2(Dev,
			VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH_2, 0xEF, data);
		}

		if (Status == VL53L0X_ERROR_NONE_2)
			VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev,
				Pin0GpioFunctionality, Functionality);

		if (Status == VL53L0X_ERROR_NONE_2)
			Status = VL53L0X_ClearInterruptMask_2(Dev, 0);

	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetGpioConfig_2(VL53L0X_DEV_2 Dev, uint8_t Pin,
	VL53L0X_DeviceModes_2 *pDeviceMode_2,
	VL53L0X_GpioFunctionality_2 *pFunctionality,
	VL53L0X_InterruptPolarity_2 *pPolarity)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	VL53L0X_GpioFunctionality_2 GpioFunctionality;
	uint8_t data;

	LOG_FUNCTION_START_2("");

	/* pDeviceMode_2 not managed by Ewok it return the current mode */

	Status = VL53L0X_GetDeviceMode_2(Dev, pDeviceMode_2);

	if (Status == VL53L0X_ERROR_NONE_2) {
		if (Pin != 0) {
			Status = VL53L0X_ERROR_GPIO_NOT_EXISTING_2;
		} else {
			Status = VL53L0X_RdByte_2(Dev,
			VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO_2, &data);
		}
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		switch (data & 0x07) {
		case 0x00:
			GpioFunctionality = VL53L0X_GPIOFUNCTIONALITY_OFF_2;
			break;
		case 0x01:
			GpioFunctionality =
			VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW_2;
			break;
		case 0x02:
			GpioFunctionality =
			VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH_2;
			break;
		case 0x03:
			GpioFunctionality =
			VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT_2;
			break;
		case 0x04:
			GpioFunctionality =
			VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY_2;
			break;
		default:
			Status = VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED_2;
		}
	}

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_RdByte_2(Dev, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH_2,
			&data);

	if (Status == VL53L0X_ERROR_NONE_2) {
		if ((data & (uint8_t)(1 << 4)) == 0)
			*pPolarity = VL53L0X_INTERRUPTPOLARITY_LOW_2;
		else
			*pPolarity = VL53L0X_INTERRUPTPOLARITY_HIGH_2;
	}

	if (Status == VL53L0X_ERROR_NONE_2) {
		*pFunctionality = GpioFunctionality;
		VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev, Pin0GpioFunctionality,
			GpioFunctionality);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetInterruptThresholds_2(VL53L0X_DEV_2 Dev,
	VL53L0X_DeviceModes_2 DeviceMode_2, FixPoint1616_t_2 ThresholdLow,
	FixPoint1616_t_2 ThresholdHigh)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint16_t Threshold16;
	LOG_FUNCTION_START_2("");

	/* no dependency on DeviceMode_2 for Ewok */
	/* Need to divide by 2 because the FW will apply a x2 */
	Threshold16 = (uint16_t)((ThresholdLow >> 17) & 0x00fff);
	Status = VL53L0X_WrWord_2(Dev, VL53L0X_REG_SYSTEM_THRESH_LOW_2, Threshold16);

	if (Status == VL53L0X_ERROR_NONE_2) {
		/* Need to divide by 2 because the FW will apply a x2 */
		Threshold16 = (uint16_t)((ThresholdHigh >> 17) & 0x00fff);
		Status = VL53L0X_WrWord_2(Dev, VL53L0X_REG_SYSTEM_THRESH_HIGH_2,
			Threshold16);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetInterruptThresholds_2(VL53L0X_DEV_2 Dev,
	VL53L0X_DeviceModes_2 DeviceMode_2, FixPoint1616_t_2 *pThresholdLow,
	FixPoint1616_t_2 *pThresholdHigh)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint16_t Threshold16;
	LOG_FUNCTION_START_2("");

	/* no dependency on DeviceMode_2 for Ewok */

	Status = VL53L0X_RdWord_2(Dev, VL53L0X_REG_SYSTEM_THRESH_LOW_2, &Threshold16);
	/* Need to multiply by 2 because the FW will apply a x2 */
	*pThresholdLow = (FixPoint1616_t_2)((0x00fff & Threshold16) << 17);

	if (Status == VL53L0X_ERROR_NONE_2) {
		Status = VL53L0X_RdWord_2(Dev, VL53L0X_REG_SYSTEM_THRESH_HIGH_2,
			&Threshold16);
		/* Need to multiply by 2 because the FW will apply a x2 */
		*pThresholdHigh =
			(FixPoint1616_t_2)((0x00fff & Threshold16) << 17);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetStopCompletedStatus_2(VL53L0X_DEV_2 Dev,
	uint32_t *pStopStatus)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t Byte = 0;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x01);

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_RdByte_2(Dev, 0x04, &Byte);

	if (Status == VL53L0X_ERROR_NONE_2)
		Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x0);

	*pStopStatus = Byte;

	if (Byte == 0) {
		Status = VL53L0X_WrByte_2(Dev, 0x80, 0x01);
		Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x01);
		Status = VL53L0X_WrByte_2(Dev, 0x00, 0x00);
		Status = VL53L0X_WrByte_2(Dev, 0x91,
			PALDevDataGet_2(Dev, StopVariable));
		Status = VL53L0X_WrByte_2(Dev, 0x00, 0x01);
		Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x00);
		Status = VL53L0X_WrByte_2(Dev, 0x80, 0x00);
	}

	LOG_FUNCTION_END_2(Status);
	return Status;
}

/* Group PAL Interrupt Functions */
VL53L0X_Error_2 VL53L0X_ClearInterruptMask_2(VL53L0X_DEV_2 Dev, uint32_t InterruptMask)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t LoopCount;
	uint8_t Byte;
	LOG_FUNCTION_START_2("");

	/* clear bit 0 range interrupt, bit 1 error interrupt */
	LoopCount = 0;
	do {
		Status = VL53L0X_WrByte_2(Dev,
			VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR_2, 0x01);
		Status |= VL53L0X_WrByte_2(Dev,
			VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR_2, 0x00);
		Status |= VL53L0X_RdByte_2(Dev,
			VL53L0X_REG_RESULT_INTERRUPT_STATUS_2, &Byte);
		LoopCount++;
	} while (((Byte & 0x07) != 0x00)
			&& (LoopCount < 3)
			&& (Status == VL53L0X_ERROR_NONE_2));


	if (LoopCount >= 3)
		Status = VL53L0X_ERROR_INTERRUPT_NOT_CLEARED_2;

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetInterruptMaskStatus_2(VL53L0X_DEV_2 Dev,
	uint32_t *pInterruptMaskStatus)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t Byte;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_RdByte_2(Dev, VL53L0X_REG_RESULT_INTERRUPT_STATUS_2, &Byte);
	*pInterruptMaskStatus = Byte & 0x07;

	if (Byte & 0x18)
		Status = VL53L0X_ERROR_RANGE_ERROR_2;

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_EnableInterruptMask_2(VL53L0X_DEV_2 Dev, uint32_t InterruptMask)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NOT_IMPLEMENTED_2;
	LOG_FUNCTION_START_2("");

	/* not implemented for VL53L0X */

	LOG_FUNCTION_END_2(Status);
	return Status;
}

/* End Group PAL Interrupt Functions */

/* Group SPAD functions */

VL53L0X_Error_2 VL53L0X_SetSpadAmbientDamperThreshold_2(VL53L0X_DEV_2 Dev,
	uint16_t SpadAmbientDamperThreshold)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x01);
	Status |= VL53L0X_WrWord_2(Dev, 0x40, SpadAmbientDamperThreshold);
	Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x00);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetSpadAmbientDamperThreshold_2(VL53L0X_DEV_2 Dev,
	uint16_t *pSpadAmbientDamperThreshold)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x01);
	Status |= VL53L0X_RdWord_2(Dev, 0x40, pSpadAmbientDamperThreshold);
	Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x00);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_SetSpadAmbientDamperFactor_2(VL53L0X_DEV_2 Dev,
	uint16_t SpadAmbientDamperFactor)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t Byte;
	LOG_FUNCTION_START_2("");

	Byte = (uint8_t)(SpadAmbientDamperFactor & 0x00FF);

	Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x01);
	Status |= VL53L0X_WrByte_2(Dev, 0x42, Byte);
	Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x00);

	LOG_FUNCTION_END_2(Status);
	return Status;
}

VL53L0X_Error_2 VL53L0X_GetSpadAmbientDamperFactor_2(VL53L0X_DEV_2 Dev,
	uint16_t *pSpadAmbientDamperFactor)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	uint8_t Byte;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_WrByte_2(Dev, 0xFF, 0x01);
	Status |= VL53L0X_RdByte_2(Dev, 0x42, &Byte);
	Status |= VL53L0X_WrByte_2(Dev, 0xFF, 0x00);
	*pSpadAmbientDamperFactor = (uint16_t)Byte;

	LOG_FUNCTION_END_2(Status);
	return Status;
}

/* END Group SPAD functions */

/*****************************************************************************
 * Internal functions
 *****************************************************************************/

VL53L0X_Error_2 VL53L0X_SetReferenceSpads_2(VL53L0X_DEV_2 Dev, uint32_t count,
	uint8_t isApertureSpads)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_set_reference_spads_2(Dev, count, isApertureSpads);

	LOG_FUNCTION_END_2(Status);

	return Status;
}

VL53L0X_Error_2 VL53L0X_GetReferenceSpads_2(VL53L0X_DEV_2 Dev, uint32_t *pSpadCount,
	uint8_t *pIsApertureSpads)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_get_reference_spads_2(Dev, pSpadCount, pIsApertureSpads);

	LOG_FUNCTION_END_2(Status);

	return Status;
}

VL53L0X_Error_2 VL53L0X_PerformRefSpadManagement_2(VL53L0X_DEV_2 Dev,
	uint32_t *refSpadCount, uint8_t *isApertureSpads)
{
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	LOG_FUNCTION_START_2("");

	Status = VL53L0X_perform_ref_spad_management_2(Dev, refSpadCount,
		isApertureSpads);

	LOG_FUNCTION_END_2(Status);

	return Status;
}
