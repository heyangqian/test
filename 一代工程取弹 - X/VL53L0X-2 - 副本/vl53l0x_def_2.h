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
*******************************************************************************/

/**
 * @file VL53L0X_def.h
 *
 * @brief Type definitions for VL53L0X API.
 *
 */


#ifndef _VL53L0X_DEF_2_H_
#define _VL53L0X_DEF_2_H_


#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup VL53L0X_globaldefine_group VL53L0X Defines
 *	@brief	  VL53L0X Defines
 *	@{
 */


/** PAL SPECIFICATION major version */
#define VL53L0X10_SPECIFICATION_VER_MAJOR_2   1
/** PAL SPECIFICATION minor version */
#define VL53L0X10_SPECIFICATION_VER_MINOR_2   2
/** PAL SPECIFICATION sub version */
#define VL53L0X10_SPECIFICATION_VER_SUB_2	   7
/** PAL SPECIFICATION sub version */
#define VL53L0X10_SPECIFICATION_VER_REVISION_2 1440

/** VL53L0X PAL IMPLEMENTATION major version */
#define VL53L0X10_IMPLEMENTATION_VER_MAJOR_2	1
/** VL53L0X PAL IMPLEMENTATION minor version */
#define VL53L0X10_IMPLEMENTATION_VER_MINOR_2	0
/** VL53L0X PAL IMPLEMENTATION sub version */
#define VL53L0X10_IMPLEMENTATION_VER_SUB_2		9
/** VL53L0X PAL IMPLEMENTATION sub version */
#define VL53L0X10_IMPLEMENTATION_VER_REVISION_2	3673

/** PAL SPECIFICATION major version */
#define VL53L0X_SPECIFICATION_VER_MAJOR_2	 1
/** PAL SPECIFICATION minor version */
#define VL53L0X_SPECIFICATION_VER_MINOR_2	 2
/** PAL SPECIFICATION sub version */
#define VL53L0X_SPECIFICATION_VER_SUB_2	 7
/** PAL SPECIFICATION sub version */
#define VL53L0X_SPECIFICATION_VER_REVISION_2 1440

/** VL53L0X PAL IMPLEMENTATION major version */
#define VL53L0X_IMPLEMENTATION_VER_MAJOR_2	  1
/** VL53L0X PAL IMPLEMENTATION minor version */
#define VL53L0X_IMPLEMENTATION_VER_MINOR_2	  0
/** VL53L0X PAL IMPLEMENTATION sub version */
#define VL53L0X_IMPLEMENTATION_VER_SUB_2	  2
/** VL53L0X PAL IMPLEMENTATION sub version */
#define VL53L0X_IMPLEMENTATION_VER_REVISION_2	  4823
#define VL53L0X_DEFAULT_MAX_LOOP_2 2000
#define VL53L0X_MAX_STRING_LENGTH_2 32


#include "vl53l0x_device_2.h"
#include "vl53l0x_types_2.h"


/****************************************
 * PRIVATE define do not edit
 ****************************************/

/** @brief Defines the parameters of the Get Version Functions
 */
typedef struct {
	uint32_t	 revision; /*!< revision number */
	uint8_t		 major;	   /*!< major number */
	uint8_t		 minor;	   /*!< minor number */
	uint8_t		 build;	   /*!< build number */
} VL53L0X_Version_t_2;


/** @brief Defines the parameters of the Get Device Info Functions		¶¨ÒåGetÉè±¸ÐÅÏ¢º¯ÊýµÄ²ÎÊý
 */
typedef struct {
	char Name[VL53L0X_MAX_STRING_LENGTH_2];
		/*!< Name of the Device e.g. Left_Distance 		Éè±¸Ãû³Æ£¬ÀýÈçleft t_distance*/
	char Type[VL53L0X_MAX_STRING_LENGTH_2];
		/*!< Type of the Device e.g VL53L0X 	Éè±¸ÀàÐÍe¡£g VL53L0X */
	char ProductId[VL53L0X_MAX_STRING_LENGTH_2];
		/*!< Product Identifier String		²úÆ·±êÊ¶·û×Ö·û´®*/
	uint8_t ProductType_2;
		/*!< Product Type, VL53L0X = 1, VL53L1 = 2 		²úÆ·ÀàÐÍ£¬VL53L0X = 1, VL53L1 = 2 */
	uint8_t ProductRevisionMajor_2;
		/*!< Product revision major		²úÆ·ÐÞ¶©×¨Òµ*/
	uint8_t ProductRevisionMinor_2;
		/*!< Product revision minor		 ²úÆ·ÐÞ¶©Ð¡ÐÞ*/
} VL53L0X_DeviceInfo_t_2;


/** @defgroup VL53L0X_define_Error_group Error and Warning code returned by API
 *	The following DEFINE are used to identify the PAL ERROR
 *	@{
 */

typedef int8_t VL53L0X_Error_2;

#define VL53L0X_ERROR_NONE_2		((VL53L0X_Error_2)	0)
#define VL53L0X_ERROR_CALIBRATION_WARNING_2	((VL53L0X_Error_2) -1)
	/*!< Warning invalid calibration data may be in used
		\a	VL53L0X_InitData()
		\a VL53L0X_GetOffsetCalibrationData
		\a VL53L0X_SetOffsetCalibrationData */
#define VL53L0X_ERROR_MIN_CLIPPED_2			((VL53L0X_Error_2) -2)
	/*!< Warning parameter passed was clipped to min before to be applied */

#define VL53L0X_ERROR_UNDEFINED_2				((VL53L0X_Error_2) -3)
	/*!< Unqualified error */
#define VL53L0X_ERROR_INVALID_PARAMS_2			((VL53L0X_Error_2) -4)
	/*!< Parameter passed is invalid or out of range */
#define VL53L0X_ERROR_NOT_SUPPORTED_2			((VL53L0X_Error_2) -5)
	/*!< Function is not supported in current mode or configuration */
#define VL53L0X_ERROR_RANGE_ERROR_2			((VL53L0X_Error_2) -6)
	/*!< Device report a ranging error interrupt status */
#define VL53L0X_ERROR_TIME_OUT_2				((VL53L0X_Error_2) -7)
	/*!< Aborted due to time out */
#define VL53L0X_ERROR_MODE_NOT_SUPPORTED_2			((VL53L0X_Error_2) -8)
	/*!< Asked mode is not supported by the device */
#define VL53L0X_ERROR_BUFFER_TOO_SMALL_2			((VL53L0X_Error_2) -9)
	/*!< ... */
#define VL53L0X_ERROR_GPIO_NOT_EXISTING_2			((VL53L0X_Error_2) -10)
	/*!< User tried to setup a non-existing GPIO pin */
#define VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED_2  ((VL53L0X_Error_2) -11)
	/*!< unsupported GPIO functionality */
#define VL53L0X_ERROR_INTERRUPT_NOT_CLEARED_2		((VL53L0X_Error_2) -12)
	/*!< Error during interrupt clear */
#define VL53L0X_ERROR_CONTROL_INTERFACE_2			((VL53L0X_Error_2) -20)
	/*!< error reported from IO functions */
#define VL53L0X_ERROR_INVALID_COMMAND_2			((VL53L0X_Error_2) -30)
	/*!< The command is not allowed in the current device state
	 *	(power down) */
#define VL53L0X_ERROR_DIVISION_BY_ZERO_2			((VL53L0X_Error_2) -40)
	/*!< In the function a division by zero occurs */
#define VL53L0X_ERROR_REF_SPAD_INIT_2			((VL53L0X_Error_2) -50)
	/*!< Error during reference SPAD initialization */
#define VL53L0X_ERROR_NOT_IMPLEMENTED_2			((VL53L0X_Error_2) -99)
	/*!< Tells requested functionality has not been implemented yet or
	 * not compatible with the device */
/** @} VL53L0X_define_Error_group */


/** @defgroup VL53L0X_define_DeviceModes_group Defines Device modes
 *	Defines all possible modes for the device
 *	@{
 */
typedef uint8_t VL53L0X_DeviceModes_2;

#define VL53L0X_DEVICEMODE_SINGLE_RANGING_2	((VL53L0X_DeviceModes_2)  0)
#define VL53L0X_DEVICEMODE_CONTINUOUS_RANGING_2	((VL53L0X_DeviceModes_2)  1)
#define VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM_2	((VL53L0X_DeviceModes_2)  2)
#define VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING_2 ((VL53L0X_DeviceModes_2) 3)
#define VL53L0X_DEVICEMODE_SINGLE_ALS_2		((VL53L0X_DeviceModes_2) 10)
#define VL53L0X_DEVICEMODE_GPIO_DRIVE_2		((VL53L0X_DeviceModes_2) 20)
#define VL53L0X_DEVICEMODE_GPIO_OSC_2		((VL53L0X_DeviceModes_2) 21)
	/* ... Modes to be added depending on device */
/** @} VL53L0X_define_DeviceModes_group */



/** @defgroup VL53L0X_define_HistogramModes_group Defines Histogram modes
 *	Defines all possible Histogram modes for the device
 *	@{
 */
typedef uint8_t VL53L0X_HistogramModes_2;

#define VL53L0X_HISTOGRAMMODE_DISABLED_2		((VL53L0X_HistogramModes_2) 0)
	/*!< Histogram Disabled */
#define VL53L0X_HISTOGRAMMODE_REFERENCE_ONLY_2	((VL53L0X_HistogramModes_2) 1)
	/*!< Histogram Reference array only */
#define VL53L0X_HISTOGRAMMODE_RETURN_ONLY_2	((VL53L0X_HistogramModes_2) 2)
	/*!< Histogram Return array only */
#define VL53L0X_HISTOGRAMMODE_BOTH_2		((VL53L0X_HistogramModes_2) 3)
	/*!< Histogram both Reference and Return Arrays */
	/* ... Modes to be added depending on device */
/** @} VL53L0X_define_HistogramModes_group */


/** @defgroup VL53L0X_define_PowerModes_group List of available Power Modes
 *	List of available Power Modes
 *	@{
 */

typedef uint8_t 	VL53L0X_PowerModes_2;

#define VL53L0X_POWERMODE_STANDBY_LEVEL1_2 ((VL53L0X_PowerModes_2) 0)
	/*!< Standby level 1 */
#define VL53L0X_POWERMODE_STANDBY_LEVEL2_2 ((VL53L0X_PowerModes_2) 1)
	/*!< Standby level 2 */
#define VL53L0X_POWERMODE_IDLE_LEVEL1_2	((VL53L0X_PowerModes_2) 2)
	/*!< Idle level 1 */
#define VL53L0X_POWERMODE_IDLE_LEVEL2_2	((VL53L0X_PowerModes_2) 3)
	/*!< Idle level 2 */

/** @} VL53L0X_define_PowerModes_group */


/** @brief Defines all parameters for the device
 */
typedef struct {
	VL53L0X_DeviceModes_2 DeviceMode;
	/*!< Defines type of measurement to be done for the next measure */
	VL53L0X_HistogramModes_2 HistogramMode;
	/*!< Defines type of histogram measurement to be done for the next
	 *	measure */
	uint32_t MeasurementTimingBudgetMicroSeconds_2;
	/*!< Defines the allowed total time for a single measurement */
	uint32_t InterMeasurementPeriodMilliSeconds_2;
	/*!< Defines time between two consecutive measurements (between two
	 *	measurement starts). If set to 0 means back-to-back mode */
	uint8_t XTalkCompensationEnable_2;
	/*!< Tells if Crosstalk compensation shall be enable or not	 */
	uint16_t XTalkCompensationRangeMilliMeter_2;
	/*!< CrossTalk compensation range in millimeter	 */
	FixPoint1616_t_2 XTalkCompensationRateMegaCps_2;
	/*!< CrossTalk compensation rate in Mega counts per seconds.
	 *	Expressed in 16.16 fixed point format.	*/
	int32_t RangeOffsetMicroMeters_2;
	/*!< Range offset adjustment (mm).	*/

	uint8_t LimitChecksEnable_2[VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS_2];
	/*!< This Array store all the Limit Check enable for this device. */
	uint8_t LimitChecksStatus_2[VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS_2];
	/*!< This Array store all the Status of the check linked to last
	* measurement. */
	FixPoint1616_t_2 	LimitChecksValue_2[VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS_2];
	/*!< This Array store all the Limit Check value for this device */

	uint8_t WrapAroundCheckEnable_2;
	/*!< Tells if Wrap Around Check shall be enable or not */
} VL53L0X_DeviceParameters_t_2;


/** @defgroup VL53L0X_define_State_group Defines the current status of the device
 *	Defines the current status of the device
 *	@{
 */

typedef uint8_t VL53L0X_State_2;

#define VL53L0X_STATE_POWERDOWN_2		 ((VL53L0X_State_2)  0)
	/*!< Device is in HW reset	*/
#define VL53L0X_STATE_WAIT_STATICINIT_2 ((VL53L0X_State_2)  1)
	/*!< Device is initialized and wait for static initialization  */
#define VL53L0X_STATE_STANDBY_2		 ((VL53L0X_State_2)  2)
	/*!< Device is in Low power Standby mode   */
#define VL53L0X_STATE_IDLE_2			 ((VL53L0X_State_2)  3)
	/*!< Device has been initialized and ready to do measurements  */
#define VL53L0X_STATE_RUNNING_2		 ((VL53L0X_State_2)  4)
	/*!< Device is performing measurement */
#define VL53L0X_STATE_UNKNOWN_2		 ((VL53L0X_State_2)  98)
	/*!< Device is in unknown state and need to be rebooted	 */
#define VL53L0X_STATE_ERROR_2		 ((VL53L0X_State_2)  99)
	/*!< Device is in error state and need to be rebooted  */

/** @} VL53L0X_define_State_group */


/** @brief Structure containing the Dmax computation parameters and data
 */
typedef struct {
	int32_t AmbTuningWindowFactor_K_2;
		/*!<  internal algo tuning (*1000) */
	int32_t RetSignalAt0mm_2;
		/*!< intermediate dmax computation value caching */
} VL53L0X_DMaxData_t_2;

/**
 * @struct VL53L0X_RangeData_t
 * @brief Range measurement data.
 */
typedef struct {
	uint32_t TimeStamp_2;		/*!< 32-bit time stamp. */
	uint32_t MeasurementTimeUsec_2;
		/*!< Give the Measurement time needed by the device to do the
		 * measurement.*/


	uint16_t RangeMilliMeter_2;	/*!< range distance in millimeter. */

	uint16_t RangeDMaxMilliMeter_2;
		/*!< Tells what is the maximum detection distance of the device
		 * in current setup and environment conditions (Filled when
		 *	applicable) */

	FixPoint1616_t_2 SignalRateRtnMegaCps_2;
		/*!< Return signal rate (MCPS)\n these is a 16.16 fix point
		 *	value, which is effectively a measure of target
		 *	 reflectance.*/
	FixPoint1616_t_2 AmbientRateRtnMegaCps_2;
		/*!< Return ambient rate (MCPS)\n these is a 16.16 fix point
		 *	value, which is effectively a measure of the ambien
		 *	t light.*/

	uint16_t EffectiveSpadRtnCount_2;
		/*!< Return the effective SPAD count for the return signal.
		 *	To obtain Real value it should be divided by 256 */

	uint8_t ZoneId_2;
		/*!< Denotes which zone and range scheduler stage the range
		 *	data relates to. */
	uint8_t RangeFractionalPart_2;
		/*!< Fractional part of range distance. Final value is a
		 *	FixPoint168 value. */
	uint8_t RangeStatus_2;
		/*!< Range Status for the current measurement. This is device
		 *	dependent. Value = 0 means value is valid.
		 *	See \ref RangeStatusPage */
} VL53L0X_RangingMeasurementData_t_2;


#define VL53L0X_HISTOGRAM_BUFFER_SIZE_2 24

/**
 * @struct VL53L0X_HistogramData_t
 * @brief Histogram measurement data.
 */
typedef struct {
	/* Histogram Measurement data */
	uint32_t HistogramData_2[VL53L0X_HISTOGRAM_BUFFER_SIZE_2];
	/*!< Histogram data */
	uint8_t HistogramType_2; /*!< Indicate the types of histogram data :
	Return only, Reference only, both Return and Reference */
	uint8_t FirstBin_2; /*!< First Bin value */
	uint8_t BufferSize_2; /*!< Buffer Size - Set by the user.*/
	uint8_t NumberOfBins_2;
	/*!< Number of bins filled by the histogram measurement */

	VL53L0X_DeviceError_2 ErrorStatus_2;
	/*!< Error status of the current measurement. \n
	see @a ::VL53L0X_DeviceError_2 @a VL53L0X_GetStatusErrorString() */
} VL53L0X_HistogramMeasurementData_t_2;

#define VL53L0X_REF_SPAD_BUFFER_SIZE_2 6

/**
 * @struct VL53L0X_SpadData_t
 * @brief Spad Configuration Data.
 */
typedef struct {
	uint8_t RefSpadEnables[VL53L0X_REF_SPAD_BUFFER_SIZE_2];
	/*!< Reference Spad Enables */
	uint8_t RefGoodSpadMap[VL53L0X_REF_SPAD_BUFFER_SIZE_2];
	/*!< Reference Spad Good Spad Map */
} VL53L0X_SpadData_t_2;

typedef struct {
	FixPoint1616_t_2 OscFrequencyMHz; /* Frequency used */

	uint16_t LastEncodedTimeout;
	/* last encoded Time out used for timing budget*/

	VL53L0X_GpioFunctionality_2 Pin0GpioFunctionality;
	/* store the functionality of the GPIO: pin0 */

	uint32_t FinalRangeTimeoutMicroSecs;
	 /*!< Execution time of the final range*/
	uint8_t FinalRangeVcselPulsePeriod;
	 /*!< Vcsel pulse period (pll clocks) for the final range measurement*/
	uint32_t PreRangeTimeoutMicroSecs;
	 /*!< Execution time of the final range*/
	uint8_t PreRangeVcselPulsePeriod;
	 /*!< Vcsel pulse period (pll clocks) for the pre-range measurement*/

	uint16_t SigmaEstRefArray;
	 /*!< Reference array sigma value in 1/100th of [mm] e.g. 100 = 1mm */
	uint16_t SigmaEstEffPulseWidth;
	 /*!< Effective Pulse width for sigma estimate in 1/100th
	  * of ns e.g. 900 = 9.0ns */
	uint16_t SigmaEstEffAmbWidth;
	 /*!< Effective Ambient width for sigma estimate in 1/100th of ns
	  * e.g. 500 = 5.0ns */


	uint8_t ReadDataFromDeviceDone; /* Indicate if read from device has
	been done (==1) or not (==0) */
	uint8_t ModuleId; /* Module ID */
	uint8_t Revision; /* test Revision */
	char ProductId[VL53L0X_MAX_STRING_LENGTH_2];
		/* Product Identifier String  */
	uint8_t ReferenceSpadCount; /* used for ref spad management */
	uint8_t ReferenceSpadType;	/* used for ref spad management */
	uint8_t RefSpadsInitialised; /* reports if ref spads are initialised. */
	uint32_t PartUIDUpper; /*!< Unique Part ID Upper */
	uint32_t PartUIDLower; /*!< Unique Part ID Lower */
	FixPoint1616_t_2 SignalRateMeasFixed400mm; /*!< Peek Signal rate
	at 400 mm*/

} VL53L0X_DeviceSpecificParameters_t_2;

/**
 * @struct VL53L0X_DevData_t
 *
 * @brief VL53L0X PAL device ST private data structure \n
 * End user should never access any of these field directly
 *
 * These must never access directly but only via macro
 */
typedef struct {
	VL53L0X_DMaxData_t_2 DMaxData;
	/*!< Dmax Data */
	int32_t	 Part2PartOffsetNVMMicroMeter;
	/*!< backed up NVM value */
	int32_t	 Part2PartOffsetAdjustmentNVMMicroMeter;
	/*!< backed up NVM value representing additional offset adjustment */
	VL53L0X_DeviceParameters_t_2 CurrentParameters;
	/*!< Current Device Parameter */
	VL53L0X_RangingMeasurementData_t_2 LastRangeMeasure;
	/*!< Ranging Data */
	VL53L0X_HistogramMeasurementData_t_2 LastHistogramMeasure;
	/*!< Histogram Data */
	VL53L0X_DeviceSpecificParameters_t_2 DeviceSpecificParameters;
	/*!< Parameters specific to the device */
	VL53L0X_SpadData_t_2 SpadData;
	/*!< Spad Data */
	uint8_t SequenceConfig;
	/*!< Internal value for the sequence config */
	uint8_t RangeFractionalEnable;
	/*!< Enable/Disable fractional part of ranging data */
	VL53L0X_State_2 PalState;
	/*!< Current state of the PAL for this device */
	VL53L0X_PowerModes_2 PowerMode;
	/*!< Current Power Mode	 */
	uint16_t SigmaEstRefArray;
	/*!< Reference array sigma value in 1/100th of [mm] e.g. 100 = 1mm */
	uint16_t SigmaEstEffPulseWidth;
	/*!< Effective Pulse width for sigma estimate in 1/100th
	* of ns e.g. 900 = 9.0ns */
	uint16_t SigmaEstEffAmbWidth;
	/*!< Effective Ambient width for sigma estimate in 1/100th of ns
	* e.g. 500 = 5.0ns */
	uint8_t StopVariable;
	/*!< StopVariable used during the stop sequence */
	uint16_t targetRefRate;
	/*!< Target Ambient Rate for Ref spad management */
	FixPoint1616_t_2 SigmaEstimate;
	/*!< Sigma Estimate - based on ambient & VCSEL rates and
	* signal_total_events */
	FixPoint1616_t_2 SignalEstimate;
	/*!< Signal Estimate - based on ambient & VCSEL rates and cross talk */
	FixPoint1616_t_2 LastSignalRefMcps;
	/*!< Latest Signal ref in Mcps */
	uint8_t *pTuningSettingsPointer;
	/*!< Pointer for Tuning Settings table */
	uint8_t UseInternalTuningSettings;
	/*!< Indicate if we use	 Tuning Settings table */
	uint16_t LinearityCorrectiveGain;
	/*!< Linearity Corrective Gain value in x1000 */
	uint16_t DmaxCalRangeMilliMeter;
	/*!< Dmax Calibration Range millimeter */
	FixPoint1616_t_2 DmaxCalSignalRateRtnMegaCps;
	/*!< Dmax Calibration Signal Rate Return MegaCps */

} VL53L0X_DevData_t_2;


/** @defgroup VL53L0X_define_InterruptPolarity_group Defines the Polarity
 * of the Interrupt
 *	Defines the Polarity of the Interrupt
 *	@{
 */
typedef uint8_t VL53L0X_InterruptPolarity_2;

#define VL53L0X_INTERRUPTPOLARITY_LOW_2	   ((VL53L0X_InterruptPolarity_2)	0)
/*!< Set active low polarity best setup for falling edge. */
#define VL53L0X_INTERRUPTPOLARITY_HIGH_2	   ((VL53L0X_InterruptPolarity_2)	1)
/*!< Set active high polarity best setup for rising edge. */

/** @} VL53L0X_define_InterruptPolarity_group */


/** @defgroup VL53L0X_define_VcselPeriod_group Vcsel Period Defines
 *	Defines the range measurement for which to access the vcsel period.
 *	@{
 */
typedef uint8_t VL53L0X_VcselPeriod_2;

#define VL53L0X_VCSEL_PERIOD_PRE_RANGE_2	((VL53L0X_VcselPeriod_2) 0)
/*!<Identifies the pre-range vcsel period. */
#define VL53L0X_VCSEL_PERIOD_FINAL_RANGE_2 	((VL53L0X_VcselPeriod_2) 1)
/*!<Identifies the final range vcsel period. */

/** @} VL53L0X_define_VcselPeriod_group */

/** @defgroup VL53L0X_define_SchedulerSequence_group Defines the steps
 * carried out by the scheduler during a range measurement.
 *	@{
 *	Defines the states of all the steps in the scheduler
 *	i.e. enabled/disabled.
 */
typedef struct {
	uint8_t		 TccOn;	   /*!<Reports if Target Centre Check On  */
	uint8_t		 MsrcOn;	   /*!<Reports if MSRC On  */
	uint8_t		 DssOn;		   /*!<Reports if DSS On  */
	uint8_t		 PreRangeOn;   /*!<Reports if Pre-Range On	*/
	uint8_t		 FinalRangeOn; /*!<Reports if Final-Range On  */
} VL53L0X_SchedulerSequenceSteps_t_2;

/** @} VL53L0X_define_SchedulerSequence_group */

/** @defgroup VL53L0X_define_SequenceStepId_group Defines the Polarity
 *	of the Interrupt
 *	Defines the the sequence steps performed during ranging..
 *	@{
 */
typedef uint8_t VL53L0X_SequenceStepId_2;

#define	 VL53L0X_SEQUENCESTEP_TCC_2		 ((VL53L0X_VcselPeriod_2) 0)
/*!<Target CentreCheck identifier. */
#define	 VL53L0X_SEQUENCESTEP_DSS_2		 ((VL53L0X_VcselPeriod_2) 1)
/*!<Dynamic Spad Selection function Identifier. */
#define	 VL53L0X_SEQUENCESTEP_MSRC_2		 ((VL53L0X_VcselPeriod_2) 2)
/*!<Minimum Signal Rate Check function Identifier. */
#define	 VL53L0X_SEQUENCESTEP_PRE_RANGE_2	 ((VL53L0X_VcselPeriod_2) 3)
/*!<Pre-Range check Identifier. */
#define	 VL53L0X_SEQUENCESTEP_FINAL_RANGE_2  ((VL53L0X_VcselPeriod_2) 4)
/*!<Final Range Check Identifier. */

#define	 VL53L0X_SEQUENCESTEP_NUMBER_OF_CHECKS_2			 5
/*!<Number of Sequence Step Managed by the API. */

/** @} VL53L0X_define_SequenceStepId_group */


/* MACRO Definitions */
/** @defgroup VL53L0X_define_GeneralMacro_group General Macro Defines
 *	General Macro Defines
 *	@{
 */

/* Defines */
#define VL53L0X_SETPARAMETERFIELD_2(Dev, field, value) \
	PALDevDataSet_2(Dev, CurrentParameters.field, value)

#define VL53L0X_GETPARAMETERFIELD_2(Dev, field, variable_2) \
	variable_2 = PALDevDataGet_2(Dev, CurrentParameters).field


#define VL53L0X_SETARRAYPARAMETERFIELD_2(Dev, field, index, value) \
	PALDevDataSet_2(Dev, CurrentParameters.field[index], value)

#define VL53L0X_GETARRAYPARAMETERFIELD_2(Dev, field, index, variable_2) \
	variable_2 = PALDevDataGet_2(Dev, CurrentParameters).field[index]


#define VL53L0X_SETDEVICESPECIFICPARAMETER_2(Dev, field, value) \
		PALDevDataSet_2(Dev, DeviceSpecificParameters.field, value)

#define VL53L0X_GETDEVICESPECIFICPARAMETER_2(Dev, field) \
		PALDevDataGet_2(Dev, DeviceSpecificParameters).field


#define VL53L0X_FIXPOINT1616TOFIXPOINT97_2(Value) \
	(uint16_t)((Value>>9)&0xFFFF)
#define VL53L0X_FIXPOINT97TOFIXPOINT1616_2(Value) \
	(FixPoint1616_t_2)(Value<<9)

#define VL53L0X_FIXPOINT1616TOFIXPOINT88_2(Value) \
	(uint16_t)((Value>>8)&0xFFFF)
#define VL53L0X_FIXPOINT88TOFIXPOINT1616_2(Value) \
	(FixPoint1616_t_2)(Value<<8)

#define VL53L0X_FIXPOINT1616TOFIXPOINT412_2(Value) \
	(uint16_t)((Value>>4)&0xFFFF)
#define VL53L0X_FIXPOINT412TOFIXPOINT1616_2(Value) \
	(FixPoint1616_t_2)(Value<<4)

#define VL53L0X_FIXPOINT1616TOFIXPOINT313_2(Value) \
	(uint16_t)((Value>>3)&0xFFFF)
#define VL53L0X_FIXPOINT313TOFIXPOINT1616_2(Value) \
	(FixPoint1616_t_2)(Value<<3)

#define VL53L0X_FIXPOINT1616TOFIXPOINT08_2(Value) \
	(uint8_t)((Value>>8)&0x00FF)
#define VL53L0X_FIXPOINT08TOFIXPOINT1616_2(Value) \
	(FixPoint1616_t_2)(Value<<8)

#define VL53L0X_FIXPOINT1616TOFIXPOINT53_2(Value) \
	(uint8_t)((Value>>13)&0x00FF)
#define VL53L0X_FIXPOINT53TOFIXPOINT1616_2(Value) \
	(FixPoint1616_t_2)(Value<<13)

#define VL53L0X_FIXPOINT1616TOFIXPOINT102_2(Value) \
	(uint16_t)((Value>>14)&0x0FFF)
#define VL53L0X_FIXPOINT102TOFIXPOINT1616_2(Value) \
	(FixPoint1616_t_2)(Value<<12)

#define VL53L0X_MAKEUINT16_2(lsb, msb) (uint16_t)((((uint16_t)msb)<<8) + \
		(uint16_t)lsb)

/** @} VL53L0X_define_GeneralMacro_group */

/** @} VL53L0X_globaldefine_group */







#ifdef __cplusplus
}
#endif


#endif /* _VL53L0X_DEF_H_ */
