#include "usbd_uvc.h"

#include "lepton_i2c.h"

#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_AGC.h"
#include "LEPTON_VID.h"
#include "LEPTON_OEM.h"
#include "LEPTON_RAD.h"
#include "usbd_uvc_lepton_xu.h"

#include <limits.h>

extern LEP_CAMERA_PORT_DESC_T hport_desc;

int8_t VC_LEP_GetAttribute (struct Lepton_Command *command, uint8_t* pbuf, uint16_t length)
{
  LEP_RESULT result = LEP_GetAttribute(&hport_desc,
                                      command->commandId,
                                      ( LEP_ATTRIBUTE_T_PTR )pbuf,
                                      length >> 1);
  return result;
}

int8_t VC_LEP_SetAttribute (struct Lepton_Command *command, uint8_t* pbuf, uint16_t length)
{
	LEP_RESULT result;
	if(command->runnable){
		result = LEP_RunCommand(&hport_desc,
								command->commandId);
	}else{
		result = LEP_SetAttribute(&hport_desc,
								  command->commandId,
								  ( LEP_ATTRIBUTE_T_PTR )pbuf,
								  length >> 1);
	}
	return result;
}

int8_t VC_LEP_GetMaxValue (struct Lepton_Command *command, void* pbuf, uint16_t len)
{
  switch (command->commandId)
  {
#if 0
  case LEP_CID_AGC_ROI:
  case LEP_CID_AGC_STATISTICS:
    *((int64_t*)pbuf) = LLONG_MAX;
    break;
#else
  case LEP_CID_AGC_ROI:
  case LEP_CID_AGC_STATISTICS:
    memset(pbuf, 0xff, len);
    break;
#endif
  case LEP_CID_AGC_CALC_ENABLE_STATE:
  case LEP_CID_AGC_ENABLE_STATE:
    *((LEP_AGC_ENABLE_E_PTR)pbuf) = LEP_END_AGC_ENABLE - 1;
    break;
  case LEP_CID_AGC_POLICY:
    *((LEP_AGC_POLICY_E_PTR)pbuf) = LEP_END_AGC_POLICY - 1;
    break;
  case LEP_CID_AGC_HEQ_SCALE_FACTOR:
    *((LEP_AGC_HEQ_SCALE_FACTOR_E_PTR)pbuf) = LEP_AGC_END_SCALE_TO - 1;
    break;
#if 0
  case LEP_CID_OEM_FLIR_PART_NUMBER:
  case LEP_CID_OEM_CUST_PART_NUMBER:
    *pbuf = 32;
    break;
#else
  case LEP_CID_OEM_FLIR_PART_NUMBER:
  case LEP_CID_OEM_CUST_PART_NUMBER:
    memset(pbuf, 0xff, len);
    break;
#endif
  case LEP_CID_OEM_SOFTWARE_VERSION:
    *((LEP_OEM_SW_VERSION_T_PTR)pbuf) = (LEP_OEM_SW_VERSION_T) {
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00
    };
    break;
  case LEP_CID_OEM_VIDEO_OUTPUT_ENABLE:
    *((LEP_OEM_VIDEO_OUTPUT_ENABLE_E_PTR)pbuf) = LEP_END_VIDEO_OUTPUT_ENABLE - 1;
    break;
  case LEP_CID_OEM_VIDEO_OUTPUT_FORMAT:
    *((LEP_OEM_VIDEO_OUTPUT_FORMAT_E_PTR)pbuf) = LEP_END_VIDEO_OUTPUT_FORMAT - 1;
    break;
  case LEP_CID_OEM_VIDEO_OUTPUT_SOURCE:
    *((LEP_OEM_VIDEO_OUTPUT_SOURCE_E_PTR)pbuf) = LEP_END_VIDEO_OUTPUT_SOURCE - 1;
    break;
  case LEP_CID_OEM_VIDEO_OUTPUT_CHANNEL:
    *((LEP_OEM_VIDEO_OUTPUT_CHANNEL_E_PTR)pbuf) = LEP_END_VIDEO_OUTPUT_CHANNEL - 1;
    break;
  case LEP_CID_OEM_VIDEO_GAMMA_ENABLE:
    *((LEP_OEM_VIDEO_GAMMA_ENABLE_E_PTR)pbuf) = LEP_END_VIDEO_GAMMA_ENABLE - 1;
    break;
  case LEP_CID_OEM_STATUS:
    *((LEP_OEM_STATUS_E_PTR)pbuf) = LEP_OEM_STATUS_END - 1;
    break;
  case LEP_CID_OEM_POWER_MODE:
    *((LEP_OEM_POWER_STATE_E_PTR)pbuf) = LEP_OEM_END_POWER_MODE - 1;
    break;
  case LEP_CID_OEM_GPIO_MODE_SELECT:
    *((LEP_OEM_GPIO_MODE_E_PTR)pbuf) = LEP_OEM_END_GPIO_MODE - 1;
    break;
  case LEP_CID_OEM_GPIO_VSYNC_PHASE_DELAY: // TODO: handle min
    *((LEP_OEM_VSYNC_DELAY_E_PTR)pbuf) = LEP_END_OEM_VSYNC_DELAY - 1;
    break;
  case LEP_CID_OEM_USER_DEFAULTS: // also a command
    *((LEP_OEM_USER_PARAMS_STATE_E_PTR)pbuf) = LEP_OEM_END_USER_PARAMS_STATE - 1;
    break;
  case LEP_CID_OEM_THERMAL_SHUTDOWN_ENABLE_STATE:
  case LEP_CID_OEM_BAD_PIXEL_REPLACE_CONTROL:
  case LEP_CID_OEM_TEMPORAL_FILTER_CONTROL:
  case LEP_CID_OEM_COLUMN_NOISE_ESTIMATE_CONTROL:
  case LEP_CID_OEM_PIXEL_NOISE_ESTIMATE_CONTROL:
    *((LEP_OEM_STATE_E_PTR)pbuf) = LEP_OEM_END_STATE - 1;
    break;
  case LEP_CID_OEM_SHUTTER_PROFILE_OBJ:
    *((LEP_OEM_SHUTTER_PROFILE_OBJ_T_PTR)pbuf) = (LEP_OEM_SHUTTER_PROFILE_OBJ_T) {
      0xffff, 0xffff
    };
    break;
  case LEP_CID_OEM_POWER_DOWN:
  case LEP_CID_OEM_STANDBY:
  case LEP_CID_OEM_LOW_POWER_MODE_1:
  case LEP_CID_OEM_LOW_POWER_MODE_2:
  case LEP_CID_OEM_BIT_TEST:
  case LEP_CID_OEM_REBOOT:
  case LEP_CID_OEM_USER_DEFAULTS_RESTORE:
    *((uint8_t*)pbuf) = 1;
    break;
#if 0
  case LEP_CID_RAD_TFPA_LUT:
  case LEP_CID_RAD_TAUX_LUT:
    *pbuf = 512;
    break;
  case LEP_CID_RAD_RESPONSIVITY_VALUE_LUT:
  case LEP_CID_RAD_TEQ_SHUTTER_LUT:
  case LEP_CID_RAD_MLG_LUT:
    *pbuf = 256;
    break;
  case LEP_CID_RAD_RBFO_INTERNAL:
  case LEP_CID_RAD_RBFO_EXTERNAL:
  case LEP_CID_RAD_RBFO_INTERNAL_LG:
  case LEP_CID_RAD_RBFO_EXTERNAL_LG:
  case LEP_CID_RAD_FLUX_LINEAR_PARAMS:
    *pbuf = 16;
    break;
  case LEP_CID_AGC_ROI:
  case LEP_CID_AGC_STATISTICS:
  case LEP_CID_RAD_THOUSING_TCP:
  case LEP_CID_RAD_SHUTTER_TCP:
  case LEP_CID_RAD_LENS_TCP:
  case LEP_CID_RAD_SPOTMETER_ROI:
    *pbuf = 8;
    break;
#else
  case LEP_CID_RAD_TFPA_LUT:
  case LEP_CID_RAD_TAUX_LUT:
  case LEP_CID_RAD_RESPONSIVITY_VALUE_LUT:
  case LEP_CID_RAD_TEQ_SHUTTER_LUT:
  case LEP_CID_RAD_MLG_LUT:
  case LEP_CID_RAD_RBFO_INTERNAL:
  case LEP_CID_RAD_RBFO_EXTERNAL:
  case LEP_CID_RAD_RBFO_INTERNAL_LG:
  case LEP_CID_RAD_RBFO_EXTERNAL_LG:
  case LEP_CID_RAD_FLUX_LINEAR_PARAMS:
  case LEP_CID_RAD_THOUSING_TCP:
  case LEP_CID_RAD_SHUTTER_TCP:
  case LEP_CID_RAD_LENS_TCP:
  case LEP_CID_RAD_SPOTMETER_ROI:
    memset(pbuf, 0xff, len);
    break;
#endif
  case LEP_CID_RAD_TSHUTTER_MODE:
    *((LEP_RAD_TS_MODE_E_PTR)pbuf) = LEP_RAD_TS_END_TS_MODE - 1;
    break;
  case LEP_CID_RAD_DEBUG_FLUX: // TODO: handle min
  case LEP_CID_RAD_TEQ_SHUTTER_FLUX:
  case LEP_CID_RAD_MFFC_FLUX:
    *((LEP_INT32*)pbuf) = LONG_MAX;
    break;
  case LEP_CID_RAD_ENABLE_STATE:
  case LEP_CID_RAD_TLINEAR_ENABLE_STATE:
  case LEP_CID_RAD_TLINEAR_AUTO_RESOLUTION:
    *((LEP_RAD_ENABLE_E_PTR)pbuf) = LEP_END_RAD_ENABLE - 1;
    break;
  case LEP_CID_RAD_TFPA_CTS_MODE:
  case LEP_CID_RAD_TAUX_CTS_MODE:
    *((LEP_RAD_TEMPERATURE_UPDATE_E_PTR)pbuf) = LEP_RAD_UPDATE_END - 1;
    break;
  case LEP_CID_RAD_RUN_STATUS:
    *((LEP_RAD_STATUS_E_PTR)pbuf) = LEP_RAD_STATUS_END - 1;
    break;
  case LEP_CID_RAD_TLINEAR_RESOLUTION:
    *((LEP_RAD_TLINEAR_RESOLUTION_E_PTR)pbuf) = LEP_RAD_END_RESOLUTION - 1;
    break;
  case LEP_CID_RAD_ARBITRARY_OFFSET_MODE:
    *((LEP_RAD_ARBITRARY_OFFSET_MODE_E_PTR)pbuf) = LEP_RAD_END_ARBITRARY_OFFSET_MODE - 1;
    break;
  case LEP_CID_RAD_ARBITRARY_OFFSET_PARAMS:
    *((LEP_RAD_ARBITRARY_OFFSET_PARAMS_T_PTR)pbuf) = (LEP_RAD_ARBITRARY_OFFSET_PARAMS_T) {
      0xffff, 0xffff
    };
    break;
  case LEP_CID_RAD_RUN_FFC:
    *((uint8_t*)pbuf) = 1;
    break;
#if 0
  case LEP_CID_SYS_CUST_SERIAL_NUMBER:
  case LEP_CID_SYS_FFC_SHUTTER_MODE_OBJ:
    *pbuf = 32;
    break;
  case LEP_CID_SYS_SCENE_STATISTICS:
  case LEP_CID_SYS_SCENE_ROI:
    *pbuf = 8;
    break;
#else
  case LEP_CID_SYS_CUST_SERIAL_NUMBER:
  case LEP_CID_SYS_FFC_SHUTTER_MODE_OBJ:
  case LEP_CID_SYS_SCENE_STATISTICS:
  case LEP_CID_SYS_SCENE_ROI:
    memset(pbuf, 0xff, len);
    break;
#endif
  case LEP_CID_SYS_CAM_STATUS:
  case LEP_CID_SYS_FLIR_SERIAL_NUMBER:
    *((int64_t*)pbuf) = LLONG_MAX;
    break;
  case LEP_CID_SYS_CAM_UPTIME:
    *((LEP_SYS_UPTIME_NUMBER_T_PTR)pbuf) = ULONG_MAX;
    break;
  case LEP_CID_SYS_TELEMETRY_ENABLE_STATE:
    *((LEP_SYS_TELEMETRY_ENABLE_STATE_E_PTR)pbuf) = LEP_END_TELEMETRY_ENABLE_STATE - 1;
    break;
  case LEP_CID_SYS_TELEMETRY_LOCATION:
    *((LEP_SYS_TELEMETRY_LOCATION_E_PTR)pbuf) = LEP_END_TELEMETRY_LOCATION - 1;
    break;
  case LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE:
    *((LEP_SYS_FRAME_AVERAGE_DIVISOR_E_PTR)pbuf) = LEP_SYS_END_FA_DIV - 1;
    break;
  case LEP_CID_SYS_SHUTTER_POSITION:
    *((LEP_SYS_SHUTTER_POSITION_E_PTR)pbuf) = LEP_SYS_SHUTTER_POSITION_END - 1;
    break;
  case LEP_CID_SYS_FFC_STATUS: // TODO: handle min
    *((LEP_SYS_STATUS_E_PTR)pbuf) = LEP_SYS_STATUS_END -1;
    break;
  case LEP_CID_SYS_PING:
  case LEP_CID_SYS_EXECTUE_FRAME_AVERAGE:
  case (FLR_CID_SYS_RUN_FFC & 0xfffc):
    *((uint8_t*)pbuf) = 1;
    break;
#if 0
  case LEP_CID_VID_LUT_TRANSFER:
    *pbuf = 1024;
    break;
  case LEP_CID_VID_FOCUS_ROI:
    *pbuf = 8;
    break;
#else
  case LEP_CID_VID_LUT_TRANSFER:
  case LEP_CID_VID_FOCUS_ROI:
    memset(pbuf, 0xff, len);
    break;
#endif
  case LEP_CID_VID_POLARITY_SELECT:
    *((LEP_POLARITY_E_PTR)pbuf) = LEP_VID_END_POLARITY - 1;
    break;
  case LEP_CID_VID_LUT_SELECT:
    *((LEP_PCOLOR_LUT_E_PTR)pbuf) = LEP_VID_END_PCOLOR_LUT - 1;
    break;
  case LEP_CID_VID_FOCUS_CALC_ENABLE:
    *((LEP_VID_FOCUS_CALC_ENABLE_E_PTR)pbuf) = LEP_VID_END_FOCUS_CALC_ENABLE - 1;
    break;
  case LEP_CID_VID_FOCUS_METRIC:
    *((LEP_VID_FOCUS_METRIC_T_PTR)pbuf) = ULONG_MAX;
    break;
  case LEP_CID_VID_FOCUS_THRESHOLD:
    *((LEP_VID_FOCUS_METRIC_THRESHOLD_T_PTR)pbuf) = ULONG_MAX;
    break;
  case LEP_CID_VID_SBNUC_ENABLE:
    *((LEP_VID_SBNUC_ENABLE_E_PTR)pbuf) = LEP_VID_END_SBNUC_ENABLE - 1;
    break;
  case LEP_CID_VID_FREEZE_ENABLE:
    *((LEP_VID_FREEZE_ENABLE_E_PTR)pbuf) = LEP_VID_END_FREEZE_ENABLE - 1;
    break;
  default:
    *((uint16_t*)pbuf) = USHRT_MAX;
    break;
  }
  return LEP_OK;
}
