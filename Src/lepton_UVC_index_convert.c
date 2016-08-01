#include "usbd_uvc.h"
#include "LEPTON_ErrorCodes.h"
#include "LEPTON_AGC.h"
#include "LEPTON_OEM.h"
#include "LEPTON_SYS.h"
#include "LEPTON_VID.h"
#include "LEPTON_RAD.h"
#include "lepton_UVC_index_convert.h"
#include "LEPTON_Types.h"

struct Lepton_Command Lepton_Command_Invalid;
struct Lepton_Command Lepton_Command_Terminal_Not_Found;

struct Lepton_Command Lepton_Command_Attributes_AGC[] = {
    {
        .commandId = LEP_CID_AGC_CALC_ENABLE_STATE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_ENABLE_STATE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_HEQ_BIN_EXTENSION,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_HEQ_CLIP_LIMIT_HIGH,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_HEQ_CLIP_LIMIT_LOW,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_HEQ_DAMPENING_FACTOR,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_HEQ_EMPTY_COUNTS,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_HEQ_MAX_GAIN,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_HEQ_MIDPOINT,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_HEQ_NORMALIZATION_FACTOR,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_HEQ_SCALE_FACTOR,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_HISTOGRAM_CLIP_PERCENT,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_HISTOGRAM_TAIL_SIZE,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_LINEAR_DAMPENING_FACTOR,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_LINEAR_MAX_GAIN,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_LINEAR_MIDPOINT,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_POLICY,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_ROI,
        .attributeWordLength = 4,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_AGC_STATISTICS,
        .attributeWordLength = 4,
        .runnable = 0
    },
};
struct Lepton_Command Lepton_Command_Attributes_OEM[] = {
    {
        .commandId = LEP_CID_OEM_BAD_PIXEL_REPLACE_CONTROL,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_BIT_TEST,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_OEM_COLUMN_NOISE_ESTIMATE_CONTROL,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_CUST_PART_NUMBER,
        .attributeWordLength = 16,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_FFC_NORMALIZATION_TARGET,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_FFC_NORMALIZATION_TARGET,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_OEM_FLIR_PART_NUMBER,
        .attributeWordLength = 16,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_GPIO_MODE_SELECT,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_GPIO_VSYNC_PHASE_DELAY,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_LOW_POWER_MODE_1,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_OEM_LOW_POWER_MODE_2,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_OEM_MASK_REVISION,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_PIXEL_NOISE_ESTIMATE_CONTROL,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_POWER_DOWN,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_OEM_POWER_MODE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_REBOOT,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_OEM_SCENE_MEAN_VALUE,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_SHUTTER_PROFILE_OBJ,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_SOFTWARE_VERSION,
        .attributeWordLength = 4,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_STANDBY,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_OEM_STATUS,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_TEMPORAL_FILTER_CONTROL,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_THERMAL_SHUTDOWN_ENABLE_STATE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_USER_DEFAULTS,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_OEM_USER_DEFAULTS,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_USER_DEFAULTS_RESTORE,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_OEM_VIDEO_GAMMA_ENABLE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_VIDEO_OUTPUT_CHANNEL,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_VIDEO_OUTPUT_CONSTANT,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_VIDEO_OUTPUT_ENABLE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_VIDEO_OUTPUT_FORMAT,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_OEM_VIDEO_OUTPUT_SOURCE,
        .attributeWordLength = 2,
        .runnable = 0
    },
};
struct Lepton_Command Lepton_Command_Attributes_RAD[] = {
    {
        .commandId = LEP_CID_RAD_ARBITRARY_OFFSET,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_ARBITRARY_OFFSET_MODE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_ARBITRARY_OFFSET_PARAMS,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_CNF_SCALE_FACTOR,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_DEBUG_FLUX,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_DEBUG_TEMP,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_ENABLE_STATE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_FLUX_LINEAR_PARAMS,
        .attributeWordLength = 8,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_FRAME_MEDIAN_VALUE,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_F_NUMBER,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_GLOBAL_GAIN,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_GLOBAL_GAIN_FFC,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_GLOBAL_OFFSET,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_HOUSING_TCP,
        .attributeWordLength = 4,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_LENS_TCP,
        .attributeWordLength = 4,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_MFFC_FLUX,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_MLG_LUT,
        .attributeWordLength = 128,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_PREVIOUS_GLOBAL_GAIN,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_PREVIOUS_GLOBAL_OFFSET,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_RADIOMETRY_FILTER,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_RBFO_EXTERNAL,
        .attributeWordLength = 8,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_RBFO_EXTERNAL_LG,
        .attributeWordLength = 8,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_RBFO_INTERNAL,
        .attributeWordLength = 8,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_RBFO_INTERNAL_LG,
        .attributeWordLength = 8,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_RESPONSIVITY_SHIFT,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_RESPONSIVITY_VALUE_LUT,
        .attributeWordLength = 128,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_RUN_FFC,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_RAD_RUN_STATUS,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_SHUTTER_TCP,
        .attributeWordLength = 4,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_SNF_SCALE_FACTOR,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_SPOTMETER_ROI,
        .attributeWordLength = 4,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_SPOTMETER_VALUE_KELVIN,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TAUX_CTS,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TAUX_CTS_MODE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TAUX_LUT,
        .attributeWordLength = 256,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TAU_LENS,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TEQ_SHUTTER_FLUX,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TEQ_SHUTTER_LUT,
        .attributeWordLength = 128,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TFPA_CTS,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TFPA_CTS_MODE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TFPA_LUT,
        .attributeWordLength = 256,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_THOUSING_TCP,
        .attributeWordLength = 4,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TLINEAR_AUTO_RESOLUTION,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TLINEAR_ENABLE_STATE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TLINEAR_RESOLUTION,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TNF_SCALE_FACTOR,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TSHUTTER,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_RAD_TSHUTTER_MODE,
        .attributeWordLength = 2,
        .runnable = 0
    },
};
struct Lepton_Command Lepton_Command_Attributes_SYS[] = {
    {
        .commandId = FLR_CID_SYS_RUN_FFC,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_SYS_AUX_TEMPERATURE_KELVIN,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_CAM_STATUS,
        .attributeWordLength = 4,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_CAM_UPTIME,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_CUST_SERIAL_NUMBER,
        .attributeWordLength = 16,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_EXECTUE_FRAME_AVERAGE,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_SYS_FFC_SHUTTER_MODE_OBJ,
        .attributeWordLength = 16,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_FFC_STATUS,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_FLIR_SERIAL_NUMBER,
        .attributeWordLength = 4,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_FPA_TEMPERATURE_KELVIN,
        .attributeWordLength = 1,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_PING,
        .attributeWordLength = 1,
        .runnable = 1
    },
    {
        .commandId = LEP_CID_SYS_SCENE_ROI,
        .attributeWordLength = 4,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_SCENE_STATISTICS,
        .attributeWordLength = 4,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_SHUTTER_POSITION,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_TELEMETRY_ENABLE_STATE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_TELEMETRY_LOCATION,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_SYS_THERMAL_SHUTDOWN_COUNT,
        .attributeWordLength = 1,
        .runnable = 0
    },
};
struct Lepton_Command Lepton_Command_Attributes_VID[] = {
    {
        .commandId = LEP_CID_VID_FOCUS_CALC_ENABLE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_VID_FOCUS_METRIC,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_VID_FOCUS_ROI,
        .attributeWordLength = 4,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_VID_FOCUS_THRESHOLD,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_VID_FREEZE_ENABLE,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_VID_LUT_SELECT,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_VID_LUT_TRANSFER,
        .attributeWordLength = 512,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_VID_POLARITY_SELECT,
        .attributeWordLength = 2,
        .runnable = 0
    },
    {
        .commandId = LEP_CID_VID_SBNUC_ENABLE,
        .attributeWordLength = 2,
        .runnable = 0
    },
};

struct Lepton_Command *cs_value_to_lepton_sdk(int cs_value, VC_TERMINAL_ID entity_id){
	struct Lepton_Command *map;
	size_t size = 0;
	switch (entity_id){
	  case VC_CONTROL_XU_LEP_AGC_ID:
		  map = Lepton_Command_Attributes_AGC;
		  size = sizeof(Lepton_Command_Attributes_AGC);
		  break;
	  case VC_CONTROL_XU_LEP_OEM_ID:
		  map = Lepton_Command_Attributes_OEM;
		  size = sizeof(Lepton_Command_Attributes_OEM);
		  break;
	  case VC_CONTROL_XU_LEP_RAD_ID:
		  map = Lepton_Command_Attributes_RAD;
		  size = sizeof(Lepton_Command_Attributes_RAD);
		  break;
	  case VC_CONTROL_XU_LEP_SYS_ID:
		  map = Lepton_Command_Attributes_SYS;
		  size = sizeof(Lepton_Command_Attributes_SYS);
		  break;
	  case VC_CONTROL_XU_LEP_VID_ID:
		  map = Lepton_Command_Attributes_VID;
		  size = sizeof(Lepton_Command_Attributes_VID);
		  break;
	  default:
		  return &Lepton_Command_Terminal_Not_Found;
	}
	int idx = cs_value - 1;
	if (idx < size / sizeof(struct Lepton_Command))
		return &map[cs_value - 1];
	return &Lepton_Command_Invalid;
}
