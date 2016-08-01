#ifndef LEPTON_UVC_INDEX_CONVERT_H
#define LEPTON_UVC_INDEX_CONVERT_H

#include "LEPTON_Types.h"

struct Lepton_Command {
	LEP_COMMAND_ID commandId;
	LEP_UINT16 attributeWordLength;
	uint8_t runnable;
};

extern struct Lepton_Command Lepton_Command_Invalid;
extern struct Lepton_Command Lepton_Command_Terminal_Not_Found;

struct Lepton_Command *cs_value_to_lepton_sdk(int cs_value, VC_TERMINAL_ID entity_id);

#endif
