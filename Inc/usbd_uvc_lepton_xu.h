#ifndef __USBD_UVC_LEPTON_XU_H
#define __USBD_UVC_LEPTON_XU_H

#include "lepton_UVC_index_convert.h"

int8_t VC_LEP_GetAttribute (struct Lepton_Command *command, uint8_t* pbuf, uint16_t length);
int8_t VC_LEP_SetAttribute (struct Lepton_Command *command, uint8_t* pbuf, uint16_t length);
int8_t VC_LEP_GetMaxValue (struct Lepton_Command *command, void* pbuf, uint16_t len);

#endif
