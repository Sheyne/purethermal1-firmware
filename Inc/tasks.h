#ifndef _TASKS_H_
#define _TASKS_H_

#include "pt.h"

// Long running tasks
PT_THREAD( lepton_task(struct pt *pt));
PT_THREAD( uart_task(struct pt *pt));
PT_THREAD( usb_task(struct pt *pt));
PT_THREAD( button_task(struct pt *pt));
PT_THREAD( convert_task(struct pt *pt));

// Temporary tasks
PT_THREAD( uart_lepton_send(struct pt *pt,char * buffer));

// Synchronization helpers
uint32_t get_lepton_buffer(y8_full_buffer **buffer);

#endif
