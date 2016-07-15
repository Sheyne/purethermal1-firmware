
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"


#include "pt.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"

#include "tasks.h"
#include "project_config.h"

static volatile int segment_number; // TODO REMOVE SHEYNE

y8_full_buffer double_buffer[2];

uint8_t filling_buffer_index = 0;
int segment_index_ready = -1, segment_index_copied = -1;

lepton_buffer *filling_segments[4];

uint32_t completed_frame_count = 0;

uint8_t lepton_i2c_buffer[36];


#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif


uint32_t get_lepton_buffer(y8_full_buffer **buffer)
{
  if (buffer != NULL)
    *buffer = &double_buffer[!filling_buffer_index];
  return completed_frame_count;
}


void init_lepton_state(void);
void init_lepton_state(void)
{

}

PT_THREAD( convert_task(struct pt *pt)){
	PT_BEGIN(pt);

	static int output_row, j, input_row, input_col, segment_index_in_progress;

	PT_YIELD_UNTIL(pt, segment_index_copied < segment_index_ready);

	segment_index_in_progress = segment_index_copied + 1;

	// 2 segment lines per output row
	for(output_row = 0 ; output_row < IMAGE_NUM_LINES/2; output_row ++){
		// j is 0 if we are on a left line, 1 if on right line
		for (j = 0; j < 2; j ++){
			// input row computed from output row and j
			input_row = output_row * 2 + j;
			for (input_col = 0; input_col < FRAME_LINE_LENGTH; input_col++){
				// output col is dependent on j
				// select the g channel b/c we are using greyscale and g will get the
				// right pixel (although not the right channel) even if the bits are
				// flipped for 16bit opposite endian and r = g = b for greyscale.
				double_buffer[filling_buffer_index].segments[segment_index_in_progress][output_row][input_col + j * FRAME_LINE_LENGTH] =
						filling_segments[segment_index_in_progress]->lines[input_row].data.image_data[input_col].g;
			}
		}
		PT_YIELD(pt);
	}

	segment_index_copied = segment_index_in_progress;

	if(segment_index_copied == 3){
		HAL_GPIO_TogglePin(SYSTEM_LED_GPIO_Port, SYSTEM_LED_Pin);
		filling_buffer_index = !filling_buffer_index;
		segment_index_copied = -1;
		segment_index_ready = -1;
		completed_frame_count ++;
	}

	PT_END(pt);
}

PT_THREAD( lepton_task(struct pt *pt))
{
	PT_BEGIN(pt);

	static lepton_buffer *current_buffer;

	while (1)
	{
		current_buffer = lepton_transfer();

		PT_YIELD_UNTIL(pt, current_buffer->status != LEPTON_STATUS_TRANSFERRING);

		if (complete_lepton_transfer(current_buffer) != LEPTON_STATUS_OK)
		{
			continue;
		}

		segment_number = (current_buffer->lines[20].header[0] >> 12) & 0x7;
		int segment_index = segment_number - 1; // number = 0, index = -1 indicates that this is a repeat
		// (and thus discardable) segment

		if (segment_index_ready + 1 == segment_index){
			segment_index_ready = segment_index;

			filling_segments[segment_index_ready] = current_buffer;
			increment_buffer_index();
		}else{
			segment_index_copied = -1;
			segment_index_ready = -1;
		}
	}
	PT_END(pt);
}
