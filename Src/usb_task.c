
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

extern volatile uint8_t uvc_stream_status;
extern struct uvc_streaming_control videoCommitControl;


static int last_frame_count;
static y8_full_buffer *last_buffer;

#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

void HAL_RCC_CSSCallback(void) {
    DEBUG_PRINTF("Oh no! HAL_RCC_CSSCallback()\r\n");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    DEBUG_PRINTF("Yay! HAL_GPIO_EXTI_Callback()\r\n");
}


static inline uint8_t clamp (float x)
{
  if (x < 0)         return 0;
  else if (x > 255)  return 255;
  else               return (uint8_t)x;
}


PT_THREAD( usb_task(struct pt *pt))
{
    static int temperature;
    static uint16_t count = 0, i;
    static uint8_t uvc_header[2] = { 2, 0 };
    static uint32_t uvc_xmit_row = 0, uvc_xmit_plane = 0;
    static uint8_t packet[VIDEO_PACKET_SIZE];
    static uint8_t frame_number = 0;

    PT_BEGIN(pt);

    while (1)
    {
        PT_WAIT_UNTIL(pt, get_lepton_buffer(NULL) != last_frame_count);
        last_frame_count = get_lepton_buffer(&last_buffer);





    // perform stream initialization
    if (uvc_stream_status == 1)
    {
      DEBUG_PRINTF("Starting UVC stream...\r\n");

      uvc_header[0] = 2;
      uvc_header[1] = 0;
      UVC_Transmit_FS(uvc_header, 2);

      uvc_stream_status = 2;
      uvc_xmit_row = 0;
      uvc_xmit_plane = 0;
    }

    // put image on stream as long as stream is open
    while (uvc_stream_status == 2) {
        count = 0;

        packet[count++] = uvc_header[0];
        packet[count++] = uvc_header[1];

        switch (videoCommitControl.bFormatIndex) {
        case VS_FMT_INDEX(YUYV): {
            while (uvc_xmit_row < SHEYNE_HEIGHT && count + SHEYNE_WIDTH * 2 <= VIDEO_PACKET_SIZE) {
            	int width = SHEYNE_WIDTH;

            	int segment_index = uvc_xmit_row / (SHEYNE_HEIGHT / 4);
            	int segment_row = uvc_xmit_row - segment_index * (SHEYNE_HEIGHT / 4);

            	for(int i = 0; i < SHEYNE_WIDTH; i++){
                	uint8_t grey = last_buffer->segments[segment_index][segment_row][i];
                	packet[count++] = 127;
                	packet[count++] = grey;
            	}

            	uvc_xmit_row++;
            }
            // image is done
            if (uvc_xmit_row == SHEYNE_HEIGHT) {
                packet[1] |= 0x2; // Flag end of frame
            }

            break;
        }
    }


      // printf("UVC_Transmit_FS(): packet=%p, count=%d\r\n", packet, count);
      // fflush(stdout);

      int retries = 1000;
      while (UVC_Transmit_FS(packet, count) == USBD_BUSY && uvc_stream_status == 2)
      {
        if (--retries == 0) {
//          DEBUG_PRINTF("UVC_Transmit_FS() failed (no one is acking)\r\n");
          break;
        }
      }

      if (packet[1] & 0x2)
      {
        uvc_header[1] ^= 1; // toggle bit 0 for next new frame
        uvc_xmit_row = 0;
        uvc_xmit_plane = 0;
        frame_number ++;
        // DEBUG_PRINTF("Frame complete\r\n");
        break;
      }
      PT_YIELD(pt);
    }
    PT_YIELD(pt);
    }
    PT_END(pt);
}
