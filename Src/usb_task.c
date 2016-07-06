
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

static int overlay_mode = 0;

void change_overlay_mode(void)
{
    overlay_mode = (overlay_mode+1) % 2;
}

static int last_frame_count;
static lepton_buffer *last_buffer;

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
            	if (uvc_xmit_row * 2 < SHEYNE_HEIGHT){
            		width = SHEYNE_WIDTH / 2;

            		// endian flip
            	    uint16_t* lineptr = (uint16_t*)last_buffer->lines[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data;
            	    while (lineptr < (uint16_t*)&last_buffer->lines[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[FRAME_LINE_LENGTH])
            	    {
            	      uint8_t* bytes = (uint8_t*)lineptr;
            	      *lineptr++ = bytes[0] << 8 | bytes[1];
            	    }

            	    // can't afford the buffers to do this in it's own task
                    for (i = 0; i < SHEYNE_WIDTH / 2; i++) {
                        rgb_t val = last_buffer->lines[IMAGE_OFFSET_LINES + uvc_xmit_row].data.image_data[i];
                        float r = val.r, g = val.g, b = val.b;
                        float y1 = 0.299f * r + 0.587f * g + 0.114f * b;

                        if ((i % 2) == 0)
                        	packet[count++] = clamp (0.496f * (b - y1) + 128.0f);
                        else
                        	packet[count++] = clamp (0.627f * (r - y1) + 128.0f);

                        packet[count++] = clamp (0.859f *      y1  +  16.0f);
                    }
//            		  memcpy(&packet[count], last_buffer->data[uvc_xmit_row], sizeof(yuv422_row_t));
//            		  count += sizeof(yuv422_row_t);
            	}

                for (int x = 0; x < width; x++) {
                    unsigned char chroma_square = x % 2 ? 120 : 0;
                    unsigned char chroma_background = x % 2 ? 0 : 120;

                    packet[count++] = x > (frame_number) && x < (frame_number + 10) && uvc_xmit_row > (frame_number) && uvc_xmit_row <  (frame_number + 10) ? chroma_square : chroma_background;
                    packet[count++] = 120;
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
