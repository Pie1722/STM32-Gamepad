#ifndef __usb_xinput_H
#define __usb_xinput_H

//#define XINPUT_RX_ENDPOINT   2      // Endpoint 2 OUT
//#define XINPUT_RX_SIZE       8

#include <inttypes.h>
#include "stdint.h"

int usb_xinput_recv(void *buffer, uint32_t timeout);
int usb_xinput_available(void);
int usb_xinput_send(const void *buffer, uint32_t timeout);


#endif // USBxinput_h_
