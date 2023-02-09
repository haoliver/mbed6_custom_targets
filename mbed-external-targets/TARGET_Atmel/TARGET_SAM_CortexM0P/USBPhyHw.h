/*
** ================================================================================
** MIT License
** 
** Copyright (c) 2022 HA Oliver
** 
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** 
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.

** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
** ================================================================================
*/

#ifndef USBPHYHW_H
#define USBPHYHW_H

#include "mbed.h"
#include "USBPhy.h"

#include "samd21g18a.h"

class USBPhyHw : public USBPhy {
public:
    USBPhyHw();
    virtual ~USBPhyHw();
    virtual void init(USBPhyEvents *events);
    virtual void deinit();
    virtual bool powered();
    virtual void connect();
    virtual void disconnect();
    virtual void configure();
    virtual void unconfigure();
    virtual void sof_enable();
    virtual void sof_disable();
    virtual void set_address(uint8_t address);
    virtual void remote_wakeup();
    virtual const usb_ep_table_t *endpoint_table();

    virtual uint32_t ep0_set_max_packet(uint32_t max_packet);
    virtual void ep0_setup_read_result(uint8_t *buffer, uint32_t size);
    virtual void ep0_read(uint8_t *data, uint32_t size);
    virtual uint32_t ep0_read_result();
    virtual void ep0_write(uint8_t *buffer, uint32_t size);
    virtual void ep0_stall();

    virtual bool endpoint_add(usb_ep_t endpoint, uint32_t max_packet, usb_ep_type_t type);
    virtual void endpoint_remove(usb_ep_t endpoint);
    virtual void endpoint_stall(usb_ep_t endpoint);
    virtual void endpoint_unstall(usb_ep_t endpoint);

    virtual bool endpoint_read(usb_ep_t endpoint, uint8_t *data, uint32_t size);
    virtual uint32_t endpoint_read_result(usb_ep_t endpoint);
    virtual bool endpoint_write(usb_ep_t endpoint, uint8_t *data, uint32_t size);
    virtual void endpoint_abort(usb_ep_t endpoint);

    virtual void process();

private:
    USBPhyEvents *events;
    bool mustSetAddress;
    uint8_t new_address;
    uint8_t *ep0_read_buffer;
    uint32_t ep0_read_size;

    static void _usbisr(void);
};

#endif
