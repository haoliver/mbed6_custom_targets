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

#include "USBPhyHw.h"

static USBPhyHw *instance;

#define USB_EPTYPE_DISABLED     0
#define OUT_BANK                0
#define IN_BANK                 1
#define USB_EP0_SIZE            64
#define NVM_USB_PAD_TRANSN_POS  45
#define NVM_USB_PAD_TRANSN_SIZE 5
#define NVM_USB_PAD_TRANSP_POS  50
#define NVM_USB_PAD_TRANSP_SIZE 5
#define NVM_USB_PAD_TRIM_POS    55
#define NVM_USB_PAD_TRIM_SIZE   3
#define USB_GCLK_GEN            0

#define USB_ALIGN __attribute__((__aligned__(4)))

#define USB_EP_SIZE_TO_BIT(x)  ((x <= 8   )?0:\
                               (x <= 16  )?1:\
                               (x <= 32  )?2:\
                               (x <= 64  )?3:\
                               (x <= 128 )?4:\
                               (x <= 256 )?5:\
                               (x <= 512 )?6:\
                                           7)

#undef ENABLE

USB_ALIGN uint8_t ep0_buf_in[USB_EP0_SIZE];
USB_ALIGN uint8_t ep0_buf_out[USB_EP0_SIZE];

const uint8_t usb_num_endpoints = USB_EPT_NUM;
UsbDeviceDescriptor usb_endpoints[USB_EPT_NUM +1];

void USBMemCopy(uint8_t *dst, uint8_t *src, uint32_t size);
void USBMemCopy(uint8_t *dst, uint8_t *src, uint32_t size) {
    if (size > 0) {
        do {
            *dst++ = *src++;
        } while (--size > 0);
    }
}

USBPhy *get_usb_phy()
{
    static USBPhyHw usbphy;
    return &usbphy;
}

USBPhyHw::USBPhyHw(): 
    events(NULL),
    mustSetAddress(false),
    new_address(0),
    ep0_read_buffer(NULL),
    ep0_read_size(0)
{
}

USBPhyHw::~USBPhyHw()
{
}

void USBPhyHw::init(USBPhyEvents *events)
{
    this->events = events;

    // Disable IRQ
    NVIC_DisableIRQ(USB_IRQn);

    // Turn on the digital interface clock
    system_apb_clock_set_mask(SYSTEM_CLOCK_APB_APBB, PM_APBBMASK_USB);

    // Set up the USB DP/DN pins
    struct system_pinmux_config pin_config;

    system_pinmux_get_config_defaults(&pin_config);
    pin_config.mux_position = MUX_PA24G_USB_DM;
    system_pinmux_pin_set_config(PIN_PA24G_USB_DM, &pin_config);
    pin_config.mux_position = MUX_PA25G_USB_DP;
    system_pinmux_pin_set_config(PIN_PA25G_USB_DP, &pin_config);

    // Setup clock for module
    struct system_gclk_chan_config gclk_chan_config;

    system_gclk_chan_get_config_defaults(&gclk_chan_config);
    gclk_chan_config.source_generator = GCLK_GENERATOR_0;
    system_gclk_chan_set_config(USB_GCLK_ID, &gclk_chan_config);
    system_gclk_chan_enable(USB_GCLK_ID);

    // Reset the module
    USB->DEVICE.CTRLA.bit.SWRST = 1;
    while (USB->DEVICE.SYNCBUSY.bit.SWRST) {
        // Sync wait
    }

    // Change QOS values to have the best performance and correct USB behaviour
    USB->DEVICE.QOSCTRL.bit.CQOS = 2;
    USB->DEVICE.QOSCTRL.bit.DQOS = 2;

    // Load Pad Calibration
    uint32_t pad_transn, pad_transp, pad_trim;

    // Load Pad Calibration
    pad_transn =( *((uint32_t *)(NVMCTRL_OTP4)
            + (NVM_USB_PAD_TRANSN_POS / 32))
        >> (NVM_USB_PAD_TRANSN_POS % 32))
        & ((1 << NVM_USB_PAD_TRANSN_SIZE) - 1);

    if (pad_transn == 0x1F) {
        pad_transn = 5;
    }

    // Set Pad Calibration
    USB->DEVICE.PADCAL.bit.TRANSN = pad_transn;

    pad_transp =( *((uint32_t *)(NVMCTRL_OTP4)
            + (NVM_USB_PAD_TRANSP_POS / 32))
            >> (NVM_USB_PAD_TRANSP_POS % 32))
            & ((1 << NVM_USB_PAD_TRANSP_SIZE) - 1);

    if (pad_transp == 0x1F) {
        pad_transp = 29;
    }

    USB->DEVICE.PADCAL.bit.TRANSP = pad_transp;

    pad_trim =( *((uint32_t *)(NVMCTRL_OTP4)
            + (NVM_USB_PAD_TRIM_POS / 32))
            >> (NVM_USB_PAD_TRIM_POS % 32))
            & ((1 << NVM_USB_PAD_TRIM_SIZE) - 1);

    if (pad_trim == 0x7) {
        pad_trim = 3;
    }

    USB->DEVICE.PADCAL.bit.TRIM = pad_trim;

    memset((void *)usb_endpoints, 0, usb_num_endpoints * sizeof(UsbDeviceDescriptor));
    USB->DEVICE.DESCADD.reg = (uint32_t)(&usb_endpoints[0]);

    // Enable needed interrupts
    USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENSET_SOF | USB_DEVICE_INTENSET_EORST;
    USB->DEVICE.DeviceEndpoint[0].EPINTENSET.reg = USB_DEVICE_EPINTENSET_RXSTP | USB_DEVICE_EPINTENSET_TRCPT1;
    USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;

    // Set speed configuration to Full speed
    USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;
    // Enable Run in Standby
    USB->DEVICE.CTRLA.bit.RUNSTDBY = true;
    // Set mode to Device mode
    USB->DEVICE.CTRLA.bit.MODE = 0;
    // Attach to the USB host
    USB->DEVICE.CTRLB.reg &= ~USB_DEVICE_CTRLB_DETACH;

    memset((uint8_t *)(&usb_endpoints[0]), 0, sizeof(usb_endpoints));

    instance = this;

    // Enable IRQ
    NVIC_SetVector(USB_IRQn, (uint32_t)&_usbisr);
    NVIC_EnableIRQ(USB_IRQn);

    // Enable USB module
    USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;

    while (USB->DEVICE.SYNCBUSY.bit.ENABLE) {
        // Sync wait
    }
}

void USBPhyHw::deinit()
{
    // Disconnect and disable interrupt
    disconnect();
    NVIC_DisableIRQ(USB_IRQn);
}

bool USBPhyHw::powered()
{
    // TODO - return true if powered false otherwise. Devices which don't support
    //    this should always return true
    return true;
}

void USBPhyHw::connect()
{
}

void USBPhyHw::disconnect()
{
}

void USBPhyHw::configure()
{
    // TODO - set device to configured. Most device will not need this
}

void USBPhyHw::unconfigure()
{
    // TODO - set device to unconfigured. Most device will not need this
}

void USBPhyHw::sof_enable()
{
    // TODO - Enable SOF interrupt
}

void USBPhyHw::sof_disable()
{
    // TODO - Disable SOF interrupt
}

void USBPhyHw::set_address(uint8_t address)
{
    if (address > 0) {
        mustSetAddress = true;
        new_address = address;
    } else {
        mustSetAddress = false;
        USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | 0;
    }
}

void USBPhyHw::remote_wakeup()
{
    // TODO - Sent remote wakeup over USB lines (if supported)
}

const usb_ep_table_t *USBPhyHw::endpoint_table()
{
    // TODO - Update the endpoint table for what your device supports

    static const usb_ep_table_t endpoints_table = {
        4096 - 32 * 4, // 32 words for endpoint buffers
        // +3 based added to interrupt and isochronous to ensure enough
        // space for 4 byte alignment
        {
            {USB_EP_ATTR_ALLOW_CTRL | USB_EP_ATTR_DIR_IN_AND_OUT, 1, 0},
            {USB_EP_ATTR_ALLOW_INT | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 2, 0},
            {USB_EP_ATTR_ALLOW_ISO | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_INT | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 2, 0},
            {USB_EP_ATTR_ALLOW_ISO | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_INT | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 2, 0},
            {USB_EP_ATTR_ALLOW_ISO | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_INT | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 2, 0},
            {USB_EP_ATTR_ALLOW_ISO | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_INT | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 2, 0},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 2, 0}
        }
    };
    return &endpoints_table;
}

uint32_t USBPhyHw::ep0_set_max_packet(uint32_t max_packet)
{
    return USB_EP0_SIZE;
}

// read setup packet
void USBPhyHw::ep0_setup_read_result(uint8_t *buffer, uint32_t size)
{
    // Copy data
    USBMemCopy(buffer, ep0_buf_out, size);
}

void USBPhyHw::ep0_read(uint8_t *buffer, uint32_t size)
{
    // Start read. Save destination buffer for ep0 OUT token data when received.
    ep0_read_buffer = buffer;
    ep0_read_size = size;
    // Control OUT endpoint now ready to receive new data.
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;
}

uint32_t USBPhyHw::ep0_read_result()
{
    // Copy last ep0 received OUT token data
    USBMemCopy(ep0_read_buffer, ep0_buf_out, ep0_read_size);

    return usb_endpoints[0].DeviceDescBank[OUT_BANK].PCKSIZE.bit.BYTE_COUNT;
}

void USBPhyHw::ep0_write(uint8_t *buffer, uint32_t size)
{
    USBMemCopy(ep0_buf_in, buffer, size);
    usb_endpoints[0].DeviceDescBank[IN_BANK].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
    usb_endpoints[0].DeviceDescBank[IN_BANK].PCKSIZE.bit.BYTE_COUNT = size;
    // Control IN endpoint now ready to send data.
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUS_BK1RDY;
}

void USBPhyHw::ep0_stall()
{
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUS_STALLRQ(0x3);
}

bool USBPhyHw::endpoint_add(usb_ep_t endpoint, uint32_t max_packet, usb_ep_type_t type)
{
    if (endpoint & 0x80) {
        usb_endpoints[endpoint & 0x3f].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_EP_SIZE_TO_BIT(max_packet);
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPCFG.bit.EPTYPE1 = type + 1;
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUS_BK1RDY
                                                                    | USB_DEVICE_EPSTATUS_STALLRQ(0x2)
                                                                    | USB_DEVICE_EPSTATUS_DTGLIN;

        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1 | USB_DEVICE_EPINTFLAG_TRFAIL1;
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT1;
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPSTATUSSET.reg = USB_DEVICE_EPSTATUS_BK1RDY;
    } else {
        usb_endpoints[endpoint & 0x3f].DeviceDescBank[0].PCKSIZE.bit.SIZE = USB_EP_SIZE_TO_BIT(max_packet);
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPCFG.bit.EPTYPE0 = type + 1;
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPSTATUSSET.reg = USB_DEVICE_EPSTATUS_BK0RDY;
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUS_STALLRQ(0x1)
                                                                    | USB_DEVICE_EPSTATUS_DTGLOUT;
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0 | USB_DEVICE_EPINTFLAG_TRFAIL0;
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT0;
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUS_BK0RDY;
    }

    return true;
}

void USBPhyHw::endpoint_remove(usb_ep_t endpoint)
{
    if (endpoint & 0x80) {
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUS_BK1RDY;
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPCFG.bit.EPTYPE1 = USB_EPTYPE_DISABLED;
    } else {
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPSTATUSSET.reg = USB_DEVICE_EPSTATUS_BK0RDY;
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPCFG.bit.EPTYPE0 = USB_EPTYPE_DISABLED;
    }
}

void USBPhyHw::endpoint_stall(usb_ep_t endpoint)
{
    if (endpoint & 0x80) {
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3F].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
    } else {
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3F].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0;
    }
}

void USBPhyHw::endpoint_unstall(usb_ep_t endpoint)
{
    if (endpoint & 0x80) {
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3F].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ1;
    } else {
        USB->DEVICE.DeviceEndpoint[endpoint & 0x3F].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ0;
    }
}

bool USBPhyHw::endpoint_read(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    // TODO - setup data buffer to receive next endpoint OUT packet and return true if successful
    usb_endpoints[endpoint & 0x3F].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = size;
    usb_endpoints[endpoint & 0x3F].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
    usb_endpoints[endpoint & 0x3F].DeviceDescBank[0].ADDR.reg = (uint32_t) data;
    USB->DEVICE.DeviceEndpoint[endpoint & 0x3F].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0 | USB_DEVICE_EPINTFLAG_TRFAIL0;
    USB->DEVICE.DeviceEndpoint[endpoint & 0x3F].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT0;
    USB->DEVICE.DeviceEndpoint[endpoint & 0x3f].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUS_BK0RDY;
    return true;
}

uint32_t USBPhyHw::endpoint_read_result(usb_ep_t endpoint)
{
    // TODO - return the size of the last OUT packet received on endpoint
    return usb_endpoints[endpoint & 0x3F].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT;
}

bool USBPhyHw::endpoint_write(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    endpoint &= 0x3f;
    usb_endpoints[endpoint].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
    usb_endpoints[endpoint].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = size;
    usb_endpoints[endpoint].DeviceDescBank[1].ADDR.reg = (uint32_t) data;
    USB->DEVICE.DeviceEndpoint[endpoint].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1 | USB_DEVICE_EPINTFLAG_TRFAIL1;
    USB->DEVICE.DeviceEndpoint[endpoint].EPSTATUSSET.reg = USB_DEVICE_EPSTATUS_BK1RDY;

    return true;
}

void USBPhyHw::endpoint_abort(usb_ep_t endpoint)
{
    // TODO - stop the current transfer on this endpoint and don't call the IN or OUT callback
}

void USBPhyHw::process()
{
    uint32_t summary = USB->DEVICE.EPINTSMRY.reg;
    uint32_t status = USB->DEVICE.INTFLAG.reg;

    if (status & USB_DEVICE_INTFLAG_SUSPEND) {
        USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_SUSPEND;
    }
    if (status & USB_DEVICE_INTFLAG_WAKEUP) {
        USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_WAKEUP;
    }
    if (status & USB_DEVICE_INTFLAG_EORST) {
        USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;

        // Clear address
        USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | 0;
        // Configure Endpoint 0 for Control IN and Control OUT
        USB->DEVICE.DeviceEndpoint[0].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1) | USB_DEVICE_EPCFG_EPTYPE1(1);
        USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
        USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
        // Configure control OUT Packet size to 64 bytes
        usb_endpoints[0].DeviceDescBank[OUT_BANK].PCKSIZE.bit.SIZE = 3;
        // Configure control IN Packet size to 64 bytes
        usb_endpoints[0].DeviceDescBank[IN_BANK].PCKSIZE.bit.SIZE = 3;
        // Configure the data buffer address for control OUT
        usb_endpoints[0].DeviceDescBank[OUT_BANK].ADDR.reg = (uint32_t)&ep0_buf_out[0];
        // Configure the data buffer address for control IN
        usb_endpoints[0].DeviceDescBank[IN_BANK].ADDR.reg = (uint32_t)&ep0_buf_in[0];

        // Set Multipacket size to 8 for control OUT and byte count to 0
        usb_endpoints[0].DeviceDescBank[IN_BANK].PCKSIZE.bit.AUTO_ZLP = false;
        usb_endpoints[0].DeviceDescBank[OUT_BANK].PCKSIZE.bit.AUTO_ZLP = false;
        usb_endpoints[0].DeviceDescBank[OUT_BANK].PCKSIZE.bit.MULTI_PACKET_SIZE = 64;
        usb_endpoints[0].DeviceDescBank[OUT_BANK].PCKSIZE.bit.BYTE_COUNT = 0;
        USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

        USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENSET_SOF | USB_DEVICE_INTENSET_EORST;
        USB->DEVICE.DeviceEndpoint[0].EPINTENSET.reg = USB_DEVICE_EPINTENSET_RXSTP | USB_DEVICE_EPINTENSET_TRCPT1 | USB_DEVICE_EPINTFLAG_TRFAIL0;
        USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;

        events->reset();

        // Re-enable interrupt
        NVIC_ClearPendingIRQ(USB_IRQn);
        NVIC_EnableIRQ(USB_IRQn);
        return;
    }
    if (USB->DEVICE.INTFLAG.reg & USB_DEVICE_INTFLAG_SOF) {
        USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_SOF;
        // SOF event, read frame number
        //events->sof(0);
    }
    if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRFAIL0 & USB_DEVICE_EPINTFLAG_TRFAIL0) {
        USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRFAIL0;
    }
    if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT0) {
        // EP0OUT ACK event (OUT data received)
        events->ep0_out();
        USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
    }
    if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_RXSTP) {
        // Prevent any incomming OUT request while processing the SETUP request
        USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
        // EP0 SETUP event (SETUP data received)
        events->ep0_setup();
        USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP;
    }
    if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT1) {
        // EP0IN ACK event (IN data sent)
        if (mustSetAddress) {
            mustSetAddress = false;
            USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | new_address;
        }
        events->ep0_in();
        USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
    }

    for (int endpoint = 1; endpoint < usb_num_endpoints; endpoint++) {
        if (summary & 1<<endpoint) {
            uint32_t flags = USB->DEVICE.DeviceEndpoint[endpoint].EPINTFLAG.reg;
            USB->DEVICE.DeviceEndpoint[endpoint].EPINTFLAG.reg = flags;

            if (flags & USB_DEVICE_EPINTFLAG_TRCPT1) {
                events->in(0x80 | endpoint);
            } else if (flags & USB_DEVICE_EPINTFLAG_TRCPT0) {
                events->out(endpoint);
            }
        }
    }

   // Re-enable interrupt
    NVIC_ClearPendingIRQ(USB_IRQn);
    NVIC_EnableIRQ(USB_IRQn);
}

void USBPhyHw::_usbisr(void)
{
    NVIC_DisableIRQ(USB_IRQn);
    instance->events->start_process();
}
