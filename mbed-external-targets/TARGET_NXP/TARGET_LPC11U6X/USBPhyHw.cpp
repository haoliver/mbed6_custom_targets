/*
 * Copyright (c) 2018-2019, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "USBPhyHw.h"
#include "USBEndpoints_LPC11U.h"

static USBPhyHw *instance;

// Valid physical endpoint numbers are 0 to (NUMBER_OF_PHYSICAL_ENDPOINTS-1)
#define LAST_PHYSICAL_ENDPOINT (NUMBER_OF_PHYSICAL_ENDPOINTS-1)

// Convert physical endpoint number to register bit
#define EP(endpoint) (1UL<<endpoint)

// Convert physical to logical
#define PHY_TO_LOG(endpoint)    ((endpoint)>>1)

// Get physical endpoint direction
#define IN_EP(endpoint)     ((endpoint) & 1U ? true : false)
#define OUT_EP(endpoint)    ((endpoint) & 1U ? false : true)

// USB RAM
#if defined(TARGET_LPC1549)
#define USB_RAM_START ((uint32_t)usbmem)
#define USB_RAM_SIZE  sizeof(usbmem)
#else
#define USB_RAM_START (0x20004000)
#define USB_RAM_SIZE  (0x00000800)
#endif

// SYSAHBCLKCTRL
#if defined(TARGET_LPC1549)
#define CLK_USB     (1UL<<23)
#else
#define CLK_USB     (1UL<<14)
#define CLK_USBRAM  (1UL<<27)
#endif

// USB Information register
#define FRAME_NR(a)     ((a) & 0x7ff)   // Frame number

// USB Device Command/Status register
#define DEV_ADDR_MASK   (0x7f)          // Device address
#define DEV_ADDR(a)     ((a) & DEV_ADDR_MASK)
#define DEV_EN          (1UL<<7)        // Device enable
#define SETUP           (1UL<<8)        // SETUP token received
#define PLL_ON          (1UL<<9)        // PLL enabled in suspend
#define DCON            (1UL<<16)       // Device status - connect
#define DSUS            (1UL<<17)       // Device status - suspend
#define DCON_C          (1UL<<24)       // Connect change
#define DSUS_C          (1UL<<25)       // Suspend change
#define DRES_C          (1UL<<26)       // Reset change
#define VBUSDEBOUNCED   (1UL<<28)       // Vbus detected

// Endpoint Command/Status list
#define CMDSTS_A                 (1UL<<31)          // Active
#define CMDSTS_D                 (1UL<<30)          // Disable
#define CMDSTS_S                 (1UL<<29)          // Stall
#define CMDSTS_TR                (1UL<<28)          // Toggle Reset
#define CMDSTS_RF                (1UL<<27)          // Rate Feedback mode
#define CMDSTS_TV                (1UL<<27)          // Toggle Value
#define CMDSTS_T                 (1UL<<26)          // Endpoint Type
#define CMDSTS_NBYTES(n)         (((n)&0x3ff)<<16)  // Number of bytes
#define CMDSTS_ADDRESS_OFFSET(a) (((a)>>6)&0xffff)  // Buffer start address

#define BYTES_REMAINING(s)       (((s)>>16)&0x3ff)  // Bytes remaining after transfer

// USB Non-endpoint interrupt sources
#define FRAME_INT   (1UL<<30)
#define DEV_INT     (1UL<<31)

static volatile int epComplete = 0;

// One entry for a double-buffered logical endpoint in the endpoint
// command/status list. Endpoint 0 is single buffered, out[1] is used
// for the SETUP packet and in[1] is not used
typedef struct {
    uint32_t out[2];
    uint32_t in[2];
} PACKED EP_COMMAND_STATUS;

typedef struct {
    uint8_t out[MAX_PACKET_SIZE_EP0];
    uint8_t in[MAX_PACKET_SIZE_EP0];
    uint8_t setup[SETUP_PACKET_SIZE];
} PACKED CONTROL_TRANSFER;

typedef struct {
    uint32_t    maxPacket;
    uint32_t    buffer[2];
    uint32_t    options;
    uint8_t     *read_buffer;
} PACKED EP_STATE;

static volatile EP_STATE endpointState[NUMBER_OF_PHYSICAL_ENDPOINTS];

// Pointer to the endpoint command/status list
static EP_COMMAND_STATUS *ep = NULL;

// Pointer to endpoint 0 data (IN/OUT and SETUP)
static CONTROL_TRANSFER *ct = NULL;

// Shadow DEVCMDSTAT register to avoid accidentally clearing flags or
// initiating a remote wakeup event.
static volatile uint32_t devCmdStat;

// Pointers used to allocate USB RAM
static uint32_t usbRamPtr = USB_RAM_START;
static uint32_t epRamPtr = 0; // Buffers for endpoints > 0 start here

#define ROUND_UP_TO_MULTIPLE(x, m) ((((x)+((m)-1))/(m))*(m))

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

USBPhyHw::USBPhyHw(): events(NULL)
{

}

USBPhyHw::~USBPhyHw()
{

}

void USBPhyHw::init(USBPhyEvents *events)
{
    this->events = events;

    NVIC_DisableIRQ(USB_IRQn);

#if defined(TARGET_LPC1549)
    /* Set USB PLL input to system oscillator */
    LPC_SYSCON->USBPLLCLKSEL = 0x01;

    /* Setup USB PLL  (FCLKIN = 12MHz) * 4 = 48MHz
       MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
       FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
       FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
    LPC_SYSCON->USBPLLCTRL = (0x3 | (1UL << 6));

    /* Powerup USB PLL */
    LPC_SYSCON->PDRUNCFG &= ~(CLK_USB);

    /* Wait for PLL to lock */
    while(!(LPC_SYSCON->USBPLLSTAT & 0x01));

    /* enable USB main clock */
    LPC_SYSCON->USBCLKSEL = 0x02;
    LPC_SYSCON->USBCLKDIV = 1;

    /* Enable AHB clock to the USB block. */
    LPC_SYSCON->SYSAHBCLKCTRL1 |= CLK_USB;

    /* power UP USB Phy */
    LPC_SYSCON->PDRUNCFG &= ~(1UL << 9);

    /* Reset USB block */
    LPC_SYSCON->PRESETCTRL1 |= (CLK_USB);
    LPC_SYSCON->PRESETCTRL1 &= ~(CLK_USB);

#else
    #if defined(TARGET_MCU_LPC11U35) || defined(TARGET_AIRIO_BASE)
    // USB_VBUS input with pull-down
    LPC_IOCON->PIO0_3 = 0x00000009;
    #endif

    // nUSB_CONNECT output
    LPC_IOCON->PIO0_6 = 0x00000001;

    // Enable clocks (USB registers, USB RAM)
    LPC_SYSCON->SYSAHBCLKCTRL |= CLK_USB | CLK_USBRAM;

    // Ensure device disconnected (DCON not set)
    LPC_USB->DEVCMDSTAT = 0;
#endif
    // to ensure that the USB host sees the device as
    // disconnected if the target CPU is reset.
    //thread_sleep_for(300);

    // Reserve space in USB RAM for endpoint command/status list
    // Must be 256 byte aligned
    usbRamPtr = ROUND_UP_TO_MULTIPLE(usbRamPtr, 256);
    ep = (EP_COMMAND_STATUS *)usbRamPtr;
    usbRamPtr += (sizeof(EP_COMMAND_STATUS) * NUMBER_OF_LOGICAL_ENDPOINTS);
    LPC_USB->EPLISTSTART = (uint32_t)(ep) & 0xffffff00;

    // Reserve space in USB RAM for Endpoint 0
    // Must be 64 byte aligned
    usbRamPtr = ROUND_UP_TO_MULTIPLE(usbRamPtr, 64);
    ct = (CONTROL_TRANSFER *)usbRamPtr;
    usbRamPtr += sizeof(CONTROL_TRANSFER);
    LPC_USB->DATABUFSTART =(uint32_t)(ct) & 0xffc00000;

    // Setup command/status list for EP0
    ep[0].out[0] = 0;
    ep[0].in[0] =  0;
    ep[0].out[1] = CMDSTS_ADDRESS_OFFSET((uint32_t)ct->setup);

    // Route all interrupts to IRQ, some can be routed to
    // USB_FIQ if you wish.
    LPC_USB->INTROUTING = 0;

    // Set device address 0, enable USB device, no remote wakeup
    devCmdStat = DEV_ADDR(0) | DEV_EN | DSUS;
    LPC_USB->DEVCMDSTAT = devCmdStat;

    // Enable interrupts for device events and EP0
    LPC_USB->INTEN = DEV_INT | EP(EP0IN) | EP(EP0OUT) | FRAME_INT;
    instance = this;

    //attach IRQ handler and enable interrupts
    NVIC_SetVector(USB_IRQn, (uint32_t)&_usbisr);
    NVIC_EnableIRQ(USB_IRQn);
}

void USBPhyHw::deinit()
{
    // Disconnect and disable interrupt
    disconnect();
    NVIC_DisableIRQ(USB_IRQn);
}

bool USBPhyHw::powered()
{
    return true;
}

void USBPhyHw::connect()
{
    NVIC_EnableIRQ(USB_IRQn);
    devCmdStat |= DCON;
    LPC_USB->DEVCMDSTAT = devCmdStat;
}

void USBPhyHw::disconnect()
{
    NVIC_DisableIRQ(USB_IRQn);
    devCmdStat &= ~DCON;
    LPC_USB->DEVCMDSTAT = devCmdStat;
}

void USBPhyHw::configure()
{
}

void USBPhyHw::unconfigure()
{
}

void USBPhyHw::sof_enable()
{
}

void USBPhyHw::sof_disable()
{
}

void USBPhyHw::set_address(uint8_t address)
{
    devCmdStat &= ~DEV_ADDR_MASK;
    devCmdStat |= DEV_ADDR(address);
    LPC_USB->DEVCMDSTAT = devCmdStat;
}

void USBPhyHw::remote_wakeup()
{
    // Clearing DSUS bit initiates a remote wakeup if the
    // device is currently enabled and suspended - otherwise
    // it has no effect.
    LPC_USB->DEVCMDSTAT = devCmdStat & ~DSUS;
}

const usb_ep_table_t *USBPhyHw::endpoint_table()
{
    static const usb_ep_table_t lpc_table = {
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
    return &lpc_table;
}

uint32_t USBPhyHw::ep0_set_max_packet(uint32_t max_packet)
{
    return 64;
}

// read setup packet
void USBPhyHw::ep0_setup_read_result(uint8_t *buffer, uint32_t size)
{
    USBMemCopy(buffer, ct->setup, size);
}

void USBPhyHw::ep0_read(uint8_t *data, uint32_t size)
{
    // Copy data
    USBMemCopy(data, ct->out, size);
}

uint32_t USBPhyHw::ep0_read_result()
{
    // Complete an endpoint 0 read
    uint32_t bytesRead;

    // Find how many bytes were read
    bytesRead = MAX_PACKET_SIZE_EP0 - BYTES_REMAINING(ep[0].out[0]);
    return bytesRead;
}

void USBPhyHw::ep0_write(uint8_t *buffer, uint32_t size)
{
    // Copy data
    USBMemCopy(ct->in, buffer, size);

    // Start transfer
    ep[0].in[0] = CMDSTS_A | CMDSTS_NBYTES(size) | CMDSTS_ADDRESS_OFFSET((uint32_t)ct->in);
    ep[0].out[0] = CMDSTS_A;
}

void USBPhyHw::ep0_stall()
{
    ep[0].in[0] = CMDSTS_S;
    ep[0].out[0] = CMDSTS_S;
}

bool USBPhyHw::endpoint_add(usb_ep_t endpoint, uint32_t max_packet, usb_ep_type_t type)
{
    uint32_t tmpEpRamPtr;

    if (endpoint & 0x80) {
        endpoint &= ~0x80;
        endpoint = (1 << endpoint) + 1;
    } else {
        endpoint = (1 << endpoint);
    }

    //TODO FIX
    uint32_t options = SINGLE_BUFFERED;

    if (endpoint > LAST_PHYSICAL_ENDPOINT) {
        return false;
    }

    // Not applicable to the control endpoints
    if ((endpoint==EP0IN) || (endpoint==EP0OUT)) {
        return false;
    }

    // Allocate buffers in USB RAM
    tmpEpRamPtr = epRamPtr;

    // Must be 64 byte aligned
    tmpEpRamPtr = ROUND_UP_TO_MULTIPLE(tmpEpRamPtr, 64);

    if ((tmpEpRamPtr + max_packet) > (USB_RAM_START + USB_RAM_SIZE)) {
        // Out of memory
        return false;
    }

    // Allocate first buffer
    endpointState[endpoint].buffer[0] = tmpEpRamPtr;
    tmpEpRamPtr += max_packet;

    if (!(options & SINGLE_BUFFERED)) {
        // Must be 64 byte aligned
        tmpEpRamPtr = ROUND_UP_TO_MULTIPLE(tmpEpRamPtr, 64);

        if ((tmpEpRamPtr + max_packet) > (USB_RAM_START + USB_RAM_SIZE)) {
            // Out of memory
            return false;
        }

        // Allocate second buffer
        endpointState[endpoint].buffer[1] = tmpEpRamPtr;
        tmpEpRamPtr += max_packet;
    }

    // Commit to this USB RAM allocation
    epRamPtr = tmpEpRamPtr;

    // Remaining endpoint state values
    endpointState[endpoint].maxPacket = max_packet;
    endpointState[endpoint].options = options;
    endpointState[endpoint].read_buffer = NULL;

    // Enable double buffering if required
    if (options & SINGLE_BUFFERED) {
        LPC_USB->EPBUFCFG &= ~EP(endpoint);
    } else {
        // Double buffered
        LPC_USB->EPBUFCFG |= EP(endpoint);
    }

    // Enable interrupt
    LPC_USB->INTEN |= EP(endpoint);

    // Enable endpoint
    endpoint_unstall(endpoint);

    return true;
}

void USBPhyHw::endpoint_remove(usb_ep_t endpoint)
{
    // TODO - disable and remove this endpoint
}

void USBPhyHw::endpoint_stall(usb_ep_t endpoint)
{
    if (IN_EP(endpoint)) {
        ep[PHY_TO_LOG(endpoint)].in[0] |= CMDSTS_S;
        ep[PHY_TO_LOG(endpoint)].in[1] |= CMDSTS_S;
    } else {
        ep[PHY_TO_LOG(endpoint)].out[0] |= CMDSTS_S;
        ep[PHY_TO_LOG(endpoint)].out[1] |= CMDSTS_S;
    }
}

void USBPhyHw::endpoint_unstall(usb_ep_t endpoint)
{
    if (LPC_USB->EPBUFCFG & EP(endpoint)) {
        // Double buffered
        if (IN_EP(endpoint)) {
            ep[PHY_TO_LOG(endpoint)].in[0] = 0; // S = 0
            ep[PHY_TO_LOG(endpoint)].in[1] = 0; // S = 0

            if (LPC_USB->EPINUSE & EP(endpoint)) {
                ep[PHY_TO_LOG(endpoint)].in[1] = CMDSTS_TR; // S = 0, TR = 1, TV = 0
            } else {
                ep[PHY_TO_LOG(endpoint)].in[0] = CMDSTS_TR; // S = 0, TR = 1, TV = 0
            }
        } else {
            ep[PHY_TO_LOG(endpoint)].out[0] = 0; // S = 0
            ep[PHY_TO_LOG(endpoint)].out[1] = 0; // S = 0

            if (LPC_USB->EPINUSE & EP(endpoint)) {
                ep[PHY_TO_LOG(endpoint)].out[1] = CMDSTS_TR; // S = 0, TR = 1, TV = 0
            } else {
                ep[PHY_TO_LOG(endpoint)].out[0] = CMDSTS_TR; // S = 0, TR = 1, TV = 0
            }
        }
    } else {
        // Single buffered
        if (IN_EP(endpoint)) {
            ep[PHY_TO_LOG(endpoint)].in[0] = CMDSTS_TR;     // S = 0, TR = 1, TV = 0
        } else {
            ep[PHY_TO_LOG(endpoint)].out[0] = CMDSTS_TR;    // S = 0, TR = 1, TV = 0
        }
    }
}

bool USBPhyHw::endpoint_read(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    uint8_t bf = 0;
    uint32_t flags = 0;

    if (endpoint & 0x80) {
        endpoint &= ~0x80;
        endpoint = (1 << endpoint) + 1;
    } else {
        endpoint = (1 << endpoint);
    }

     uint32_t maximumSize = endpointState[endpoint].maxPacket;

    //check which buffer must be filled
    if (LPC_USB->EPBUFCFG & EP(endpoint)) {
        // Double buffered
        if (LPC_USB->EPINUSE & EP(endpoint)) {
            bf = 1;
        } else {
            bf = 0;
        }
    }

    // if isochronous endpoint, T = 1
    if(endpointState[endpoint].options & ISOCHRONOUS)
    {
        flags |= CMDSTS_T;
    }

    endpointState[endpoint].read_buffer = data;

    // Active the endpoint for reading
    ep[PHY_TO_LOG(endpoint)].out[bf] = CMDSTS_A | CMDSTS_NBYTES(maximumSize) \
                                       | CMDSTS_ADDRESS_OFFSET(endpointState[endpoint].buffer[bf]) | flags;

    return true;
}

uint32_t USBPhyHw::endpoint_read_result(usb_ep_t endpoint)
{
    uint8_t bf = 0;
    uint32_t bytesRead = 0;

    if (endpoint & 0x80) {
        endpoint &= ~0x80;
        endpoint = (1 << endpoint) + 1;
    } else {
        endpoint = (1 << endpoint);
    }

    if (!(epComplete & EP(endpoint))) {
        return bytesRead;
    } else {
        epComplete &= ~EP(endpoint);

        //check which buffer has been filled
        if (LPC_USB->EPBUFCFG & EP(endpoint)) {
            // Double buffered (here we read the previous buffer which was used)
            if (LPC_USB->EPINUSE & EP(endpoint)) {
                bf = 0;
            } else {
                bf = 1;
            }
        }

        // Find how many bytes were read
        bytesRead = (uint32_t) (endpointState[endpoint].maxPacket - BYTES_REMAINING(ep[PHY_TO_LOG(endpoint)].out[bf]));

        if (endpointState[endpoint].read_buffer && bytesRead) {
            USBMemCopy(endpointState[endpoint].read_buffer, (uint8_t *)endpointState[endpoint].buffer[bf], bytesRead);
        }

        return bytesRead;
    }   
}

bool USBPhyHw::endpoint_write(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    uint32_t flags = 0;
    uint32_t bf;

    if (endpoint & 0x80) {
        endpoint &= ~0x80;
        endpoint = (1 << endpoint) + 1;
    } else {
        endpoint = (1 << endpoint);
    }

    // Validate parameters
    if (data == NULL) {
        return false;
    }

    if (endpoint > LAST_PHYSICAL_ENDPOINT) {
        return false;
    }

    if ((endpoint==EP0IN) || (endpoint==EP0OUT)) {
        return false;
    }

    if (size > endpointState[endpoint].maxPacket) {
        return false;
    }

    if (LPC_USB->EPBUFCFG & EP(endpoint)) {
        // Double buffered
        if (LPC_USB->EPINUSE & EP(endpoint)) {
            bf = 1;
        } else {
            bf = 0;
        }
    } else {
        // Single buffered
        bf = 0;
    }

    // Check if already active
    if (ep[PHY_TO_LOG(endpoint)].in[bf] & CMDSTS_A) {
        return false;
    }

    // Check if stalled
    if (ep[PHY_TO_LOG(endpoint)].in[bf] & CMDSTS_S) {
        return false;
    }

    // Copy data to USB RAM
    USBMemCopy((uint8_t *)endpointState[endpoint].buffer[bf], data, size);

    // Add options
    if (endpointState[endpoint].options & RATE_FEEDBACK_MODE) {
        flags |= CMDSTS_RF;
    }

    if (endpointState[endpoint].options & ISOCHRONOUS) {
        flags |= CMDSTS_T;
    }

    // Add transfer
    ep[PHY_TO_LOG(endpoint)].in[bf] = CMDSTS_ADDRESS_OFFSET( \
                                      endpointState[endpoint].buffer[bf]) \
                                      | CMDSTS_NBYTES(size) | CMDSTS_A | flags;

    return true;
}

void USBPhyHw::endpoint_abort(usb_ep_t endpoint)
{
    // TODO - stop the current transfer on this endpoint and don't call the IN or OUT callback
}

EP_STATUS USBPhyHw::endpoint_write_result(uint8_t endpoint) {
    uint32_t bf;

    // Validate parameters
    if (endpoint > LAST_PHYSICAL_ENDPOINT) {
        return EP_INVALID;
    }

    if (OUT_EP(endpoint)) {
        return EP_INVALID;
    }

    if (LPC_USB->EPBUFCFG & EP(endpoint)) {
        // Double buffered     // TODO: FIX THIS
        if (LPC_USB->EPINUSE & EP(endpoint)) {
            bf = 1;
        } else {
            bf = 0;
        }
    } else {
        // Single buffered
        bf = 0;
    }

    // Check if endpoint still active
    if (ep[PHY_TO_LOG(endpoint)].in[bf] & CMDSTS_A) {
        return EP_PENDING;
    }

    // Check if stalled
    if (ep[PHY_TO_LOG(endpoint)].in[bf] & CMDSTS_S) {
        return EP_STALLED;
    }

    return EP_COMPLETED;
}

bool USBPhyHw::get_endpoint_stall_state(unsigned char endpoint) {
    if (IN_EP(endpoint)) {
        if (LPC_USB->EPINUSE & EP(endpoint)) {
            if (ep[PHY_TO_LOG(endpoint)].in[1] & CMDSTS_S) {
                return true;
            }
        } else {
            if (ep[PHY_TO_LOG(endpoint)].in[0] & CMDSTS_S) {
                return true;
            }
        }
    } else {
        if (LPC_USB->EPINUSE & EP(endpoint)) {
            if (ep[PHY_TO_LOG(endpoint)].out[1] & CMDSTS_S) {
                return true;
            }
        } else {
            if (ep[PHY_TO_LOG(endpoint)].out[0] & CMDSTS_S) {
                return true;
            }
        }
    }

    return false;
}

void USBPhyHw::endpoints_disable(void) {
    uint32_t logEp;

    // Ref. Table 158 "When a bus reset is received, software
    // must set the disable bit of all endpoints to 1".

    for (logEp = 1; logEp < NUMBER_OF_LOGICAL_ENDPOINTS; logEp++) {
        ep[logEp].out[0] = CMDSTS_D;
        ep[logEp].out[1] = CMDSTS_D;
        ep[logEp].in[0] =  CMDSTS_D;
        ep[logEp].in[1] =  CMDSTS_D;
    }

    // Start of USB RAM for endpoints > 0
    epRamPtr = usbRamPtr;
}

void USBPhyHw::_usbisr(void)
{
    NVIC_DisableIRQ(USB_IRQn);
    instance->events->start_process();
}

void USBPhyHw::process()
{
    uint32_t stat = LPC_USB->INTSTAT;

    // reset interrupt
    if (stat & DEV_INT) {

        LPC_USB->INTSTAT = DEV_INT;

        if (LPC_USB->DEVCMDSTAT & DSUS_C) {
            // Suspend status changed
            LPC_USB->DEVCMDSTAT = devCmdStat | DSUS_C;
            if (LPC_USB->DEVCMDSTAT & DSUS) {
                events->suspend(true);
            } else {
                events->suspend(false);
            }
        }

        if (LPC_USB->DEVCMDSTAT & DRES_C) {
            // Bus reset
            LPC_USB->DEVCMDSTAT = devCmdStat | DRES_C;

            // Disable endpoints > 0
            endpoints_disable();

            events->reset();
        }

        if (LPC_USB->DEVCMDSTAT & DCON_C) {
            // Bus connected
            LPC_USB->DEVCMDSTAT = devCmdStat | DCON_C;

            events->power(true);
        }
        
        // Re-enable interrupt
        NVIC_ClearPendingIRQ(USB_IRQn);
        NVIC_EnableIRQ(USB_IRQn);
        return;
    }

    // sof interrupt
    if (stat & FRAME_INT) {
        LPC_USB->INTSTAT = FRAME_INT;
        // SOF event, read frame number
        events->sof(FRAME_NR(LPC_USB->INFO));
    }

    // endpoint interrupt
    if (stat & EP(EP0IN)) {
        LPC_USB->INTSTAT = EP(EP0IN);
        // EP0IN ACK event (IN data sent)
        events->ep0_in();
    }

    if (stat & EP(EP0OUT)) {
        // Check if SETUP
        LPC_USB->INTSTAT = EP(EP0OUT);

        if (LPC_USB->DEVCMDSTAT & SETUP) {
            // Disable control endpoint while processing setup stage
            ep[0].in[0] = 0;
            ep[0].out[0] = 0;

            // Clear EP0IN interrupt
            LPC_USB->INTSTAT = EP(EP0IN);

            // Clear SETUP (and INTONNAK_CI/O) in device status register
            LPC_USB->DEVCMDSTAT = devCmdStat | SETUP;

            // EP0 SETUP event (SETUP data received)
            events->ep0_setup();
        } else {
            // EP0OUT ACK event (OUT data received)
            events->ep0_out();
        }
    }

    for (uint8_t endpoint = 2; endpoint < NUMBER_OF_PHYSICAL_ENDPOINTS; endpoint++) {
        if (LPC_USB->INTSTAT & EP(endpoint)) {
            LPC_USB->INTSTAT = EP(endpoint);
            epComplete |= EP(endpoint);
            if (endpoint & 1U) {
                events->in(0x80 | PHY_TO_LOG(endpoint));
            } else {
                events->out(PHY_TO_LOG(endpoint));
            }
        }
    }

    // Re-enable interrupt
    NVIC_ClearPendingIRQ(USB_IRQn);
    NVIC_EnableIRQ(USB_IRQn);
}