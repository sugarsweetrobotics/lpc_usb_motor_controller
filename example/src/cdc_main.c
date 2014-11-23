/*
 * @brief Motor Controller with Vitual communication port
 *
 * @note
 * Copyright(C) SUGAR SWEET ROBOTICS CO., LTD., 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include <stdio.h>
#include <string.h>
#include "app_usbd_cfg.h"
#include "cdc_vcom.h"
#include "pwm_13xx.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

static USBD_HANDLE_T g_hUsb;
static uint32_t g_buffCounter;
static uint8_t g_rxBuff[256];
const USBD_API_T *g_pUsbApi;

/* For packet handling */
enum __read_mode {
	WAITING_HEADER, WAITING_FOOTER,
};
const char PACKET_HEADER = '$';
const char PACKET_FOOTER = '#';

/* For ADC */
#define _LPC_ADC_ID LPC_ADC
#define _LPC_ADC_IRQ ADC_IRQn
static ADC_CLOCK_SETUP_T ADCSetup;
#define ADC_IRQ_HANDLER  ADC_IRQHandler	/* GPIO interrupt IRQ function name */
#define ADC_NVIC_NAME    ADC_IRQn	/* GPIO interrupt NVIC interrupt name */

/* For PWM */
static int handle0, handle1;

/* For Control */
static int32_t g_target = 4096 / 2;
static int32_t g_Kp = 50;
static int32_t g_Ti = 20;
static int32_t g_sumEpsilon = 0;
static int32_t g_epsilon_old = 0;
static int32_t g_Td = 10;
static uint16_t g_dataADC;
#define CONTROL_INTERVAL   (10) // [ms]


/*****************************************************************************
 * Private functions
 ****************************************************************************/
/**
 * Control Procedure called periodically
 */
static inline void onControl() {
	int32_t epsilon = g_target - g_dataADC;
	g_sumEpsilon += epsilon;
	int32_t deltaEpsilon = (epsilon - g_epsilon_old) / 10;
	int32_t out = g_Kp * (epsilon + g_sumEpsilon / 100 / g_Ti + deltaEpsilon / g_Td);
	uint16_t ch0, ch1;
	if (out < 0) {
		ch0 = -out;
		ch1 = 0;
		if(ch0 > 65000) ch0 = 65000;
	} else {
		ch0 = 0;
		ch1 = out;
		if(ch1 > 65000) ch1 = 65000;
	}

	Chip_PWM_SetDuty(handle0, ch0);
	Chip_PWM_SetDuty(handle1, ch1);

	g_epsilon_old = epsilon;
}

/**
 * Conversion Utility (String -> INT16)
 */
static inline uint16_t str2uint16(char* str, int len) {
	uint16_t data = 0;
	int i, j;
	for (i = 0; i < len; i++) {
		uint16_t buf = str[i] - 48;
		for (j = 0; j < (len - 1) - i; j++) {
			buf *= 10;
		}
		data += buf;
	}
	return data;
}

/**
 * Called When Valid Packet is received.
 */
static inline void onReceivePacket() {
	/**
	 * Global Variables
	 * g_rxBuff : Packet Data (not include header and footer)
	 * g_buffCounter : Length of Packet data (byte)
	 * g_dataADC : result of ADC
	 * g_target  : target valueof control
	 *
	 */
	if (strncmp(g_rxBuff, "ping", g_buffCounter) == 0) {
		vcom_write("$ping#", 4 + 2);
	} else if (strncmp(g_rxBuff, "get", g_buffCounter) == 0) {
		uint16_t data = g_dataADC;
		char out_str[6];
		out_str[0] = '$';
		int j;
		for (j = 4; j != 0; j--) {
			out_str[j] = (data % 10) + 48;
			data /= 10;
		}
		out_str[5] = '#';
		vcom_write((unsigned char*) out_str, 6);
	} else if (strncmp((char*) g_rxBuff, "set", 3) == 0) {
		if (g_buffCounter != (3 + 4)) {
			vcom_write("$error#", 5 + 2);
		} else {
			uint16_t data = str2uint16(g_rxBuff + 3, 4);
			g_target = data;
			vcom_write("$set#", 5);
		}
	} else if (strncmp((char*) g_rxBuff, "kp", 2) == 0) {
		if (g_buffCounter != (2 + 4)) {
			vcom_write("$error#", 5 + 2);
		} else {
			uint16_t data = str2uint16(g_rxBuff + 2, 4);
			g_Kp = data;
			vcom_write("$kp#", 6);
		}
	}else if (strncmp((char*) g_rxBuff, "ti", 2) == 0) {
		if (g_buffCounter != (2 + 4)) {
			vcom_write("$error#", 5 + 2);
		} else {
			uint16_t data = str2uint16(g_rxBuff + 2, 4);
			g_Ti = data;
			vcom_write("$ti#", 6);
		}
	}
	else if (strncmp((char*) g_rxBuff, "td", 2) == 0) {
			if (g_buffCounter != (2 + 4)) {
				vcom_write("$error#", 5 + 2);
			} else {
				uint16_t data = str2uint16(g_rxBuff + 2, 4);
				g_Td = data;
				vcom_write("$td#", 6);
			}
	}
	else {
		vcom_write("$error#", 5 + 2);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	Handle interrupt from USB0
 * @return	Nothing
 */
void USB_IRQHandler(void) {
	uint32_t *addr = (uint32_t *) LPC_USB->EPLISTSTART;

	/*	WORKAROUND for artf32289 ROM driver BUG:
	 As part of USB specification the device should respond
	 with STALL condition for any unsupported setup packet. The host will send
	 new setup packet/request on seeing STALL condition for EP0 instead of sending
	 a clear STALL request. Current driver in ROM doesn't clear the STALL
	 condition on new setup packet which should be fixed.
	 */
	if ( LPC_USB->DEVCMDSTAT & _BIT(8)) { /* if setup packet is received */
		addr[0] &= ~(_BIT(29)); /* clear EP0_OUT stall */
		addr[2] &= ~(_BIT(29)); /* clear EP0_IN stall */
	}
	USBD_API->hw->ISR(g_hUsb);
}

/* Find the address of interface descriptor for given class type. */
USB_INTERFACE_DESCRIPTOR *find_IntfDesc(const uint8_t *pDesc,
		uint32_t intfClass) {
	USB_COMMON_DESCRIPTOR *pD;
	USB_INTERFACE_DESCRIPTOR *pIntfDesc = 0;
	uint32_t next_desc_adr;

	pD = (USB_COMMON_DESCRIPTOR *) pDesc;
	next_desc_adr = (uint32_t) pDesc;

	while (pD->bLength) {
		/* is it interface descriptor */
		if (pD->bDescriptorType == USB_INTERFACE_DESCRIPTOR_TYPE) {

			pIntfDesc = (USB_INTERFACE_DESCRIPTOR *) pD;
			/* did we find the right interface descriptor */
			if (pIntfDesc->bInterfaceClass == intfClass) {
				break;
			}
		}
		pIntfDesc = 0;
		next_desc_adr = (uint32_t) pD + pD->bLength;
		pD = (USB_COMMON_DESCRIPTOR *) next_desc_adr;
	}

	return pIntfDesc;
}

/**
 * Interrupt Handler called when ADC is finished.
 */
void ADC_IRQ_HANDLER(void) {
	/**
	 * g_dataADC : result of ADC
	 */
	// Read ADC Channel 0
	Chip_ADC_ReadValue(_LPC_ADC_ID, ADC_CH0, &g_dataADC);
}

/**
 * Interrupt Handler called when RITIMER tick
 */
void RIT_IRQHandler(void) {
	Chip_RIT_ClearInt(LPC_RITIMER);
	onControl();
}

/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void) {
	USBD_API_INIT_PARAM_T usb_param;
	USB_CORE_DESCS_T desc;
	ErrorCode_t ret = LPC_OK;
	uint32_t prompt = 0;

	SystemCoreClockUpdate();
	/* Initialize board and chip */
	Board_Init();
	Board_ADC_Init();

	/* Initialize PWM Units */
	handle0 = Chip_PWM_Init(0, 18, 100);
	handle1 = Chip_PWM_Init(0, 13, 100);

	/* enable clocks and pinmux */
	Chip_USB_Init();

	/* initialize USBD ROM API pointer. */
	g_pUsbApi = (const USBD_API_T *) LPC_ROM_API->usbdApiBase;

	/* initialize call back structures */
	memset((void *) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
	usb_param.usb_reg_base = LPC_USB0_BASE;
	/*	WORKAROUND for artf44835 ROM driver BUG:
	 Code clearing STALL bits in endpoint reset routine corrupts memory area
	 next to the endpoint control data. For example When EP0, EP1_IN, EP1_OUT,
	 EP2_IN are used we need to specify 3 here. But as a workaround for this
	 issue specify 4. So that extra EPs control structure acts as padding buffer
	 to avoid data corruption. Corruption of padding memory doesn’t affect the
	 stack/program behaviour.
	 */
	usb_param.max_num_ep = 3 + 1;
	usb_param.mem_base = USB_STACK_MEM_BASE;
	usb_param.mem_size = USB_STACK_MEM_SIZE;

	/* Set the USB descriptors */
	desc.device_desc = (uint8_t *) &USB_DeviceDescriptor[0];
	desc.string_desc = (uint8_t *) &USB_StringDescriptor[0];
	/* Note, to pass USBCV test full-speed only devices should have both
	 descriptor arrays point to same location and device_qualifier set to 0.
	 */
	desc.high_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
	desc.full_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
	desc.device_qualifier = 0;

	/* USB Initialization */
	ret = USBD_API->hw->Init(&g_hUsb, &desc, &usb_param);
	if (ret == LPC_OK) {

		/*	WORKAROUND for artf32219 ROM driver BUG:
		 The mem_base parameter part of USB_param structure returned
		 by Init() routine is not accurate causing memory allocation issues for
		 further components.
		 */
		usb_param.mem_base = USB_STACK_MEM_BASE
				+ (USB_STACK_MEM_SIZE - usb_param.mem_size);

		/*
		 Initialize ADC
		 */
		Chip_ADC_Init(_LPC_ADC_ID, &ADCSetup);
		Chip_ADC_EnableChannel(_LPC_ADC_ID, ADC_CH0, ENABLE);
		Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, ADC_CH0, ENABLE);
		NVIC_SetPriority(_LPC_ADC_IRQ, 1);
		NVIC_EnableIRQ(_LPC_ADC_IRQ);

		/* Init VCOM interface */
		ret = vcom_init(g_hUsb, &desc, &usb_param);
		if (ret == LPC_OK) {
			/*  enable USB interrupts */
			NVIC_SetPriority(USB0_IRQn, 1);
			NVIC_EnableIRQ(USB0_IRQn);
			/* now connect */
			USBD_API->hw->Connect(g_hUsb, 1);
		}

	}

	DEBUGSTR("USB CDC class based virtual Comm port example!\r\n");

	/* Start BURST Mode (Continuously Convert and Interrupt) */
	Chip_ADC_SetBurstCmd(_LPC_ADC_ID, ENABLE);
	Chip_RIT_Init(LPC_RITIMER);
	Chip_RIT_SetTimerInterval(LPC_RITIMER, CONTROL_INTERVAL);

	NVIC_EnableIRQ(RIT_IRQn);

	int read_mode = WAITING_HEADER;

	while (1) {

		/* Check if host has connected and opened the VCOM port */

		if ((vcom_connected() != 0) && (prompt == 0)) {
			//vcom_write("Hello World!!\r\n", 15);
			prompt = 1;
		}

		if (prompt) {

			unsigned char c;
			if (vcom_bread(&c, 1) != 0) {
				switch (read_mode) {
				case WAITING_HEADER:
					if (c == PACKET_HEADER) {
						g_buffCounter = 0;
						read_mode = WAITING_FOOTER;
					}
					break;
				case WAITING_FOOTER:
					if (c == PACKET_FOOTER) {
						onReceivePacket();
						read_mode = WAITING_HEADER;
					} else {
						g_rxBuff[g_buffCounter] = c;
						g_buffCounter++;
					}
					break;
				default:
					break;
				}
			}
		}
		/* Sleep until next IRQ happens */
		//__WFI();

	}
}
