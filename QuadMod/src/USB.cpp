#include "USB.h"

USB usb;
/*
extern "C" {
// To enable USB for printf commands (To print floats enable 'Use float with printf from newlib-nano' MCU Build Settings)
size_t _write(int handle, const unsigned char* buf, size_t bufSize)
{
	if (usb.devState == USB::DeviceState::Configured) {
		return usb.SendString(buf, bufSize);
	} else {
		return 0;
	}
}
}
*/
inline void ClearRxInterrupt(uint8_t ep)
{
	uint16_t wRegVal = (USB_EPR[ep].EPR & USB_CHEP_REG_MASK) & ~USB_EP_VTRX;
	USB_EPR[ep].EPR = wRegVal | USB_EP_VTTX;
}


inline void ClearTxInterrupt(uint8_t ep)
{
	uint16_t wRegVal = (USB_EPR[ep].EPR & USB_CHEP_REG_MASK) & ~USB_EP_VTTX;
	USB_EPR[ep].EPR = wRegVal | USB_EP_VTRX;
}


inline void SetTxStatus(uint8_t ep, uint16_t status)		// Set endpoint transmit status - have to use XOR to toggle bits
{
	uint16_t wRegVal = (USB_EPR[ep].EPR & USB_CHEP_TX_DTOGMASK) ^ status;
	USB_EPR[ep].EPR = wRegVal | USB_EP_VTRX | USB_EP_VTTX;
}


inline void SetRxStatus(uint8_t ep, uint16_t status)		// Set endpoint receive status - have to use XOR to toggle bits
{
	uint16_t wRegVal = (USB_EPR[ep].EPR & USB_CHEP_RX_DTOGMASK) ^ status;
	USB_EPR[ep].EPR = wRegVal | USB_EP_VTRX | USB_EP_VTTX;
}


void USB::ReadPMA(uint32_t pma, USBHandler* handler)
{
	volatile uint32_t* pmaBuff = reinterpret_cast<volatile uint32_t*>(USB_DRD_PMAADDR + pma);		// Eg 0x40006018

	for (uint32_t i = 0; i < (handler->outBuffCount + 3) / 4; i++) {
		reinterpret_cast<volatile uint32_t*>(handler->outBuff)[i] = *pmaBuff++;				// pma buffer can only be read in 16 bit words
	}

#if (USB_DEBUG)
	usbDebug[usbDebugNo].PacketSize = handler->outBuffCount;
	usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)handler->outBuff)[0];
	usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)handler->outBuff)[1];
#endif
}


void USB::WritePMA(uint32_t pma, uint32_t bytes, USBHandler* handler)
{
	volatile uint32_t* pmaBuff = reinterpret_cast<volatile uint32_t*>(USB_DRD_PMAADDR + pma);

	for (int i = 0; i < (bytes + 3) / 4; i++) {
		pmaBuff[i] = reinterpret_cast<const uint32_t*>(handler->inBuff)[i];
	}
}


void USB::ProcessSetupPacket()
{
	req.loadData((uint8_t*)classByEP[0]->outBuff);		// Parse the setup request into the req object

#if (USB_DEBUG)
	usbDebug[usbDebugNo].Request = req;
#endif
	// Previously USBD_StdDevReq
	if ((req.RequestType & recipientMask) == RequestRecipientDevice && (req.RequestType & requestTypeMask) == RequestTypeStandard) {
		switch (static_cast<Request>(req.Request)) {
		case Request::GetDescriptor:
			GetDescriptor();
			break;

		case Request::SetAddress:
			devAddress = static_cast<uint8_t>(req.Value) & 0x7F;			// Address address is set on the next interrupt - hold in temp storage

			EPStartXfer(Direction::in, 0, 0);
			devState = DeviceState::Addressed;
			break;

		case Request::SetConfiguration:
			if (devState == DeviceState::Addressed) {
				devState = DeviceState::Configured;

				for (auto c : classByEP) {
					c->ActivateEP();
				}

				EPStartXfer(Direction::in, 0, 0);
			}
			break;

		default:
			SetTxStatus(0, USB_EP_TX_STALL);
			break;
		}

	// Previously USBD_StdItfReq
	} else if ((req.RequestType & recipientMask) == RequestRecipientInterface && (req.RequestType & requestTypeMask) == RequestTypeClass) {

		// req.Index holds interface - call the appropriate handler's setup
		if (req.Length > 0) {
			classesByInterface[req.Index]->ClassSetup(req);
		} else {
			EPStartXfer(Direction::in, 0, 0);
		}

	} else {
		SetTxStatus(0, USB_EP_TX_STALL);
	}
}


// EPStartXfer setup and starts a transfer over an EP
void USB::EPStartXfer(const Direction direction, uint8_t endpoint, uint32_t len)
{
	uint8_t epIndex = (endpoint & epAddrMask);

	if (direction == Direction::in) {						// IN endpoint
		if (len > ep_maxPacket) {
			len = ep_maxPacket;
		}

		WritePMA(USB_PMA[epIndex].GetTXAddr(), len, classByEP[epIndex]);
		USB_PMA[epIndex].SetTXCount(len);

#if (USB_DEBUG)
				usbDebug[usbDebugNo].PacketSize = len;
				if (len > 0) {
					usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)classByEP[epIndex]->inBuff)[0];
					usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)classByEP[epIndex]->inBuff)[1];
				}
#endif

		SetTxStatus(epIndex, USB_EP_TX_VALID);
	} else {												// OUT endpoint
		SetRxStatus(0, USB_EP_RX_VALID);
	}
}


void USB::USBInterruptHandler()						// Originally in Drivers\STM32F4xx_HAL_Driver\Src\stm32f4xx_hal_pcd.c
{
	// Handle spurious interrupt
	USBP->ISTR &= ~(USB_ISTR_SOF | USB_ISTR_ESOF);
	if ((USBP->ISTR) == 0) {
		return;
	}


	/////////// 	8000 		USB_ISTR_CTR: Correct Transfer
	while (ReadInterrupts(USB_ISTR_CTR)) {					// Originally PCD_EP_ISR_Handler
		uint8_t epIndex = USBP->ISTR & USB_ISTR_IDN;		// Extract highest priority endpoint number

#if (USB_DEBUG)
		usbDebug[usbDebugNo].endpoint = epIndex;
#endif

		if (epIndex == 0) {
			if ((USBP->ISTR & USB_ISTR_DIR) == 0) {			// DIR = 0: Direction IN
				ClearTxInterrupt(0);

				uint16_t txBytes = USB_PMA[0].GetTXCount();
				classByEP[epIndex]->inBuff += txBytes;

				if (classByEP[epIndex]->inBuffRem > ep_maxPacket) {
					classByEP[epIndex]->inBuffRem -= ep_maxPacket;
					EPStartXfer(Direction::in, 0, classByEP[epIndex]->inBuffRem);
					EPStartXfer(Direction::out, 0, 0);
				} else {
					// FIXME if (rem_length ==  maxpacket) etc - where non zero size packet and last packet is a multiple of max packet size
					SetTxStatus(0, USB_EP_TX_STALL);
					EPStartXfer(Direction::out, 0, 0);
				}

				if (devAddress > 0 && txBytes == 0) {
					USBP->DADDR = (devAddress | USB_DADDR_EF);
					devAddress = 0;
				}

			} else {										// DIR = 1: Setup or OUT interrupt

				if ((USBP->CHEP0R & USB_EP_SETUP) != 0) {
					classByEP[0]->outBuffCount = USB_PMA[0].GetRXCount();
					ReadPMA(USB_PMA[0].GetRXAddr(), classByEP[0]);	// Read setup data into  receive buffer
					ClearRxInterrupt(0);						// clears 8000 interrupt
					ProcessSetupPacket();						// Parse setup packet into request, locate data (eg descriptor) and populate TX buffer

				} else {
					ClearRxInterrupt(0);
					classByEP[0]->outBuffCount = USB_PMA[0].GetRXCount();
					if (classByEP[0]->outBuffCount != 0) {
						ReadPMA(USB_PMA[0].GetRXAddr(), classByEP[0]);

						if (devState == DeviceState::Configured && classPendingData) {
							if ((req.RequestType & requestTypeMask) == RequestTypeClass) {
								// Previous OUT interrupt contains instruction (eg host sending CDC LineCoding); next command sends data (Eg LineCoding data)
								classesByInterface[req.Index]->ClassSetupData(req, (uint8_t*)ep0.outBuff);
							}
							classPendingData = false;
							EPStartXfer(Direction::in, 0, 0);
						}
					}
					SetRxStatus(0, USB_EP_RX_VALID);
				}
			}

		} else {
			// Non zero endpoint
			if ((USB_EPR[epIndex].EPR & USB_EP_VTRX) != 0) {
				ClearRxInterrupt(epIndex);
				
				classByEP[epIndex]->outBuffCount = USB_PMA[epIndex].GetRXCount();
				if (classByEP[epIndex]->outBuffCount != 0) {
					ReadPMA(USB_PMA[epIndex].GetRXAddr(), classByEP[epIndex]);
				}
				SetRxStatus(epIndex, USB_EP_RX_VALID);


				classByEP[epIndex]->DataOut();
			}

			if ((USB_EPR[epIndex].EPR & USB_EP_VTTX) != 0) {
				transmitting = false;
				ClearTxInterrupt(epIndex);

				uint16_t txBytes = USB_PMA[epIndex].GetTXCount();
				if (classByEP[epIndex]->inBuffSize >= txBytes) {					// Transmitting data larger than buffer size
					classByEP[epIndex]->inBuffSize -= txBytes;
					classByEP[epIndex]->inBuff += txBytes;
					EPStartXfer(Direction::in, epIndex, classByEP[epIndex]->inBuffSize);
				}
			}

		}
	}


	/////////// 	1000 		USB_ISTR_WKUP: Wake Up
	if (ReadInterrupts(USB_ISTR_WKUP)) {
		USBP->CNTR &= ~USB_CNTR_SUSPEN;
		//USBP->CNTR &= ~USB_CNTR_LPMODE;		// FIXME
		USBP->ISTR &= ~USB_ISTR_WKUP;
	}

	/////////// 	800 		SUSP: Suspend Interrupt
	if (ReadInterrupts(USB_ISTR_SUSP)) {
		USBP->CNTR |= USB_CNTR_SUSPEN;
		USBP->ISTR &= ~USB_ISTR_SUSP;
		//USBP->CNTR |= USB_CNTR_LPMODE;		// FIXME
		devState = DeviceState::Suspended;
	}

	/////////// 	400 		RESET: Reset Interrupt
	if (ReadInterrupts(USB_ISTR_RESET))	{
		USBP->ISTR &= ~USB_ISTR_RESET;

		pmaAddress = pmaStartAddr;						// Reset PMA allocation start address
		ActivateEndpoint(0, Direction::out, Control);
		ActivateEndpoint(0, Direction::in,  Control);

		USBP->DADDR = USB_DADDR_EF;						// Enable endpoint and set address to 0
	}

	/////////// 	100 		USB_ISTR_ESOF: Expected Start of frame
	if (ReadInterrupts(USB_ISTR_ESOF)) {
		USBP->ISTR &= ~USB_ISTR_ESOF;
	}

	/////////// 	2000 		ERR: Error Interrupt
	if (ReadInterrupts(USB_ISTR_ERR)) {
		USBP->ISTR &= ~USB_ISTR_ERR;
	}
}


void USB::InitUSB()
{
	RCC->APB2ENR |= RCC_APB2ENR_USBEN;					// USB2OTG (OTG_HS2) Peripheral Clocks Enable
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;				// Enable GPIO Clock

	RCC->CR |= RCC_CR_HSI48ON;							// Enable USB clock
	while ((RCC->CR & RCC_CR_HSI48RDY) == 0);			// Wait till ready

	RCC->CCIPR4 |= RCC_CCIPR4_USBSEL;					// USB clock source: 00: no clock; 01: pll1_q_ck; 10: pll3_q_ck; *11: hsi48_ker_ck

	// MODER 00: Input mode, 01: General purpose output mode, 10: Alternate function mode, 11: Analog mode (reset state)
	GPIOA->MODER &= ~(GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0);
	GPIOA->AFR[1] |= (10 << GPIO_AFRH_AFSEL11_Pos) | (10 << GPIO_AFRH_AFSEL12_Pos);

	NVIC_SetPriority(USB_DRD_FS_IRQn, 3);
	NVIC_EnableIRQ(USB_DRD_FS_IRQn);

	USB_DRD_FS->CNTR &= ~USB_CNTR_HOST;					// Set mode to device
	USB_DRD_FS->CNTR |= USB_CNTR_USBRST;				// Force USB Reset
	USB_DRD_FS->CNTR &= ~USB_CNTR_USBRST;				// Release reset
	USB_DRD_FS->ISTR = 0;								// Clear pending interrupts
	USB_DRD_FS->CNTR = USB_CNTR_CTRM  | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_ERRM | USB_CNTR_RESETM | USB_CNTR_L1REQM;

	USB_DRD_FS->BCDR |= USB_BCDR_DPPU;					// Connect internal PU resistor on USB DP line
}


void USB::ActivateEndpoint(uint8_t endpoint, Direction direction, EndPointType eptype)
{
	endpoint = endpoint & 0xF;
	uint16_t ep_type = 0;
	switch (eptype) {
		case Control:		ep_type = USB_EP_CONTROL;		break;
		case Isochronous:	ep_type = USB_EP_ISOCHRONOUS;	break;
		case Bulk:			ep_type = USB_EP_BULK;			break;
		case Interrupt:		ep_type = USB_EP_INTERRUPT;		break;
	}

	// Set the address (EA=endpoint) and type (EP_TYPE=ep_type)
	USB_EPR[endpoint].EPR = (USB_EPR[endpoint].EPR & USB_EP_T_MASK) | (endpoint | ep_type | USB_EP_VTRX | USB_EP_VTTX);

	if (direction == Direction::in) {
		USB_PMA[endpoint].SetTXAddr(pmaAddress);			// Offset of PMA used for EP TX

		// Clear tx data toggle (data packets must alternate 1 and 0 in the data field)
		if ((USB_EPR[endpoint].EPR & USB_EP_DTOG_TX) != 0) {
			uint16_t wEPVal = USB_EPR[endpoint].EPR & USB_CHEP_REG_MASK;
			USB_EPR[endpoint].EPR = wEPVal | USB_EP_VTRX | USB_EP_VTTX | USB_EP_DTOG_TX;
		}

		SetTxStatus(endpoint, USB_EP_TX_NAK);

	} else {
		USB_PMA[endpoint].SetRXAddr(pmaAddress);			// Offset of PMA used for EP RX
		USB_PMA[endpoint].SetRXBlkSize(1);					// configure block size = 1 (32 Bytes)
		USB_PMA[endpoint].SetRXBlocks(1);					// number of blocks = 2 (64 bytes)

		// Clear rx data toggle
		if ((USB_EPR[endpoint].EPR & USB_EP_DTOG_RX) != 0) {
			uint16_t wEPVal = USB_EPR[endpoint].EPR & USB_CHEP_REG_MASK;
			USB_EPR[endpoint].EPR = wEPVal | USB_EP_VTRX | USB_EP_VTTX | USB_EP_DTOG_RX;
		}

		SetRxStatus(endpoint, USB_EP_RX_VALID);
	}

	// Increment PMA address in 64 byte chunks
	pmaAddress += ep_maxPacket;
}


// procedure to allow classes to pass configuration data back via endpoint 0 (eg CDC line setup, MSC MaxLUN etc)
void USB::EP0In(const uint8_t* buff, const uint32_t size)
{
	ep0.inBuff = buff;
	ep0.inBuffRem = size;
	ep0.inBuffSize = std::min(size, static_cast<uint32_t>(req.Length));
	EPStartXfer(Direction::in, 0, ep0.inBuffSize);		// sends blank request back

#if (USB_DEBUG)
	usbDebug[usbDebugNo].PacketSize = ep0.inBuffSize;
	usbDebug[usbDebugNo].xferBuff0 = (uint32_t)ep0.inBuff;
#endif
}


void USB::GetDescriptor()
{
	uint32_t strSize;

	switch (static_cast<Descriptor>(req.Value >> 8))	{
	case DeviceDescriptor:
		return EP0In(USBD_FS_DeviceDesc, sizeof(USBD_FS_DeviceDesc));
		break;

	case ConfigurationDescriptor:
		return EP0In(configDescriptor, MakeConfigDescriptor());		// Construct config descriptor from individual classes
		break;

	case BosDescriptor:
		return EP0In(USBD_FS_BOSDesc, sizeof(USBD_FS_BOSDesc));
		break;

	case StringDescriptor:

		switch ((uint8_t)(req.Value)) {
		case StringIndex::LangId:				// 300
			return EP0In(USBD_LangIDDesc, sizeof(USBD_LangIDDesc));
			break;

		case StringIndex::Manufacturer:			// 301
			strSize = StringToUnicode(manufacturerString, stringDescr);
			return EP0In(stringDescr, strSize);
			break;

		case StringIndex::Product:				// 302
			strSize = StringToUnicode(productString, stringDescr);
			return EP0In(stringDescr, strSize);
			break;

		case StringIndex::Serial:				// 303
			SerialToUnicode();
			return EP0In(stringDescr, stringDescr[0]);				// length is 24 bytes (x2 for unicode padding) + 2 for header
			break;

		case StringIndex::AudioClass:			// 307
			strSize = StringToUnicode(midiString, stringDescr);
			return EP0In(stringDescr, strSize);
			break;

	    case StringIndex::CommunicationClass:	// 306
	    	strSize = StringToUnicode(cdcString, stringDescr);
	    	return EP0In(stringDescr, strSize);
	    	break;

		default:
			SetTxStatus(0, USB_EP_TX_STALL);
			return;
		}
		break;

	default:
		SetTxStatus(0, USB_EP_TX_STALL);
		return;
	}

	if (req.Length == 0) {
		EPStartXfer(Direction::in, 0, 0);
	}
}


uint32_t USB::MakeConfigDescriptor()
{
	// Construct the configuration descriptor from the various class descriptors with header
	static constexpr uint8_t descrHeaderSize = 9;
	uint32_t descPos = descrHeaderSize;
	for (auto c : classByEP) {
		if (c != nullptr) {
			const uint8_t* descBuff = nullptr;
			uint32_t descSize = c->GetInterfaceDescriptor(&descBuff);
			memcpy(&configDescriptor[descPos], descBuff, descSize);
			descPos += descSize;
		}
	}

	// Insert config descriptor header
	const uint8_t descriptorHeader[] = {
		0x09,								// bLength: Configuration Descriptor size
		ConfigurationDescriptor,			// bDescriptorType: Configuration
		LOBYTE(descPos),					// wTotalLength
		HIBYTE(descPos),
		interfaceCount,						// bNumInterfaces: 4 [2 CDC, 2 MIDI]
		0x01,								// bConfigurationValue: Configuration value
		0x04,								// iConfiguration: Index of string descriptor describing the configuration
		0xC0,								// bmAttributes: self powered
		0x32,								// MaxPower 0 mA
	};
	memcpy(&configDescriptor[0], descriptorHeader, descrHeaderSize);

	return descPos;
}


uint32_t USB::StringToUnicode(const std::string_view desc, uint8_t *unicode)
{
	uint32_t idx = 2;
	for (auto c: desc) {
		unicode[idx++] = c;
		unicode[idx++] = 0;
	}
	unicode[0] = idx;
	unicode[1] = StringDescriptor;

	return idx;
}



void USB::SerialToUnicode()
{
	// FIXME - accessing UID_BASE data hard faults
	char uidBuff[usbSerialNoSize + 1] = "Mountjoy_Quango_12345678";

	//const uint32_t* uidAddr = (uint32_t*)UID_BASE;			// Location in memory that holds 96 bit Unique device ID register
	//snprintf(uidBuff, usbSerialNoSize + 1, "%08lx%08lx%08lx", uidAddr[0], uidAddr[1], uidAddr[2]);

	stringDescr[0] = usbSerialNoSize * 2 + 2;				// length is 24 bytes (x2 for unicode padding) + 2 for header
	stringDescr[1] = StringDescriptor;
	for (uint8_t i = 0; i < usbSerialNoSize; ++i) {
		stringDescr[i * 2 + 2] = uidBuff[i];
	}
}


bool USB::ReadInterrupts(uint32_t interrupt)
{
#if (USB_DEBUG)
	if ((USBP->ISTR & interrupt) == interrupt && usbDebugEvent < USB_DEBUG_COUNT) {
		usbDebugNo = usbDebugEvent % USB_DEBUG_COUNT;
		usbDebug[usbDebugNo].eventNo = usbDebugEvent;
		usbDebug[usbDebugNo].Interrupt = USBP->ISTR;
		usbDebugEvent++;
	}
#endif

	return (USBP->ISTR & interrupt) == interrupt;
}


size_t USB::SendData(const uint8_t* data, uint16_t len, uint8_t endpoint)
{
	if (devState == DeviceState::Configured && !transmitting) {
		transmitting = true;
		classByEP[endpoint & epAddrMask]->inBuff = (uint8_t*)data;
		classByEP[endpoint & epAddrMask]->inBuffSize = len;
		EPStartXfer(Direction::in, endpoint, len);
		return len;
	} else {
		return 0;
	}
}


void USB::SendString(const char* s)
{
	uint16_t counter = 0;
	while (transmitting && counter < 10000) {
		++counter;
	}
	SendData((uint8_t*)s, strlen(s), CDC_In);
}


void USB::SendString(std::string s)
{
	SendString(s.c_str());
}


size_t USB::SendString(const unsigned char* s, size_t len)
{
	uint16_t counter = 0;
	while (transmitting && counter < 10000) {
		++counter;
	}
	return SendData((uint8_t*)s, len, CDC_In);
}

#if (USB_DEBUG)

std::string IntToString(const int32_t& v) {
	return std::to_string(v);
}

std::string HexToString(const uint32_t& v, const bool& spaces) {
	char buf[20];
	if (spaces) {
		if (v != 0) {
			uint8_t* bytes = (uint8_t*)&v;
			sprintf(buf, "%02X%02X%02X%02X", bytes[0], bytes[1], bytes[2], bytes[3]);
		} else {
			sprintf(buf, " ");
		}
	} else {
		sprintf(buf, "%X", (unsigned int)v);
	}
	return std::string(buf);

}

std::string HexByte(const uint16_t& v) {
	char buf[50];
	sprintf(buf, "%X", v);
	return std::string(buf);

}

void USB::OutputDebug()
{
	uart.SendString("Event,Interrupt,Name,Desc,Endpoint,mRequest,Request,Value,Index,Length,PacketSize,XferBuff,\n");
	uint16_t evNo = usbDebugEvent % USB_DEBUG_COUNT;
	std::string interrupt, subtype;

	for (int i = 0; i < USB_DEBUG_COUNT; ++i) {
		if ((usbDebug[evNo].Interrupt & USB_ISTR_CTR) == USB_ISTR_CTR) {
			if ((usbDebug[evNo].Interrupt & USB_ISTR_DIR) == USB_ISTR_DIR) {
				interrupt = "CTR_OUT";
				if (usbDebug[evNo].Request.Request == 6) {
					switch (static_cast<Descriptor>(usbDebug[evNo].Request.Value >> 8))	{
					case DeviceDescriptor:
						subtype = "Get Device Descriptor";
						break;
					case ConfigurationDescriptor:
						subtype = "Get Configuration Descriptor";
						break;
					case BosDescriptor:
						subtype = "Get BOS Descriptor";
						break;

					case StringDescriptor:

						switch ((uint8_t)(usbDebug[evNo].Request.Value & 0xFF)) {
						case StringIndex::LangId:				// 300
							subtype = "Get Lang Str Descriptor";
							break;
						case StringIndex::Manufacturer:					// 301
							subtype = "Get Manufacturor Str Descriptor";
							break;
						case StringIndex::Product:				// 302
							subtype = "Get Product Str Descriptor";
							break;
						case StringIndex::Serial:				// 303
							subtype = "Get Serial Str Descriptor";
							break;
					    case StringIndex::CommunicationClass:					// 304
							subtype = "Get CDC Str Descriptor";
							break;
						}
						break;
					default:
						subtype = "Get Descriptor";
					}
				} else if (usbDebug[evNo].Request.Request == 5) {
					subtype = "Set Address to " + std::to_string(usbDebug[evNo].Request.Value);
				} else if (usbDebug[evNo].Request.Request == 9) {
					subtype = "SET_CONFIGURATION";
				} else if ((usbDebug[evNo].Request.RequestType & requestTypeMask) == RequestTypeClass) {
					switch (usbDebug[evNo].Request.Request) {
					case 0x20:
						subtype = "CDC: Set Line Coding";
						break;
					case 0x21:
						subtype = "CDC: Get Line Coding";
						break;
					case 0x22:
						subtype = "CDC: Set Control Line State";
						break;
					}
				} else {
					subtype = "";
				}
			} else {
				interrupt = "CTR_IN";
				subtype = "";
			}
		}

		if ((usbDebug[evNo].Interrupt & USB_ISTR_SUSP) == USB_ISTR_SUSP) {
			interrupt = "SUSP";
		}

		if ((usbDebug[evNo].Interrupt & USB_ISTR_WKUP) == USB_ISTR_WKUP) {
			interrupt = "WKUP";
		}

		if ((usbDebug[evNo].Interrupt & USB_ISTR_RESET) == USB_ISTR_RESET) {
			interrupt = "RESET";
		}


		if (usbDebug[evNo].Interrupt != 0) {
			uart.SendString(std::to_string(usbDebug[evNo].eventNo) + ","
					+ HexToString(usbDebug[evNo].Interrupt, false) + ","
					+ interrupt + "," + subtype + ","
					+ std::to_string(usbDebug[evNo].endpoint) + ","
					+ HexByte(usbDebug[evNo].Request.RequestType) + ","
					+ HexByte(usbDebug[evNo].Request.Request) + ","
					+ HexByte(usbDebug[evNo].Request.Value) + ","
					+ HexByte(usbDebug[evNo].Request.Index) + ","
					+ HexByte(usbDebug[evNo].Request.Length) + ","
					+ HexByte(usbDebug[evNo].PacketSize) + ","
					+ HexToString(usbDebug[evNo].xferBuff0, true) + " "
					+ HexToString(usbDebug[evNo].xferBuff1, true) + "\n");

		}
		evNo = (evNo + 1) % USB_DEBUG_COUNT;
	}
}


/* startup sequence:
Event		Interrupt	Desc							mReq	Req	Value	Ind	Len	PacketSize	[XferBuff]
	400		RESET
1	400		RESET
2	800		SUSP
3	1000	WKUP
4	400		RESET
5	8010	CTR_OUT		Get Device Descriptor			80		6	100		0	40	12	[12 01 01 02 EF 02 01 40]
6	8000	CTR_IN
7	8010	CTR_OUT
8	8010	CTR_OUT		Set Address to 52				5		34	0		0
9	8000	CTR_IN
10	8010	CTR_OUT		Get Device Descriptor			80		6	100		0	12	12	[12 01 01 02 EF 02 01 40]
11	8000	CTR_IN
12	8010	CTR_OUT
13	8010	CTR_OUT		Get Configuration Descriptor	80		6	200		0	FF	4B	[09 02 4B 00 02 01 00 C0]
14	8000	CTR_IN
15	8000	CTR_IN
16	8010	CTR_OUT
17	8010	CTR_OUT		Get BOS Descriptor				80		6	F00		0	FF	C	[05 0F 0C 00 01 07 10 02]
18	8000	CTR_IN
19	8010	CTR_OUT
20	8010	CTR_OUT		Get Serial Str Descriptor		80		6	303		409	FF	1A	[1A 03 30 00 30 00 36 00]
21	8000	CTR_IN
22	8010	CTR_OUT
23	8010	CTR_OUT		Get Lang Str Descriptor			80		6	300		0	FF	4	[04 03 09 04]
24	8000	CTR_IN
25	8010	CTR_OUT
26	8010	CTR_OUT		Get Product Str Descriptor		80		6	302		409	FF	26	[26 03 4D 00 6F 00 75 00]
27	8000	CTR_IN
28	8010	CTR_OUT
29	8010	CTR_OUT		Get Descriptor					80		6	600		0	A
30	8010	CTR_OUT		SET_CONFIGURATION				9		1
31	8000	CTR_IN
32	8010	CTR_OUT		Get CDC Str Descriptor			80		6	304		409	4	2E	[2E 03 4D 00 6F 00 75 00]
33	8000	CTR_IN
34	8010	CTR_OUT
35	8010	CTR_OUT		Get CDC Str Descriptor			80		6	304		409	2E	2E	[2E 03 4D 00 6F 00 75 00]
36	8000	CTR_IN
37	8010	CTR_OUT
38	8010	CTR_OUT		Get Lang Str Descriptor			80		6	300		0	FF	4	[04 03 09 04 2E 03 4D 00]
39	8000	CTR_IN
40	8010	CTR_OUT
41	8010	CTR_OUT		Get Manufacturor Str Descriptor	80		6	301		409	FF	22	[22 03 4D 00 6F 00 75 00]
42	8000	CTR_IN
43	8010	CTR_OUT
44	8010	CTR_OUT		CDC: Get Line Coding			A1		21	0		0	7
45	8000	CTR_IN
46	8010	CTR_OUT
47	8010	CTR_OUT		Get Product Str Descriptor		80		6	302		409	FF	26	[26 03 4D 00 6F 00 75 00]
48	8000	CTR_IN
49	8010	CTR_OUT
50	8010	CTR_OUT		CDC: Set Control Line State		21		22
51	8000	CTR_IN
52	8010	CTR_OUT		CDC: Set Line Coding			21		20	0		0	7
53	8010	CTR_OUT
54	8000	CTR_IN
55	8010	CTR_OUT		CDC: Get Line Coding			A1		21	0		0	7
56	8000	CTR_IN
57	8010	CTR_OUT
58	8010	CTR_OUT		Get Descriptor					80		6	600		0	A
59	8010	CTR_OUT		Get Lang Str Descriptor			80		6	300		0	1FE	4	[04 03 09 04 26 03 4D 00]
60	8000	CTR_IN
61	8010	CTR_OUT
62	8010	CTR_OUT		Get Manufacturor Str Descriptor	80		6	301		409	1FE	22	[22 03 4D 00 6F 00 75 00]
63	8000	CTR_IN
64	8010	CTR_OUT
65	8010	CTR_OUT		Get Product Str Descriptor		80		6	302		409	1FE	26	[26 03 4D 00 6F 00 75 00]
66	8000	CTR_IN
67	8010	CTR_OUT
68	8010	CTR_OUT		Get CDC Str Descriptor			80		6	304		409	1FE	2E	[2E 03 4D 00 6F 00 75 00]
69	8000	CTR_IN
70	8010	CTR_OUT
71	8010	CTR_OUT										80		6	305		409	1FE
72	8010	CTR_OUT										80		6	3EE		409	1FE
*/
#endif


