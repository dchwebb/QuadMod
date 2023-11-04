#include "initialisation.h"

// 8MHz / 4(M) * 250(N) / 2(P) = 250MHz
#define PLL_M 4
#define PLL_N 250
#define PLL_P 1				// 01: PLLP = /2
#define PLL_Q 1				// 01: PLLQ = /2

void InitSystemClock(void) {
	PWR->VOSCR |= PWR_VOSCR_VOS;						// Set power scaling to 11: VOS0 (highest frequency)
	while ((PWR->VOSSR & PWR_VOSSR_VOSRDY) == 0); 		// Delay after an RCC peripheral clock enabling

//	RCC->CR |= RCC_CR_CSION;							// Enable low power internal RC oscillator
//	while ((RCC->CR & RCC_CR_CSIRDY) == 0);				// Wait until CSI ready

	RCC->CR |= RCC_CR_HSEON;							// HSE ON
	while ((RCC->CR & RCC_CR_HSERDY) == 0);				// Wait till HSE is ready

	// Configure PLL1
	RCC->PLL1CFGR = (PLL_M << RCC_PLL1CFGR_PLL1M_Pos) | (RCC_PLL1CFGR_PLL1SRC);		// PLL1SRC 11: HSE selected as PLL clock
	RCC->PLL1DIVR = ((PLL_N - 1) << RCC_PLL1DIVR_PLL1N_Pos) | (PLL_P << RCC_PLL1DIVR_PLL1P_Pos) | (PLL_Q << RCC_PLL1DIVR_PLL1Q_Pos);

	// Settings needed for fractional adjustment:
//	RCC->PLL1CFGR |= RCC_PLL1CFGR_PLL1FRACEN;			// HAL Enables this but not sure if needed
//	RCC->PLL1CFGR |= RCC_PLL1CFGR_PLL1RGE;				// PLL1 input frequency range: 00: 1-2MHz; 01: 2-4 MHz; 10: 4-8 MHz; 11: 8-16 MHz;
//	RCC->PLL1CFGR &= ~RCC_PLL1CFGR_PLL1VCOSEL;

	RCC->CR |= RCC_CR_PLL1ON;							// Enable PLL1
	RCC->PLL1CFGR = RCC_PLL1CFGR_PLL1PEN |				// Enable PLL P (drives AHB clock)
					RCC_PLL1CFGR_PLL1QEN;				// Enable PLL Q (for SPI1 clock)
	while ((RCC->CR & RCC_CR_PLL1RDY) == 0);			// Wait till PLL1 is ready

	// Configure Flash prefetch and wait state
	FLASH->ACR |= FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN;
	FLASH->ACR &= ~FLASH_ACR_LATENCY_2WS;

	RCC->CFGR1 |= RCC_CFGR1_SW;							// Select PLL1 as system clock source
	while ((RCC->CFGR1 & RCC_CFGR1_SWS) != RCC_CFGR1_SWS);	// Wait till the main PLL is used as system clock source

	// Configure PLL2 to output 12.286MHz clock for SAI: 8MHz (HSE) / 5 (M) * 192 (N) / 25(P)
	RCC->PLL2CFGR = (5 << RCC_PLL2CFGR_PLL2M_Pos) | RCC_PLL2CFGR_PLL2SRC;		// Configure HSE as source for PLL2
	RCC->PLL2DIVR = (191 << RCC_PLL2DIVR_PLL2N_Pos) | (24 << RCC_PLL2DIVR_PLL2P_Pos);
	RCC->CR |= RCC_CR_PLL2ON;							// Enable PLL2
	RCC->PLL2CFGR = RCC_PLL2CFGR_PLL2PEN;				// Enable PLL P (drives SAI clock)
	while ((RCC->CR & RCC_CR_PLL2RDY) == 0);			// Wait till PLL2 is ready


	ICACHE->CR |= ICACHE_CR_EN;							// Enable instruction cache
	DCACHE1->CR |= DCACHE_CR_EN;						// Enable data cache

	// These settings are not documented at present, but seem required to apply the pull-up on the USB VP line
	PWR->USBSCR |= PWR_USBSCR_USB33DEN;					// VDDUSB voltage level detector enable
	while ((PWR->VMSR & PWR_VMSR_USB33RDY) == 0);		// Wait till ready

	PWR->USBSCR |= PWR_USBSCR_USB33SV;					// Independent USB supply valid

	SystemCoreClockUpdate();							// Update SystemCoreClock (system clock frequency) from settings of oscillators, prescalers and PLL
}


void InitHardware()
{
	InitSysTick();
	InitMPU();

	// Debug pins - PG12, PG6
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;				// GPIO clock
	GPIOG->MODER &= ~GPIO_MODER_MODE12_1;				// 00: Input, 01: Output, 10: Alternate function, 11: Analog (reset state)
	GPIOG->MODER &= ~GPIO_MODER_MODE6_1;				// 00: Input, 01: Output, 10: Alternate function, 11: Analog (reset state)
}


void InitAudioCodec()
{
	// Initialise pins and functions needed for Audio Codec

	// Enable SPI

	// PA5: SPI1_SCK; PB4: SPI1_MISO; PB5: SPI1_MOSI; PG10: SPI1_NSS (AF5)
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;					// SPI1 clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOGEN;			// GPIO clocks

	// PA5: SPI1_SCK
	GPIOA->MODER  &= ~GPIO_MODER_MODE5_0;				// 10: Alternate function mode
	GPIOA->AFR[0] |= 5 << GPIO_AFRL_AFSEL5_Pos;			// Alternate Function 5 (SPI1)

	// PB5: SPI1_MISO
	GPIOB->MODER  &= ~GPIO_MODER_MODE4_0;				// 10: Alternate function mode
	GPIOB->AFR[0] |= 5 << GPIO_AFRL_AFSEL4_Pos;			// Alternate Function 5 (SPI1)

	// PB5: SPI1_MOSI
	GPIOB->MODER  &= ~GPIO_MODER_MODE5_0;				// 10: Alternate function mode
	GPIOB->AFR[0] |= 5 << GPIO_AFRL_AFSEL5_Pos;			// Alternate Function 5 (SPI1)

	// PG10: SPI1_NSS (uses GPIO rather than hardware NSS which doesn't work with 24 bit data)
	//GPIOG->MODER &= ~GPIO_MODER_MODE10_1;				// 01: Output mode
	//GPIOG->ODR |= GPIO_ODR_OD10;						// Set pin to high
	GPIOG->MODER &= ~GPIO_MODER_MODE10_0;				// 10: Alternate function mode
	GPIOG->AFR[1] |= 5 << GPIO_AFRH_AFSEL10_Pos;		// Alternate Function 5 (SPI1)

	// Configure SPI
	SPI1->CFG1 |= SPI_CFG1_MBR_2;						// Baud rate (250Mhz/x): 000: /2; 001: /4; 010: /8; 011: /16; *100: /32; 101: /64
	SPI1->CR2  |= 1;									// Transfer size of 1
	SPI1->CFG2 |= SPI_CFG2_SSOE;						// Hardware NSS management
	SPI1->CFG1 |= 0b11111 << SPI_CFG1_DSIZE_Pos;		// Data Size: 0b111 = 8 bit; 0b11111 = 32bit
	SPI1->CFG2 |= SPI_CFG2_MASTER;						// Master mode

	SPI1->CR1 |= SPI_CR1_SPE;							// Enable SPI

	// PD15 is PDN pin - has external pull-down to ground; pull high to enable
	//
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;				// Enable GPIO Clock
	GPIOD->MODER &= ~GPIO_MODER_MODE15_1;				// 00: Input, 01: Output, 10: Alternate function, 11: Analog (reset state)

}


void InitSAI()
{
	// SAI peripheral configured to output 2 stereo signals on SAI1 and receive 2 stereo signals on SAI2
	// Master is SAI1_A and all other blocks use the same clock
	RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;					// Enable SAI peripheral
	RCC->APB2ENR |= RCC_APB2ENR_SAI2EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;				// Enable GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;				// Enable GPIO Clock
	RCC->CCIPR5 |= RCC_CCIPR5_SAI1SEL_0;				// SAI Clock Source: 000: pll1_q_ck; *001: pll2_p_ck; 010: pll3_p_ck; 011: AUDIOCLK; 100: per_ck

	// MODER 00: Input, 01: Output, 10: Alternate function, 11: Analog (reset state)

	// PE2 SAI1_MCLK_A  AF6
	// PE3 SAI1_SD_B    AF6
	// PE4 SAI1_FS_A    AF6
	// PE5 SAI1_SCK_A   AF6
	// PE6 SAI1_SD_A    AF6
	GPIOE->MODER &= ~(GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0);
	GPIOE->AFR[0] |= (6 << GPIO_AFRL_AFSEL2_Pos) | (6 << GPIO_AFRL_AFSEL3_Pos) | (6 << GPIO_AFRL_AFSEL4_Pos) | (6 << GPIO_AFRL_AFSEL5_Pos) | (6 << GPIO_AFRL_AFSEL6_Pos);

	// PD11 SAI2_SD_A   AF10
	// PE11 SAI2_SD_B   AF10
	GPIOD->MODER &= ~GPIO_MODER_MODE11_0;
	GPIOD->AFR[1] |= (10 << GPIO_AFRH_AFSEL11_Pos);
	GPIOE->MODER &= ~GPIO_MODER_MODE11_0;
	GPIOE->AFR[1] |= (10 << GPIO_AFRH_AFSEL11_Pos);

	SAI1_Block_A->CR1 |= SAI_xCR1_DS;					// 110: 24 bits; 111: 32 bits
	SAI1_Block_A->CR1 |= SAI_xCR1_CKSTR;				// 0: Signals generated by SAI change on rising edge, signals received by SAI sampled on falling edge; 	1: Signals generated by SAI change on falling edge, signals received sampled on rising edge.
	SAI1_Block_A->CR1 |= 0x0 << SAI_xCR1_MCKDIV_Pos;	// SAI clock set up for 12.288MHz (48kHz * 256) so no division required
	SAI1_Block_A->CR1 |= SAI_xCR1_MCKEN;				// Enable Master clock output

	SAI1_Block_A->CR2 |= SAI_xCR2_FTH_1;				// Set FIFO interrupt level to half full

	SAI1_Block_A->FRCR |= (63 << SAI_xFRCR_FRL_Pos);	// Set Frame size to 32 bits
	SAI1_Block_A->FRCR |= SAI_xFRCR_FSDEF;				// Frame synchronization: 0: FS is start frame signal; 1: FS is start of frame signal + channel side identification
	SAI1_Block_A->FRCR |= SAI_xFRCR_FSOFF;				// Frame synchronization offset: 0: FS asserted on first bit of slot 0. 1: FS asserted one bit before first bit of slot 0.
	SAI1_Block_A->FRCR |= 31 << SAI_xFRCR_FSALL_Pos;	// Frame synchronization active level length: set to toggle on left/right transition

	SAI1_Block_A->SLOTR |= SAI_xSLOTR_NBSLOT_0;			// Number of slots = 2
	SAI1_Block_A->SLOTR |= SAI_xSLOTR_SLOTEN;			// FIXME - enable all slots for now


	// Initialise SAI1 block B as slave transmitter
	SAI1_Block_B->CR1 |= SAI_xCR1_MODE_1;				// 00: Master transmitter; 01: Master receiver; 10: Slave transmitter; 11: Slave receiver
	SAI1_Block_B->CR1 |= SAI_xCR1_DS;					// 110: 24 bits; 111: 32 bits
	SAI1_Block_B->CR1 |= SAI_xCR1_CKSTR;				// 0: Signals generated by SAI change on rising edge, signals received by SAI sampled on falling edge; 	1: Signals generated by SAI change on falling edge, signals received sampled on rising edge.
	SAI1_Block_B->CR1 |= SAI_xCR1_SYNCEN_0;				// 00: asynchronous mode; *01: synchronous with internal subblock; 10: synchronous with external SAI

	SAI1_Block_B->CR2 |= SAI_xCR2_FTH_1;				// Set FIFO interrupt level to half full

	SAI1_Block_B->FRCR = SAI1_Block_A->FRCR;			// Duplicate block A Frame settings
	SAI1_Block_B->SLOTR = SAI1_Block_A->SLOTR;			// Duplicate block A Slot settings


	//------------------------------------------
	// Initialise SAI2 A and B as slave receiver
	SAI2_Block_A->CR1 |= SAI_xCR1_MODE;					// 00: Master transmitter; 01: Master receiver; 10: Slave transmitter; 11: Slave receiver
	SAI2_Block_A->CR1 |= SAI_xCR1_DS;					// 110: 24 bits; 111: 32 bits
	SAI2_Block_A->CR1 |= SAI_xCR1_CKSTR;				// 0: Signals generated by SAI change on rising edge, signals received by SAI sampled on falling edge; 	1: Signals generated by SAI change on falling edge, signals received sampled on rising edge.
	SAI2_Block_A->CR1 |= SAI_xCR1_SYNCEN_1;				// 00: asynchronous mode; 01: synchronous with internal subblock; *10: synchronous with external SAI

	SAI2_Block_B->CR1 |= SAI_xCR1_MODE;					// 00: Master transmitter; 01: Master receiver; 10: Slave transmitter; 11: Slave receiver
	SAI2_Block_B->CR1 |= SAI_xCR1_DS;					// 110: 24 bits; 111: 32 bits
	SAI2_Block_B->CR1 |= SAI_xCR1_CKSTR;				// 0: Signals generated by SAI change on rising edge, signals received by SAI sampled on falling edge; 	1: Signals generated by SAI change on falling edge, signals received sampled on rising edge.
	SAI2_Block_B->CR1 |= SAI_xCR1_SYNCEN_1;				// 00: asynchronous mode; 01: synchronous with internal subblock; *10: synchronous with external SAI

	SAI2_Block_A->FRCR = SAI1_Block_A->FRCR;			// Duplicate SAI1 block A Frame settings
	SAI2_Block_A->SLOTR = SAI1_Block_A->SLOTR;			// Duplicate SAI1 block A Slot settings
	SAI2_Block_B->FRCR = SAI1_Block_A->FRCR;			// Duplicate SAI1 block A Frame settings
	SAI2_Block_B->SLOTR = SAI1_Block_A->SLOTR;			// Duplicate SAI1 block A Slot settings


	// Initialise synching between master on SAI1 and slave on SAI2
	SAI1->GCR |= SAI_GCR_SYNCOUT_0;						// 00: No sync; 01: Block A used to sync; 10: Block B used to sync
	SAI2->GCR &= ~SAI_GCR_SYNCIN;						//* 00: SAI2 syncs to SAI1; 01: SAI1 syncs to SA2

	// Initialise interrupt on SAI1 block A master transmitter
	SAI1_Block_A->IMR |= SAI_xIMR_FREQIE;				// Interrupt on FIFO empty

	NVIC_SetPriority(SAI1_IRQn, 1);						// Lower is higher priority
	NVIC_EnableIRQ(SAI1_IRQn);

	SAI1_Block_A->DR = 0;								// Load the transmitter FIFO with the first left/right values
	SAI1_Block_A->DR = 0;
	SAI1_Block_B->DR = 0;
	SAI1_Block_B->DR = 0;

	// Initialise SAI
	SAI1_Block_B->CR1 |= SAI_xCR1_SAIEN;
	SAI2_Block_A->CR1 |= SAI_xCR1_SAIEN;
	SAI2_Block_B->CR1 |= SAI_xCR1_SAIEN;
	SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
}






void InitSysTick()
{
	SysTick_Config(SystemCoreClock / SYSTICK);		// gives 1ms
	NVIC_SetPriority(SysTick_IRQn, 0);
}


void InitMPU()
{
	// Use the Memory Protection Unit (MPU) to set up a region of memory with data caching disabled to access UID
	MPU->RNR = 0;									// Memory region number

	MPU->RBAR = (UID_BASE & 0xFFFFFFE0UL) |			// Store the address of the UID into the region base address register
				(0    << MPU_RBAR_SH_Pos) |			// Shareable
				(0b11 << MPU_RBAR_AP_Pos) |			// Access Permission: Read-only by any privilege level
				(1    << MPU_RBAR_XN_Pos);			// Execution not permitted

	MPU->RLAR = ((UID_BASE | 0xFFFF) & 0xFFFFFFE0UL) |	// Set the upper memory limit for the region
				(0 << MPU_RLAR_AttrIndx_Pos)         |	// Set attribute index to 0
				MPU_RLAR_EN_Msk;						// Enable the region

	MPU->CTRL |= (1 << MPU_CTRL_PRIVDEFENA_Pos) |	// Enable PRIVDEFENA - this allows use of default memory map for memory areas other than those specific regions defined above
				 (1 << MPU_CTRL_ENABLE_Pos);		// Enable the MPU

}

/*
void InitDAC()
{
	// Configure 4 DAC outputs PA4 and PA5 are regular DAC1 buffered outputs; PA2 and PB1 are DAC3 via OpAmp1 and OpAmp3 (Manual p.782)

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;			// Enable GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;			// Enable GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN | RCC_AHB2ENR_DAC2EN | RCC_AHB2ENR_DAC3EN | RCC_AHB2ENR_DAC4EN;				// Enable DAC Clock

	DAC1->MCR &= ~DAC_MCR_MODE1_Msk;				// Set to normal mode: DAC1 channel1 is connected to external pin with Buffer enabled
	DAC1->CR |= DAC_CR_EN1;							// Enable DAC using PA4 (DAC_OUT1)

	DAC1->MCR &= ~DAC_MCR_MODE2_Msk;				// Set to normal mode: DAC1 channel2 is connected to external pin with Buffer enabled
	DAC1->CR |= DAC_CR_EN2;							// Enable DAC using PA5 (DAC_OUT2)

	DAC2->MCR &= ~DAC_MCR_MODE1_Msk;				// Set to normal mode: DAC2 channel1 is connected to external pin with Buffer enabled
	DAC2->CR |= DAC_CR_EN1;							// Enable DAC using PA6 (DAC_OUT1)

	// output triggered with DAC->DHR12R1 = x;

	// Opamp for DAC3 Channel 1: Follower configuration mode - output on PA2
	DAC3->MCR |= DAC_MCR_MODE1_0 | DAC_MCR_MODE1_1;	// 011: DAC channel1 is connected to on chip peripherals with Buffer disabled
	DAC3->CR |= DAC_CR_EN1;							// Enable DAC

	OPAMP1->CSR |= OPAMP_CSR_VMSEL;					// 11: Opamp_out connected to OPAMPx_VINM input
	OPAMP1->CSR |= OPAMP_CSR_VPSEL;					// 11: DAC3_CH1  connected to OPAMP1 VINP input
	OPAMP1->CSR |= OPAMP_CSR_OPAMPxEN;				// Enable OpAmp: voltage on pin OPAMPx_VINP is buffered to pin OPAMPx_VOUT (PA2)

	// Opamp for DAC3 Channel 2: Follower configuration mode - output on PB1
	DAC3->MCR |= DAC_MCR_MODE2_0 | DAC_MCR_MODE2_1;	// 011: DAC channel2 is connected to on chip peripherals with Buffer disabled
	DAC3->CR |= DAC_CR_EN2;							// Enable DAC

	OPAMP3->CSR |= OPAMP_CSR_VMSEL;					// 11: Opamp_out connected to OPAMPx_VINM input
	OPAMP3->CSR |= OPAMP_CSR_VPSEL;					// 11: DAC3_CH2  connected to OPAMP1 VINP input
	OPAMP3->CSR |= OPAMP_CSR_OPAMPxEN;				// Enable OpAmp: voltage on pin OPAMPx_VINP is buffered to pin OPAMPx_VOUT (PB1)

	// Opamp for DAC4 Channel 1: Follower configuration mode - output on PB12
	DAC4->MCR |= DAC_MCR_MODE1_0 | DAC_MCR_MODE1_1;	// 011: DAC channel1 is connected to on chip peripherals with Buffer disabled
	DAC4->CR |= DAC_CR_EN1;							// Enable DAC

	OPAMP4->CSR |= OPAMP_CSR_VMSEL;					// 11: Opamp_out connected to OPAMPx_VINM input
	OPAMP4->CSR |= OPAMP_CSR_VPSEL;					// 11: DAC4_CH1  connected to OPAMP1 VINP input
	OPAMP4->CSR |= OPAMP_CSR_OPAMPxEN;				// Enable OpAmp: voltage on pin OPAMPx_VINP is buffered to pin OPAMPx_VOUT (PB12)

	// Opamp for DAC4 Channel 2: Follower configuration mode - output on PA8
	DAC4->MCR |= DAC_MCR_MODE2_0 | DAC_MCR_MODE2_1;	// 011: DAC channel2 is connected to on chip peripherals with Buffer disabled
	DAC4->CR |= DAC_CR_EN2;							// Enable DAC

	OPAMP5->CSR |= OPAMP_CSR_VMSEL;					// 11: Opamp_out connected to OPAMPx_VINM input
	OPAMP5->CSR |= OPAMP_CSR_VPSEL;					// 11: DAC4_CH2  connected to OPAMP1 VINP input
	OPAMP5->CSR |= OPAMP_CSR_OPAMPxEN;				// Enable OpAmp: voltage on pin OPAMPx_VINP is buffered to pin OPAMPx_VOUT (PA8)

}


void InitIO()
{
	// MODER 00: Input mode, 01: General purpose output mode, 10: Alternate function mode, 11: Analog mode (reset state)
	// PUPDR 01: Pull-up; 10: Pull-down

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;			// GPIO A clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;			// GPIO B clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;			// GPIO C clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;			// GPIO D clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;			// GPIO F clock

	//	Init Multiplexer pins PA1, PA3, PF2
	GPIOA->MODER &= ~GPIO_MODER_MODER1_1;			// PA1: Multiplexer A0 out
	GPIOA->MODER &= ~GPIO_MODER_MODER3_1;			// PA3: Multiplexer A1 out
	GPIOF->MODER &= ~GPIO_MODER_MODER2_1;			// PF2: Multiplexer A2 out

	// Init Gate outputs PD2 - PD5
	GPIOD->MODER &= ~GPIO_MODER_MODER2_1;			// PD2: Gate1 Out
	GPIOD->MODER &= ~GPIO_MODER_MODER3_1;			// PD3: Gate2 Out
	GPIOD->MODER &= ~GPIO_MODER_MODER4_1;			// PD4: Gate3 Out
	GPIOD->MODER &= ~GPIO_MODER_MODER5_1;			// PD5: Gate4 Out

	// Calibration buttons input
	GPIOB->MODER &= ~GPIO_MODER_MODER10;			// PB10: channel A
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0;			// Activate pull-up
	GPIOB->MODER &= ~GPIO_MODER_MODER11;			// PB11: channel B
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD11_0;			// Activate pull-up

	// Octave Switch pins PD0, PC12
	GPIOD->MODER &= ~GPIO_MODER_MODER0;				// PD0: Octave
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD0_1;				// Activate pull-down
	GPIOC->MODER &= ~GPIO_MODER_MODER12;			// PC12: Octave
	GPIOC->PUPDR |= GPIO_PUPDR_PUPD12_1;			// Activate pull-down
}





void InitSPI2()
{
	// Controls MCP48CMB21 single channel DAC for channel B envelope 4
	// PA10: SPI2_MISO; PB13: SPI2_SCK; PB15: SPI2_MOSI; PD15: SPI2_NSS
	RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;			// SPI2 clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIODEN;			// GPIO clocks

	// PB13: SPI2_SCK
	GPIOB->MODER  &= ~GPIO_MODER_MODE13_0;			// 10: Alternate function mode
	GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL13_Pos;	// Alternate Function 5 (SPI2)

	// PB15: SPI2_MOSI
	GPIOB->MODER  &= ~GPIO_MODER_MODE15_0;			// 10: Alternate function mode
	GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL15_Pos;	// Alternate Function 5 (SPI2)

	// PD15: SPI2_NSS
	GPIOD->MODER  &= ~GPIO_MODER_MODE15_1;			// 01: Output mode

	// Configure SPI
	SPI2->CR1 |= SPI_CR1_MSTR;						// Master mode
	SPI2->CR1 |= SPI_CR1_SSI;						// Internal slave select
	SPI2->CR1 |= SPI_CR1_BR_1;						// Baud rate (170Mhz/x): 000: /2; 001: /4; *010: /8; 011: /16; 100: /32; 101: /64
	SPI2->CR1 |= SPI_CR1_SSM;						// Software NSS management
	SPI2->CR2 |= 0b111 << SPI_CR2_DS_Pos;			// Data Size: 0b1011 = 12-bit; 0b111 = 8 bit

	SPI2->CR1 |= SPI_CR1_SPE;						// Enable SPI

	GPIOD->ODR |= GPIO_ODR_OD15;					// SPI2 NSS - initialise high
}


void InitMidiUART()
{
	// PC11: UART4_RX (AF5) or USART3_RX (AF7)

	RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;			// UART4 clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;			// GPIO port C

	GPIOC->MODER &= ~GPIO_MODER_MODE11_0;			// Set alternate function on PE0
	GPIOC->AFR[1] |= 5 << GPIO_AFRH_AFSEL11_Pos;	// Alternate function on PC11 for USART4_RX is AF5

	// By default clock source is muxed to peripheral clock 1 which is system clock
	// Calculations depended on oversampling mode set in CR1 OVER8. Default = 0: Oversampling by 16

	UART4->BRR = SystemCoreClock / 31250;			// clk / desired_baud
	UART4->CR1 &= ~USART_CR1_M;						// 0: 1 Start bit, 8 Data bits, n Stop bit; 	1: 1 Start bit, 9 Data bits, n Stop bit
	UART4->CR1 |= USART_CR1_RE;						// Receive enable
	UART4->CR2 |= USART_CR2_RXINV;					// Invert UART receive to allow use of inverting buffer

	// Set up interrupts
	UART4->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(UART4_IRQn, 1);				// Lower is higher priority
	NVIC_EnableIRQ(UART4_IRQn);

	UART4->CR1 |= USART_CR1_UE;						// UART Enable
}


void InitPWMTimer()
{
	// TIM3: Channel 1: PE2 (AF2)
	// 		 Channel 2: PE3 (AF2)
	// 		 Channel 3: PE4 (AF2)
	// 		 Channel 4: PE5 (AF2)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;			// Enable GPIO Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

	// Enable channels 1 - 4 PWM output pins on PE2-5
	GPIOE->MODER &= ~(GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0);
	GPIOE->AFR[0] |= GPIO_AFRL_AFSEL2_1 | GPIO_AFRL_AFSEL3_1 | GPIO_AFRL_AFSEL4_1 | GPIO_AFRL_AFSEL5_1;			// AF2

	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;					// Output compare 1 preload enable
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;					// Output compare 2 preload enable
	TIM3->CCMR2 |= TIM_CCMR2_OC3PE;					// Output compare 3 preload enable
	TIM3->CCMR2 |= TIM_CCMR2_OC4PE;					// Output compare 4 preload enable

	TIM3->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);	// 0110: PWM mode 1 - In upcounting, channel 1 active if TIMx_CNT<TIMx_CCR1
	TIM3->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);	// 0110: PWM mode 1 - In upcounting, channel 2 active if TIMx_CNT<TIMx_CCR2
	TIM3->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);	// 0110: PWM mode 1 - In upcounting, channel 3 active if TIMx_CNT<TIMx_CCR3
	TIM3->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);	// 0110: PWM mode 1 - In upcounting, channel 3 active if TIMx_CNT<TIMx_CCR3

	TIM3->CCR1 = 0;									// Initialise PWM level to 0
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;

	// Timing calculations: Clock = 170MHz / (PSC + 1) = 21.25m counts per second
	// ARR = number of counts per PWM tick = 4095
	// 21.25m / ARR ~= 5.2kHz of PWM square wave with 4095 levels of output

	TIM3->ARR = 4095;								// Total number of PWM ticks
	TIM3->PSC = 7;									// Should give ~5.2kHz
	TIM3->CR1 |= TIM_CR1_ARPE;						// 1: TIMx_ARR register is buffered
	TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);		// Capture mode enabled / OC1 signal is output on the corresponding output pin
	TIM3->EGR |= TIM_EGR_UG;						// 1: Re-initialize the counter and generates an update of the registers

	TIM3->CR1 |= TIM_CR1_CEN;						// Enable counter

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TIM4: PD12 TIM4_CH1 (AF2)
	// 		 PD13 TIM4_CH2 (AF2)
	// 		 PD14 TIM4_CH3 (AF2)
	// 		 PB9  TIM4_CH4 (AF2)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;			// Enable GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;			// Enable GPIO Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;

	// Enable channels 1 - 3 PWM output pins on PD12-14
	GPIOD->MODER &= ~(GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0);
	GPIOD->AFR[1] |= GPIO_AFRH_AFSEL12_1 | GPIO_AFRH_AFSEL13_1 | GPIO_AFRH_AFSEL14_1;			// AF2

	// Enable channel 4 PWM output pin on PB9
	GPIOB->MODER &= ~GPIO_MODER_MODE9_0;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL9_1;			// AF2

	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;					// Output compare 1 preload enable
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;					// Output compare 2 preload enable
	TIM4->CCMR2 |= TIM_CCMR2_OC3PE;					// Output compare 3 preload enable
	TIM4->CCMR2 |= TIM_CCMR2_OC4PE;					// Output compare 4 preload enable

	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);	// 0110: PWM mode 1 - In upcounting, channel 1 active if TIMx_CNT<TIMx_CCR1
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);	// 0110: PWM mode 1 - In upcounting, channel 2 active if TIMx_CNT<TIMx_CCR2
	TIM4->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);	// 0110: PWM mode 1 - In upcounting, channel 3 active if TIMx_CNT<TIMx_CCR3
	TIM4->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);	// 0110: PWM mode 1 - In upcounting, channel 3 active if TIMx_CNT<TIMx_CCR3

	TIM4->CCR1 = 0;									// Initialise PWM level to 0
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 0;

	// Timing calculations: Clock = 170MHz / (PSC + 1) = 21.25m counts per second
	// ARR = number of counts per PWM tick = 2047
	// 21.25m / ARR ~= 5.2kHz of PWM square wave with 2047 levels of output

	TIM4->ARR = 4095;								// Total number of PWM ticks
	TIM4->PSC = 7;									// Should give ~5.2kHz
	TIM4->CR1 |= TIM_CR1_ARPE;						// 1: TIMx_ARR register is buffered
	TIM4->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);		// Capture mode enabled / OC1 signal is output on the corresponding output pin
	TIM4->EGR |= TIM_EGR_UG;						// 1: Re-initialize the counter and generates an update of the registers

	TIM4->CR1 |= TIM_CR1_CEN;						// Enable counter
}



//	Setup Timer 2 on an interrupt to trigger sample output
void InitEnvTimer() {
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;			// Enable Timer 2
	TIM2->PSC = 16;									// Set prescaler
	TIM2->ARR = 500; 								// Set auto reload register - 170Mhz / (PSC + 1) / ARR = ~20kHz

	TIM2->DIER |= TIM_DIER_UIE;						// DMA/interrupt enable register
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 2);					// Lower is higher priority

	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->EGR |= TIM_EGR_UG;						//  Re-initializes counter and generates update of registers
}


//	Setup Timer 5 on an interrupt to trigger tuner sample capture
void InitTunerTimer()
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;			// Enable Timer 5
	TIM5->PSC = 1;									// Set prescaler
	TIM5->ARR = 500; 								// Set auto reload register - 170Mhz / (PSC + 1) / ARR = ~10kHz

	TIM5->DIER |= TIM_DIER_UIE;						// DMA/interrupt enable register
	NVIC_EnableIRQ(TIM5_IRQn);
	NVIC_SetPriority(TIM5_IRQn, 0);					// Lower is higher priority
}


void InitAdcPins(ADC_TypeDef* ADC_No, std::initializer_list<uint8_t> channels) {
	uint8_t sequence = 1;

	for (auto channel: channels) {
		// Set conversion sequence to order ADC channels are passed to this function
		if (sequence < 5) {
			ADC_No->SQR1 |= channel << ((sequence) * 6);
		} else if (sequence < 10) {
			ADC_No->SQR2 |= channel << ((sequence - 5) * 6);
		} else if (sequence < 15) {
			ADC_No->SQR3 |= channel << ((sequence - 10) * 6);
		} else {
			ADC_No->SQR4 |= channel << ((sequence - 15) * 6);
		}

		// 000: 3 cycles, 001: 15 cycles, 010: 28 cycles, 011: 56 cycles, 100: 84 cycles, 101: 112 cycles, 110: 144 cycles, 111: 480 cycles
		if (channel < 10)
			ADC_No->SMPR1 |= 0b010 << (3 * channel);
		else
			ADC_No->SMPR2 |= 0b010 << (3 * (channel - 10));

		sequence++;
	}
}



void InitADC1(volatile uint16_t* buffer, uint16_t channels)
{
	// Initialize Clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
	RCC->CCIPR |= RCC_CCIPR_ADC12SEL_1;				// 00: no clock, 01: PLL P clk clock, *10: System clock

	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel1->CCR |= DMA_CCR_CIRC;				// Circular mode to keep refilling buffer
	DMA1_Channel1->CCR |= DMA_CCR_MINC;				// Memory in increment mode
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel1->CCR |= DMA_CCR_PL_0;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High

	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF1_Pos;		// clear all five interrupts for this stream

	DMAMUX1_Channel0->CCR |= 5; 					// DMA request MUX input 5 = ADC1 (See p.427)
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF0; // Channel 1 Clear synchronization overrun event flag

	ADC1->CR &= ~ADC_CR_DEEPPWD;					// Deep power down: 0: ADC not in deep-power down	1: ADC in deep-power-down (default reset state)
	ADC1->CR |= ADC_CR_ADVREGEN;					// Enable ADC internal voltage regulator

	// Wait until voltage regulator settled
	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}
	while ((ADC1->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	ADC12_COMMON->CCR |= ADC_CCR_CKMODE;			// adc_hclk/4 (Synchronous clock mode)
	ADC1->CFGR |= ADC_CFGR_CONT;					// 1: Continuous conversion mode for regular conversions
	ADC1->CFGR |= ADC_CFGR_OVRMOD;					// Overrun Mode 1: ADC_DR register is overwritten with the last conversion result when an overrun is detected.
	ADC1->CFGR |= ADC_CFGR_DMACFG;					// 0: DMA One Shot Mode selected, 1: DMA Circular Mode selected
	ADC1->CFGR |= ADC_CFGR_DMAEN;					// Enable ADC DMA

	// For scan mode: set number of channels to be converted
	ADC1->SQR1 |= (channels - 1);

	// Start calibration
	ADC1->CR &= ~ADC_CR_ADCALDIF;					// Calibration in single ended mode
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};


//	Configure ADC Channels to be converted:
//	PC0 ADC12_IN6		Pitch Detect Audio
//	PC2 ADC12_IN8		ChannelALevel

	InitAdcPins(ADC1, {6, 8});

	// Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}

	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF0; // Channel 1 Clear synchronization overrun event flag
	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF1_Pos;		// clear all five interrupts for this stream

	DMA1_Channel1->CNDTR |= channels;				// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));	// Configure the peripheral data register address 0x40022040
	DMA1_Channel1->CMAR = (uint32_t)(buffer);		// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Channel1->CCR |= DMA_CCR_EN;				// Enable DMA and wait
	wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}

	ADC1->CR |= ADC_CR_ADSTART;						// Start ADC
}




void InitADC3(volatile uint16_t* buffer, uint16_t channels)
{
	// Initialize Clocks
	//RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	//RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
	RCC->CCIPR |= RCC_CCIPR_ADC345SEL_1;			// 00: no clock, 01: PLL P clk clock, *10: System clock

	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CCR |= DMA_CCR_CIRC;				// Circular mode to keep refilling buffer
	DMA1_Channel2->CCR |= DMA_CCR_MINC;				// Memory in increment mode
	DMA1_Channel2->CCR |= DMA_CCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel2->CCR |= DMA_CCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel2->CCR |= DMA_CCR_PL_0;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High

	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF2_Pos;		// clear all five interrupts for this stream

	DMAMUX1_Channel1->CCR |= 37; 					// DMA request MUX input 37 = ADC3 (See p.426)
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF1; // Channel 2 Clear synchronization overrun event flag

	ADC3->CR &= ~ADC_CR_DEEPPWD;					// Deep power down: 0: ADC not in deep-power down	1: ADC in deep-power-down (default reset state)
	ADC3->CR |= ADC_CR_ADVREGEN;					// Enable ADC internal voltage regulator

	// Wait until voltage regulator settled
	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}
	while ((ADC3->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	ADC345_COMMON->CCR |= ADC_CCR_CKMODE;			// adc_hclk/4 (Synchronous clock mode)
	ADC3->CFGR |= ADC_CFGR_CONT;					// 1: Continuous conversion mode for regular conversions
	ADC3->CFGR |= ADC_CFGR_OVRMOD;					// Overrun Mode 1: ADC_DR register is overwritten with the last conversion result when an overrun is detected.
	ADC3->CFGR |= ADC_CFGR_DMACFG;					// 0: DMA One Shot Mode selected, 1: DMA Circular Mode selected
	ADC3->CFGR |= ADC_CFGR_DMAEN;					// Enable ADC DMA

	// For scan mode: set number of channels to be converted
	ADC3->SQR1 |= (channels - 1);

	// Start calibration
	ADC3->CR &= ~ADC_CR_ADCALDIF;					// Calibration in single ended mode
	ADC3->CR |= ADC_CR_ADCAL;
	while ((ADC3->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};


//	Configure ADC Channels to be converted:
//	PE7 ADC3_IN4		Env1 Attack
//	PE8 ADC345_IN6		Env1 Decay
//	PE9 ADC3_IN2		Env1 Sustain
//	PE10 ADC345_IN14	Env1 Release

	InitAdcPins(ADC3, {4, 6, 2, 14});

	// Enable ADC
	ADC3->CR |= ADC_CR_ADEN;
	while ((ADC3->ISR & ADC_ISR_ADRDY) == 0) {}

	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF1; // Channel 2 Clear synchronization overrun event flag
	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF2_Pos;		// clear all five interrupts for this stream

	DMA1_Channel2->CNDTR |= channels;				// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Channel2->CPAR = (uint32_t)(&(ADC3->DR));	// Configure the peripheral data register address 0x40022040
	DMA1_Channel2->CMAR = (uint32_t)(buffer);		// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Channel2->CCR |= DMA_CCR_EN;				// Enable DMA and wait
	wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}

	ADC3->CR |= ADC_CR_ADSTART;						// Start ADC
}


void InitADC4(volatile uint16_t* buffer, uint16_t channels)
{
	// Initialize Clocks
	//RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	//RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
	//RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
	//RCC->CCIPR |= RCC_CCIPR_ADC345SEL_1;			// 00: no clock, 01: PLL P clk clock, *10: System clock

	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_CIRC;				// Circular mode to keep refilling buffer
	DMA1_Channel3->CCR |= DMA_CCR_MINC;				// Memory in increment mode
	DMA1_Channel3->CCR |= DMA_CCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel3->CCR |= DMA_CCR_PL_0;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High

	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF3_Pos;		// clear all five interrupts for this stream

	DMAMUX1_Channel2->CCR |= 38; 					// DMA request MUX input 38 = ADC4 (See p.426)
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF2; // Channel 2 Clear synchronization overrun event flag

	ADC4->CR &= ~ADC_CR_DEEPPWD;					// Deep power down: 0: ADC not in deep-power down	1: ADC in deep-power-down (default reset state)
	ADC4->CR |= ADC_CR_ADVREGEN;					// Enable ADC internal voltage regulator

	// Wait until voltage regulator settled
	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}
	while ((ADC4->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	//ADC345_COMMON->CCR |= ADC_CCR_CKMODE;			// adc_hclk/4 (Synchronous clock mode)
	ADC4->CFGR |= ADC_CFGR_CONT;					// 1: Continuous conversion mode for regular conversions
	ADC4->CFGR |= ADC_CFGR_OVRMOD;					// Overrun Mode 1: ADC_DR register is overwritten with the last conversion result when an overrun is detected.
	ADC4->CFGR |= ADC_CFGR_DMACFG;					// 0: DMA One Shot Mode selected, 1: DMA Circular Mode selected
	ADC4->CFGR |= ADC_CFGR_DMAEN;					// Enable ADC DMA

	// For scan mode: set number of channels to be converted
	ADC4->SQR1 |= (channels - 1);

	// Start calibration
	ADC4->CR &= ~ADC_CR_ADCALDIF;					// Calibration in single ended mode
	ADC4->CR |= ADC_CR_ADCAL;
	while ((ADC4->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};


//	Configure ADC Channels to be converted:
//	PD9 ADC4_IN13 		ChannelBLevel
//	PE11 ADC345_IN15	Env2 Attack
//	PE12 ADC345_IN16	Env2 Decay
//	PE14 ADC4_IN1		Env2 Sustain
//	PE15 ADC4_IN2		Env2 Release


	InitAdcPins(ADC4, {13, 15, 16, 1, 2});

	// Enable ADC
	ADC4->CR |= ADC_CR_ADEN;
	while ((ADC4->ISR & ADC_ISR_ADRDY) == 0) {}

	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF2; // Channel 3 Clear synchronization overrun event flag
	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF3_Pos;		// clear all five interrupts for this stream

	DMA1_Channel3->CNDTR |= channels;				// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Channel3->CPAR = (uint32_t)(&(ADC4->DR));	// Configure the peripheral data register address 0x40022040
	DMA1_Channel3->CMAR = (uint32_t)(buffer);		// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Channel3->CCR |= DMA_CCR_EN;				// Enable DMA and wait
	wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}

	ADC4->CR |= ADC_CR_ADSTART;						// Start ADC
}


void InitCordic()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
}



*/



