#include "initialisation.h"
#include "GpioPin.h"
#include <initializer_list>

// 12MHz / 6(M) * 250(N) / 2(P) = 250MHz
#define PLL_M 6
#define PLL_N 250
#define PLL_P 1				// 01: PLLP = /2
#define PLL_Q 4				// Divide by PPL_Q + 1

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

	// Configure PLL2 to output 12.286MHz clock for SAI: 12MHz (HSE) / 5 (M) * 128 (N) / 25(P)
	RCC->PLL2CFGR = (5 << RCC_PLL2CFGR_PLL2M_Pos) | RCC_PLL2CFGR_PLL2SRC;		// Configure HSE as source for PLL2
	RCC->PLL2DIVR = (127 << RCC_PLL2DIVR_PLL2N_Pos) | (24 << RCC_PLL2DIVR_PLL2P_Pos);
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
	InitADC2(reinterpret_cast<volatile uint16_t*>(&adc), 10);
	InitCordic();
	//InitDAC();

//	GpioPin::Init(GPIOA, 6, GpioPin::Type::Input);	// PA6 - clock in

	// Debug pins - PG12, PG6
//	GpioPin::Init(GPIOG, 6, GpioPin::Type::Output);
//	GpioPin::Init(GPIOG, 12, GpioPin::Type::Output);
}


void InitHyperRAM()
{
	// Cypress S27KL0641 DABHI020: 3.0v, 64Mb, 100MHz
	GpioPin::Init(GPIOA, 3, GpioPin::Type::AlternateFunction, 3, 0b11);		// PA3  OCTOSPI_CLK AF3
	GpioPin::Init(GPIOB, 1, GpioPin::Type::AlternateFunction, 6, 0b11);		// PB1  OCTOSPI_IO0 AF6
	GpioPin::Init(GPIOB, 0, GpioPin::Type::AlternateFunction, 6, 0b11);		// PB0  OCTOSPI_IO1 AF6
	GpioPin::Init(GPIOC, 2, GpioPin::Type::AlternateFunction, 9, 0b11);		// PC2  OCTOSPI_IO2 AF9
	GpioPin::Init(GPIOA, 6, GpioPin::Type::AlternateFunction, 6, 0b11);		// PA6  OCTOSPI_IO3 AF6
	GpioPin::Init(GPIOE, 7, GpioPin::Type::AlternateFunction, 10, 0b11); 	// PE7  OCTOSPI_IO4 AF10
	GpioPin::Init(GPIOE, 8, GpioPin::Type::AlternateFunction, 10, 0b11); 	// PE8  OCTOSPI_IO5 AF10
	GpioPin::Init(GPIOC, 3, GpioPin::Type::AlternateFunction, 6, 0b11);		// PC3  OCTOSPI_IO6 AF6
	GpioPin::Init(GPIOC, 0, GpioPin::Type::AlternateFunction, 10, 0b11);	// PC0  OCTOSPI_IO7 AF10
	GpioPin::Init(GPIOB, 10, GpioPin::Type::AlternateFunction, 9, 0b11);	// PB10 OCTOSPI_NCS AF10
	GpioPin::Init(GPIOB, 2, GpioPin::Type::AlternateFunction, 10, 0b11);	// PB2  OCTOSPI_DQS AF10

	RCC->AHB4ENR |= RCC_AHB4ENR_OCTOSPI1EN;
	//RCC->CCIPR4 &= ~RCC_CCIPR4_OCTOSPISEL;					// kernel clock 250MHz: 00 rcc_hclk4 (default); 01 pll1_q_ck; 10 pll2_r_ck; 11 per_ck
	RCC->CCIPR4 |= RCC_CCIPR4_OCTOSPISEL_0;					// kernel clock 100MHz: 00 rcc_hclk4; *01 pll1_q_ck; 10 pll2_r_ck; 11 per_ck

	// Various settings below taken from AN5050
	OCTOSPI1->CR |= (3 << OCTOSPI_CR_FTHRES_Pos);			// FIFO Threshold
	OCTOSPI1->DCR1 |= (22 << OCTOSPI_DCR1_DEVSIZE_Pos);		// No. of bytes = 2^(DEVSIZE+1): 64Mb = 2^23 bytes
	OCTOSPI1->DCR1 |= (7 << OCTOSPI_DCR1_CSHT_Pos);			// CSHT + 1: min no CLK cycles where NCS must stay high between commands - Min 10ns
	OCTOSPI1->DCR1 &= ~OCTOSPI_DCR1_CKMODE;					// Clock mode 0: CLK is low NCS high
	OCTOSPI1->DCR1 |= (0b100 << OCTOSPI_DCR1_MTYP_Pos);		// 100: HyperBus memory mode; 101: HyperBus register mode
	OCTOSPI1->DCR2 |= (4 << OCTOSPI_DCR2_PRESCALER_Pos);	// Set prescaler to n + 1 => 100MHz / 1 = 100MHz
	//OCTOSPI1->DCR1 |= OCTOSPI_DCR1_DLYBYP;				// Delay block is bypassed
	//OCTOSPI1->DCR1 |= OCTOSPI_DCR1_FRCK;					// Free running clock
	OCTOSPI1->DCR3 |= (23 << OCTOSPI_DCR3_CSBOUND_Pos);		// Set Chip select boundary
	OCTOSPI1->DCR4 = 250; 									// Refresh Time: The chip select should be released every 4us
	OCTOSPI1->TCR |= OCTOSPI_TCR_DHQC;						// Delay hold quarter cycle; See RM p881
	OCTOSPI1->HLCR |= (6 << OCTOSPI_HLCR_TACC_Pos);			// 40ns: Access time according to memory latency, in no of communication clock cycles
	OCTOSPI1->HLCR |= (3 << OCTOSPI_HLCR_TRWR_Pos);			// 40ns: Minimum recovery time in number of communication clock cycles
	//OCTOSPI1->HLCR |= OCTOSPI_HLCR_WZL;					// Set write zero latency
	OCTOSPI1->HLCR |= OCTOSPI_HLCR_LM;						// Latency mode	0: Variable initial latency; 1: Fixed latency
}


void InitAudioCodec()
{
	// Enable SPI
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;					// SPI1 clock enable

	GpioPin::Init(GPIOA, 5, GpioPin::Type::AlternateFunction, 5);	// PA5: SPI1_SCK
	GpioPin::Init(GPIOB, 4, GpioPin::Type::AlternateFunction, 5);	// PB4: SPI1_MISO
	GpioPin::Init(GPIOB, 5, GpioPin::Type::AlternateFunction, 5);	// PB5: SPI1_MOSI
	GpioPin::Init(GPIOG, 10, GpioPin::Type::AlternateFunction, 5);	// PG10: SPI1_NSS

	// Configure SPI
	SPI1->CFG1 |= SPI_CFG1_MBR_1;						// Baud rate (100Mhz/x): 000: /2; 001: /4; *010: /8; 011: /16; 100: /32; 101: /64
	SPI1->CR2  |= 1;									// Transfer size of 1
	SPI1->CFG2 |= SPI_CFG2_SSOE;						// Hardware NSS management
	SPI1->CFG1 |= 0b11111 << SPI_CFG1_DSIZE_Pos;		// Data Size: 0b111 = 8 bit; 0b11111 = 32bit
	SPI1->CFG2 |= SPI_CFG2_MASTER;						// Master mode

	SPI1->CR1 |= SPI_CR1_SPE;							// Enable SPI
}


void InitSAI()
{
	// SAI peripheral configured to output 2 stereo signals on SAI1 and receive 2 stereo signals on SAI2
	// Master is SAI1_A and all other blocks use the same clock
	RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;					// Enable SAI peripheral
	RCC->APB2ENR |= RCC_APB2ENR_SAI2EN;
	RCC->CCIPR5 |= RCC_CCIPR5_SAI1SEL_0;				// SAI Clock Source: 000: pll1_q_ck; *001: pll2_p_ck; 010: pll3_p_ck; 011: AUDIOCLK; 100: per_ck

	GpioPin::Init(GPIOE, 2, GpioPin::Type::AlternateFunction, 6); 	// PE2 SAI1_MCLK_A  AF6
	GpioPin::Init(GPIOE, 3, GpioPin::Type::AlternateFunction, 6); 	// PE3 SAI1_SD_B    AF6
	GpioPin::Init(GPIOE, 4, GpioPin::Type::AlternateFunction, 6); 	// PE4 SAI1_FS_A    AF6
	GpioPin::Init(GPIOE, 5, GpioPin::Type::AlternateFunction, 6); 	// PE5 SAI1_SCK_A   AF6
	GpioPin::Init(GPIOE, 6, GpioPin::Type::AlternateFunction, 6); 	// PE6 SAI1_SD_A    AF6
	GpioPin::Init(GPIOD, 11, GpioPin::Type::AlternateFunction, 10); // PD11 SAI2_SD_A   AF10
	GpioPin::Init(GPIOE, 11, GpioPin::Type::AlternateFunction, 10); // PE11 SAI2_SD_B   AF10

	SAI1_Block_A->CR1 |= SAI_xCR1_DS;					// 110: 24 bits; 111: 32 bits
	SAI1_Block_A->CR1 |= SAI_xCR1_CKSTR;				// 0: Signals generated by SAI change on rising edge, signals received by SAI sampled on falling edge; 	1: Signals generated by SAI change on falling edge, signals received sampled on rising edge.
	SAI1_Block_A->CR1 |= 0x0 << SAI_xCR1_MCKDIV_Pos;	// SAI clock set up for 12.288MHz (48kHz * 256) so no division required
	SAI1_Block_A->CR1 |= SAI_xCR1_MCKEN;				// Enable Master clock output

	SAI1_Block_A->CR2 |= SAI_xCR2_FTH_0;				// Set FIFO interrupt level to quarter full

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

	SAI1_Block_B->CR2 |= SAI_xCR2_FTH_0;				// Set FIFO interrupt level to quarter full

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
	SAI2_Block_B->CR1 |= SAI_xCR1_CKSTR;				// 0: Generated signals change on rising edge, sampled on falling edge;	1: Generated signals change on falling edge, sampled on rising edge.
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
	SysTick_Config(SystemCoreClock / sysTick);		// gives 1ms
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


void InitAdcPins(ADC_TypeDef* ADC_No, std::initializer_list<uint8_t> channels)
{
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

		// Use slowest sampling rate: 111: 640.5 ADC clock cycles
		if (channel < 10)
			ADC_No->SMPR1 |= 0b111 << (3 * channel);
		else
			ADC_No->SMPR2 |= 0b111 << (3 * (channel - 10));

		sequence++;
	}
}


void InitADC2(volatile uint16_t* buffer, uint16_t channels)
{
	// Initialize Clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
	RCC->CCIPR5 |= (3 << RCC_CCIPR5_ADCDACSEL_Pos);	// 000: rcc_hclk; 001: sys_ck; 010: pll2_r_ck; *011: hse_ck; 100: hsi_ker_ck; 101: csi_ker_ck
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

	ADC2->CR &= ~ADC_CR_DEEPPWD;					// Deep power down: 0: not in deep-power down	1: ADC in deep-power-down (default reset state)
	ADC2->CR |= ADC_CR_ADVREGEN;					// Enable ADC internal voltage regulator

	// Wait until voltage regulator settled
	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}
	while ((ADC2->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	ADC12_COMMON->CCR |= ADC_CCR_CKMODE;			// adc_hclk/4 (Synchronous clock mode)
	ADC2->CFGR |= ADC_CFGR_CONT;					// 1: Continuous conversion mode for regular conversions
	ADC2->CFGR |= ADC_CFGR_OVRMOD;					// Overrun Mode 1: ADC_DR register is overwritten with the last conversion result when an overrun is detected.
	ADC2->CFGR |= ADC_CFGR_DMACFG;					// 0: DMA One Shot Mode selected, 1: DMA Circular Mode selected
	ADC2->CFGR |= ADC_CFGR_DMAEN;					// Enable ADC DMA

	// For scan mode: set number of channels to be converted
	ADC2->SQR1 |= (channels - 1);

	// Start calibration
	ADC2->CR &= ~ADC_CR_ADCALDIF;					// Calibration in single ended mode
	ADC2->CR |= ADC_CR_ADCAL;
	while ((ADC2->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};


	/*--------------------------------------------------------------------------------------------
	Configure ADC Channels to be converted:
	PA1 ADC12_INP1	Delay Time
	PA2 ADC12_INP14	Delay Feedback
	PA3 ADC12_INP15	FX Mix
	PA6 ADC12_INP3	LFO Range
	PA7 ADC12_INP7	Delay Mix
	PB0	ADC12_INP9	LFO Base Frequency
	PB1 ADC12_IN5   Delay Filter
	PC1 ADC12_INP11	Delay Time Pot
	PC4	ADC12_INP4	LFO Speed
	PC5 ADC12_INP8	FX Feedback
	*/
	InitAdcPins(ADC2, {1, 14, 15, 3, 7, 9, 5, 11, 4, 8});

	// Enable ADC
	ADC2->CR |= ADC_CR_ADEN;
	while ((ADC2->ISR & ADC_ISR_ADRDY) == 0) {}

	// Initialise DMA registers
	GPDMA1_Channel0->CCR &= ~DMA_CCR_EN;
	GPDMA1_Channel0->CTR1 |= DMA_CTR1_DINC;					// Increment destination address
	GPDMA1_Channel0->CTR1 |= DMA_CTR1_SDW_LOG2_0;			// Source data size: 00: 8 bit; 01 = 16 bit; 10 = 32 bit
	GPDMA1_Channel0->CTR1 |= DMA_CTR1_DDW_LOG2_0;			// Destination data size: 00: 8 bit; 01 = 16 bit; 10 = 32 bit
	GPDMA1_Channel0->CTR2 |= (1 << DMA_CTR2_REQSEL_Pos);	// See page 621 for DMA channel assignments
	GPDMA1_Channel0->CBR1 |= channels * 2;					// Number of bytes (data items * 2) to transfer
	GPDMA1_Channel0->CSAR= (uint32_t)(&(ADC2->DR));			// Configure the source data register address
	GPDMA1_Channel0->CDAR = (uint32_t)(buffer);				// Configure the memory address

	static DMALinkedList dmaLinkedList;						// Create a linked list node which holds register values to be reloaded each cycle
	GPDMA1_Channel0->CLBAR = (uint32_t)&dmaLinkedList & 0xFFFF0000;		// Linked list base address (top 16 bits)
	uint16_t addrLow = ((uint32_t)&dmaLinkedList & 0xFFFF);	// Get the linked list address low 16 bits

	// Configure the linked list register with the low bits of the linked list memory address and the registers to load on restart
	GPDMA1_Channel0->CLLR = addrLow | DMA_CLLR_UT1 | DMA_CLLR_UT2 | DMA_CLLR_UB1 | DMA_CLLR_UDA | DMA_CLLR_USA | DMA_CLLR_ULL;

	// Store the configured register values in the linked list node to be reloaded when conversion complete
	dmaLinkedList.TR1 = GPDMA1_Channel0->CTR1;
	dmaLinkedList.TR2 = GPDMA1_Channel0->CTR2;
	dmaLinkedList.BR1 = GPDMA1_Channel0->CBR1;
	dmaLinkedList.SAR = GPDMA1_Channel0->CSAR;
	dmaLinkedList.DAR = GPDMA1_Channel0->CDAR;
	dmaLinkedList.LLR = GPDMA1_Channel0->CLLR;

	GPDMA1_Channel0->CFCR = 0x7F << DMA_CFCR_TCF_Pos;	// clear all interrupts for this stream
	GPDMA1_Channel0->CCR |= DMA_CCR_EN;					// Enable DMA

	ADC2->CR |= ADC_CR_ADSTART;							// Start ADC
}


void InitDAC()
{
	// Configure 4 DAC outputs PA4 and PA5 are regular DAC1 buffered outputs; PA2 and PB1 are DAC3 via OpAmp1 and OpAmp3 (Manual p.782)
	RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;				// Enable DAC Clock

	DAC1->MCR &= ~DAC_MCR_MODE1_Msk;				// Set to normal mode: DAC1 channel1 is connected to external pin with Buffer enabled
	DAC1->CR |= DAC_CR_EN1;							// Enable DAC using PA4 (DAC_OUT1)

//	DAC1->MCR &= ~DAC_MCR_MODE2_Msk;				// Set to normal mode: DAC1 channel2 is connected to external pin with Buffer enabled
//	DAC1->CR |= DAC_CR_EN2;							// Enable DAC using PA5 (DAC_OUT2)
}


void InitCordic()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
}






