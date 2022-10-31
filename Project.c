/*
 * This file is part of the ÂµOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
#define myTIM3_PRESCALER ((uint32_t)0x0000BB80)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)
#define myTIM3_PERIOD ((uint32_t)0x00000027)

uint32_t getADC(void);
void delay(int);
int convertres(uint32_t);
void myGPIO_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myEXTI0_Init(void);
void myDAC_Init(void);
void myADC_Init(void);
void myLCD_Init(void);
void cmd(uint16_t);
void send_char(char);
void write_char(int,int);


unsigned int mode = 0x0;
volatile int timertriggered = 0;
unsigned int frequency;
unsigned int resistance;

int main(int argc, char* argv[])
{
  // At this stage the system clock should have already been configured
  // at high speed.

	uint32_t restemp = 0;
	myGPIO_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myTIM3_Init();		/* Initialize timer TIM3 */
	myEXTI0_Init();		/* Initialize EXTI1 */
	myDAC_Init();		/* Initialize DAC */
	myADC_Init();		/* Initialize ADC */
	myLCD_Init();		/* Initialize LCD */

  // Infinite loop
  while (1)
    {
//	  delay(1000);
	  restemp = getADC();  				//gets ADC value of resistance (32 bit unsignt int)
	  resistance = convertres(restemp); //resistance converted to an int from 32bit unsigned int
	  DAC->DHR12R1 = ADC1->DR;			 //Maps the output of the ADC (Data Register) to the input of the DAC
	  delay(10);
	  write_char(frequency, resistance);
	  trace_printf("F: %u R: %u\n", frequency, resistance);
	  }

 }

uint32_t getADC(){
	ADC1->CR |= ADC_CR_ADSTART;//start ADC

	while(!(ADC1->ISR & ADC_ISR_EOC)){}; //wait for adc conversion

	ADC1->ISR &= ~(ADC_ISR_EOC); // resets conversion flag
	//trace_printf("ADC Value: %x",ADC_DR_DATA);
	return ((ADC1 ->DR) & ADC_DR_DATA); // Returns on ADC1's data masking the rest
}


int convertres(uint32_t restemp){

	float percent = (float)(restemp / 4095.0); // Converts the value read from a 0 - 4095 value to a percent

	int newres = (int)(percent * 5000); // Scales the percent by the maximum resistance possible

	return newres; // Returns the equivalent resistance for a given input
}

/*GPIOA Initialization [Untested & Not Done]*/
void myGPIO_Init()
{
	/*MODER sets the mode of the pin input/general purpose out/alt mode/analog mode*/
	/*PUPDR sets the push/pull resistance*/

	/* Enable clock for GPIOA peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //GPIO A is enabled

	/*PA0 initialization pushbutton to switch modes*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

	/*PA1 timer input pins*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	/*PA4 DAC output pin*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER4);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

	/*PA5 ADC analog input pin*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER5);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5);

	/*Port B initialization for LCD*/
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //GPIO B is enabled
	GPIOB->MODER = (GPIOB->MODER & 0xFFFFFF00) | (0x55551500);
	GPIOB->PUPDR = (GPIOB->PUPDR & 0xFFFFFF00) | (0x0 << 8);


}

/*Timer2 Initialization [Untested]*/
void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;

	//Timer Pulse
	TIM2->CR1 |= TIM_CR1_CEN;
}

void myTIM2_IRQHandler(){

	if((TIM2->SR & TIM_SR_UIF)!=0){

		//traceprintf("\n*** Overflow! ***\n");
		TIM2->SR &= ~(TIM_SR_UIF); //flip bit 1 from 0 -> 1
		TIM2->CR1 |= (TIM_CR1_CEN); //timer 2 enable
	}
}


void myTIM3_Init(){
		/* Enable clock for TIM3 peripheral */
		// Relevant register: RCC->APB1ENR
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

		/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
		 * enable update events, interrupt on overflow only */
		// Relevant register: TIM2->CR1
		TIM3->CR1 = ((uint16_t)0x008C);

		/* Set clock prescaler value */
		TIM3->PSC = myTIM3_PRESCALER;
		/* Set auto-reloaded delay */
		TIM3->ARR = myTIM3_PERIOD;

		/* Update timer registers */
		// Relevant register: TIM2->EGR
		TIM3->EGR = ((uint16_t)0x0001);

}

void delay(int time){

	TIM3->CNT = ((uint32_t) 0x00000004); // Clear the timer

	TIM3->ARR = time; // Set the timeout value from parameter

	TIM3->EGR |= ((uint16_t) 0x0001); // Update registers

	TIM3->CR1 |= ((uint16_t) 0x0001); // Start the timer

	while((TIM3->SR & 0x0001) == 0); // Wait until timer hits the desired count

	TIM3->SR &= ~((uint16_t) 0x0001); // Clear update interrupt flag

	TIM3->CR1 &= ~0x0001; // Stop timer (TIM3->CR1)
}

void myTIM3_IRQHandler(){

	if((TIM3->SR & TIM_SR_UIF)!=0){

		write_char(frequency,resistance);

		TIM3->SR = ((uint16_t)0x0000);
		TIM3->CR1 |= ((uint16_t)0x0001);
	}
}

/*External Interrupt Initialization [Untested]*/
void myEXTI0_Init()
{
	/* Map EXTI0 line to PA1 */
	SYSCFG->EXTICR[0] = ((uint32_t)0x00000080);

	/* EXTI1 line interrupts: set rising-edge trigger */
	EXTI->RTSR = EXTI_RTSR_TR1; //Needs fixing

	/* Unmask interrupts from EXTI1 line */
	EXTI->IMR = EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(EXTI0_1_IRQn,0);

	NVIC_EnableIRQ(EXTI0_1_IRQn); //Enable 555 timer interrupts
	//NVIC_DisableIRQ(EXTI2_3_IRQn); //Disable function gen interrupts
}

/*ADC Initialization*/
void myADC_Init(){

		RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // Enable clock for ADC

		ADC1->CHSELR |= ADC_CHSELR_CHSEL5; // Select channel 5 of ADC

		ADC1->SMPR |= ADC_SMPR_SMP; //Select maximum sampling time

		ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD); //Continuous conversion and turn on overrun mode

		ADC1->CR |= ADC_CR_ADEN;//Enable ADC

		while(!(ADC1->ISR & ADC_ISR_ADRDY)){}; //Wait until ADC is ready

		ADC1->CR |= ADC_CR_ADSTART;//Start ADC, values can now be read from ADC_DR
}

/*DAC Initialization [Untested]*/
void myDAC_Init(){
	RCC->APB1ENR |= RCC_APB1ENR_DACEN; //Enable DAC clock
	DAC->CR |= DAC_CR_EN1; //Enable DAC channel 1
}

void myLCD_Init(){
	/*clear LCD screen*/
	GPIOB->BRR = 0xFFFF;  //set all bit reset register to 1 (reset bits)
	cmd((uint16_t) 0x28); //DL set 1, N set 1, F set 0 (8bit ram, two lines, 8 chars)
	cmd((uint16_t) 0x0C); //Disp set to 1, cursor set to 0, blinking set to 0,
	cmd((uint16_t) 0x06); //Address auto increment 1, shift = 0
	cmd((uint16_t) 0x01); //clear display
}

void cmd (uint16_t command){
	unsigned int temp;

	GPIOB->BRR = GPIO_Pin_5; //set RS to 0 (reset) command mode

	GPIOB->BRR = GPIO_Pin_6; //set Read/Write to 1 (reading)

	temp = GPIOB->ODR & ~((uint16_t) 0xFF00); //set temp variable to the upper half of the data register

	GPIOB->ODR = temp | ((uint16_t) command << 8); // flush next 8 chars

	GPIOB->BSRR = GPIO_Pin_4; //Set bit 4 to 1 (enable lcd handshake)
	while (((GPIOB->IDR) & GPIO_Pin_7) == 0){};//wait for pin 7 input
	GPIOB->BRR |= GPIO_Pin_4; //Handshaking done
	while (((GPIOB->IDR) & GPIO_Pin_7) != 0){};//wait for pin 7 input (ready for another handhsake)

}

void send_char (char c){
	unsigned int temp;

	GPIOB->BSRR = GPIO_Pin_5; //set RS to 0 (command mode)
	GPIOB->BRR = GPIO_Pin_6; //set Read/Write to 1

	temp = GPIOB->ODR & ~((uint16_t) 0xFF00); //Isolate upper half

	GPIOB->ODR = temp | ((uint16_t) c << 8); //shift chars up 8 bits (flush)
	GPIOB->BSRR = GPIO_Pin_4; //Set bit 4 to 0 ENB

	while (((GPIOB->IDR) & GPIO_Pin_7) == 0){//wait for pin 7 input
	};

	GPIOB->BRR |= GPIO_Pin_4; //Set bit 4 to 1 ENB

	while (((GPIOB->IDR) & GPIO_Pin_7) != 0){//wait for pin 7 input
	};
}

void write_char(int freq, int res){
	int freq1 = 0;
	int freq10 = 0;
	int freq100 = 0;
	int freq1000 = 0; //frequency and resistance placeholders
	int res1 = 0;
	int res10 = 0;
	int res100 = 0;
	int res1000 = 0;


	freq1000 = (freq/1000) % 10;
	freq100 = (freq/100) % 10;
	freq10 = (freq/10) % 10;
	freq1 = (freq) % 10;

	res1000 = (res/1000) % 10;
	res100 = (res/100) % 10;
	res10 = (res/10) % 10;
	res1 = (res) % 10;

	cmd(0x80); //cursor set to the first line

	//sends "F: [frequency of wave] Hz"
	send_char('F');
	send_char(':');
	send_char(freq1000 + '0');
	send_char(freq100 + '0');
	send_char(freq10 + '0');
	send_char(freq1 + '0');
	send_char('H');
	send_char('z');

	cmd(0x00C0); //set cursor to the second line

	//sends "R: [resistance calculated] Oh"
	send_char('R');
	send_char(':');
	send_char(res1000 + '0');
	send_char(res100 + '0');
	send_char(res10 + '0');
	send_char(res1 + '0');
	send_char('O');
	send_char('h');
}


void EXTI0_1_IRQHandler(){
	/* Check if EXTI0 interrupt pending flag is indeed set */
	float frequency_temp;
	if ((EXTI->PR & EXTI_PR_PR1) != 0) {

			float counter_value = TIM2->CNT; //timer2 value

			if (timertriggered == 0x0) {    // if the signal is lo and rising edge
				TIM2->CNT = 0x0000; 		// Clear count register (TIM2->CNT).
				TIM2->CR1 |= 0x0001;	 	// Start timer (TIM2->CR1).
				timertriggered = 0x1;       // Set signal as high
			}
			else {
				timertriggered = 0;
				TIM2->CR1 |= 0x0000;
				EXTI->IMR &= ~(EXTI_IMR_MR1); //set the external interrupt enable to disabled
				frequency_temp = 48000000/counter_value; //Add value here based on clock frequency
				frequency = (int)frequency_temp; //convert back to int value
				TIM2->CNT = 0x0000;
				EXTI->IMR |= (EXTI_IMR_MR1);  //re-enable the external interrupts
			}
			EXTI->PR |= EXTI_PR_PR1; //Clear External Interrupt 0 Pending flag
		}
}


#pragma GCC diagnostic pop