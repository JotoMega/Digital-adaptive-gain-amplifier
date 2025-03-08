#include <msp430.h>
#include <stdint.h>

#define LED_PWM_PERIOD  (66)    //   LED PWM period set to 66 = 2ms

#define LED_TIME_ON (5)        // LED PWM time on set to 40

#define HIGHEST_VALUE (1024) //     Maximum number of LSB for 10bit ADC conversion

#define REF_AMPLITUDE (465) //      Reference amplitude set to 615 = 1.5V

#define VDD (3.3)  //   Defined expected supply voltage of 3.3V

#define LOW_THRESHOLD   (62) //     Low threshold voltage set to 82 = 0.2V so we can escape the division by small numbers

#define PWM_PERIOD (66)    //       PWM period set to 66 = 2ms

#define TIMER_PERIOD_ADC (99)  //   Set to 99 = 3ms

uint16_t dutycycle = 0; //     Defined startup duty cycle

uint16_t output_sensed_data = 0; //   Data converted by the ADC thats sensing the output

uint16_t input_sensed_data = 0; //    Data converted by the ADC thats sensing the input

uint16_t output_data = 0;      //     Data that is the output of the system

uint16_t ccr3_new = 0;         //      Variable for the new of value of TA0CCR3

uint16_t faulty_flag = 1;      //      Flag that shows that the input signal amplitude level is less than LOW_THRESHOLD - at the begining its set to 1 indicating the default non-nominal regime of work

float gain = 1; //  Starting Gain of the system

float ratio = 1; // Starting ratio of the output data and the maximum number of LSB
/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	P6SEL |= BIT2;  // configuring P6.2 as the ADC channel 2 input - sensing the envelope of the output signal
	P6SEL |= BIT4;  // configuring P6.4 as the ADC channel 4 input - sensing the input signal
	P1SEL |= BIT4;  // configuring P1.4 as TA0.3 output function - PWM output of the system
	P1DIR |= BIT4;  // configuring output function

	P1SEL |= BIT2;  // configuring alternate function of P1.2
	P1DIR |= BIT2;   // P1.2 is TA0.1 output pin - connected to LED


	//TIMER B0 USED FOR ADC SAMPLING PERIOD---------------------------------------------------------
	TB0CCTL0 |= OUTMOD_4;   // setting the output mode to toggle
	TB0CCR0 = TIMER_PERIOD_ADC/2 - 1; // configuring timer period
	TBCTL |= TBSSEL__ACLK; //configuring timer to work at ACLK
	//-----------------------------------------------------------------------------------------------


	//TIMER A0 USED FOR PWM OUTPUT GENERATION--------------------------------------------------------
    TA0CCR0 = PWM_PERIOD;    //  maximum value of counter set
    TA0CTL |= TASSEL__ACLK;      // timer A set to work at ACLK
    TA0CCR3 = dutycycle;         // initial state is Duty cycle = 0


    //TIMER USED FOR FAULTY LED PWM GENERATION--------------------------------------------------------
    TA0CCR1 = dutycycle;
    TA0CCTL1 = OUTMOD_7;
    //------------------------------------------------------------------------------------------------


    //------------------------------------------------------------------------------------------------
    TA0CCTL3 = OUTMOD_7;        // Out mode set to Reset/Set
    TA0CCTL0 = CCIE;            // enabling interrupt for timer A0 when it reaches its final value
    //------------------------------------------------------------------------------------------------


    //ADC12-------------------------------------------------------------------------------------------
	ADC12CTL0 &= ~ADC12ENC;      // ADC disabled
	ADC12CTL0 |= ADC12ON;       // ADC turned on

	ADC12MCTL0 |= ADC12INCH_2;    //reference AVCC and AVSS, channel A2
	ADC12MCTL1 |= ADC12INCH_4;    //reference AVCC and AVSS, channel A3

	ADC12CTL1 |= ADC12SHS_2;      // timer B0CCR0 source for sampling
	ADC12CTL1 |= ADC12SSEL1;    // ADC clock set to be ACLK
	ADC12CTL1 |= ADC12CONSEQ_3;  // ADC conversion mode set to repeated multiple channel
	ADC12CTL2 &= ~ADC12RES_2;    // setting resolution bits of ADC to zero
	ADC12CTL2 |= ADC12RES_1;    // setting the resolution of ADC to 10bits
	ADC12IE |= ADC12IE0;        // enable interrupt when MEM0 is written
	ADC12IE |= ADC12IE1;         // enable interrupt when MEM1 is written
	ADC12CTL0 |= ADC12ENC;      // ADC enabled
	//TIMER STARTUP-----------------------------------------------------------------------------------------------
	TB0CTL |= MC__UP;           // starting the timer B0
	TA0CTL |= MC__UP;           // starting the timer A0
	__enable_interrupt();       // enabling interrupts
	//-----------------------------------------------------------------------------------------------

	// Main loop - empty
	while(1)
	{

	}
}

void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TACC0ISR (void){
    // Determining the new TA0CCR1 limit after the end of the timer period
    TA0CCR3 = ccr3_new;
    if(faulty_flag == 0){
        TA0CCR1 = LED_TIME_ON;
    }
    else{
        TA0CCR1 = 0;
    }
    // Flag is automatically cleared
}


void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12ISR (void){
    /* Algorithm:
     * - converting the data from the output of the envelope detector
     * - calculating the gain based on the amplitude value (output_sensed_data) and the reference amplitude (REF_AMPLITUDE)
     * - sensing the value of the input signal
     * - amplifying the input signal - multiplying with gain (input_sensed_data * gain)
     * - determining the PWM duration of the timer so that the mean value of the output produce that value (ccr3_new = PWM_PERIOD * ratio)
     */

    switch(ADC12IV){
    // IR with output envelope signal sensing
    case ADC12IV_ADC12IFG0:
        output_sensed_data = ADC12MEM0;
        if(output_sensed_data >= LOW_THRESHOLD){
            gain = (float) REF_AMPLITUDE/ output_sensed_data;
            faulty_flag = 0;                                  //no break here because
        }                                                     //the next case should
        else{                                                 //be served right
            gain = 1;                                         // after
            faulty_flag = 1;
            break;
        }
    // IR with input signal sensing
    case ADC12IV_ADC12IFG1:
        input_sensed_data = ADC12MEM1;
        output_data = input_sensed_data * gain;
        ratio = (float)output_data/HIGHEST_VALUE;
        ccr3_new = PWM_PERIOD * ratio;
        break;
    default:
        gain = 1;
        break;
    }
}
