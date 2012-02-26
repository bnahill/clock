/* Blink LED example */

#include <msp430.h>
#include <stdint.h>

#define BIT(x) (1 << x)

#define FLASH_LEDS 0

// Pin 1.0
#define LED1_OUT  P1OUT
#define LED1_DIR  P1DIR
#define LED1_MASK BIT(0)

// Pin 1.6
#define LED2_OUT  P1OUT
#define LED2_DIR  P1DIR
#define LED2_MASK BIT(6)

// Tick
#define TICK_OUT P1OUT
#define TICK_DIR P1DIR
#define TICKP_MASK BIT(4)
#define TICKN_MASK BIT(5)

#define TICK_PER_S      4
#define TPS_MASK        0b11
// The number of ticks (at 32kHz) to wait before disabling the output
// Should be less than 32768/TICK_PER_S...
#define DUTY_CYCLE_LEN  3000

static int16_t error;

// State for Marsaglia's MWC RNG algorithm
static union {
	struct {
		uint32_t m_w;
		uint32_t m_z;
	};
	uint64_t seed;
} rng_state;

void rng_init(void);
uint32_t rng_next(void);
uint16_t rng_next16(void);

int main(void) {
	uint8_t ticks;
	uint16_t rand;
	uint8_t cnt;
	
	WDTCTL = WDTPW | WDTHOLD;

	// Set MCLK for 16MHz
	DCOCTL = CALDCO_1MHZ;
	BCSCTL1 = CALBC1_1MHZ;
	// Set SMCLK for MCLK/8 = 2MHz
	BCSCTL2 |= 0b00000110;

	__enable_interrupt();

#if FLASH_LEDS
	LED1_DIR |= LED1_MASK;
	LED1_OUT |= LED1_MASK;

	LED2_DIR |= LED2_MASK;
	LED2_OUT &= ~LED2_MASK;
#endif
	
	TICK_OUT = TICK_OUT & ~(TICKP_MASK | TICKN_MASK);
	TICK_DIR |= TICKP_MASK | TICKN_MASK;

	rng_init();

	TA0R = 0;
	TA0CCR0 = 32768 / TICK_PER_S;
	TA0CTL = 0b0000000100010010;
	TA0CCR1 = DUTY_CYCLE_LEN;
	TA0CCTL1 = 0b0000000000110000;

	ticks = 0;
	error = 0;
	cnt = 0;
	while(1){
		LPM3;
		if((cnt++ & TPS_MASK) == 0){
			error -= 1;
		}
		
		// Move on to next tick
#if FLASH_LEDS
		LED2_OUT ^= LED2_MASK;
		LED1_OUT ^= LED1_MASK;
#endif

		if(error > (30 * TICK_PER_S)){
			continue;
		}
		if(error < (-30 * TICK_PER_S)){
			ticks += 1;
			error += 1;
			if(ticks & 0x0001){
				TICK_OUT = (TICK_OUT & ~TICKP_MASK) | TICKN_MASK;
			} else {
				TICK_OUT = (TICK_OUT & ~TICKN_MASK) | TICKP_MASK;
			}
			continue;
		}
		rand = rng_next16();
		if((rand & TPS_MASK) == 0){
			ticks += 1;
			error += 1;
			if(ticks & 0x0001){
				TICK_OUT = (TICK_OUT & ~TICKP_MASK) | TICKN_MASK;
			} else {
				TICK_OUT = (TICK_OUT & ~TICKN_MASK) | TICKP_MASK;
			}
		}
	}
}

void rng_init(void){
	uint16_t ticks;

	WDTCTL = WDTPW | 0b0000000000011111;
	
	TA0R = 0;
	// Configure timer for fast up-counting
	TA0CTL = 0b0000001000100000;

	// Perform RNG
	for(ticks = 0; ticks < 64; ticks++){
		// Wait for watchdog interrupt
		while((IFG1 & WDTIFG) == 0);
		IFG1 &= ~WDTIFG;
		rng_state.seed = (rng_state.seed << 1) | (TA0R & 0x0001);
		BCSCTL1 += 5;
	}
	
	BCSCTL1 -= 5 * 64;
	
	WDTCTL = WDTPW | WDTHOLD;
	TA0CTL = 0;
}

uint32_t rng_next(void){
	rng_state.m_z = 36969 * (rng_state.m_z & 0xFFFF) + (rng_state.m_z >> 16);
	rng_state.m_w = 18000 * (rng_state.m_w & 0xFFFF) + (rng_state.m_w >> 16);
	return (rng_state.m_z << 16) + rng_state.m_w;
}

uint16_t rng_next16(void){
	rng_state.m_z = 36969 * (rng_state.m_z & 0xFFFF) + (rng_state.m_z >> 16);
	rng_state.m_w = 18000 * (rng_state.m_w & 0xFFFF) + (rng_state.m_w >> 16);
	return rng_state.m_w;
}


#ifdef __MSP430G2231
#define TA0IV_TAIFG TAIV_TAIFG
#define TA0IV_TACCR1 TAIV_TACCR1
__attribute__((interrupt(TIMERA1_VECTOR)))
#else
__attribute__((interrupt(TIMER0_A1_VECTOR)))
#endif
void timer_tick_isr(void){
	switch(TA0IV){
	case TA0IV_TAIFG:
		// Clear that flag
		TA0CTL &= ~0x0001;
		TA0CCTL1 |= CCIE;
		LPM3_EXIT;
		break;
	case TA0IV_TACCR1:
		TICK_OUT = TICK_OUT & ~(TICKP_MASK | TICKN_MASK);
		// Clear that flag
		TA0CCTL1 &= ~CCIE;
		TA0CCTL1 &= ~0x0001;
		break;
	}
}

