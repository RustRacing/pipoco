#include <avr/io.h>
#include <stdint.h>

#define CRANK_BIT PD2
#define CAM_BIT PD3
#define INJ1_BIT PH5
#define IGN1_BIT PG1

static void delay_cycles(volatile uint16_t count) {
    while (count--) {
        __asm__ __volatile__("nop");
    }
}

static uint16_t read_adc3(void) {
    ADMUX = (1U << REFS0) | 3U;
    ADCSRA |= (1U << ADSC);
    while (ADCSRA & (1U << ADSC)) {
    }
    return ADC;
}

static void pulse_outputs(void) {
    PORTH |= (1U << INJ1_BIT);
    PORTG |= (1U << IGN1_BIT);
    delay_cycles(32000);
    PORTH &= (uint8_t)~(1U << INJ1_BIT);
    PORTG &= (uint8_t)~(1U << IGN1_BIT);
}

int main(void) {
    DDRD &= (uint8_t)~((1U << CRANK_BIT) | (1U << CAM_BIT));
    DDRH |= (1U << INJ1_BIT);
    DDRG |= (1U << IGN1_BIT);
    PORTH &= (uint8_t)~(1U << INJ1_BIT);
    PORTG &= (uint8_t)~(1U << IGN1_BIT);

    ADCSRA = (1U << ADEN) | (1U << ADPS2) | (1U << ADPS1) | (1U << ADPS0);

    uint8_t last_pind = PIND;
    uint8_t cam_seen = 0;
    uint8_t tooth_count = 0;

    for (;;) {
        uint8_t pind = PIND;
        uint8_t crank_rising = (uint8_t)((pind & (1U << CRANK_BIT)) && !(last_pind & (1U << CRANK_BIT)));
        uint8_t cam_rising = (uint8_t)((pind & (1U << CAM_BIT)) && !(last_pind & (1U << CAM_BIT)));

        if (cam_rising) {
            cam_seen = 1;
        }

        if (crank_rising) {
            tooth_count++;
            if (cam_seen && tooth_count >= 3 && read_adc3() > 120) {
                pulse_outputs();
                tooth_count = 0;
            }
        }

        last_pind = pind;
    }
}
