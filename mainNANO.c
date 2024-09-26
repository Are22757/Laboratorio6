#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Prototipos
void setup(void);
void UART_init(unsigned int ubrr);
void UART_transmit(unsigned char data);
void debounce(void);

int main(void) {
    cli();
    setup();
    UART_init(103);  
    sei();
    while (1);  // Loop vacío, interrupciones manejan todo
}

void setup(void) {
    // Configuración de pines PB0-PB4 y PD7 como entradas con pull-up
    DDRB &= ~(0x1F);   // PB0-PB4 como entradas
    PORTB |= 0x1F;     // Pull-up PB0-PB4
    DDRD &= ~(1 << DDD7); 
    PORTD |= (1 << PORTD7);  // Pull-up PD7

    // Habilitar PCINT para PB0-PB4 y PD7
    PCICR |= (1 << PCIE0) | (1 << PCIE2);
    PCMSK0 |= 0x1F;    // PCINT para PB0-PB4
    PCMSK2 |= (1 << PCINT23);  // PD7
}

void UART_init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void debounce(void) {
    _delay_ms(50);
}

ISR(PCINT0_vect) {
    debounce();
    if (!(PINB & (1 << PINB0))) UART_transmit('U');
    if (!(PINB & (1 << PINB1))) UART_transmit('D');
    if (!(PINB & (1 << PINB2))) UART_transmit('L');
    if (!(PINB & (1 << PINB3))) UART_transmit('R');
    if (!(PINB & (1 << PINB4))) UART_transmit('A');
}

ISR(PCINT2_vect) {
    debounce();
    if (!(PIND & (1 << PIND7))) UART_transmit('B');
}
