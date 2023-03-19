#include <avr/io.h>
#include <util/delay.h>
#define MYDELAY 5


void uart_init(void);
void uart_send(unsigned char ch);
int main(void){
    unsigned char ledDigits[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D,
    0x07, 0x7F, 0x67};  // NUmber values
    unsigned int i=0;
    unsigned char DIG4, DIG3, DIG2, DIG1;

    uart_init();
    DDRC = 0x0F;  // To avoid issues with using TX, set lower bits to Analog in (DDRC)
    DDRD = 0xF0;  //pins on the7 segment
    DDRB = 0xFF;   // Digit enable pins (9,10,11,12)
    

    while(1){
    i++;
        if (i>9999)  i=0;

        DIG4 = i%10;  // 1st
        PORTC = ledDigits[DIG4];
        PORTD = ledDigits[DIG4];
        uart_send(DIG4+'0');
        PORTB = ~ (1<<1);   // Enables digit
        _delay_ms(MYDELAY);

        DIG3 = (i/10)%10;  //10th
        PORTC = ledDigits[DIG3];
        PORTD = ledDigits[DIG3];
        uart_send(DIG3+'0');
        PORTB = ~ (1<<2);
        _delay_ms(MYDELAY);

        DIG2 = (i/100)%10; //100th
        PORTC = ledDigits[DIG2];
        PORTD = ledDigits[DIG2];
        uart_send(DIG2+'0');
        PORTB = ~ (1<<3);
        _delay_ms(MYDELAY);

        DIG1 = (i/1000);  // 1000th
        PORTC = ledDigits[DIG1];
        PORTD = ledDigits[DIG1];
        uart_send(DIG1+'0');
        PORTB = ~ (1<<4);
        _delay_ms(MYDELAY);

        PORTB = 0xFF;
        uart_send(13);
        uart_send(10);
    }
}


void uart_init(void){
    UCSR0B = (1<<TXEN0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
    UBRR0L = 103;
}

void uart_send(unsigned char ch){
    while(!(UCSR0A&(1<<UDRE0)));
    UDR0=ch;
}


