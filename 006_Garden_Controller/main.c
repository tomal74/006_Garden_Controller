/*
 * main.c
 *
 *  Created on: 4 maj 2020
 *      Author: G505s
 */


#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#include "LCD/lcd44780.h"
#include "encoder.h"

#define TIME_TO_OFF_LCD 700 //czas wygaszenia ekranu w bezczynnosci

#define BAT_LOW_VOL 400

//encoder switch (INT1)
#define EN_SW PD3
#define EN_SW_PIN PIND
#define EN_SW_MASK (1<<PD3)
//#define EN_A PD5
//#define EN_B PB2

//#define EN_A_STATE ( PIND & (1<<EN_A) )
//#define EN_B_STATE ( PINB & (1<<EN_B) )

#define BUZZER PC1	//buzzer
#define RELAY PC4	//przekaznik
#define VALVE PB1   //zawor wody (sterowanie PWMem z powodu cewki przewinietej na ok 4V a sterowania z 12V)
#define LCD_BACK_LIGHT PC2 //podswietlenie LCD

#define LCD_ON PORTC |= (1<<LCD_BACK_LIGHT)
#define LCD_OFF PORTC &= ~(1<<LCD_BACK_LIGHT)

#define BUZ_ON PORTC |= (1<<BUZZER)
#define BUZ_OFF PORTC &= ~(1<<BUZZER)
#define BUZ_TOG PORTC ^= (1<<BUZZER)

volatile uint8_t valve_on_flag;
volatile uint8_t int1_flag;
uint8_t buz_flag_tog;


uint8_t on_time_min, on_time_sec;
volatile uint16_t on_time = 10;

uint16_t interval_time_min = 60;
uint16_t remaining_time_to_on;
volatile int32_t interval_time = 3600;

volatile uint32_t sec_time;	// czas w sekundach

uint16_t bat_voltage; //baterry voltage
uint8_t low_bat_flag;

// timery programowe
volatile uint16_t Timer1, Timer2, Timer3, Timer4, Timer5;


const uint8_t flower[] PROGMEM = {32,4,10,4,21,14,4,32};	// wzór znaku buŸki w pamiêci FLASH
//baterie od pustej do calej
const uint8_t bat0[] PROGMEM = {14,27,17,17,17,17,31,0};
const uint8_t bat1[] PROGMEM = {14,27,17,17,17,31,31,0};
const uint8_t bat2[] PROGMEM = {14,27,17,17,31,31,31,0};
const uint8_t bat3[] PROGMEM = {14,27,17,31,31,31,31,0};
const uint8_t bat4[] PROGMEM = {14,31,31,31,31,31,31,0};


void valve_off(uint8_t * state);
void valve_on(uint8_t * state);
uint8_t RelaySuperDebounce( uint8_t * key_state, volatile uint8_t *KPIN, uint8_t key_mask,
		volatile uint16_t *soft_timer, void (*push_proc)(void) );
void key_fun(void);
void my_encoder(void);
void INT1_EVENT(void);
void buzzer_process(void);
void lcd_display(void);
void battery_status(void);
void menu(void);
void VALVE_EVENT(void);
void bat_measure(void);


// main begin
int main(void)
{

	TCCR0 |= (1 << CS00) | (1 << CS01); //presacler for timer0 = 64 -> f_IRQ=100Hz
	TCNT0 = 100;
	TIMSK |= (1 << TOIE0);

	ADCSRA |= (1 << ADEN) | (1 << ADPS2);	//ADC on, ADC prescaler = 16
	ADMUX |= (1 << REFS1) | (1 << REFS0);	//Internal 2.56V Voltage Reference
	ADMUX |= (1 << MUX0) | (1 << MUX2);		//chanel ADC5

	DDRC |= (1 << BUZZER) | (1 << LCD_BACK_LIGHT);
	LCD_ON; //wlacz podswietlenie LCD

	//sprzêtowy (PB1) PWM jako WYJŒCIE
	DDRB |= (1 << VALVE);

	PORTD |= (1 << EN_SW); //int1 pull-up (encoder switch)
	MCUCR |= (1 << ISC11); //int1 falling edge
	GICR |= (1 << INT1);   //int1 (encoder switch) IRQ

	// Encoder pull-up
	//PORTB |= (1 << EN_B);
	//PORTD |= (1 << EN_A);

	ASSR |= (1 << AS2); // set Timer/Counter2 to be asynchronous from the CPU clock with a 32,768kHz
	while (ASSR & 0x07);// Wait until TC2 is updated
	TCCR2 |= (1 << CS22) | (1 << CS20); // prescale the timer to be clock source / 128 to make it

	//Inicjalizacja Timer1 (PWM)
	TCCR1A |= (1 << COM1A1) //Zmiana stanu wyjœcia OC1A na niski przy porównaniu A
	| (1 << WGM11); //Tryb 14 (FAST PWM, TOP=ICR1)

	TCCR1B |= (1 << WGM13) | (1 << WGM12)  //Tryb 14 (FAST PWM, TOP=ICR1)
			| (1 << CS10);                 //prescaler = 1

	ICR1 = 1000;  //Wartoœæ maksymalna (dla trybu 14)			  //a wiêc czêstotliwoœæ = CLK/ICR1 = 1kHz
	OCR1A = 0;    //Wartoœæ pocz¹tkowa porównania A (Wyjœcie OC1A - PB1),
	TIMSK |= (1<<TOIE2); //wlacz przerwanie od zegarka co 1s (overflow)

	lcd_init();
	lcd_defchar_P(0x80, flower);
	lcd_defchar_P(0x81, bat0);
	lcd_defchar_P(0x82, bat1);
	lcd_defchar_P(0x83, bat2);
	lcd_defchar_P(0x84, bat3);
	lcd_defchar_P(0x85, bat4);

	mk_encoder_init();
	register_enc_event_callback( my_encoder );

	lcd_locate(0, 1);
	lcd_str_P(PSTR("\x80 STEROWNIK \x80"));
	lcd_locate(1, 2);
	lcd_str_P(PSTR("OGRODOWY v1.0"));

	BUZ_ON;
	_delay_ms(300);
	BUZ_OFF;
	_delay_ms(1200);

	Timer4 = TIME_TO_OFF_LCD;
	int1_flag = 1;
	lcd_display();

	sei();


	// ***************** while begin ***************** //
	while (1) {

		INT1_EVENT();

		lcd_display();

		VALVE_EVENT();

		bat_measure();

	    menu();

	    buzzer_process();

	}
	// while end
}
//main end




void my_encoder(void) {
	// nie wygaszaj ekranu LCD jesli krencony enkoder
	 Timer4 = TIME_TO_OFF_LCD;
	 buz_flag_tog = 1;
}

int8_t menu_position, menu_lvl;


void INT1_EVENT(void) {
	static uint8_t key_state1 = 0;
	// INT1 (encoder switch) request occur
	if (int1_flag) {
		ENCODER_EVENT();

		//wygas ekran jesli brak aktywnosci enkodera
		if (!Timer4) {
			LCD_OFF;		//wygas ekran
			BUZ_OFF; 		//upewni sie ze buzzer jest off
			int1_flag = 0;	//wyzeruj flage przerwania int1
			menu_lvl = 0; 	//main menu return
		}
		//obsluga klawisza ekodera (debouncer) wykonujaca przez wskanik funkcje key_fun
		RelaySuperDebounce(&key_state1, &EN_SW_PIN, EN_SW_MASK, &Timer3, key_fun);
	} //INT1 end
}

void buzzer_process(void) {
	if(buz_flag_tog) {
		static uint8_t buz_in_state;
		if(!Timer5 && !buz_in_state) {
			BUZ_ON;
			buz_in_state = 1;
			Timer5 = buz_flag_tog;
		}
		if(!Timer5 && buz_in_state) {
			BUZ_OFF;
			buz_in_state = 0;
			buz_flag_tog = 0;
			Timer5 = buz_flag_tog;
		}
	}
}



void battery_status(void) {
	if(bat_voltage <= 593) lcd_str_P(PSTR("\x81")); //critical battery voltage (~11,2V)
	else if(bat_voltage > 593 && bat_voltage <= 635) lcd_str_P(PSTR("\x82")); // bat 1 bar (to ~12V)
	else if(bat_voltage > 635 && bat_voltage <= 658) lcd_str_P(PSTR("\x83")); // bat 2 bars (to ~12,4V)
	else if(bat_voltage > 658 && bat_voltage <= 680) lcd_str_P(PSTR("\x85")); // bat 3 bars (to ~12,8V)
	else if(bat_voltage > 680) lcd_str_P(PSTR("\x85")); // bat full bars (>12,8V)
}


void lcd_display(void) {
	if (!Timer2) {
		if (int1_flag) {
			lcd_cls();

			lcd_locate(menu_position, 0);
			if(menu_lvl == 2) lcd_str("\x7F");
			else lcd_str("\x7E");
			lcd_locate(0, 1);
			lcd_str_P(PSTR("co:"));
			lcd_int(interval_time_min);
			lcd_str_P(PSTR("min"));
			lcd_locate(1, 1);
			lcd_str_P(PSTR("przez: "));
			lcd_int(on_time);
			lcd_str_P(PSTR("s"));
		}

		lcd_locate(0, 10);
		lcd_str_P(PSTR("("));
		cli();
		remaining_time_to_on = interval_time_min - (sec_time / 60UL);
		lcd_int(remaining_time_to_on);
		sei();
		lcd_str_P(PSTR(")"));

		lcd_locate(1, 15);

		battery_status();

	}
}


void menu(void) {
	if (int1_flag) {

		switch(menu_lvl) {
		case 0 ... 1:
			if(get_encoder() > 1) set_encoder(1);
			else if(get_encoder() < 0) set_encoder(0);
			menu_position = get_encoder();
			break;
		case 2:
			if(get_encoder() < 1) set_encoder(1);
			if(menu_position==0) {
				interval_time_min = get_encoder();
				cli();
				interval_time = 60UL * interval_time_min;
				sei();
			}
			else if(menu_position==1) {
				cli();
				on_time = get_encoder();
				sei();
			}
			break;
		default:
			menu_lvl = 0;
			break;
		}
	}
}


void VALVE_EVENT(void) {
	static uint8_t state = 0;
	// valve on
	if(valve_on_flag == 2) {
		valve_off(&state);
	}
	// valve off
	else if(valve_on_flag == 1) {
		valve_on(&state);
	}
}


void bat_measure(void) {
	if (!Timer2) {

		ADCSRA |= (1 << ADSC);
		while ((ADCSRA & (1 << ADSC)));
		bat_voltage = ADCW;

		if(bat_voltage <= BAT_LOW_VOL) low_bat_flag = 1;
		if(bat_voltage >= BAT_LOW_VOL + 100) low_bat_flag = 0;

		if(int1_flag) Timer2 = 10;
		else Timer2 = 1000;
	}
}


void key_fun(void) {
	menu_lvl++;
	if(menu_lvl >= 3) menu_lvl = 1;

	if(menu_lvl == 2) {
		if(!menu_position) set_encoder(interval_time_min);
		else if(menu_position == 1) set_encoder(on_time);
	}

	buz_flag_tog = 6;
}


/************** funkcja RelaySuperDebounce do obs³ugi np.pojedynczych wyjœæ przekaŸników ***************
 * 							AUTOR: Miros³aw Kardaœ
 * 							MODYFIKACJA: Tomasz Konieczka 13.03.2020r
 * 						   (KORONAWIRUS U BRAM... ZAMKNIÊTE SZKO£Y ITD.)
 * ZALETY:
 * 		- nie wprowadza najmniejszego spowalnienia
 * 		- mo¿na przydzieliæ ró¿ne akcje dla trybu klikniêcia
 *
 * Wymagania:
 * 	Timer programowy utworzony w oparciu o Timer sprzêtowy (przerwanie 100Hz)
 *
 * 	Parametry wejœciowe:
 *
 * 	*key_state - wskaŸnik na zmienn¹ w pamiêci RAM (1 bajt) - do przechowywania stanu klawisza
 *  *KPIN - nazwa PINx portu na którym umieszczony jest klawisz, np: PINB
 *  key_mask - maska klawisza np: (1<<PB3)
 *  push_proc - wskaŸnik do w³asnej funkcji wywo³ywanej raz po zwolenieniu przycisku
 *
 *  return:
 *
 *  0 - klawisz niewcisniety
 *  1 - klawisz wciœniêty
 *  2 - zbocze narastajace
 *  3 - zbocze opadajace
 **************************************************************************************/
uint8_t RelaySuperDebounce( uint8_t * key_state, volatile uint8_t *KPIN, uint8_t key_mask,
		volatile uint16_t *soft_timer, void (*push_proc)(void) ) {

	enum {idle, debounce, rising_edge, pressed};

	uint8_t key_press = !(*KPIN & key_mask);

	if( key_press && !*key_state ) {
		*key_state = debounce;
		*soft_timer = 3;
	}
	else if( *key_state  ) {
		if( key_press && debounce==*key_state && !*soft_timer ) {
			*key_state = rising_edge;
			*soft_timer = 3;
		}
		else if( key_press && rising_edge==*key_state && !*soft_timer ) {
			*key_state = pressed;
			return 2;
		}
		else if( key_press && pressed==*key_state && !*soft_timer ) {
			return 1;
		}
		else if( !key_press && *key_state>1 ) {
			if(push_proc) push_proc();						/* KEY_UP */
			*key_state=idle;
			return 3;
		}
	}
	return 0;
}


void valve_off(uint8_t * istate) {
		OCR1A = 0;
		PORTC |= (1 << RELAY);
		DDRC &= ~(1 << RELAY);
		sec_time = 0;
		*istate = 0;
		valve_on_flag = 0;
}


void valve_on(uint8_t * istate) {
	if(!(*istate)) {
		DDRC |= (1 << RELAY);
		PORTC &= ~(1 << RELAY);
		(*istate)++;
		Timer1 = 30;
	}
	else if((*istate) == 1 && !Timer1) {
		OCR1A = 660;
		(*istate)++;
		Timer1 = 30;
	}
	 if((*istate) == 2 && !Timer1) {
		 OCR1A = 330;
	}
}


//int1 (encoder switch) IRQ
ISR(INT1_vect) {
	int1_flag = 1;
	LCD_ON;
	Timer2 = 10; //brak oczekiwania na odswiezenie ekranu
	Timer4 = TIME_TO_OFF_LCD;
}


// timer0 int - 10ms (100Hz)
ISR(TIMER0_OVF_vect) {
	uint16_t n;

	n = Timer1;
	if(n) Timer1 = --n;

	n = Timer2;
	if(n) Timer2 = --n;

	n = Timer3;
	if(n) Timer3 = --n;

	n = Timer4;
	if(n) Timer4 = --n;

	n = Timer5;
	if(n) Timer5 = --n;


	TCNT0 = 100;
}


// RTC, przerwanie co 1 sekunde
ISR(TIMER2_OVF_vect) {
	uint32_t off_time = interval_time + on_time;

	sec_time++; //aktualny czas w sekundach

	if (sec_time == interval_time - 5UL) buz_flag_tog = 200;
	if (sec_time >= interval_time) valve_on_flag = 1;
	if (sec_time >= off_time) valve_on_flag = 2;
}
