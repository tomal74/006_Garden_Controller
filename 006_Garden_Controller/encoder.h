/*
 * mkencoder.c
 *
 *  Obs�uga ENKODERA z przyciskiem
 *
 *  Created on: 8 lut 2016
 *      Author: Miros�aw Karda�
 *      website: www.atnel.pl
 */

#ifndef ENCODER_H_
#define ENCODER_H_

/*--------------------- inicjalizacja / obs�uga enkodera w main.c -----------------------------
    mk_encoder_init();
    register_enc_event_callback( enc_event );    // w�asna funckja obs�ugi enkodera
    register_enc_event_sw_callback( enc_sw );    // w�asna funckja obs�ugi klawisza enkodera

    // w programie g��wnym u�ytkownik ma do dyspozycji zmienne:
     1. enco_cnt - licznik enkodera (-32768 do 32767)
     2. enco_dir - kierunek ( 1 - left, 2 - right )

    while(1) {
        ENCODER_EVENT();
    }
-----------------------------------------------------------------------------------------------
*/

//****************************** konfiguracja sprz�towa enkodera ******************************

#define HALF_STEP        1       //     0 - fullstep encoder
//    1 - halfstep encoder

#define    USE_INT_IRQ        0       //  0 - obs�uga typu pooling
//    1 - obs�uga na przerwaniach

#define USE_ENC_SWITCH		0

#if USE_INT_IRQ == 1

#define    ENC_INT            -1        // ( -1 - INT0 & INT1, -2 - PCINT )

#if ENC_INT == -2

#define PCINT_IRQ_VECT    PCINT2_vect    // wektor przerwania PCINT
#define PCMSK_REG        PCMSK2        // nazwa rejestru maskuj�cego grupy przerwa� PCINT
#define PCINT_A            PCINT18        // nazwa przerwania PCINTx dla pinu A enkodera
#define PCINT_B            PCINT19        // nazwa przerwania PCINTx dla pinu B enkodera
#endif

#endif


//------------ piny A i B enkodera -----------------------
#define ENC_B        (1<<PD5)    // pin pod��czony do wej�cia INTx
#define ENC_A        (1<<PB2)    // pin pod��czony do innego wej�cia tego samego portu
#define ENC_B_PIN     PIND
#define ENC_B_PORT    PORTD
#define ENC_A_PIN     PINB
#define ENC_A_PORT    PORTB


//------------- przycisk enkodera -------------------------
#define ENC_SW            (1<<PD4)    // pin pod��czony do dowolnego wej�cia
#define ENC_SW_PORT        PORTD
#define ENC_SW_PIN        PIND
#define ENC_SW_DIR        DDRD

//**************************************************** koniec konfiguracji sprz�towej ****************


#ifdef GIMSK
#define GICR     GIMSK
#endif



#define ENC_SW_ON        (ENC_SW_PIN & ENC_SW)

#define enc_A_HI        (ENC_A_PIN & ENC_A)
#define enc_B_HI        (ENC_B_PIN & ENC_B)



#define ENC_LEFT     0x10
#define ENC_RIGHT     0x20


//extern volatile int enco_cnt;
extern volatile uint8_t enco_dir;



//--------- funkcje biblioteki enkodera -------------------
void mk_encoder_init( void );    // inicjalizacja obs�ugi enkodera
void ENCODER_EVENT( void );        // zdarzenie enkodera do p�tli g��wnej

// rejestracja w�asnej funkcji obs�ugi przycisku enkodera
void register_enc_event_sw_callback( void ( *callback )( void ) );

// rejestracja w�asnej funkcji obs�ugi enkodera
void register_enc_event_callback( void ( *callback )( void ) );


void encoder_proc( void );
int get_encoder( void );
void set_encoder( int val );



#endif /* MKENCODER_H_ */
