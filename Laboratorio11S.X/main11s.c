/* 
 * File:   main11.c
 * Author: Daniel Casasola
 *
 * Created on May 14, 2022, 10:25 PM
 */
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000
#define BOTON PORTBbits.RB0     // Asignamos un alias a RB0
#define BOTON1 PORTBbits.RB1
#define FLAG_SPI 0xFF
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor m ximo de entrada del potenciometro?
#define OUT_MIN 0               // Valor minimo de ancho de pulso de se al PWM?
#define OUT_MAX 503             // Valor m ximo de ancho de pulso de se al PWM??
/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t cont = 0;           // Contador que env a el maestro al esclavo?
uint8_t valores;
char cont_master = 0;
char cont_slave = 0xFF;
char val_temporal = 0;
unsigned short CCPR = 0;      

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PORTAbits.RA0){
        if(PIR1bits.SSPIF){             //  Recibi  datos el esclavo???
            
            SSPBUF = valores;        // Cargamos contador del esclavo al buffer
            if (val_temporal != FLAG_SPI){  // Es env�o solo para generar los pulsos de reloj?
                PORTD = valores;
                SSPBUF = valores;        // Cargamos contador del esclavo al buffer
            }
            
        PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupci n?

        }
        if(INTCONbits.RBIF){        // Fue interrupci�n del PORTB
            if (!BOTON) {            // Verificamos si fue RB0 quien gener� la interrupci�n
                valores++;            // Incremento del PORTC
                INTCONbits.RBIF = 0;}    // Limpiamos bandera de interrupci�n
        
            if (!BOTON1){             // Verificamos si fue RB0 quien gener� la interrupci�n
                valores--;            // Incremento del PORTC
                INTCONbits.RBIF = 0;}    // Limpiamos bandera de interrupci�n    
        }
    }
    
    else{
         if(ADCON0bits.CHS == 0){
            CCPR = map(SSPBUF, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de anchode pulso
            CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
         }
    }    
    return;
}

void main(void) {
    setup();
    while(1){   

    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0b00000010; //AN0 Y AN1 ENTRADA ANALOGICA
    ANSELH = 0;                 // I/O digitales
    
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    TRISA = 0b00100011;         // SS y RA0 como entradas
    PORTA = 0;
    
    TRISD = 0;
    PORTD = 0;
    TRISB = 0;
    PORTB = 0;
    valores = 0;
    if(PORTAbits.RA0){
        TRISC = 0b00011000; // -> SDI y SCK entradas, SD0 como salida
        PORTC = 0;
        TRISE = 0;
        PORTE = 0;
        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0100;   // -> SPI Esclavo, SS hablitado
        SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
        SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
        SSPSTATbits.SMP = 0;        // -> Dato al final del pulso de reloj
        
        TRISBbits.TRISB0 = 1;       // RB0 como entrada (configurada con bits de control)
        TRISBbits.TRISB1 = 1; 
        OPTION_REGbits.nRBPU = 0;   // Habilitamos resistencias de pull-up del PORTB
        WPUBbits.WPUB0 = 1;         // Habilitamos resistencia de pull-up de RB0
        WPUBbits.WPUB1 = 1;

        
        INTCONbits.RBIE = 1;        // Habilitamos interrupciones del PORTB
        IOCBbits.IOCB0 = 1;         // Habilitamos interrupci�n por cambio de estado para RB0
        IOCBbits.IOCB1 = 1;
        INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupci�n

        
        PIR1bits.SSPIF = 0;         // Limpiamos bandera de SPI
        PIE1bits.SSPIE = 1;         // Habilitamos int. de SPI
        INTCONbits.PEIE = 1;
        INTCONbits.GIE = 1;
    }
    // Configs del esclavo
    else{
        
        TRISC = 0b00011000; // -> SDI y SCK entradas, SD0 como salida
        PORTC = 0;
      
        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0100;   // -> SPI Esclavo, SS hablitado
        SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
        SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
        SSPSTATbits.SMP = 0;        // -> Dato al final del pulso de reloj
        
        __delay_us(100);             // Sample time
        // Configuraci n PWM?
        TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
        PR2 = 0b01111100;                  // periodo de 2ms
    
        // Configuraci n CCP?
        CCP1CON = 0;                // Apagamos CCP1
        CCP1CONbits.P1M = 0;        // Modo single output
        CCP1CONbits.CCP1M = 0b1100; // PWM
    
        CCPR1L = 125>>2;
        CCP1CONbits.DC1B = 125 & 0b11;
        
        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
        T2CON = 0b00000101;
        while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 
        TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
        
        
        PIR1bits.SSPIF = 0;         // Limpiamos bandera de SPI
        PIE1bits.SSPIE = 1;         // Habilitamos int. de SPI
        INTCONbits.PEIE = 1;
        INTCONbits.GIE = 1;
    }
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
    