/*
//* Authors: Manal Khalili and Masa Issa
*/
#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdio.h>
#include "my_ser.h"
#include "my_adc.h"
#include "lcd_x8.h"

#define STARTVALUE  1214
short secminhor = -1;
short cnt = 0;
char ReceivedChar;
unsigned char* key = "";
unsigned short cnt2=-1;
int commandStart=0;
int readMode=0;
int readD=0;
int readCooler=0;
int readHeater=0;
int readTime=0;
int readA0=0;
int readA1=0;
int readA2=0;
int writeMode=0;
int writeD0=0;
int writeD1=0;
int writeD2=0;
int writeD3=0;
int writeD4=0;
int writeD5=0;
int writeD6=0;
int writeD7=0;
int readAnalog=0;
float temp ;
float tempf;
int ft=0;
int hours=0, minutes=0, seconds=0;
int mode=0; //int1
int mode1=0; //setup int2
int heater_flag=0;
int cooler_flag=0;
int count=2; 
char x;
int y; 
char ti[6]; 
int h, m,s; 
int tmp;//temporary 
int clock_mode=0;
void reloadTimer0(void)
{  
    TMR0H = (unsigned char) ((STARTVALUE >>  8) & 0x00FF);
    TMR0L =  (unsigned char)(STARTVALUE & 0x00FF );   
}

void checkTimeValidation(void) {
    if (seconds >= 60) {
        minutes++;
        seconds = 0;
    }
    if (minutes >= 60) {
        hours++;
        minutes = 0;
    }
    if (hours >= 23) {
        hours = 0;
    }


}

void Timer0_isr(void)
{
    INTCONbits.TMR0IF=0;
    ft=1;
    reloadTimer0();  
}

void EXT_Int2_isr(void)
{
    INTCON3bits.INT2IF=0;
    if(mode==1){
        if(mode1==2){
            mode1=0; 
        }
        else{
            mode1+=1;
        }
     }
}

void EXT_Int1_isr(void)
{
    INTCON3bits.INT1IF=0;
    if(mode==1)
    {
        mode=0;
    }
    else{
        mode++;   
    }
}
void __interrupt(high_priority)highIsr(void)
//void interrupt high_priority highIsr(void)
{
    CLRWDT();
    if (INTCONbits.TMR0IF) Timer0_isr();
   // if (PIR1bits.RCIF) RX_isr();
    if (INTCON3bits.INT1F) EXT_Int1_isr();
    if (INTCON3bits.INT2F) EXT_Int2_isr();
    
   
    
    
}
void delay_ms(unsigned int n)
{
    int i;
    for (i=0; i < n; i++){
         __delaywdt_ms(1) ; 
    }
}

void init_int(void){
  
          INTCON = 0; // disable interrupts first, then enable the ones i want
    RCONbits.IPEN =0;
    INTCONbits.INT0IE = 1;
    INTCONbits.TMR0IE=1;
    INTCON2 = 0;
    INTCON3 = 0;
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT2IE = 1;
    INTCON2bits.INTEDG1 = 1;
    INTCON2bits.INTEDG0= 1;
    T0CON=0X80;
    PIE1 = 0;
    PIR1 = 0;
    IPR1 = 0;
    PIE2 = 0;
    PIE2 = 0;
    PIR2 = 0;
    IPR2 = 0;
    INTCONbits.GIEH = 1;  // enable global interrupt bits
    INTCONbits.GIEL = 1;
    RCONbits.IPEN = 0;
    T0CON=0b10000011;
    INTCON=0b11100000;
    INTCON2=0;
    PIE1bits.RC1IE=1;
    INTCON3=0b00011000;      
    INTCONbits.INT0E=1;
    
}


void setupPorts(void)
{
    
    ADCON1 =0b00001100; //3 analog input
    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x80; // RX input , others output
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE= 0x00;  // All outputs  
     LATA = LATB = LATC = LATD = LATE = 0;
}
void display(void){
    char LCD[64];
    char LCDP[64];
    char NOR[64];
    char H[64];
    char M[64];
    char S[64];
    unsigned char Cl, Ht;
    
    //to display time end tmp 
    lcd_gotoxy(1, 1);
    sprintf(LCD, "%d ", hours);
    lcd_puts(LCD);
    lcd_gotoxy(3, 1);
    sprintf(LCD, ":%d ", minutes);
    lcd_puts(LCD);
    lcd_gotoxy(7, 1);
    sprintf(LCD, ":%d ", seconds);
    lcd_puts(LCD);
    
    lcd_gotoxy(10, 1);
    sprintf(LCD, "%6.2fC", temp);
    lcd_puts(LCD);
    
   //line 2 to display on or off for heater and cooler
    lcd_gotoxy(1, 2);
    if(PORTCbits.RC2 ==1){
    sprintf(LCD, "C:ON ");
    lcd_puts(LCD);
    }
    else{
       sprintf(LCD, "C:OFF ");
    lcd_puts(LCD);
    }
     
    lcd_gotoxy(8, 2);
    if(PORTCbits.RC5 ==1){
    sprintf(LCD, "H:ON ");
    lcd_puts(LCD);
    }
    else{
       sprintf(LCD, "H:OFF ");
    lcd_puts(LCD);
    }
    
    //to display digital clk modes  
    switch(mode){
        case 0: //normal mode 
             lcd_gotoxy(1, 3);
            sprintf(NOR, "NORMAL          " );
            lcd_puts(NOR);
            break;
        case 1: //setup mode 
            if(mode1==0){
                 lcd_gotoxy(1, 3);
               sprintf(S, "SETUP  seconds"  ); 
               lcd_puts(S);
            }
            else if(mode1==1){
                 lcd_gotoxy(1, 3);
                 sprintf(M, "SETUP  minutes"  ); 
                  lcd_puts(M);
            }
            else{
                 lcd_gotoxy(1, 3);
               sprintf(H, "SETUP  hours       "  );  
                lcd_puts(H);
            }
            break;
    } 
    //to display our first names
    lcd_gotoxy(1, 4);
    sprintf(LCD, "Manal");
    lcd_puts(LCD);
    lcd_gotoxy(8, 4);
    sprintf(LCD, "Masa");
    lcd_puts(LCD);
}
//inc or dec digital clk
void normal(void){
    seconds++;
        if(seconds==60 && minutes!=60){
            seconds=0; 
            minutes+=1; 
        }
        if(minutes==60 && hours!=24){
            minutes=0; 
            hours+=1; 
        }
        if(hours==24){
            hours=0;
            //seconds+=1; 
        }
}

void setup_inc(void){
    if(mode1==0){  
        if(seconds == 59 && minutes ==59 && hours==23){
         seconds=0; 
         minutes=0; 
         hours=0;
        }
        else if(seconds==59 && minutes ==59 && hours !=23){
            seconds=0; 
            minutes=0;
            hours+=1;
        }
        else if(seconds==59 && minutes !=59 && hours !=23){
            seconds=0; 
            minutes+=1;
        }
        
        else{
           seconds+=1;  
        }   
    }
    else if(mode1==1){
        if(minutes==59 && hours!=23){
            minutes=0; 
            hours+=1; 
        }
        else if(minutes == 59 && hours==23){
           minutes=0; 
           hours=0; 
        }
        else{
               minutes+=1;    
        }   
    }
    else{
        if(hours==23){
            hours=0; 
        }
        else{
           hours+=1; 
        } 
    }
}

void setup_dec(void){
    if(mode1==0){
        if(seconds==0 && minutes==0 && hours==0){
           seconds=59;
           minutes=59;
           hours=23;
        }
        else if(seconds==0 && minutes!=0 && hours!=0){
            seconds=59; 
            minutes-=1;
        }
        else if(seconds==0 && minutes==0 && hours!=0){
            seconds=59;
            minutes=59;
            hours-=1; 
        }
        else{
           seconds-=1;  
        }   
    }
    else if(mode1==1){
        if(minutes==0 && hours !=0){
            minutes=59; 
            hours-=1; 
        }
        else if(minutes==0 && hours==0){
            hours=23; 
            minutes=59; 
        } 
        else{
            minutes-=1;
        }
    }
    else{
        if(hours!=0){
           hours-=1;  
        }
        else{
            hours=23;
        }
    }
}


void reloadIT(void) {
    commandStart = 0;
    cnt2 = -1;
    readMode = 0;
    readD = 0;
    readHeater = 0;
    readCooler = 0;
    readTime = 0;
    readA0 = 0;
    readA1 = 0;
    readA2 = 0;
    writeMode = 0;
    writeD0 = 0;
    writeD1 = 0;
    writeD2 = 0;
    writeD3 = 0;
    writeD4 = 0;
    writeD5 = 0;
    writeD6 = 0;
    writeD7 = 0;
    readAnalog = 0;
}

void RX_isr(void) {
    CLRWDT();
    ReceivedChar = RCREG;
    ++cnt2;
    if (ReceivedChar == '<') {
        commandStart = 1;
    }
    if (commandStart != 1 && cnt2 > 1) {
        reloadIT();
        PORTDbits.RD3 = !PORTDbits.RD3;
    }
    if (cnt2 == 1) {

        if (commandStart == 1 && ReceivedChar == 'R') {

            readMode = 1;


        } else if (commandStart && ReceivedChar == 'W') {
            readMode = 2;
        }

    }
    if (cnt2 == 2 && readMode == 1) {
        if (ReceivedChar == 't') {
            readTime = 1;
        } else if (ReceivedChar == 'A') {
            readAnalog = 1;
        } else if (ReceivedChar == 'H') {
            readHeater = 1;
        } else if (ReceivedChar == 'C') {
            readCooler = 1;
        } else if (ReceivedChar == 'D') {
            readD = 1;
        } else if (ReceivedChar == 't') {
            readTime = 1;
        }

    } else if (readMode == 2 && cnt2 == 2) {
        if (ReceivedChar == 'H') {
            readHeater = 2;
        } else if (ReceivedChar == 'C') {
            readCooler = 2;
        } else if (ReceivedChar == 'D') {
            readD = 2;
        } else if (ReceivedChar == 't') {
            readTime = 2;
        } else if (ReceivedChar == '0') {
            writeD0 = 1;
        } else if (ReceivedChar == '1') {
            writeD1 = 1;
        } else if (ReceivedChar == '2') {
            writeD2 = 1;
        } else if (ReceivedChar == '3') {
            writeD3 = 1;
        } else if (ReceivedChar == '4') {
            writeD4 = 1;
        } else if (ReceivedChar == '5') {
            writeD5 = 1;
        } else if (ReceivedChar == '6') {
            writeD6 = 1;
        } else if (ReceivedChar == '7') {
            writeD7 = 1;
        }
    }
    if (cnt2 == 3) {
        if (readAnalog == 1 && readMode == 1) {

            if (ReceivedChar == '0') {
                readA0 = 1;
                PORTDbits.RD3 = !PORTDbits.RD3;
            } else if (ReceivedChar == '1')readA1 = 1;
            else if (ReceivedChar == '2')readA2 = 1;
        } else if (readMode == 2) {
            if (ReceivedChar != 'V') {
                ReceivedChar = 'V';
                cnt2 = 4;
            }
        }


    }
    if (cnt2 == 4 && readMode == 2 && ReceivedChar != '>') {
        if (ReceivedChar == '0') {
            writeMode = 0;

        }
        if (ReceivedChar == '1') {
            writeMode = 1;
        }
    }

}


void main(void) {
    
 setupPorts();
    init_adc_no_lib();
     char *Cooler;
        char *Heater;
    setupSerial();
    lcd_init();
    setupPorts();
    lcd_putc('\f');
    init_int();
    lcd_send_byte(0, 1); 
    PORTDbits.RD0 = 1;
    PORTDbits.RD1 = 1;
    float AN[3]; // To store the voltages of AN0, AN1, AN2
    int raw_val;
    unsigned char channel;
    float Tempreture;
    char Buffer[30];
    short j = 0;
     char command[32];
     
    while(1)
    {        
       CLRWDT();  
       PORTCbits.RC2=1;
       PORTCbits.RC5=1;
        tempf =read_adc_voltage(2);
        temp=tempf*100.0;
        
  
    
        //for digital clk inc dec or normal
        if(mode==0){ //normal
            delay_ms(150);//toggle every second  
            normal();   
        }
        
         if(mode==1){ //setup
            while(PORTBbits.RB3 ==0){
                delay_ms(300);
                setup_inc();
            }
            while(PORTBbits.RB4 ==0){
                delay_ms(300);
                setup_dec();
            } 
            
        }
          display();
           char command[32];
           
        if (readTime == 1) {
            send_string_no_lib((unsigned char *) "TIME");
            sprintf(command, " %2d:%2d:%2d HH \n", hours, minutes, seconds);
            send_string_no_lib(command);
            reloadIT();
        } else if (readCooler == 1) {
            send_string_no_lib((unsigned char *) "CoolerState: ");

            sprintf(command, "%3s \n", Cooler);
            send_string_no_lib(command);
            reloadIT();
        } else if (readHeater == 1) {
            send_string_no_lib((unsigned char *) "HeaterState: ");
            sprintf(command, "%3s \n", Heater);
            send_string_no_lib(command);
            reloadIT();

        } else if (readA0 == 1) {
            send_string_no_lib((unsigned char *) "A0 Value : ");
            sprintf(command, "%4.2f \n", AN[0]);
            send_string_no_lib(command);
            reloadIT();
        } else if (readA1 == 1) {
            send_string_no_lib((unsigned char *) "A1 Value : ");
            sprintf(command, "%4.2f \n", AN[1]);
            send_string_no_lib(command);
            reloadIT();
        } else if (readA2 == 1) {
            send_string_no_lib((unsigned char *) "A2 Value : ");
            sprintf(command, "%4.2f \n", AN[2]);
            send_string_no_lib(command);
            reloadIT();
        } else if (readD == 1) {
            send_string_no_lib((unsigned char *) "PortD : ");
            sprintf(command, "%8d \n", LATD);
            send_string_no_lib(command);
            reloadIT();
        } else if (readHeater == 2) {
            Heater = writeMode;
            PORTCbits.RC5 = Heater;
            reloadIT();

        } else if (readCooler == 2) {
            Cooler = writeMode;
            PORTCbits.RC2 = Cooler;
            reloadIT();

        } else {
            reloadIT();
        }
    }
   
}

    
    



















































