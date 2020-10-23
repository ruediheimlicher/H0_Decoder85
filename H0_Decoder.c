//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//


/*
//#include <avr/io.h>


//#include <avr/pgmspace.h>

//#include <avr/eeprom.h>
//#include <inttypes.h>

//#include <stdint.h>

//#include "lcd.c"

//#include "adc.c"
*/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "defines.h"
//***********************************
						
uint8_t  LOK_ADRESSE = 0xCC; //	11001100	Trinär
//									
//***********************************

/*
 commands
 LO     0x0202  // 0000001000000010
 OPEN   0x02FE  // 0000001011111110
 HI     0xFEFE  // 1111111011111110
 */


//#define OUTPORT	PORTD		// Ausgang fuer Motor

//#define INPORT   PORTD  // Input signal auf INT0
//#define INPIN   PIND  // Input signal

#define DATAPIN  2 



//volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[buffer_size];



volatile uint8_t	INT0status=0x00;				
volatile uint8_t	signalstatus=0x00; // status TRIT
volatile uint8_t  pausestatus=0x00;


volatile uint8_t   address=0x00; 
volatile uint8_t   data=0x00;   





volatile uint16_t	HIimpulsdauer=00;			//	Dauer des LOimpulsdaueres Definitiv
volatile uint8_t	HIimpulsdauerPuffer=22;		//	Puffer fuer HIimpulsdauer
volatile uint8_t	HIimpulsdauerSpeicher=0;		//	Speicher  fuer HIimpulsdauer

volatile uint8_t   LOimpulsdauerOK=0;   

volatile uint16_t   pausecounter = 0; //  neue ädaten detektieren
volatile uint16_t   abstandcounter = 0; // zweites Paket detektieren

volatile uint8_t   tritposition = 0; // nummer des trit im Paket
volatile uint8_t   lokadresse = 0;

volatile uint8_t   lokadresseA = 0;
volatile uint8_t   lokadresseB = 0;

volatile uint8_t   deflokadresse = 0;
volatile uint8_t   lokstatus=0x00; // Funktion, Richtung

volatile uint8_t   oldlokdata = 0;
volatile uint8_t   lokdata = 0;
volatile uint8_t   deflokdata = 0;

//volatile uint16_t   startdelaycounter = 0; // 
//volatile uint16_t   newlokdata = 0;

//volatile uint16_t   blinkWait = 0x2FFF; 
//volatile uint16_t   blinkOK = 0x1FFF; 

volatile uint16_t   rawdataA = 0;
volatile uint16_t   rawdataB = 0;
//volatile uint32_t   oldrawdata = 0;

volatile uint8_t   speed = 0;

volatile uint8_t   oldfunktion = 0;
volatile uint8_t   funktion = 0;
volatile uint8_t   deffunktion = 0;
volatile uint8_t   waitcounter = 0;
volatile uint8_t   richtungcounter = 0; // delay fuer Richtungsimpuls

//volatile uint8_t	Potwert=45;
			//	Zaehler fuer richtige Impulsdauer
//uint8_t				Servoposition[]={23,33,42,50,60};
// Richtung invertiert
//volatile uint8_t				Servoposition[]={60,50,42,33,23};

volatile uint16_t	taktimpuls=0;

volatile uint16_t   motorPWM=0;

volatile uint8_t   wdtcounter = 0;







void slaveinit(void)
{
 	OSZIPORT |= (1<<OSZIA);	//Pin 6 von PORT D als Ausgang fuer OSZI A
	OSZIDDR |= (1<<OSZIA);	//Pin 7 von PORT D als Ausgang fuer SOSZI B

   
	LOOPLEDPORT |=(1<<LOOPLED);
   LOOPLEDDDR |=(1<<LOOPLED); // HI

   MOTORDDR |= (1<<MOTOROUT);  // Motor PWM   
   MOTORPORT &= ~(1<<MOTOROUT); // LO

   MOTORDDR |= (1<<MOTORDIR);  // Motor PWM
   MOTORPORT &= ~(1<<MOTORDIR); // LO

   MOTORDDR |= (1<<LAMPE);  // Motor PWM
   MOTORPORT &= ~(1<<LAMPE); // LO

  
}




void int0_init(void)
{
   MCUCR = (1<<ISC00 | (1<<ISC01)); // raise int0 on rising edge
   GIMSK |= (1<<INT0); // enable external int0
   INT0status |= (1<<INT0_RISING);
   INT0status = 0;
   
}


/*
void timer0 (void) 
{ 
// Timer fuer Exp
TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
//	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	
//   TCCR0 |= (1<<CS00); // no prescaler
//Timer fuer 	
   
//	TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64 Intervall 64 us
	
	TIFR |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<TOIE0);			//Overflow Interrupt aktivieren
	TCNT0 = 0x00;					//Rücksetzen des Timers
  
}
*/
/*
ISR(TIMER0_COMP_vect) 
{
   
}
*/
void timer2 (uint8_t wert) 
{ 
   // set up timer with prescaler = 1 and CTC mode
   TCCR0A |= (1<<WGM01);
   TCCR0B |= (1<<CS01);
   
   // initialize counter
   TCNT0 = 0;
   
   // initialize compare value
   OCR0A = wert; //
   
   // clear interrupt flag as a precaution
   TIFR |= 0x01;   
   
   // enable compare interrupt
   TIMSK |= (1 << OCIE0A);
   
   // enable global interrupts
   sei();
} 
ISR(INT0_vect) 
{
   //OSZIATOG;
   if (INT0status == 0) // neue Daten beginnen
   {
      INT0status |= (1<<INT0_START);
      INT0status |= (1<<INT0_WAIT); // delay, um Wert des Eingangs zum richtigen Zeitpunkt zu messen
      
      INT0status |= (1<<INT0_PAKET_A); // erstes Paket lesen
      //OSZIPORT &= ~(1<<PAKETA); 
      //TESTPORT &= ~(1<<TEST2);
  //    OSZIALO; 
 //     OSZIBLO;
      
      
      pausecounter = 0; // pausen detektieren, reset fuer jedes HI
      abstandcounter = 0;// zweites Paket detektieren, 
      
      waitcounter = 0;
      tritposition = 0;
      lokadresse = 0;
      lokdata = 0;
      funktion = 0;
      
      HIimpulsdauer = 0;
 //     OSZIAHI;
   } 
   
   else // Data im Gang, neuer Interrupt
   {
      INT0status |= (1<<INT0_WAIT);
      
      pausecounter = 0;
      abstandcounter = 0; 
      waitcounter = 0;
 //     OSZIALO;
      //   INT0status |= (1<<INT0_RISING); // wait for next rise
      //    MCUCR = (1<<ISC00) |(1<<ISC01); // raise int0 on rising edge
   }
}

#pragma mark ISR Timer2
ISR(TIMER0_COMPA_vect) // Schaltet Impuls an MOTOROUT LO
{
   //MOTORPORT ^= (1<<MOTOROUT);return;
   
   motorPWM++;
   if (motorPWM > speed)
   {
      //  MOTORPORT &= ~(1<<MOTOROUT); // active LO
      MOTORPORT |= (1<<MOTOROUT); // active HI
   }
  // if (motorPWM >= 0xFF)
   if (motorPWM >= 14*SPEEDFAKTOR)
   {
      motorPWM = 0;
      //    MOTORPORT |= (1<<MOTOROUT);
      {
         MOTORPORT &= ~(1<<MOTOROUT);
      }
   }
#pragma mark TIMER0 INT0
   if (INT0status & (1<<INT0_WAIT))
   {
      waitcounter++;
      if (waitcounter > 2)
      {
         INT0status &= ~(1<<INT0_WAIT);
         if (INT0status & (1<<INT0_PAKET_A))
         {
            /*
             if (richtungcounter> 1)
             {
             richtungcounter--;
             }
             else if (richtungcounter == 1)
             {
             //lokstatus |= (1<<OLDRICHTUNGBIT);
             // lokstatus &= ~(1<<RICHTUNGBIT);
             
             richtungcounter = 0;
             }
             */
            //             TESTPORT &= ~(1<<TEST1);
            if (tritposition < 8) // Adresse)
            {
               
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  lokadresseA |= (1<<tritposition); // bit ist 1
               }
               else // 
               {
                  lokadresseA &= ~(1<<tritposition); // bit ist 0
               }
               //tritposition ++;
            }
            else
            {
               
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawdataA |= (1<<(tritposition-8)); // bit ist 1
               }
               else // 
               {
                  rawdataA &= ~(1<<(tritposition-8)); // bit ist 0
               }
            }
            //TESTPORT |= (1<<TEST1);
            
         }
         
         if (INT0status & (1<<INT0_PAKET_B))
         {
            //            TESTPORT &= ~(1<<TEST2);
            if (tritposition < 8) // Adresse)
            {
               
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  lokadresseB |= (1<<tritposition); // bit ist 1
               }
               else // 
               {
                  lokadresseB &= ~(1<<tritposition); // bit ist 0
               }
               //tritposition ++;
            }
            else
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawdataB |= (1<<tritposition-8); // bit ist 1
                  if ((tritposition > 9) )
                  {
                     //                  lokdata |= (1<<((tritposition - 10))); // bit ist 1
                  }
               }
               else 
               {
                  rawdataB &= ~(1<<tritposition-8); // bit ist 0
                  if ((tritposition > 9) )
                  {
                     //                  lokdata &= ~(1<<((tritposition - 10))); // bit ist 1
                  }
               }
            }
            
            if (!(lokadresseB == LOK_ADRESSE))
            {
               
            }
         }
         
         
         if (INT0status & (1<<INT0_PAKET_B))
         {
            //           TESTPORT |= (1<<TEST2);
         }
         if (INT0status & (1<<INT0_PAKET_A))
         {
            //           TESTPORT |= (1<<TEST1);
         }
         
         if (tritposition < 17)
         {
            tritposition ++;
         }
         else // Paket gelesen
         {
            // Paket A?
            if (INT0status & (1<<INT0_PAKET_A)) // erstes Paket, Werte speichern
            {
               //oldrawdata = rawdataA;
               //rawdataA = 0;
               //STATUSPORT ^= (1<<FUNKTIONOK);
               oldfunktion = funktion;
               //funktion = 0;
               
               INT0status &= ~(1<<INT0_PAKET_A); // Bit fuer erstes Paket weg
               //              OSZIPORT |= (1<<PAKETA); 
               
               
               INT0status |= (1<<INT0_PAKET_B); // Bit fuer zweites Paket setzen
               //              OSZIPORT &= ~(1<<PAKETB);   
               tritposition = 0;
               
            }
            
            else if (INT0status & (1<<INT0_PAKET_B)) // zweites Paket, Werte testen
            {
               
               if (INT0status & (1<<INT0_PAKET_B))
               {
                  //         TESTPORT &= ~(1<<TEST2);
               }
               
#pragma mark EQUAL
               if ((rawdataA == rawdataB) && (lokadresseA == lokadresseB)) // Lokadresse und Data OK
               {
                  if (lokadresseB == LOK_ADRESSE)
                  {
                     // Daten uebernehmen
                     //   STATUSPORT |= (1<<DATAOK); // LED ON
                     //  STATUSPORT |= (1<<ADDRESSOK); // LED ON
                     //deflokadresse = rawdataA & 0xFF;
                     
                     lokstatus |= (1<<ADDRESSBIT);
                     deflokadresse = lokadresseB;
                     deffunktion = (rawdataB & 0x03); // bit 0,1
                     if (deffunktion)
                     {
                        lokstatus |= (1<<FUNKTIONBIT);
                        MOTORPORT |= (1<<LAMPE);
                     }
                     else
                     {
                        lokstatus &= ~(1<<FUNKTIONBIT);
                        MOTORPORT &= ~(1<<LAMPE);
                     }
                     for (uint8_t i=0;i<8;i++)
                     {
                        if ((rawdataB & (1<<(2+i))))
                        {
                           deflokdata |= (1<<i);
                        }
                        else 
                        {
                           deflokdata &= ~(1<<i);
                        }
                     }
                     
                     // Richtung
                     if (deflokdata == 0x03) // Wert 1, Richtung togglen
                     {
                        if (!(lokstatus & (1<<RICHTUNGBIT)))
                        {
                           lokstatus |= (1<<RICHTUNGBIT);
                           richtungcounter = 0xFF;
                           //motorPWM = 0;
                           speed = 0;
                           MOTORPORT ^= (1<<MOTORDIR); // Richtung umpolen
                           
                        }
                     }
                     else 
                     {  
                        lokstatus &= ~(1<<RICHTUNGBIT); 
                        if (deflokdata == 0)
                        {
                           speed = 0;
                        }
                        else 
                        {
                           
                           switch (deflokdata)
                           {
                              case 0x0C:
                                 speed = 1;
                                 break;
                              case 0x0F:
                                 speed = 2;
                                 break;
                              case 0x30:
                                 speed = 3;
                                 break;
                              case 0x33:
                                 speed = 4;
                                 break;
                              case 0x3C:
                                 speed = 5;
                                 break;
                              case 0x3F:
                                 speed = 6;
                                 break;
                              case 0xC0:
                                 speed = 7;
                                 break;
                              case 0xC3:
                                 speed = 8;
                                 break;
                              case 0xCC:
                                 speed = 9;
                                 break;
                              case 0xCF:
                                 speed = 10;
                                 break;
                              case 0xF0:
                                 speed = 11;
                                 break;
                              case 0xF3:
                                 speed = 12;
                                 break;
                              case 0xFC:
                                 speed = 13;
                                 break;
                              case 0xFF:
                                 speed = 14;
                                 break;
                                 
                           }
                           //  speed = 10;
                           speed *= SPEEDFAKTOR;
                           
                        } // 
                     }
                     
                     // rawdataA = 0;
                  }
                  else 
                  {
                     // aussteigen
                     deflokdata = 0xCA;
                     INT0status == 0;
                     return;
                     
                  }
                  
               }
               else 
               {
                  lokstatus &= ~(1<<ADDRESSBIT);
                  // aussteigen
                  deflokdata = 0xCA;
                  INT0status == 0;
                  return;
                  
               }
               
               //      INT0status &= ~(1<<INT0_PAKET_B); // Bit fuer zweites Paket weg
               INT0status |= (1<<INT0_END);
               //     OSZIPORT |= (1<<PAKETB);
               if (INT0status & (1<<INT0_PAKET_B))
               {
                  //               TESTPORT |= (1<<TEST2);
               }
               //tritposition = 0;
            } // End Paket B
            
            
            
            //INT0status == 0;
         }
         
      } // waitcounter > 2
   } // if INT0_WAIT
   
   if (INPIN & (1<<DATAPIN)) // Pin HI, input   im Gang
   {
      HIimpulsdauer++; // zaehlen
   }
   else  // LO, input fertig, Bilanz
   {
      if (abstandcounter < 20)
      {
         abstandcounter++;
      }
      else //if (abstandcounter ) // Paket 2
      {
         abstandcounter = 0;
         //   OSZIAHI;
         // tritposition = 0;
         
         
         //      INT0status &= ~(1<<INT0_PAKET_A); // Bit fuer erstes Paket weg
         //      OSZIPORT |= (1<<PAKETA); 
         
         
         //     INT0status |= (1<<INT0_PAKET_B); // Bit fuer zweites Paket setzen
         //    OSZIPORT &= ~(1<<PAKETB);   
      }
      
      if (pausecounter < 120)
      {
         pausecounter ++; // pausencounter incrementieren
      }
      else 
      {
         //OSZIBHI; //pause detektiert
         pausecounter = 0;
         INT0status = 0; //Neue Daten abwarten
         return;
      }
      
    } // input LO
}
/*
ISR(WDT_vect)
{
      WDTCR |= 1<<WDIE; // re-enable the watch dog interrupt
      wdt_reset(); // also reset in WDT ISR
   
}
*/

void main (void) 
{
   //WDT ausschalte 
   MCUSR = 0;
   wdt_disable();

   slaveinit();
   /*
   for (uint8_t i=0;i<4;i++)
   {
      LOOPLEDPORT |=(1<<LOOPLED);
      _delay_ms(100);
      LOOPLEDPORT &=~(1<<LOOPLED);
      _delay_ms(100);
   }
    */
   int0_init();
   
   timer2(4);
   uint16_t loopcount0=0;
   //_delay_ms(2);
   oldfunktion = 0x03; // 0x02
   oldlokdata = 0xCC; // 
   
   // WDT
   // https://bigdanzblog.wordpress.com/2015/07/20/resetting-rebooting-attiny85-with-watchdog-timer-wdt/
  /* 
   WDTCR|=(1<<WDCE)|(1<<WDE);  // https://www.instructables.com/ATtiny85-Watchdog-reboot-Together-With-SLEEP-Andor/
   WDTCR=0x00; // disable watchdog
   
   // #define WDTO_15MS   0

   WDTCR = 0xD8 | WDTO_15MS;
  */ 
   wdt_enable(WDTO_15MS);  // Set watchdog timeout to 15 milliseconds
   
   wdt_reset();
   
   sei();
   
   
   while (1)
   {	
      // loop: 40 us, takt 85us, mit if-teil 160 us
     // LOOPLEDPORT &= ~(1<<LOOPLED);
      wdt_reset();
      //Blinkanzeige
      
      loopcount0++;
      if (loopcount0==0x1FFF)
      {
         
         loopcount0=0;
         //LOOPLEDPORT |=(1<<LOOPLED);
         LOOPLEDPORT ^= (1<<LOOPLED);
         /*
         wdtcounter++;
         if (wdtcounter > 2)
         {
           // _delay_ms(1);
         }
         */
         //LOOPLEDPORT &= ~(1<<LOOPLED);
      }

      
     // LOOPLEDPORT |=(1<<LOOPLED);
   }//while
   
   
   // return 0;
}
