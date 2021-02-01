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

#define LOK_TYP_DIESEL  1
#define LOK_TYP_RE44  2


//***********************************
						
uint8_t  LOK_ADRESSE = 0xF3; //	1011 binaer 11001111	Trinär
//									
//***********************************

uint8_t LOK_TYP = LOK_TYP_RE44;

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




volatile uint8_t   INT0status=0x00;            
volatile uint8_t   signalstatus=0x00; // status TRIT
volatile uint8_t   pausestatus=0x00;


volatile uint8_t   address=0x00; 
volatile uint8_t   data=0x00;   

//volatile uint16_t   HIimpulsdauer=0;         //   Dauer des LOimpulsdaueres Definitiv
volatile uint8_t   HIimpulsdauerPuffer=22;      //   Puffer fuer HIimpulsdauer
volatile uint8_t   HIimpulsdauerSpeicher=0;      //   Speicher  fuer HIimpulsdauer

volatile uint8_t   LOimpulsdauerOK=0;   

volatile uint8_t   pausecounter = 0; //  neue ädaten detektieren
volatile uint8_t   abstandcounter = 0; // zweites Paket detektieren

volatile uint8_t   tritposition = 0; // nummer des trit im Paket
//volatile uint8_t   lokadresse = 0;

volatile uint8_t   lokadresseA = 0;
volatile uint8_t   lokadresseB = 0;

volatile uint8_t   deflokadresse = 0;
volatile uint8_t   lokstatus=0x00; // Funktion, Richtung

volatile uint8_t   rawfunktionA = 0;
volatile uint8_t   rawfunktionB = 0;

volatile uint8_t   deffunktiondata = 0;

volatile uint8_t   oldlokdata = 0;
//volatile uint8_t   lokdata = 0;
volatile uint8_t   deflokdata = 0;

//volatile uint16_t   startdelaycounter = 0; // 
//volatile uint16_t   newlokdata = 0;

//volatile uint16_t   blinkWait = 0x2FFF; 
//volatile uint16_t   blinkOK = 0x1FFF; 

volatile uint8_t   rawdataA = 0;
volatile uint8_t   rawdataB = 0;
//volatile uint32_t   oldrawdata = 0;

volatile uint8_t   speed = 0;

volatile uint8_t   oldfunktion = 0;
volatile uint8_t   funktion = 0;
volatile uint8_t   deffunktion = 0;
volatile uint8_t   waitcounter = 0;
volatile uint8_t   richtungcounter = 0; // delay fuer Richtungsimpuls


volatile uint8_t   motorPWM=0;
volatile uint8_t   motorcounter=0;

volatile uint8_t   lampePWM=0;
volatile uint8_t   lampecounter=0;


volatile uint8_t   wdtcounter = 0;

volatile uint8_t   int0counter = 0; // detektiert datenfluss

// linear
//volatile uint8_t   speedlookup[15] = {0,18,36,54,72,90,108,126,144,162,180,198,216,234,252};


//volatile uint8_t   speedlookup[15] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140};
// linear 100
//volatile uint8_t   speedlookup[15] = {0,7,14,21,28,35,42,50,57,64,71,78,85,92,100};

//Diesel
// linear 80 
//volatile uint8_t   speedlookup[15] = {0,5,11,17,22,28,34,40,45,51,57,62,68,74,80};

// linear 250 mit offset 20
//volatile uint8_t   speedlookup[15] = {0,36,52,69,85,102,118,135,151,167,184,200,217,233,250};

// linear mit offset 30
//volatile uint8_t   speedlookup[15] = {0,33,37,40,44,47,51,55,58,62,65,69,72,76,80};

// linear mit offset 22
//volatile uint8_t   speedlookup[15] = {0,26,30,34,38,42,46,51,55,59,63,67,71,75,80};

// linear mit offset 20
//volatile uint8_t   speedlookup[15] = {0,24,28,32,37,41,45,50,54,58,62,67,71,75,80};

// Quadratisch  mit 250
//volatile uint8_t   speedlookup[15] = {0,21,24,30,38,49,62,77,95,115,137,161,188,218,250};
//volatile uint8_t   speedlookup[15] = {0,46,54,63,73,84,97,112,127,144,163,183,204,226,250,};

// Re44
//volatile uint8_t   speedlookup[15] = {0,59,64,71,80,90,101,115,129,146,163,183,204,226,250,};// 

// 10/ 80
//volatile uint8_t   speedlookup[15] = {0,12,14,17,20,24,28,33,38,44,50,57,64,72,80,};

// 12/100
volatile uint8_t   speedlookup[15] = {0,13,14,17,20,24,29,35,42,49,58,67,77,88,100};

volatile uint8_t   maxspeed =  252;

volatile uint8_t   lastDIR =  0;
uint8_t loopledtakt = 0x1F;


void slaveinit(void)
{
 	//OSZIPORT |= (1<<OSZIA);	//Pin 6 von PORT D als Ausgang fuer OSZI A
	//OSZIDDR |= (1<<OSZIA);	//Pin 7 von PORT D als Ausgang fuer SOSZI B


   //   LOOPLEDDDR |=(1<<LOOPLED); // HI
   //   LOOPLEDPORT |=(1<<LOOPLED);
 
   // in loop verschoben
//   MOTORDDR |= (1<<MOTOROUT);  // Output Motor PWM   
//   MOTORPORT |= (1<<MOTOROUT); // HI, Motor OFF
   
   //MOTORDDR &= ~(1<<2);
    MOTORDDR |= (1<<MOTORINT0); 
   MOTORPORT &= ~(1<<MOTORINT0); 
   
   MOTORDDR |= (1<<LAMPE);  // Lampe
   //MOTORPORT |= (1<<LAMPE); // HI
   switch (LOK_TYP)
   {
      case  LOK_TYP_DIESEL:
      {
         MOTORPORT |= (1<<LAMPE); // lampe ON HI
      }break;
      case  LOK_TYP_RE44:
      {
         //MOTORPORT &= ~(1<<LAMPE); // PWM in ISR
      }break;
      default:
      {
         MOTORPORT |= (1<<LAMPE);
      }break;
         
   }// switch lok_typ
   
   maxspeed =  252;//speedlookup[14];
   INT0status = 0;
}




void int0_init(void)
{
   MCUCR = (1<<ISC00 | (1<<ISC01)); // raise int0 on rising edge
   GIMSK |= (1<<INT0); // enable external int0
   INT0status |= (1<<INT0_RISING);
   INT0status = 0;
   INT0status |= (1<<INT0_WAIT);
   
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
   
 //  sei();
} 

#pragma mark INT0
ISR(INT0_vect) 
{
   //OSZIATOG;
   //MOTORPORT &= ~(1<<LAMPE); // 
   
   if (INT0status == 0) // neue Daten beginnen
   {
      //MOTORPORT &= ~(1<<LAMPE); // 6 us insgesamt
      INT0status |= (1<<INT0_RUN); // datenfluss im Gang
      int0counter = 0;
      
 //     INT0status |= (1<<INT0_START);
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
      deflokadresse = 0;
      deflokdata = 0;
      
      funktion = 0;
      // parameter resetten
      lokadresseA = 0;
      lokadresseB = 0;
 //     rawdataA = 0;
//      rawdataB = 0;
      
      rawfunktionA=0;
      rawfunktionB=0;
      deffunktiondata=0;
      
      //     HIimpulsdauer = 0;
      //     OSZIAHI;
   } 
   
   else // Data in Gang, neuer Interrupt
   {
      INT0status |= (1<<INT0_WAIT);
      
      pausecounter = 0;
      abstandcounter = 0; 
      waitcounter = 0;
      //     OSZIALO;
      //   INT0status |= (1<<INT0_RISING); // wait for next rise
      //    MCUCR = (1<<ISC00) |(1<<ISC01); // raise int0 on rising edge
   }
   //MOTORPORT |= (1<<LAMPE);
   
}

#pragma mark ISR Timer2
ISR(TIMER0_COMPA_vect) // Schaltet Impuls an MOTOROUT LO wenn speed > 0
{
   //OSZIATOG;
   if (speed)
   {
      motorcounter++;
      if (motorcounter > MOTORTEILER)
      {
         motorPWM++;
         motorcounter = 0;
      }
   }
   if ((motorPWM > speed) || (speed == 0)) // Impulszeit abgelaufen oder speed ist 0
   {
      MOTORPORT |= (1<<MOTOROUT); // OFF, Motor ist active LO
   }
   if ((motorPWM >= 254) && speed) //ON, neuer Motorimpuls
   {
      MOTORPORT &= ~(1<<MOTOROUT);
      motorPWM = 0;
      motorcounter = 0;
   }
   
    // Lampe PWM bei Re44
   if ((lokstatus & (1<<FUNKTIONBIT)) && (LOK_TYP == LOK_TYP_RE44)) // Lampe ist ON
   {
      
      lampePWM++;
      if (lampePWM > LAMPEMAX)
      {
         MOTORPORT |= (1<<LAMPE); // lampe OFF
      }
      if (lampePWM == 0) // nach 0xFF
      {
         MOTORPORT &= ~(1<<LAMPE); // Lampe wieder ON
         //lampePWM = 0;
      }
       
   } // if lampe ON


#pragma mark TIMER0 INT0_WAIT
   if (INT0status & (1<<INT0_WAIT))
   {
      
      waitcounter++;
      if (waitcounter > 2)
      {
         
         INT0status &= ~(1<<INT0_WAIT);
         if (INT0status & (1<<INT0_PAKET_A))
         {
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
            }
            else if (tritposition < 10)
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawfunktionA |= (1<<tritposition-8); // bit ist 1
               }
               else // 
               {
                  rawfunktionA &= ~(1<<tritposition-8); // bit ist 0
               }
               
            }
            
            else
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawdataA |= (1<<(tritposition-10)); // bit ist 1
               }
               else // 
               {
                  rawdataA &= ~(1<<(tritposition-10)); // bit ist 0
               }
            }
         }
         
         if (INT0status & (1<<INT0_PAKET_B))
         {
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
            }
            else if (tritposition < 10) // bit 8,9: funktion
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawfunktionB |= (1<<tritposition-8); // bit ist 1
               }
               else // 
               {
                  rawfunktionB &= ~(1<<tritposition-8); // bit ist 0
               }
               
            }
            
            
            else
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawdataB |= (1<<tritposition-10); // bit ist 1
               }
               else 
               {
                  rawdataB &= ~(1<<tritposition-10); // bit ist 0
               }
            }
            
            if (!(lokadresseB == LOK_ADRESSE))
            {
               
            }
         }
         
         // Paket anzeigen
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
               oldfunktion = funktion;
               
               INT0status &= ~(1<<INT0_PAKET_A); // Bit fuer erstes Paket weg
               INT0status |= (1<<INT0_PAKET_B); // Bit fuer zweites Paket setzen
               tritposition = 0;
            }
            else if (INT0status & (1<<INT0_PAKET_B)) // zweites Paket, Werte testen
            {
               
               
#pragma mark EQUAL
               if (lokadresseA && ((rawfunktionA == rawfunktionB) && (rawdataA == rawdataB) && (lokadresseA == lokadresseB))) // Lokadresse > 0 und Lokadresse und Data OK
               {
                  if (lokadresseB == LOK_ADRESSE) // Lok stimmt, Data vollständig
                  {
                     // Daten uebernehmen
                     //   STATUSPORT |= (1<<DATAOK); // LED ON
                     //  STATUSPORT |= (1<<ADDRESSOK); // LED ON
                     
                     lokstatus |= (1<<ADDRESSBIT);
                     deflokadresse = lokadresseB;
                     //deffunktion = (rawdataB & 0x03); // bit 0,1 funktion als eigene var
                     deffunktion = rawfunktionB;
                     uint8_t speedcode = 0;
                     
                     if (deffunktion)
                     {
                        lokstatus |= (1<<FUNKTIONBIT);
                         
                      
                        switch (LOK_TYP)
                        {
                           case  LOK_TYP_DIESEL:
                           {
                              MOTORPORT |= (1<<LAMPE);
                           }break;
                           case  LOK_TYP_RE44:
                           {
                              //MOTORPORT &= ~(1<<LAMPE); // PWM in ISR 
                           }break;
                           default:
                           {
                              MOTORPORT |= (1<<LAMPE);
                           }break;
                              
                        }// switch lok_typ
                       
                     }
                     else
                     {
                        lokstatus &= ~(1<<FUNKTIONBIT);
                        
                        switch (LOK_TYP)
                         {
                            case  LOK_TYP_DIESEL:
                            {
                               MOTORPORT &= ~(1<<LAMPE);
                            }break;
                            case  LOK_TYP_RE44:
                            {
                               //MOTORPORT |= (1<<LAMPE);// PWM in ISR 
                            }break;
                              
                            default:
                            {
                               MOTORPORT &= ~(1<<LAMPE);
                            }break;
                         }// switch lok_typ
                       
                     }
                     
                     
                     for (uint8_t i=0;i<8;i++)
                     {
                        //if ((rawdataB & (1<<(2+i))))
                        if ((rawdataB & (1<<i)))
                        {
                           deflokdata |= (1<<i);
                        }
                        else 
                        {
                           deflokdata &= ~(1<<i);
                        }
                     }
                     
                     // Richtung
                     if ((deflokdata == 0x03)) // Wert 1, Richtung togglen
                     {
                         if ((!(lokstatus & (1<<RICHTUNGBIT))) )
                        {
                           lokstatus |= (1<<RICHTUNGBIT);
                           speed = 0;
                           richtungcounter = 16;
                           MOTORPORT ^= (1<<MOTORDIR); // Richtung umpolen
                        }
                     }
                     else 
                     {  
                           if (richtungcounter--) // erneutes Umschalten verhindern
                            {
                               lokstatus &= ~(1<<RICHTUNGBIT);
                            }

                         
#pragma mark speed      
                                        
                        switch (deflokdata)
                        {
                           case 0:
                              speedcode = 0;
                              MOTORPORT |= (1<<MOTOROUT);
                              break;
                           case 0x0C:
                              speedcode = 1;
                              break;
                           case 0x0F:
                              speedcode = 2;
                              break;
                           case 0x30:
                              speedcode = 3;
                              break;
                           case 0x33:
                              speedcode = 4;
                              break;
                           case 0x3C:
                              speedcode = 5;
                              break;
                           case 0x3F:
                              speedcode = 6;
                              break;
                           case 0xC0:
                              speedcode = 7;
                              break;
                           case 0xC3:
                              speedcode = 8;
                              break;
                           case 0xCC:
                              speedcode = 9;
                              break;
                           case 0xCF:
                              speedcode = 10;
                              break;
                           case 0xF0:
                              speedcode = 11;
                              break;
                           case 0xF3:
                              speedcode = 12;
                              break;
                           case 0xFC:
                              speedcode = 13;
                              break;
                           case 0xFF:
                              speedcode = 14;
                              break;
                           default:
                              speedcode = 0;
                              MOTORPORT |= (1<<MOTOROUT);
                              break;
                              
                        }
                        speed = speedlookup[speedcode];
                     }
                  }
                  else // Lok stimmt nicht
                  {
                     // aussteigen
               //      deflokdata = 0xCA;
                     deflokdata = 0;
                     INT0status = 0;
                     return;
                  }
               }
               else // Lok stimmt nicht, Data unvollständig
               {
                  lokstatus &= ~(1<<ADDRESSBIT);
                  // aussteigen
             //     deflokdata = 0xCA;
                  deflokdata = 0;
                  INT0status = 0;
                  return;
                  
               }
               
               INT0status |= (1<<INT0_END); 
               //     OSZIPORT |= (1<<PAKETB);
               if (INT0status & (1<<INT0_PAKET_B))
               {
                  INT0status = 0;
                  //               TESTPORT |= (1<<TEST2);
               }
            } // End Paket B
         }
         
      } // waitcounter > 2
   } // if INT0_WAIT
   
   if (INPIN & (1<<DATAPIN)) // Pin HI, input   im Gang
   {
      //      HIimpulsdauer++; // zaehlen
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
         //     OSZIPORT |= (1<<PAKETA); 
         //    OSZIPORT &= ~(1<<PAKETB);   
      }
      
      if (pausecounter < 120)
      {
         pausecounter ++; // pausencounter incrementieren
      }
      else  //pause detektiert
      {
         //OSZIBHI;
         pausecounter = 0;
         INT0status = 0; //Neue Daten abwarten
         return;
      }
   } // input LO
}
/*
ISR(WDT_vect) // nicht verwendet
{
      WDTCR |= 1<<WDIE; // re-enable the watch dog interrupt
      wdt_reset(); // also reset in WDT ISR
}
*/

void main (void) 
{
   //WDT ausschalten 
   MCUSR = 0;
   wdt_disable();
  
   MOTORDDR &= ~(1<<MOTORAUX);  // Input, AUX, Sniffer fuer DIR nach reset
   MOTORDDR |= (1<<MOTORDIR);  // Output Motor DIR 
   
   
   if (MOTORPIN & (1<<MOTORAUX)) // AUX ist noch HI
   {
      lastDIR = 1;
      MOTORPORT |= (1<<MOTORDIR); 
  //    MOTORPORT |= (1<<MOTOROUT); 
      //loopledtakt = 0x1FFF;
   }
   else 
   {
      lastDIR = 0;
      MOTORPORT &= ~(1<<MOTORDIR);
  //    MOTORPORT |= (1<<MOTOROUT); 
      // loopledtakt = 0x0FFF;
   }
   
//   lastDIR = 1;
//   MOTORPORT |= (1<<MOTORDIR); 
   slaveinit();
  
 //  int0_init();
   
//   timer2(4);
   uint8_t loopcount0=0;
   uint8_t loopcount1=0;
   
    uint8_t firstruncount=0;
   
   
   
   
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
   INT0status = 0;
   
  // sei();
   MOTORDDR |= (1<<MOTOROUT);  // Output Motor PWM   
   MOTORPORT |= (1<<MOTOROUT); // HI, Motor OFF
   
   
   while (1)
   {   
      // Timing: loop: 40 us, takt 85us, mit if-teil 160 us
      wdt_reset();
      if (firstruncount < FIRSTRUN_END)
      {
         firstruncount++;
         if (firstruncount == FIRSTRUN_END)
         {
            MOTORDDR &= ~(1<<MOTORINT0); 
            int0_init();
            timer2(4);
            INT0status =0;  
            sei();
    //        MOTORDDR |= (1<<MOTOROUT);  // Output Motor PWM   
    //        MOTORPORT |= (1<<MOTOROUT); // HI, Motor OFF
            
         }

      }
      
      if (INT0status & (1<<INT0_RUN)) // Ende Data abwarten, nicht verwendet
      {
         int0counter++;
         /*
         if (int0counter > 20)
         {
            INT0status &= ~(1<<INT0_RUN);
            //MOTORPORT |= (1<<LAMPE);
            //INT0status = 0;
            int0counter = 0;
         }
          */
      }
      //Blinkanzeige
      /*
       if (lastDIR)
       {
       
       }
       else 
       {
       
       }
       */
      loopcount0++;
      
      if (loopcount0>=loopledtakt)
      {
         //MOTORPORT ^= (1<<LAMPE);
         //LOOPLEDPORT ^= (1<<LOOPLED); // Kontrolle lastDIR
         loopcount0=0;
         loopcount1++;
         if (loopcount1 >= loopledtakt)
         {
            loopcount1 = 0;
            // wdt-delay, fuer test
            /*
            wdtcounter++;
            if (wdtcounter > 60)
            {
               wdtcounter=0;
               //              _delay_ms(1000);
            }
             */
         }
         
      }
   }//while
}
