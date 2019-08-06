#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include "spi.h"
#include "flash.h"

uint8_t samples=3;
uint32_t sampleStart[]={0, 380250, 696041};
uint32_t sampleLength[]={380250, 315791, 186131};
uint8_t sampleNum=0;
uint32_t waitCycles=0;

volatile uint16_t debounce1=0, debounce2=0;
uint8_t inhibit1=0, inhibit2=0;

typedef enum
{
  STOPPED,
  RUNNING,
  WAITING,
} voice_state_t;

volatile voice_state_t voiceState=STOPPED;

//initialize fast PWM with prescaler 1 at PB1
void pwm_init()
{
  DDRB |= _BV(PB6);
	TCCR1A |= _BV(COM1A1) | _BV(WGM10);
  TCCR1B |= _BV(CS10) | _BV(WGM12);
}

//=====================================================================

void timerInit()
{
  cli();
	TCCR0A=0;
  //TCCR0B=_BV(CS01) | _BV(CS00);
  TCCR0B=_BV(CS00);
  sei();
}

void timerStart()
{
  cli();
  TCNT0=0;
  TIMSK0 |= _BV(TOIE1);
  sei();
}

void timerStop()
{
  cli();
  TIMSK0 &= ~_BV(TOIE1);
  sei();
}

//=====================================================================

void led(uint8_t state);
void ledInit()
{
  DDRB |= _BV(PB0);
  led(0);
}

void led(uint8_t state)
{
  if (state)
  {
    PORTB |= _BV(PB0);
  }
  else
  {
    PORTB &= ~_BV(PB0);
  }
}

void ptt(uint8_t state);
void pttInit()
{ // PD4
  DDRD |= _BV(PD4);
  ptt(0);
}

void ptt(uint8_t state)
{
  if (state)
  {
    PORTD |= _BV(PD4);
  }
  else
  {
    PORTD &= ~_BV(PD4);
  }
}


//=====================================================================

void seekToTrack(uint8_t track)
{
  initRead(sampleStart[track]);
  sampleCounter=sampleLength[track];
}

//=====================================================================

int main()
{
  spi_init();			//init spi. MODE 3
  pwm_init();			//fast pwm, prescaler 1

  //ledInit();
  //pttInit();

  timerInit();
  timerStart();
  
  //PORTD |= _BV(PD2) | _BV(PD3);
  //DDRD &= ~_BV(PD2) | _BV(PD3);

  _delay_ms(100);

  while(1)
  {
    if (voiceState!=RUNNING)
    {
      voiceState=RUNNING;
    }
//    if (!debounce1)
//    {
//      if (!(PIND & _BV(PD2)))
//      {
//        if (!inhibit1)
//        {
//          voice_state_t newState=voiceState;
//          debounce1=4000;
//          inhibit1=1;
//          if (voiceState==STOPPED)
//          {
//            led(1);
//            ptt(1);
//            newState=RUNNING;
//            seekToTrack(sampleNum);
//          }
//          else
//          {
//            newState=STOPPED;
//            ptt(0);
//            led(0);
//          }
//          voiceState=newState;
//        }
//      }
//      else
//      {
//        inhibit1=0;
//      }
//    }
//    if (!debounce2)
//    {
//      if (!(PIND & _BV(PD3)))
//      {
//        if (!inhibit2)
//        {
//          debounce2=4000;
//          inhibit2=1;
//          sampleNum++;
//          if (sampleNum==samples)
//          {
//            sampleNum=0;
//          }
//        }
//      }
//      else
//      {
//        inhibit2=0;
//      }
//    }
  //  OCR1A = spi_transmit_receive (0xff);	//read spi data and send to
  //  _delay_us (25);		//delay between samples (adjust it to play song fast or slow)
  }
}

//=====================================================================

volatile uint8_t intCnt=0;
ISR(TIMER0_OVF_vect)
{
  intCnt++;
  if (intCnt==2)
  {
    voice_state_t newState=voiceState;
    if (voiceState==RUNNING)
    {
      OCR1A = readSample(); //read spi data and send to
      if (sampleCounter==0)
      {
        seekToTrack(sampleNum);
        newState=WAITING;
        waitCycles=120000;
        ptt(0);
        led(0);
      }
    }
    else if (voiceState==WAITING)
    {
      waitCycles--;
      if (waitCycles==0)
      {
        newState=RUNNING;
        ptt(1);
        led(1);
      }
    }
    voiceState=newState;

    if (debounce1)
    {
      debounce1--;
    }
    if (debounce2)
    {
      debounce2--;
    }
    intCnt=0;
  }
}
