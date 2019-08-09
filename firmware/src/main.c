#include <avr/io.h>
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

void ledInit()
{
  //LEDs: PD7, PE6
  PORTD &= ~_BV(PD7);
  DDRD |= _BV(PD7);

  PORTE &= ~_BV(PE6);
  DDRE |= _BV(PE6);
}

void ledSet(uint8_t led, uint8_t state)
{
  if (led==0)
  {
    if (state==0)
    {
      PORTD &= ~_BV(PD7);
    }
    else
    {
      PORTD |= _BV(PD7);
    }
  }
  else if (led==1)
  {
    if (state==0)
    {
      PORTE &= ~_BV(PE6);
    }
    else
    {
      PORTE |= _BV(PE6);
    }
  }
}

void buttonInit()
{
  //Inputs, pullup enabled.

  //A3, PF4
  DDRF &= ~_BV(PF4);
  PORTF |= _BV(PF4);

  //A2, PF5
  DDRF &= ~_BV(PF5);
  PORTF |= _BV(PF5);

  //A0, PF7
  DDRF &= ~_BV(PF7);
  PORTF |= _BV(PF7);

  //D9,  PB5
  DDRB &= ~_BV(PB5);
  PORTB |= _BV(PB5);

  //D8,  PB4
  DDRB &= ~_BV(PB4);
  PORTB |= _BV(PB4);
}

void relayInit()
{
  //D4, PD4
  PORTD &= ~_BV(PD4);
  DDRD |= _BV(PD4);

  //D5, PC6
  PORTC &= ~_BV(PC6);
  DDRC |= _BV(PC6);
}

void relay(uint8_t state)
{
  if (state==0)
  {
    PORTD |=  _BV(PD4);
    _delay_ms(10);
    PORTD &= ~_BV(PD4);
  }
  else
  {
    PORTC |=  _BV(PC6);
    _delay_ms(10);
    PORTC &= ~_BV(PC6);
  }
}

void pttInit()
{
  //D2, PD1
  PORTD &= ~_BV(PD1);
  DDRD |= _BV(PD1);

  //A1, PTT sense, TODO: ADC init
}

void ptt(uint8_t state)
{
  if (state==0)
  {
    PORTD &= ~_BV(PD1);
  }
  else
  {
    PORTD |= _BV(PD1);
  }
}

//initialize fast PWM with prescaler 1 at PB1
void pwm_init()
{
  // PB6, OC1B
  DDRB |= _BV(PB6);
	TCCR1A |= _BV(COM1B1) | _BV(WGM10);
  TCCR1B |= _BV(CS10) | _BV(WGM12); // With WGM12: Fast PWM, without WGM12: Normal PWM
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

void seekToTrack(uint8_t track)
{
  initRead(sampleStart[track]);
  sampleCounter=sampleLength[track];
}

//=====================================================================

int main()
{
  USBCON = 0;  // Disable USB interrupts
  spi_init();			//init spi. MODE 3
  pwm_init();			//fast pwm, prescaler 1
  buttonInit();
  ledInit();
  relayInit();
  seekToTrack(0);

  ledInit();
  pttInit();

  timerInit();
  timerStart();

  ledSet(0, 1);
  _delay_ms(500);
  ledSet(0, 0);

  while(1)
  {
    if (!debounce1)
    {
      if (!(PINF & _BV(PF4)))
      {
        if (!inhibit1)
        {
          voice_state_t newState=voiceState;
          debounce1=4000;
          inhibit1=1;
          if (voiceState==STOPPED)
          {
            ledSet(0, 1);
            ptt(1);
            newState=RUNNING;
            seekToTrack(sampleNum);
          }
          else
          {
            newState=STOPPED;
            ptt(0);
            ledSet(0, 0);
          }
          voiceState=newState;
        }
      }
      else
      {
        inhibit1=0;
      }
    }
    if (!debounce2)
    {
      if (!(PINF & _BV(PF5)))
      {
        if (!inhibit2)
        {
          debounce2=4000;
          inhibit2=1;
          sampleNum++;
          if (sampleNum==samples)
          {
            sampleNum=0;
          }
        }
      }
      else
      {
        inhibit2=0;
      }
    }
  //  OCR1B = spi_transmit_receive (0xff);	//read spi data and send to
  //  _delay_us (25);		//delay between samples (adjust it to play song fast or slow)
  }
}

//=====================================================================

volatile uint8_t intCnt=0;
ISR(TIMER0_OVF_vect)
{
  static volatile uint16_t ct=0;
  ct++;
  intCnt++;
  if (ct & 32768)
  {
    ledSet(1, 1);
  }
  else
  {
    ledSet(1, 0);
  }
  if (intCnt==2)
  {
    voice_state_t newState=voiceState;
    if (voiceState==RUNNING)
    {
      OCR1B = readSample(); //read spi data and send to
      if (sampleCounter==0)
      {
        seekToTrack(sampleNum);
        newState=WAITING;
        waitCycles=120000;
        ptt(0);
        ledSet(0, 0);
      }
    }
    else if (voiceState==WAITING)
    {
      waitCycles--;
      if (waitCycles==0)
      {
        newState=RUNNING;
        ptt(1);
        ledSet(0, 1);
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
