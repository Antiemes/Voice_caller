#include <avr/io.h>
#include <util/delay.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include "spi.h"
#include "flash.h"

#include <avr/wdt.h>
#include <avr/power.h>
#include <string.h>
#include <stdio.h>

#include "Descriptors.h"

#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/Board/Joystick.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint           =
					{
						.Address          = CDC_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
	};

static FILE USBSerialStream;

uint8_t samples=6;
uint32_t sampleStart[]={0, 380250, 696041, 882172, 1203165, 1203165+119160};
//uint32_t sampleLength[]={380250, 315791, 186131, 320993, 274181};
//uint32_t waitMillis[]={5000, 5000, 5000, 5000, 5000};
uint32_t sampleLength[]={380250, 315791, 186131, 320993, 274181, 60339};
uint32_t waitMillis[]={5000, 5000, 5000, 5000, 5000, 800};
uint16_t ledPattern[]={0x8000, 0xA000, 0xA800, 0xEA00, 0xEE00, 0xEEE0};
uint8_t sampleNum=0;
uint32_t waitCycles=0;
#define BUFSIZE 532
uint8_t buf[BUFSIZE];
uint16_t bufpos=0;
volatile uint16_t ledCounter=0;
volatile uint16_t ADCCounter=0;
volatile uint8_t ledFlag=0, ADCFlag=0, ADCFlag2=0;
const uint16_t ledTimeout=8192;
const uint16_t ADCTimeout=256;
volatile uint8_t pttOn=0;

volatile uint16_t debounce1=0, debounce2=0;
uint8_t inhibit1=0, inhibit2=0;

uint8_t micPTTCounter=0;

typedef enum
{
  STOPPED,
  RUNNING,
  WAITING,
} voice_state_t;

typedef enum
{
  RELAY_RX,
  RELAY_TX,
} relay_state_t;

volatile voice_state_t voiceState=STOPPED;

void ledInit(void)
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

void ledBlink(uint8_t led)
{
  volatile static uint8_t state[2]={0, 0};
  state[led]=1-state[led];
  ledSet(led, state[led]);
}

void buttonInit(void)
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

void ADCInit(void)
{
  //PTT sense: A1, PF6, ADC6
  //PORTF &= ~_BV(PF6); //No pullup
  DDRF  &= ~_BV(PF6); //Input
  
  //PORTF |= _BV(PF6); //No pullup
  //DDRF  |= _BV(PF6); //Input

  //ADCSRA
  //  ADPS[2:0]=1 --> 128 prescaler, 125 kHz clock
  //  ADEN: enable
  //  ADSC: start conversion
  //ADCH, ADCL: result
  ADMUX=0xc6; // 11000110 //2.56V ref, right adjust, 
  ADCSRA=0x87; // 10000111 //enable, don't start, disable and clear interrupt, 128 prescaler
  ADCSRB=0; //No high speed mode
  DIDR0=0x40; // Disable input buffer for this pin
}

void ADCStart(void)
{
  ADCSRA |= _BV(ADSC);
}

uint8_t ADCDone(void)
{
  return (ADCSRA & _BV(ADIF)) > 0;
}

void ADCClear(void)
{
  ADCSRA &= ~_BV(ADIF);
}

uint16_t ADCRes(void)
{
  return ADCL+((uint16_t)ADCH<<8);
}

void relayInit(void)
{
  //D4, PD4
  PORTD &= ~_BV(PD4);
  DDRD |= _BV(PD4);

  //D5, PC6
  PORTC &= ~_BV(PC6);
  DDRC |= _BV(PC6);
}

void relay(relay_state_t state)
{
  if (state==RELAY_RX)
  {
    PORTC |=  _BV(PC6);
    _delay_ms(10);
    PORTC &= ~_BV(PC6);
  }
  else if (state==RELAY_TX)
  {
    PORTD |=  _BV(PD4);
    _delay_ms(10);
    PORTD &= ~_BV(PD4);
  }
}

void pttInit(void)
{
  //D2, PD1
  PORTD &= ~_BV(PD1);
  DDRD |= _BV(PD1);

  //A1, PTT sense, TODO: ADC init
}

void ptt(uint8_t state)
{
  pttOn=state;
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
void pwm_init(void)
{
  // PB6, OC1B
  DDRB |= _BV(PB6);
	TCCR1A |= _BV(COM1B1) | _BV(WGM10);
  TCCR1B |= _BV(CS10) | _BV(WGM12); // With WGM12: Fast PWM, without WGM12: Normal PWM
}

//=====================================================================

void timerInit(void)
{
  cli();
  OCR0A=249; // Counting up to 250, generates 64000 interrupts / sec
  //OCR0B=250; // Counting up to 250, generates 64000 interrupts / sec
	TCCR0A=_BV(WGM01); // CTC mode
  //TCCR0B=_BV(CS01) | _BV(CS00);
  TCCR0B=_BV(CS00); // No prescaler
  //TCCR0B=_BV(CS01); // div8 prescaler
  sei();
}

void timerStart(void)
{
  cli();
  TCNT0=0;
  //TIMSK0 |= _BV(TOIE1);
  TIMSK0 |= _BV(OCIE0A);
  sei();
}

void timerStop(void)
{
  cli();
  TIMSK0 &= ~_BV(TOIE1);
  sei();
}

//=====================================================================

void seekToTrack(uint8_t track)
{
  initRead(sampleStart[track]);
  setSampleCounter(sampleLength[track]);
}

//=====================================================================

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	//LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	//LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** CDC class driver callback function the processing of changes to the virtual
 *  control lines sent from the host..
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo)
{
	/* You can get changes to the virtual CDC lines in this callback; a common
	   use-case is to use the Data Terminal Ready (DTR) flag to enable and
	   disable CDC communications in your application when set to avoid the
	   application blocking while waiting for a host to become ready and read
	   in the pending data from the USB endpoints.
	*/
	//bool HostReady = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR) != 0;
}

//=====================================================================

uint8_t hex2dec(uint8_t ch)
{
  if (ch>='0' && ch<='9')
  {
    return ch-'0';
  }
  else if (ch>='a' && ch<='f')
  {
    return ch-'a'+10;
  }
  else if (ch>='A' && ch<='F')
  {
    return ch-'A'+10;
  }
  else
  {
    return 0;
  }
}

uint8_t get_hex_byte(uint8_t* ptr)
{
  return hex2dec(*ptr)*16+hex2dec(*(ptr+1));
}

uint32_t get_hex_dword(uint8_t* ptr)
{
  uint8_t i;
  uint32_t tmp=0;
  for (i=0; i<8; i+=2)
  {
    tmp<<=8;
    tmp+=get_hex_byte((uint8_t*)ptr+i);
  }
  return tmp;
}

void send_byte_hex(uint8_t b)
{
  uint8_t buf[3];

  buf[0]=b>>4;
  if (buf[0]<=9)
  {
    buf[0]+='0';
  }
  else
  {
    buf[0]+='A'-10;
  }
  buf[1]=b&15;
  if (buf[1]<=9)
  {
    buf[1]+='0';
  }
  else
  {
    buf[1]+='A'-10;
  }
  buf[2]=0;
  CDC_Device_SendString(&VirtualSerial_CDC_Interface, (char*)buf);
}

void send_lf(void)
{
  CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n");
}

void send_string(char* str)
{
  CDC_Device_SendString(&VirtualSerial_CDC_Interface, str);
}

//=====================================================================

int main(void)
{
  static uint8_t ledStep=0;
  USBCON = 0;  // Disable USB interrupts
  spi_init();			//init spi. MODE 3
  pwm_init();			//fast pwm, prescaler 1
  buttonInit();
  ledInit();
  relayInit();
  ADCInit();
  seekToTrack(0);

  ledInit();
  pttInit();

  timerInit();
	
  MCUSR &= ~(1 << WDRF);
	wdt_disable();
	clock_prescale_set(clock_div_1);
	USB_Init();
	
  CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);
	GlobalInterruptEnable();

  timerStart();

  ledSet(0, 1);
  _delay_ms(500);
  ledSet(0, 0);

  relay(RELAY_RX);

  while(1)
  {
	  int16_t ch=CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
    if (ch>0)
    {
      if (ch==10 || ch==13)
      {
        if (bufpos>0)
        {
          buf[bufpos]=0;
          if (buf[0]=='R')
          {
            uint32_t i;
            uint32_t addr=0;
            uint32_t numBytes=0;
            addr=get_hex_dword((uint8_t*)buf+1);
            numBytes=get_hex_dword((uint8_t*)buf+9);
            initRead(addr);
            for (i=0; i<numBytes; i++)
            {
              send_byte_hex(readSample());
            }
            send_lf();
          }
          else if (buf[0]=='W')
          {
            uint32_t i;
            uint32_t addr=0;
            uint32_t numBytes=0;
            addr=get_hex_dword((uint8_t*)buf+1);
            numBytes=get_hex_dword((uint8_t*)buf+9);
            program_start(addr);
            for (i=0; i<numBytes; i++)
            {
              program_byte(get_hex_byte((uint8_t*)buf+17+i*2));
              //send_byte_hex(get_hex_byte((uint8_t*)buf+17+i*2));
            }
            program_end();
            while(write_in_progress()); //TODO: Timeout.
            send_string("OK\n");
          }
          else if (buf[0]=='E')
          {
            uint32_t addr=0;
            addr=get_hex_dword((uint8_t*)buf+1);
            erase_block(addr);
            while(write_in_progress()); //TODO: Timeout.
            send_string("OK\n");
          }
          else if (buf[0]=='U')
          {
            flash_unlock();
            //while(write_in_progress()); //TODO: Timeout.
            send_string("OK\n");
          }
          else if (buf[0]=='s')
          {
            uint16_t i;

            for (i=0; i<=255; i++)
            {
              send_byte_hex((uint8_t)i);
            }
            send_lf();
          }
        }
        bufpos=0;
      }
      else if (bufpos<BUFSIZE-2)
      {
        buf[bufpos]=(uint8_t)ch;
        bufpos++;
      }
    }

	  CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	  USB_USBTask();

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
            relay(RELAY_TX);
            ptt(1);
            newState=RUNNING;
            seekToTrack(sampleNum);
          }
          else
          {
            newState=STOPPED;
            relay(RELAY_RX);
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
          ledStep=0;
        }
      }
      else
      {
        inhibit2=0;
      }
    }
  //  OCR1B = spi_transmit_receive (0xff);	//read spi data and send to
  //  _delay_us (25);		//delay between samples (adjust it to play song fast or slow)
    if (ledFlag)
    {
      uint8_t st;
      ledFlag=0;
      st=(ledPattern[sampleNum] & (1 << (15-ledStep))) != 0;
      if (st)
      {
        ledSet(1, 1);
      }
      else
      {
        ledSet(1, 0);
      }
      ledStep++;
      if (ledStep>=12)
      {
        ledStep=0;
      }
    }
    if (ADCFlag)
    {
      ADCFlag=0; 
      ADCFlag2=1; 
      ADCStart();
    }
    if (ADCFlag2 && ADCDone())
    {
      ADCFlag2=0;
      uint16_t res;
      res=ADCRes(); //Mic PTT: 10, our PTT: 25    OLD: --> Mic PTT: 10 (24 mV), our PTT: 33 (83 mV)
      ADCClear();
      //send_byte_hex(res);
      //send_lf();
      if (res<18)
      {
        if (micPTTCounter<255)
        {
          micPTTCounter++;
        }
        if (micPTTCounter>=9 && voiceState!=STOPPED) //A few tests showed 6, sometimes 7.
        {
          //ledBlink(0);
          voiceState=STOPPED;
          relay(RELAY_RX);
          ptt(0);
          ledSet(0, 0);
        }
      }
      else
      {
        micPTTCounter=0;
      }
    }
  }
}

//=====================================================================

volatile uint8_t intCnt=0;
//ISR(TIMER0_OVF_vect)
ISR(TIMER0_COMPA_vect) //64 kHz
{
  if (ledCounter>0)
  {
    ledCounter--;
  }
  else
  {
    ledCounter=ledTimeout;
    ledFlag=1;
  }
  if (ADCCounter>0)
  {
    ADCCounter--;
  }
  else
  {
    ADCCounter=ADCTimeout;
    ADCFlag=1;
  }
  if (intCnt==1)
  {
    voice_state_t newState=voiceState;
    if (voiceState==RUNNING)
    {
      OCR1B = readSample(); //read spi data and send to
      if (getSampleCounter()==0)
      {
        seekToTrack(sampleNum);
        newState=WAITING;
        waitCycles=waitMillis[sampleNum]*32;
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
  else
  {
    intCnt++;
  }
}
