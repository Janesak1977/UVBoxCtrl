// UV Box Control
// v1.0


#include <EEPROM.h>
#include <TM1637.h>

#define CLK 2//pins definitions for TM1637 and can be changed to other ports
#define DIO 3
#define LEDPIN 8
#define OUTPUTPIN 9
#define UPButtonPIN 4
#define DOWNButtonPIN 5
#define SETButtonPIN 6
#define STARTSTOPButtonPIN 7
#define DEBOUNCE_TIME 20    //in ms
#define LONG_PRESS_THLD 2   // Long time press in sec.
#define LONG_QUIET_THLD 10
#define PREHEAT_TIME 120   //Preheat time in sec.

// keyboard events
#define KB_EVENT_UP    (1 << 0)
#define KB_EVENT_DOWN  (1 << 1)
#define KB_EVENT_SET   (1 << 2)
#define KB_EVENT_START (1 << 3)
#define KB_EVENT_START (1 << 3)
#define KB_EVENT_UP_LONG  (1 << 4)
#define KB_EVENT_NONE_LONG (1 << 5)

// names for keys
#define KBI_UP  (1 << 4)  // UP Button
#define KBI_DOWN (1 << 5)  // DOWN Button
#define KBI_SET    (1 << 6)  // SET Button
#define KBI_START  (1 << 7)  // START/STOP Encoder

enum State_t {ST_IDLE, ST_RUN, ST_END, ST_SET} State;

uint8_t DispBuff[4] = {0x00,0x00,0x00,0x00};
uint8_t SetDigit[4] = {0x00,0x00,0x00,0x00};
uint16_t ActualTime = 0;
uint16_t DesiredTime = 0;
uint8_t DebounceTimer = 0;
uint32_t tmpdivider = 0;
unsigned long CountdownTimer = 0;
uint32_t tempmillis = 0;
static uint8_t state_front_prev;
bool kbtimeout = true;
static volatile uint8_t keys = 0;  // must be volatile
uint16_t kb_events = 0;
uint8_t BlinkDigitFlags = 0x08;   // Enbale Blinking digit, if bit0=1 then Blink digit 3;
bool BlinkFlag = false;
bool BtnFirstPress = true;
bool LEDBlink = false;
uint8_t LEDState = 0;
static uint8_t long_press;
static uint8_t long_quiet;

TM1637 tm1637(CLK,DIO);

// Interrupt is called once a 1ms, 
SIGNAL(TIMER0_COMPA_vect) 
{
  tmpdivider++;

  if ((CountdownTimer!=0) && (State==ST_RUN))
  {
   CountdownTimer--;
   if (CountdownTimer%1000==0)
    ActualTime--;
  }

  if ((tmpdivider%10)==0)    //scan button every 20ms
    keys = ~PIND & (KBI_UP | KBI_DOWN | KBI_SET | KBI_START);

  if ((tmpdivider%250)==0)
  {
    if (BlinkFlag==true)
      BlinkFlag = false;
    else
      BlinkFlag = true;
  }

  if ((tmpdivider%150)==0)
  {
    if (LEDBlink==true)
    {
      if (LEDState==0)
        LEDState = 1;
      else
        LEDState = 0;
        
      digitalWrite(LEDPIN, LEDState);
    }
    else
    {
      LEDState = 0;
      digitalWrite(LEDPIN, LEDState);
    }
  }
    

  if (DebounceTimer != 0)
    DebounceTimer--;
  else
    kbtimeout = true;

}
  

void ReadKeyboard(void)
{
  uint8_t front = keys & ( KBI_UP | KBI_DOWN | KBI_SET | KBI_START);
  if (front != state_front_prev)          //some button pressed
  {
    if (front && (state_front_prev == 0) && kbtimeout)   //only if any button was not pressed in prev cycle 
    {
      if (front == KBI_UP)
        kb_events |= KB_EVENT_UP;
      else
      if (front == KBI_DOWN)
        kb_events |= KB_EVENT_DOWN;
      else
      if (front == KBI_SET)
        kb_events |= KB_EVENT_SET;
      else
      if (front == KBI_START)
        kb_events |= KB_EVENT_START;
    }
    if (kbtimeout)
    {
      kbtimeout = false;
      DebounceTimer = 20;
    }
    state_front_prev = front;
    long_press = 0; // long press detection RESET
    long_quiet = 0;
  }
}

void DigitsToDispBufferWithBlink(uint8_t* Digit)
{
 unsigned int temp;

 if (BlinkDigitFlags & 0x01)
    {
      if (BlinkFlag)
        DispBuff[3]=Digit[3];
      else
        DispBuff[3]=0x7F;
    }
 else
    DispBuff[3]=Digit[3];
 
 if (BlinkDigitFlags & 0x02)
    {
      if (BlinkFlag)
        DispBuff[2]= Digit[2];
      else
         DispBuff[2]= 0x7F;
    }
 else
    DispBuff[2]= Digit[2];
 
    
 if (BlinkDigitFlags & 0x04)
    {
      if (BlinkFlag)
        DispBuff[1]=Digit[1];
      else
        DispBuff[1]=0x7F;
    }
 else
    DispBuff[1]=Digit[1];
   
    
    DispBuff[0]=0x7F;
} 


void IntegerToDispBuffer(int val)
{
 unsigned int temp;

 DispBuff[3]=val % 10;
 
 temp = val / 10;
 if (temp==0)
    DispBuff[2] = 0x7F;
 else
    DispBuff[2]= temp % 10;
    
 temp = temp / 10;
 if (temp==0)
    DispBuff[1] = 0x7F;
  else
    DispBuff[1]=temp % 10;
    
    DispBuff[0] = 0x7F;
}


void SetTime()
{
  bool SetTimerExit = false;
  SetDigit[0] = 0x00;
  SetDigit[1] = 0x00;
  SetDigit[2] = 0x00;
  SetDigit[3] = 0x00;
  BlinkDigitFlags = 0x04;
  do
  {
    ReadKeyboard();
    if (kb_events & KB_EVENT_UP)
    {
       if (BlinkDigitFlags==0x04)
        SetDigit[1] = (SetDigit[1] + 1) % 10;
       if (BlinkDigitFlags==0x02)
        SetDigit[2] = (SetDigit[2] + 1) % 10;
       if (BlinkDigitFlags==0x01)
        SetDigit[3] = (SetDigit[3] + 1) % 10;
       kb_events = 0;
    }

    if (kb_events & KB_EVENT_DOWN)
    {
      if (BlinkDigitFlags==0x04)
      if (SetDigit[1]==0)
          SetDigit[1] = 9;
      else
          SetDigit[1] = (SetDigit[1]-1) % 10;
      if (BlinkDigitFlags==0x02)
      if (SetDigit[2]==0)
          SetDigit[2] = 9;
      else
          SetDigit[2] = (SetDigit[2]-1) % 10;
      if (BlinkDigitFlags==0x01)
      if (SetDigit[3]==0)
          SetDigit[3] = 9;
      else
          SetDigit[3] = (SetDigit[3]-1) % 10;
      kb_events = 0;
    }

    if (kb_events & KB_EVENT_SET)
    {
      BlinkDigitFlags = BlinkDigitFlags >> 1;
      if (BlinkDigitFlags==0x00)
      {
        SetTimerExit = true;
        DesiredTime = SetDigit[1]*100 + SetDigit[2]*10 + SetDigit[3]*1;
        EEPROM.put(0x0001, DesiredTime);
      }
      kb_events = 0;
    }

    if (kb_events & KB_EVENT_START)
    {
       SetTimerExit = true;
       kb_events = 0;
    }
    
    DigitsToDispBufferWithBlink(SetDigit);
    tm1637.display(DispBuff);
  }while(SetTimerExit==false);
  
}

void LongPressDetect()
{
  if (! state_front_prev)
  {
    if (++long_quiet == 0)
    {
      // overload protection, nothing to do, event was generated previous
      long_quiet--;
      return;
    }
    if (long_quiet == LONG_QUIET_THLD)
    {
      kb_events |= KB_EVENT_NONE_LONG;
    }
  }
  else
  {
    if (++long_press == 0)
    {
      // overload protection, nothing to do, event was generated previous
      long_press--;
      return;
    }
    if (long_press == LONG_PRESS_THLD)
    {
      switch (state_front_prev)
      {
        case KBI_UP:
          kb_events |= KB_EVENT_UP_LONG;
        break;
       

        default:
        break;
      }
    }
  }
}

void setup()
{
  pinMode(STARTSTOPButtonPIN, INPUT);
  digitalWrite(STARTSTOPButtonPIN, HIGH);            // turn on pullup resistors
  pinMode(UPButtonPIN, INPUT);
  digitalWrite(UPButtonPIN, HIGH);            // turn on pullup resistors
  pinMode(DOWNButtonPIN, INPUT);
  digitalWrite(DOWNButtonPIN, HIGH);            // turn on pullup resistors
  pinMode(SETButtonPIN, INPUT);
  digitalWrite(SETButtonPIN, HIGH);            // turn on pullup resistors
  pinMode(OUTPUTPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);

  digitalWrite(OUTPUTPIN, LOW);            // turn off UV light
  digitalWrite(LEDPIN, LOW);
  EEPROM.get(0x0001, DesiredTime);
  if ((DesiredTime == 0) || (DesiredTime==0xFFFF))
    DesiredTime = 300;

  ActualTime = DesiredTime;
    
  State = ST_IDLE;
  
  tm1637.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  tm1637.init();
  
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xFA; //250 * 4us = 1000us
  TIMSK0 |= _BV(OCIE0A);
}

void loop()
{
 ReadKeyboard();
 if ((tmpdivider % 1000) == 0)
    LongPressDetect();
 
 if (kb_events & KB_EVENT_START)
 {
    if (State == ST_IDLE)
    {
      cli();
      CountdownTimer = (ActualTime * (unsigned long)1000);
      State = ST_RUN;
      sei();
      LEDBlink = true;
      digitalWrite(OUTPUTPIN, HIGH);            // turn ON UV light
      kb_events = 0;
    }
    else
      if (State == ST_RUN)
      {
        State = ST_IDLE;
        ActualTime = DesiredTime;
        LEDBlink = false;
        digitalWrite(OUTPUTPIN, LOW);            // turn OFF UV light
        kb_events = 0;
      }
    else
      if (State == ST_END)
      {
        State = ST_IDLE;
        ActualTime = DesiredTime;
        kb_events = 0;
      }
 }

 if (kb_events & KB_EVENT_SET)
 {
    if (State!=ST_RUN)
    {
       State = ST_SET;
       kb_events = 0;
       SetTime();
       ActualTime = DesiredTime;
       State = ST_IDLE;
    }
 }

 if ((kb_events & KB_EVENT_UP_LONG) && State==ST_IDLE)
 {
        bool PreheatExit = false;
        ActualTime = PREHEAT_TIME;
        State = ST_RUN;
        cli();
        CountdownTimer = ActualTime * (unsigned long)1000;
        sei();
        LEDBlink = true;
        digitalWrite(OUTPUTPIN, HIGH);            // turn ON UV light
        kb_events = 0;
        do
        {
          ReadKeyboard();
          
          if (kb_events & KB_EVENT_START)
            PreheatExit = true;
          
          if (ActualTime==0)
          {
            State = ST_IDLE;
            PreheatExit = true;
            LEDBlink = false;
            digitalWrite(OUTPUTPIN, LOW);            // turn OFF UV light
          }
          
          IntegerToDispBuffer(ActualTime);
          tm1637.display(DispBuff);
        }while(PreheatExit==false);
        
        State = ST_IDLE;
        LEDBlink = false;
        digitalWrite(OUTPUTPIN, LOW);            // turn OFF UV light
        kb_events = 0;
        ActualTime = DesiredTime;
 }

 if ((kb_events & KB_EVENT_UP) && State==ST_IDLE)
 {        
          ActualTime = ActualTime + 10;
          if (ActualTime > 999) ActualTime = 999;
          kb_events = 0;         
 }

 if ((kb_events & KB_EVENT_DOWN) && State==ST_IDLE)
 {
        if (ActualTime!=0)
        {
          ActualTime = ActualTime - 10;
          if (ActualTime == 0) ActualTime = ActualTime + 10;
          if (ActualTime > 999) ActualTime = 999;
        }
        kb_events = 0;
 }
 
 if (ActualTime==0 && State!=ST_END)
 {
    State = ST_END;
    LEDBlink = false;
    digitalWrite(OUTPUTPIN, LOW);            // turn OFF UV light
 }

 IntegerToDispBuffer(ActualTime);
 tm1637.display(DispBuff);
}
