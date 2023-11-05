#include "Arduino.h"
#include <avr/io.h>


#define LIGHT_PIN A3
#define PIR_PIN PB2
#define LED_PIN PB1
#define PIR_FALL_DELAY 10000
#define SERIAL_BAUD 9600
#define LIGHT_ON_THRESHOLD 600
#define LIGHT_OFF 0
#define LIGHT_ON 1



volatile int state;
volatile bool pir_rise;
unsigned long pir_fall_delay_millis;
volatile int watchdog_counter;
volatile int halfhour_counter;



void setup_watchdog(int ii)
{
  byte bb;
  int ww;

  if(ii > 9)
  {
    ii = 9;
  }

  bb = ii & 7;

  if(ii > 7)
  {
    bb |= (1<<5);
  }

  bb |= (1<<WDCE);

  ww == bb;

  MCUSR &= ~(1<<WDRF);

  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);

  // set new watchdog timeout value
  WDTCR = bb;

  WDTCR |= _BV(WDIE);
}


void movementEvent()
{
  if(pir_fall_delay_millis == 0)
  {
    pir_fall_delay_millis = millis();

    return;
  }

  if(abs(millis() - pir_fall_delay_millis) < PIR_FALL_DELAY)
  {
    return;
  }
  
  pir_fall_delay_millis = 0;
  pir_rise = false;
}




void light_off()
{
  if( (pir_rise == true) && (analogRead(LIGHT_PIN) < LIGHT_ON_THRESHOLD))
  {
    watchdog_counter = 0;
    halfhour_counter = 0;
    state = LIGHT_ON;
    return;
  }
  
  digitalWrite(LED_PIN, LOW);
}


void light_on()
{
  if( (pir_rise == true) && (analogRead(LIGHT_PIN) < LIGHT_ON_THRESHOLD))
  {
    watchdog_counter = 0;
    halfhour_counter = 0;
  }
  
  //~ 30 min
  if(watchdog_counter > 180)
  {
    watchdog_counter = 0;
    halfhour_counter++;

    if(halfhour_counter >= 1)
    {
      state = LIGHT_OFF;
      return;
    }
  }
  
  digitalWrite(LED_PIN, HIGH);
}


void setup_external_interrupt()
{
  GIMSK |= (1<<INT0);
  MCUCR |= (1<<ISC00) | (1<<ISC01);  
}


ISR(INT0_vect)
{
  pir_rise = true;
}


ISR(WDT_vect)
{  
  watchdog_counter++;
}


void setup()
{
  pinMode(LIGHT_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  setup_external_interrupt();

  // ~ 10s
  setup_watchdog(9);                 

  watchdog_counter = 0;
  halfhour_counter = 0;
  pir_rise = false;
  pir_fall_delay_millis = 0;

  state = LIGHT_ON;
}


void loop()
{
  if(pir_rise == true)
  {
    movementEvent();
  }

  switch(state)
  {
    case LIGHT_OFF:
      light_off();
      break;
    case LIGHT_ON:
      light_on();
      break;
  }
}


