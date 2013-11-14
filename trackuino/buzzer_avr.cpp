/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifdef AVR

#include "config.h"
#include "buzzer.h"
#include "pin.h"
#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include <SoftwareSerial.h>
extern SoftwareSerial mySerial;

// Module constants
static const unsigned long PWM_PERIOD = F_CPU / BUZZER_FREQ;
static const unsigned long ON_CYCLES = BUZZER_FREQ * BUZZER_ON_TIME;
static const unsigned long OFF_CYCLES = BUZZER_FREQ * BUZZER_OFF_TIME;

static const unsigned long PWM_PERIOD1 = F_CPU / BUZZER_FREQ1;
static const unsigned long ON_CYCLES1 = BUZZER_FREQ1 * BUZZER_ON_TIME;
static const unsigned long OFF_CYCLES1 = BUZZER_FREQ1 * BUZZER_OFF_TIME;

int fff=0;


boolean whichf=1;





#if BUZZER_TYPE == 0  // active buzzer
static const uint16_t DUTY_CYCLE = PWM_PERIOD;
static const uint16_t DUTY_CYCLE1 = PWM_PERIOD1;
#endif
#if BUZZER_TYPE == 1  // passive buzzer
static const uint16_t DUTY_CYCLE = PWM_PERIOD / 2;
static const uint16_t DUTY_CYCLE1 = PWM_PERIOD1 / 2;
#endif

// Module variables
static volatile bool is_buzzer_on;
static volatile bool buzzing;
static volatile unsigned long alarm;

// Exported functions
void buzzer_setup()
{
  pinMode(BUZZER_PIN, OUTPUT);
  pin_write(BUZZER_PIN, LOW);
  buzzing = false;
  is_buzzer_on = false;
  alarm = 1;

  // Top is ICR1 (WGM1=14), p.135
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12);

  // Set top to PWM_PERIOD
  //ICR1 = PWM_PERIOD;
  if (whichf) ICR1 = PWM_PERIOD;
  else ICR1 = PWM_PERIOD1;
  
  // Enable interrupts on timer overflow
  TIMSK1 |= _BV(TOIE1);

  // Start the timer, no prescaler (CS1=1)
  TCCR1B |= _BV(CS10);
}

void buzzer_on()
{
  is_buzzer_on = true;
}

void buzzer_off()
{
  is_buzzer_on = false;
}

// Interrupt Service Routine for TIMER1. This is used to switch between the
// buzzing and quiet periods when ON_CYCLES or OFF_CYCLES are reached.
ISR (TIMER1_OVF_vect)
{
  alarm--;
  if (alarm == 0) {
//     mySerial.println(fff);
//     mySerial.println(whichf);
//     mySerial.println("----");
    if (fff%2) whichf = !whichf; 
    buzzing = !buzzing;
    if (is_buzzer_on && buzzing) {
      switch(BUZZER_PIN) {
        case 9:
          // Non-inverting pin 9 (COM1A=2), p.135
          TCCR1A |= _BV(COM1A1);
          if (whichf ) OCR1A = DUTY_CYCLE;
          else OCR1A = DUTY_CYCLE1;
          break;
        case 10:
          // Non-inverting pin 10 (COM1B=2), p.135
          TCCR1A |= _BV(COM1B1);
          if (whichf ) OCR1B = DUTY_CYCLE;
          else OCR1B = DUTY_CYCLE1;
          break;
      }

      
      if (whichf)      alarm = ON_CYCLES;
      else             alarm = ON_CYCLES1;
      
    } else {
      switch(BUZZER_PIN) {
        // Disable PWM on pin 9/10
        case 9:  TCCR1A &= ~_BV(COM1A1); break;
        case 10: TCCR1A &= ~_BV(COM1B1); break;
      }
      pin_write(BUZZER_PIN, LOW);
            //alarm = OFF_CYCLES;
      if (whichf)       alarm = OFF_CYCLES;
      else              alarm = OFF_CYCLES1;  
    }
    
    fff++;
    //if (fff%2) whichf = !whichf; 
    //whichf = !whichf; 
  }
}

#endif // #ifdef AVR
