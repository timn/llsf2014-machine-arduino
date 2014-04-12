
/***************************************************************************
 *  llsf_machine.ino - RoboCup LLSF Arduino-based machine
 *
 *  Created: Sat Apr 12 12:09:34 2014 (Nobis Pontstrasse, Aachen)
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <TimerOne.h>

#define PIN_GREEN   13
#define PIN_YELLOW  8
#define PIN_RED     9

/** Light signal state. */
typedef enum {
  SIGNAL_OFF = 0,	//< Light is turned off
  SIGNAL_ON,		//< Light is turned on
  SIGNAL_BLINK	//< Light is blinking.
} SignalState;

bool blink_state_ = false;
SignalState state_red_    = SIGNAL_OFF;
SignalState state_yellow_ = SIGNAL_OFF;
SignalState state_green_  = SIGNAL_OFF;

void setup()
{
  Serial.begin(9600);

  pinMode(PIN_GREEN,  OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_RED,    OUTPUT);

  digitalWrite(PIN_GREEN, LOW);
  digitalWrite(PIN_YELLOW, LOW);
  digitalWrite(PIN_RED, LOW);

  Timer1.initialize(250000);
  Timer1.attachInterrupt(blink_timer);
}

void loop()
{
  delay(10);
}


/** Timer to blink lights if set to do so. */
void blink_timer()
{
  blink_state_ = ! blink_state_;

  if (state_red_ == SIGNAL_BLINK)
    digitalWrite(PIN_RED, blink_state_ ? HIGH : LOW);

  if (state_green_ == SIGNAL_BLINK)
    digitalWrite(PIN_GREEN, blink_state_ ? HIGH : LOW);

  if (state_yellow_ == SIGNAL_BLINK)
    digitalWrite(PIN_YELLOW, blink_state_ ? HIGH : LOW);
}
