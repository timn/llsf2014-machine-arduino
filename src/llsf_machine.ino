
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
#include <XBee.h>
#include <SoftwareSerial.h>

#define PIN_GREEN     7
#define PIN_YELLOW    8
#define PIN_RED       9

#define PIN_DEBUG_RX 11
#define PIN_DEBUG_TX 12

#define PIN_ERR_LED  13

/** Light signal state. */
typedef enum {
  SIGNAL_OFF   = 0,	//< Light is turned off
  SIGNAL_ON    = 1,	//< Light is turned on
  SIGNAL_BLINK = 2	//< Light is blinking.
} SignalState;

// ERR 1 - 3 are reserved for XBee library
#define ERR_REFBOX_UNKNOWN    5
#define ERR_UNKNOWN_PACKET    6
#define ERR_SHORT_PACKET      7
#define ERR_INVALID_RESPONSE  8

static const char * ERROR_CODES[] = {
  "No error",
  "Checksum failure",
  "Invalid AT command",
  "Invalid AT parameter",
  "No response to AT command",
  "RefBox unknown",
  "Unknown packet type",
  "Packet too short",
  "Invalid response received"
};

#define MSG_TYPE_SIGNAL_INSTRUCT  1
#define MSG_TYPE_RFID_DATA        2
#define MSG_TYPE_RFID_WRITE       3

#pragma pack(push,1)
typedef struct {
  uint32_t tag_id;
} RFIDMessage;

typedef struct {
  uint8_t signal_state_red;
  uint8_t signal_state_yellow;
  uint8_t signal_state_green;
} SignalInstructMessage;
#pragma pack(pop)

XBeeAddress64 refbox_hw_addr_(0,0);
uint16_t      refbox_net_addr_ = 0;
bool          refbox_dn_running_ = false;

bool blink_state_ = false;
SignalState state_red_    = SIGNAL_OFF;
SignalState state_yellow_ = SIGNAL_OFF;
SignalState state_green_  = SIGNAL_OFF;

SoftwareSerial debug(PIN_DEBUG_RX, PIN_DEBUG_TX); // RX, TX
XBee xbee;

// from Ethernet library, repeat to avoid pulling in the whole thing
#define htons(x) ( ((x)<<8) | (((x)>>8)&0xFF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

void setup()
{
  pinMode(PIN_GREEN,   OUTPUT);
  pinMode(PIN_YELLOW,  OUTPUT);
  pinMode(PIN_RED,     OUTPUT);

  pinMode(PIN_ERR_LED, OUTPUT);

  digitalWrite(PIN_GREEN,   LOW);
  digitalWrite(PIN_YELLOW,  LOW);
  digitalWrite(PIN_RED,     LOW);
  digitalWrite(PIN_ERR_LED, HIGH);

  Timer1.initialize(250000);
  Timer1.attachInterrupt(blink_timer);

  debug.begin(9600);
  debug.println("STARTED");

  Serial.begin(9600);
  xbee.begin(Serial);
}

void loop()
{
  if (refbox_hw_addr_.getMsb() == 0 && ! refbox_dn_running_) {
    report_error(ERR_REFBOX_UNKNOWN);
    digitalWrite(PIN_ERR_LED, HIGH);
    AtCommandRequest req((uint8_t*)"DN", (uint8_t*)"RefBox", 6);
    xbee.send(req);
    refbox_dn_running_ = true;
  }

  xbee.readPacket();
  if (xbee.getResponse().isError()) {
    debug.print("Response Error: ");
    debug.println(xbee.getResponse().getErrorCode());
    report_error(xbee.getResponse().getErrorCode());
  } else if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      ZBRxResponse rx;
      xbee.getResponse().getZBRxResponse(rx);
      //process message
      uint16_t data_length = rx.getDataLength();
      if (data_length >= 1) {
	digitalWrite(PIN_ERR_LED, LOW);

	uint8_t *data = rx.getFrameData() + rx.getDataOffset();

	debug.print("Received: ");
	for (unsigned int i = 0; i < data_length; ++i) {
	  debug.print(data[i], HEX);
	}
	debug.println("");

	uint8_t msg_type = data[0];
	switch (msg_type) {
	case MSG_TYPE_SIGNAL_INSTRUCT:
	  if (data_length == sizeof(SignalInstructMessage) + 1) {
	    SignalInstructMessage *msg = (SignalInstructMessage *)&data[1];
	    state_red_    = (SignalState)msg->signal_state_red;
	    state_yellow_ = (SignalState)msg->signal_state_yellow;
	    state_green_  = (SignalState)msg->signal_state_green;

	    debug.print("Received SignalInstructMessage: R ");
	    debug.print(state_red_);
	    debug.print("   Y "); debug.print(state_yellow_);
	    debug.print("   G "); debug.println(state_green_);

	    update_lights();
	  } else {
	    debug.println("Invalid SignalInstructMessage data length");
	  }
	  break;
	default: // unknown packet type
	  debug.print("Received unknown packet");
	  report_error(ERR_UNKNOWN_PACKET);
	  break;
	}
      } else {
	debug.print("Packet too short");
	report_error(ERR_SHORT_PACKET);
      }
    } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
      ModemStatusResponse msr;
      xbee.getResponse().getModemStatusResponse(msr);
      debug.print("Modem status:");
      debug.println(msr.getStatus());
    } else if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
      AtCommandResponse atr;
      xbee.getResponse().getAtCommandResponse(atr);
      uint8_t *command = atr.getCommand();
      if (command[0] == 'D' && command[1] == 'N') {
	refbox_dn_running_ = false;

	if (atr.getStatus() == SUCCESS) {
	  uint8_t *payload = atr.getValue();
	  refbox_net_addr_ = ((uint64_t)payload[0] << 8)  +  payload[1];

	  uint32_t msb = 
	    ((uint32_t)payload[2] << 24) + ((uint32_t)payload[3] << 16) +
	    ((uint32_t)payload[4] << 8)  +  payload[5];
	  uint32_t lsb = 
	    ((uint32_t)payload[6] << 24) + ((uint32_t)payload[7] << 16) +
	    ((uint32_t)payload[8] << 8)  +  payload[9];

	  refbox_hw_addr_.setMsb(msb);
	  refbox_hw_addr_.setLsb(lsb);

	  debug.print("RefBox: Net ");
	  for (int i = 0; i < 2; ++i)  debug.print(payload[i], HEX);
	  debug.print("  HW ");
	  for (int i = 2; i < 10; ++i)  debug.print(payload[i], HEX);
	  debug.println("");

	  digitalWrite(PIN_ERR_LED, LOW);
	}
      }
    } else {
      debug.println("Invalid response");
      report_error(ERR_INVALID_RESPONSE);
    }
  } // else no new data to process and no error

  delay(50);
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


void
update_lights()
{
  if (state_red_ == SIGNAL_OFF) {
    debug.println("Red OFF");
    digitalWrite(PIN_RED, LOW);
  } else if (state_red_ == SIGNAL_ON) {
    debug.println("Red ON");
    digitalWrite(PIN_RED, HIGH);
  }

  if (state_yellow_ == SIGNAL_OFF) {
    debug.println("Yellow OFF");
    digitalWrite(PIN_YELLOW, LOW);
  } else if (state_yellow_ == SIGNAL_ON) {
    debug.println("Yellow ON");
    digitalWrite(PIN_YELLOW, HIGH);
  }

  if (state_green_ == SIGNAL_OFF) {
    debug.println("Green OFF");
    digitalWrite(PIN_GREEN, LOW);
  } else if (state_green_ == SIGNAL_ON) {
    debug.println("Green ON");
    digitalWrite(PIN_GREEN, HIGH);
  }
}

void
report_error(uint8_t err_num)
{
  debug.print("Error: "); debug.print(err_num);
  debug.print(" - "); debug.println(ERROR_CODES[err_num]);

  digitalWrite(PIN_ERR_LED, LOW);
  delay(300);

  for (int i = 0; i < err_num; ++i) {
    digitalWrite(PIN_ERR_LED, HIGH);
    delay(200);
    digitalWrite(PIN_ERR_LED, LOW);
    delay(200);
  }
}
