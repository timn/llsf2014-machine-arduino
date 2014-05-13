
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
#include <SeeedRFIDLib.h>

// *** PIN assignment for devices
#define PIN_GREEN     7
#define PIN_YELLOW    8
#define PIN_RED       9

#define PIN_RFID_RX   4
#define PIN_RFID_TX   5
#define PIN_RFID_POW  6

#define PIN_DEBUG_RX 11
#define PIN_DEBUG_TX 12

#define PIN_ERR_LED  13


#define RFID_TIMEOUT 1500

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
#define ERR_INVALID_RFID_TAG  9

static const char * ERROR_CODES[] = {
  "No error",
  "Checksum failure",
  "Invalid AT command",
  "Invalid AT parameter",
  "No response to AT command",
  "RefBox unknown",
  "Unknown packet type",
  "Packet too short",
  "Invalid response received",
  "Invalid tag read"
};

// *** network message types
#define MSG_TYPE_SIGNAL_INSTRUCT  1
#define MSG_TYPE_SET_RED          2
#define MSG_TYPE_SET_YELLOW       3
#define MSG_TYPE_SET_GREEN        4
#define MSG_TYPE_RFID_DATA        5
#define MSG_TYPE_RFID_REMOVED     6
#define MSG_TYPE_RFID_WRITE       7

#pragma pack(push,1)
typedef struct {
  uint8_t  msg_type;
  uint32_t tag_id;
} RFIDMessage;

typedef struct {
  uint8_t signal_state_red;
  uint8_t signal_state_yellow;
  uint8_t signal_state_green;
} SignalInstructMessage;
#pragma pack(pop)

// *** refbox address data
XBeeAddress64 refbox_hw_addr_(0,0);
uint16_t      refbox_net_addr_ = 0;
uint8_t       refbox_dn_frame_id_ = 0;

RFIDMessage   rfid_report_msg_;
uint8_t       rfid_report_frame_id_ = 0;

// *** Internal state info
bool blink_state_ = false;
SignalState state_red_    = SIGNAL_ON;
SignalState state_yellow_ = SIGNAL_ON;
SignalState state_green_  = SIGNAL_ON;

uint8_t     frame_id_     = 0;

bool          rfid_visible_ = false;
unsigned long rfid_last_seen_ = 0;


// *** Interaction devices
SoftwareSerial debug(PIN_DEBUG_RX, PIN_DEBUG_TX);
XBee           xbee;
SeeedRFIDLib   rfid(PIN_RFID_RX, PIN_RFID_TX);


// from Ethernet library, repeat to avoid pulling in the whole thing
#define htons(x) ( ((x)<<8) | (((x)>>8)&0xFF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

// *** Forward declarations
void blink_timer();
void update_lights();
void report_error(uint8_t err_num);
uint8_t next_frame_id();


// *** Setup program
void setup()
{
  pinMode(PIN_GREEN,    OUTPUT);
  pinMode(PIN_YELLOW,   OUTPUT);
  pinMode(PIN_RED,      OUTPUT);

  pinMode(PIN_ERR_LED,  OUTPUT);
  pinMode(PIN_RFID_POW, OUTPUT);

  digitalWrite(PIN_GREEN,    HIGH);
  digitalWrite(PIN_YELLOW,   HIGH);
  digitalWrite(PIN_RED,      HIGH);
  digitalWrite(PIN_ERR_LED,  HIGH);
  digitalWrite(PIN_RFID_POW, HIGH);

  Timer1.initialize(250000);
  Timer1.attachInterrupt(blink_timer);

  debug.begin(9600);
  debug.println("STARTED");

  // Restart once to make the RFID's internal SoftwareSerial become
  // the active listener -- there can only be one
  // we don't care for reading on the debug SoftwareSerial anyway
  rfid.restart();

  Serial.begin(9600);
  xbee.begin(Serial);
}


// *** Program loop
void loop()
{
  // if refbox address is unknown, send request. Re-send if the failed
  if (refbox_hw_addr_.getMsb() == 0 && refbox_dn_frame_id_ == 0) {
    report_error(ERR_REFBOX_UNKNOWN);
    digitalWrite(PIN_ERR_LED, HIGH);
    refbox_dn_frame_id_ = next_frame_id();
    AtCommandRequest req((uint8_t*)"DN", (uint8_t*)"RefBox", 6);
    req.setFrameId(refbox_dn_frame_id_);
    xbee.send(req);
  }

  // Read incoming XBee messages
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

	case MSG_TYPE_SET_RED:
	  if (data_length == 2) {
	    state_red_    = (SignalState)data[1];
	    update_lights();
	  } else {
	    debug.println("Invalid SET_RED data length");
	  }
	  break;

	case MSG_TYPE_SET_YELLOW:
	  if (data_length == 2) {
	    state_yellow_    = (SignalState)data[1];
	    update_lights();
	  } else {
	    debug.println("Invalid SET_RED data length");
	  }
	  break;

	case MSG_TYPE_SET_GREEN:
	  if (data_length == 2) {
	    state_green_    = (SignalState)data[1];
	    update_lights();
	  } else {
	    debug.println("Invalid SET_RED data length");
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
      debug.print("Modem status: ");
      debug.println(msr.getStatus());
    } else if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
      AtCommandResponse atr;
      xbee.getResponse().getAtCommandResponse(atr);
      uint8_t *command = atr.getCommand();
      if (command[0] == 'D' && command[1] == 'N' && atr.getFrameId() == refbox_dn_frame_id_) {
	refbox_dn_frame_id_ = 0;

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
	  debug.print(" (");
	  debug.print(refbox_hw_addr_.getMsb(), HEX);
	  debug.print(" ");
	  debug.print(refbox_hw_addr_.getLsb(), HEX);
	  debug.print(")");
	  debug.println("");

	  digitalWrite(PIN_ERR_LED, LOW);
	}
      }
    } else if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      ZBTxStatusResponse txr;
      xbee.getResponse().getZBTxStatusResponse(txr);
      uint8_t frame_id = txr.getFrameId();
      if (rfid_report_frame_id_ == frame_id) {
	rfid_report_frame_id_ = 0;
      }
    } else {
      debug.println("Invalid response");
      report_error(ERR_INVALID_RESPONSE);
    }
  } // else no new data to process and no error

  // Read and send RFID tag if received
  // do not send if refbox address unknown or sending currently in progress
  if (refbox_hw_addr_.getMsb() != 0 && rfid_report_frame_id_ == 0) {
    if(rfid.isIdAvailable()) {
      // we have an RFID tag, read it and send it if valid
      RFIDTag tag = rfid.readId();
      if (tag.valid) {
	if (! rfid_visible_) {
	  debug.print("RFID tag detected: ");
	  debug.println(tag.id, HEX);
	  rfid_visible_ = true;
	}
	rfid_last_seen_ = millis();

	// restart RFID so we get continuous readings
	digitalWrite(PIN_RFID_POW, LOW);
	delay(5);
	digitalWrite(PIN_RFID_POW, HIGH);
	rfid.restart();

	rfid_report_msg_.tag_id   = tag.id;

	rfid_report_frame_id_ = next_frame_id();
	rfid_report_msg_.msg_type = MSG_TYPE_RFID_DATA;
      
	ZBTxRequest txr(refbox_hw_addr_, (uint8_t*)&rfid_report_msg_, sizeof(RFIDMessage));
	txr.setAddress16(refbox_net_addr_);
	txr.setFrameId(rfid_report_frame_id_);
	xbee.send(txr);
      } else {
	report_error(ERR_INVALID_RFID_TAG);
      }
    } else {
      unsigned long now = millis();
      if (now < rfid_last_seen_) {
	debug.println("RFID last seen overflow");
	// overflow, restart
	rfid_last_seen_ = now;
      }
      if (rfid_visible_ && ((now - rfid_last_seen_) > RFID_TIMEOUT)) {
	debug.println("RFID tag lost");
	rfid_visible_ = false;

	uint8_t msg_type = MSG_TYPE_RFID_REMOVED;

	rfid_report_frame_id_ = next_frame_id();
	ZBTxRequest txr(refbox_hw_addr_, &msg_type, 1);
	txr.setAddress16(refbox_net_addr_);
	txr.setFrameId(rfid_report_frame_id_);
	xbee.send(txr);
      }
    }
  }

  delay(20);
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


void update_lights()
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

void report_error(uint8_t err_num)
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

uint8_t next_frame_id()
{
  // zero means ignore frame id
  if (++frame_id_ == 0)  frame_id_ = 1;
  return frame_id_;
}
