/*
  xdrv_08_serial_bridge.ino - serial bridge support for Tasmota

  Copyright (C) 2020  Theo Arends and Dániel Zoltán Tolnai

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_SERIAL_BRIDGE
/*********************************************************************************************\
 * Serial Bridge using Software Serial library (TasmotaSerial)
\*********************************************************************************************/

#define XDRV_08                    8

const uint8_t SERIAL_BRIDGE_BUFFER_SIZE = 50;  //Dont exceed 256!!! 
const uint8_t SERIAL_BRIDGE_PACKET_MARKER_BUFFER_SIZE = 5;
const uint8_t CRCPoly = 0x9b;

// const crcArray[]={0x0,0x9B,0xAD,0x36,0xC1,0x5A,0x6C,0xF7,0x19,0x82,0xB4,0x2F,0xD8,0x43,0x75,0xEE,0x32,0xA9,0x9F,0x4,0xF3,0x68,0x5E,0xC5,0x2B,0xB0,0x86,0x1D,0xEA,0x71,0x47,0xDC,0x64,
//   0xFF,0xC9,0x52,0xA5,0x3E,0x8,0x93,0x7D,0xE6,0xD0,0x4B,0xBC,0x27,0x11,0x8A,0x56,0xCD,0xFB,0x60,0x97,0xC,0x3A,0xA1,0x4F,0xD4,0xE2,0x79,0x8E,0x15,0x23,0xB8,0xC8,0x53,0x65,0xFE,0x9,0x92,
//   0xA4,0x3F,0xD1,0x4A,0x7C,0xE7,0x10,0x8B,0xBD,0x26,0xFA,0x61,0x57,0xCC,0x3B,0xA0,0x96,0xD,0xE3,0x78,0x4E,0xD5,0x22,0xB9,0x8F,0x14,0xAC,0x37,0x1,0x9A,0x6D,0xF6,0xC0,0x5B,0xB5,0x2E,0x18,
//   0x83,0x74,0xEF,0xD9,0x42,0x9E,0x5,0x33,0xA8,0x5F,0xC4,0xF2,0x69,0x87,0x1C,0x2A,0xB1,0x46,0xDD,0xEB,0x70,0xB,0x90,0xA6,0x3D,0xCA,0x51,0x67,0xFC,0x12,0x89,0xBF,0x24,0xD3,0x48,0x7E,0xE5,
//   0x39,0xA2,0x94,0xF,0xF8,0x63,0x55,0xCE,0x20,0xBB,0x8D,0x16,0xE1,0x7A,0x4C,0xD7,0x6F,0xF4,0xC2,0x59,0xAE,0x35,0x3,0x98,0x76,0xED,0xDB,0x40,0xB7,0x2C,0x1A,0x81,0x5D,0xC6,0xF0,0x6B,0x9C,
//   0x07,0x31,0xAA,0x44,0xDF,0xE9,0x72,0x85,0x1E,0x28,0xB3,0xC3,0x58,0x6E,0xF5,0x2,0x99,0xAF,0x34,0xDA,0x41,0x77,0xEC,0x1B,0x80,0xB6,0x2D,0xF1,0x6A,0x5C,0xC7,0x30,0xAB,0x9D,0x6,0xE8,0x73,
//   0x45,0xDE,0x29,0xB2,0x84,0x1F,0xA7,0x3C,0xA,0x91,0x66,0xFD,0xCB,0x50,0xBE,0x25,0x13,0x88,0x7F,0xE4,0xD2,0x49,0x95,0xE,0x38,0xA3,0x54,0xCF,0xF9,0x62,0x8C,0x17,0x21,0xBA,0x4D,0xD6,0xE0,0x7B}

const char kSerialBridgeCommands[] PROGMEM = "|"  // No prefix
  D_CMND_SSERIALSEND "|" D_CMND_SBAUDRATE;

void (* const SerialBridgeCommand[])(void) PROGMEM = {
  &CmndSSerialSend, &CmndSBaudrate };

#include <TasmotaSerial.h>

TasmotaSerial *SerialBridgeSerial = nullptr;

unsigned long serial_bridge_polling_window = 0;
char *serial_bridge_buffer = nullptr;
int serial_bridge_packet_marker_buffer[SERIAL_BRIDGE_PACKET_MARKER_BUFFER_SIZE];
int serial_bridge_in_byte_counter = 0;
int serial_bridge_packet_counter = 0;
bool serial_bridge_active = true;
bool serial_bridge_raw = true;
int in_sync = 0;
uint8_t temp_command=0;
uint8_t crc=0x00;
int curr_packet_bytes=0;
int packet_start_byte_counter=0;
bool ready_to_parse=false;

uint8_t prev_cmd_18[18]={0};
uint8_t prev_cmd_02[2]={0};
uint8_t prev_cmd_03[3]={0};
uint8_t prev_cmd_04[4]={0};
uint8_t prev_cmd_05[5]={0};
uint8_t prev_cmd_06[6]={0};
uint8_t prev_cmd_07[7]={0};

uint8_t* prev_array = nullptr;

void SerialBridgeInput(void)
{
  while (SerialBridgeSerial->available()) {
    yield();
    if (!serial_bridge_raw) {                        // Discard binary data above 127 if no raw reception allowed
      serial_bridge_in_byte_counter = 0;
      SerialBridgeSerial->flush();
      return;
    }
    if (serial_bridge_in_byte_counter > (SERIAL_BRIDGE_BUFFER_SIZE-3 )|| serial_bridge_packet_counter > (SERIAL_BRIDGE_PACKET_MARKER_BUFFER_SIZE-2)){
      AddLog_P2(LOG_LEVEL_INFO, PSTR("Buffer about to overflow!!! in_sync: %d, num_packets=%d, bytes=%d" ),in_sync, serial_bridge_packet_counter,serial_bridge_in_byte_counter);
      in_sync=0;
      serial_bridge_in_byte_counter=0;
      serial_bridge_packet_counter=0;
    } 
    uint8_t serial_in_byte = SerialBridgeSerial->read();

    switch(in_sync){
      case 0:
        ready_to_parse=false;
        temp_command=0;
        if (serial_in_byte==0x5a) {
          in_sync=1;
          //serial_bridge_buffer[serial_bridge_in_byte_counter++] = serial_in_byte;  // Dont save the 0x5a byte starter
        }
        break;
      case 1:
        temp_command=serial_in_byte;
        if (temp_command==0x12 || (temp_command >0 && temp_command <8)){
          in_sync=2;
          packet_start_byte_counter=serial_bridge_in_byte_counter;       //Save the current buffer marker
          serial_bridge_buffer[serial_bridge_in_byte_counter++] = temp_command;
          curr_packet_bytes=0;
          crc=0x00;
          gencrc(&crc,temp_command);                             //CRC includes length/cmd in calculation
        } else {
          serial_bridge_in_byte_counter=packet_start_byte_counter;     //wrong length so discard
          AddLog_P2(LOG_LEVEL_INFO, PSTR("Wrong command! %X"),temp_command);
          in_sync=0;
        }
        ready_to_parse=false;
        break;
      case 2:
        if (curr_packet_bytes<(temp_command)){
          serial_bridge_buffer[serial_bridge_in_byte_counter++] = serial_in_byte;
          curr_packet_bytes++;
          gencrc(&crc,serial_in_byte);
        } else {                                                        // We should have gotten CRC
          if (crc==serial_in_byte){                                     //CRC matched / dont save it
            serial_bridge_packet_marker_buffer[serial_bridge_packet_counter++]=packet_start_byte_counter;
            if (temp_command==0x01){
              in_sync=1;                                                //We treat the response just like any other packet but we're already aligned.
              ready_to_parse=false;
            } else {
              ready_to_parse=true;
              in_sync=0;                                                
            }
          } else {                                                       // CRC didnt check - discard
            serial_bridge_in_byte_counter=packet_start_byte_counter;     //wrong length so discard
            in_sync=0;
            AddLog_P2(LOG_LEVEL_INFO, PSTR("CRC missmatch! - expected %2X got %2X"),serial_in_byte, crc);
          }
        }
        break;    
    } 

    if (ready_to_parse) {
      //AddLog_P2(LOG_LEVEL_INFO, PSTR("would be sending received %d packets with %d bytes"),serial_bridge_packet_counter, serial_bridge_in_byte_counter);
      for (int i=0; i< serial_bridge_packet_counter; i++){
        int pkt_start = serial_bridge_packet_marker_buffer[i];
        uint8_t my_command = serial_bridge_buffer[pkt_start];
        switch (my_command) {
          case 0x12:
            prev_array=prev_cmd_18;
            break;
          case 2:
            prev_array=prev_cmd_02;
            break;
          case 3:
            prev_array=prev_cmd_03;
            break;
          case 4:
            prev_array=prev_cmd_04;
            break;
          case 5:
            prev_array=prev_cmd_05;
            break;
          case 6:
            prev_array=prev_cmd_06;
            break;
          case 7:
            prev_array=prev_cmd_07;
            break;
          default:
            prev_array=nullptr;
        }
        if  (prev_array!=nullptr) {
          if(memcmp(serial_bridge_buffer + pkt_start + 1, prev_array, my_command)){           //if current message different to previous (of same cmd) print and send
              memcpy(prev_array,serial_bridge_buffer + pkt_start + 1,my_command);             //copy current  to prev_cmd array
              if(0x12==my_command){
                //ToHex_P((unsigned char*)prev_array, my_command-1-2, hex_param, sizeof(hex_param));
                uint16_t rpm = prev_array[2]*256 + prev_array[3];
                uint16_t level = prev_array[10]*256 + prev_array[11];
                Response_P(PSTR("{\"wmCmd%d\":{\"door\":%u, \"rpm\":%u, \"temp1\":%u, \"temp2\":%u, \"temp3\":%u, \"cond1\":%u, \"levl1\":%u}}"),
                  my_command,
                  prev_array[1],
                  rpm,
                  prev_array[8],
                  prev_array[14],
                  prev_array[15],
                  prev_array[9],
                  level);
              } else {
                char hex_param[my_command*2+1];
                ToHex_P((unsigned char*)prev_array, my_command, hex_param, sizeof(hex_param));
                Response_P(PSTR("{\"wmCmd%d\":{\"wmParam1\":\"%s\"}}"),
                  my_command, hex_param );
              }
              MqttPublishPrefixTopic_P(RESULT_OR_TELE, PSTR(D_JSON_SSERIALRECEIVED));
          }
        }
      }
      ready_to_parse = false;
      serial_bridge_packet_counter = 0;
      serial_bridge_in_byte_counter = 0;
    }
    //XdrvRulesProcess();
  }
}


/********************************************************************************************/

void SerialBridgeInit(void)
{
  serial_bridge_active = false;
  if (PinUsed(GPIO_SBR_RX) && PinUsed(GPIO_SBR_TX)) {
    SerialBridgeSerial = new TasmotaSerial(Pin(GPIO_SBR_RX), Pin(GPIO_SBR_TX), 1);
    if (SerialBridgeSerial->begin(Settings.sbaudrate * 300)) {  // Baud rate is stored div 300 so it fits into 16 bits
      if (SerialBridgeSerial->hardwareSerial()) {
        ClaimSerial();
        //AddLog_P2(LOG_LEVEL_INFO, PSTR(D_LOG_COMMAND "Using Hardware Serial"));
        serial_bridge_buffer = serial_in_buffer;  // Use idle serial buffer to save RAM
      } else {
        serial_bridge_buffer = (char*)(malloc(SERIAL_BRIDGE_BUFFER_SIZE));
      }
      serial_bridge_active = true;
      SerialBridgeSerial->flush();
    }
  }
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

void CmndSSerialSend(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= 5)) {
    serial_bridge_raw = (XdrvMailbox.index > 3);
    if (XdrvMailbox.data_len > 0) {
      if (1 == XdrvMailbox.index) {
        SerialBridgeSerial->write(XdrvMailbox.data, XdrvMailbox.data_len);  // "Hello Tiger"
        SerialBridgeSerial->write("\n");                                    // "\n"
      }
      else if ((2 == XdrvMailbox.index) || (4 == XdrvMailbox.index)) {
        SerialBridgeSerial->write(XdrvMailbox.data, XdrvMailbox.data_len);  // "Hello Tiger" or "A0"
      }
      else if (3 == XdrvMailbox.index) {                                    // "Hello\f"
        SerialBridgeSerial->write(Unescape(XdrvMailbox.data, &XdrvMailbox.data_len), XdrvMailbox.data_len);
      }
      else if (5 == XdrvMailbox.index) {
        char *p;
        char stemp[3];
        uint8_t code;

        char *codes = RemoveSpace(XdrvMailbox.data);
        int size = strlen(XdrvMailbox.data);

        while (size > 1) {
          strlcpy(stemp, codes, sizeof(stemp));
          code = strtol(stemp, &p, 16);
          SerialBridgeSerial->write(code);                                  // "AA004566" as hex values
          size -= 2;
          codes += 2;
        }
      }
      ResponseCmndDone();
    }
  }
}

void CmndSBaudrate(void)
{
  if (XdrvMailbox.payload >= 300) {
    XdrvMailbox.payload /= 300;  // Make it a valid baudrate
    Settings.sbaudrate = XdrvMailbox.payload;
    SerialBridgeSerial->begin(Settings.sbaudrate * 300);  // Reinitialize serial port with new baud rate
  }
  ResponseCmndNumber(Settings.sbaudrate * 300);
}


/*********************************************************************************************\
 * crc8 calculator
 * For a byte array whose accumulated crc value is stored in crc, computes
 * resultant crc obtained by appending m to the byte array
\*********************************************************************************************/

// uint8_t crc8table(uint8_t crc, uint8_t m)
//      /*
//       * For a byte array whose accumulated crc value is stored in *crc, computes
//       * resultant crc obtained by appending m to the byte array
//       */
// {
//   crc = CRCTable[(crc) ^ m];
//   crc &= 0xFF;
//   return crc;
// }

void gencrc(uint8_t *crc, uint8_t data)
{
    size_t j;
      *crc ^= data;
      for (j = 0; j < 8; j++) {
          if ((*crc & 0x80) != 0)
            *crc = (uint8_t)((*crc << 1) ^ CRCPoly );
          else
            *crc <<= 1;
      }
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv08(uint8_t function)
{
  bool result = false;

  if (serial_bridge_active) {
    switch (function) {
      case FUNC_LOOP:
        if (SerialBridgeSerial) { SerialBridgeInput(); }
        break;
      case FUNC_PRE_INIT:
        SerialBridgeInit();
        break;
      case FUNC_COMMAND:
        result = DecodeCommand(kSerialBridgeCommands, SerialBridgeCommand);
        break;
    }
  }
  return result;
}

#endif // USE_SERIAL_BRIDGE
