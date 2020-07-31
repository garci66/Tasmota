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

const uint8_t SERIAL_BRIDGE_BUFFER_SIZE = 200;  //Dont exceed 256!!! 
const uint8_t SERIAL_BRIDGE_PACKET_MARKER_BUFFER_SIZE = 32;

const char kSerialBridgeCommands[] PROGMEM = "|"  // No prefix
  D_CMND_SSERIALSEND "|" D_CMND_SBAUDRATE;

void (* const SerialBridgeCommand[])(void) PROGMEM = {
  &CmndSSerialSend, &CmndSBaudrate };

#include <TasmotaSerial.h>

TasmotaSerial *SerialBridgeSerial = nullptr;

unsigned long serial_bridge_polling_window = 0;
char *serial_bridge_buffer = nullptr;
uint8_t serial_bridge_packet_marker_buffer[SERIAL_BRIDGE_PACKET_MARKER_BUFFER_SIZE];
int serial_bridge_in_byte_counter = 0;
int previous_packet_size=0;
int serial_bridge_packet_counter = 0;
bool serial_bridge_active = true;
bool serial_bridge_raw = true;
bool in_sync = false;

uint8_t prev_cmd_01[22]={0};
uint8_t prev_cmd_02[4]={0};
uint8_t prev_cmd_03[5]={0};
uint8_t prev_cmd_04[6]={0};
uint8_t prev_cmd_06[8]={0};

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
    uint8_t serial_in_byte = SerialBridgeSerial->read();
    serial_bridge_buffer[serial_bridge_in_byte_counter++] = serial_in_byte;
    if (serial_bridge_in_byte_counter > ( SERIAL_BRIDGE_BUFFER_SIZE -1)){
      serial_bridge_polling_window = 0;
    } else {
      serial_bridge_polling_window = millis();                               // Wait for more data
    }
  }

  if ( (serial_bridge_in_byte_counter > ( SERIAL_BRIDGE_BUFFER_SIZE -1)) ||          // Buffer is full and we didnt catch it via millis
      serial_bridge_in_byte_counter && (millis() > (serial_bridge_polling_window + SERIAL_POLLING))) {  //A Packet was completed - lets mark it.
    //serial_bridge_packet_marker_buffer[serial_bridge_packet_counter++]=serial_bridge_in_byte_counter-previous_packet_size;  // Log length of current packet
    //previous_packet_size=serial_bridge_in_byte_counter; 
    serial_bridge_packet_marker_buffer[serial_bridge_packet_counter++]=serial_bridge_in_byte_counter;  // Log start of next packet
    
    //AddLog_P2(LOG_LEVEL_INFO, PSTR(D_LOG_COMMAND "Adding Packet with %d bytes:"), serial_bridge_in_byte_counter-previous_packet_size);
    
    if ((serial_bridge_in_byte_counter > (SERIAL_BRIDGE_BUFFER_SIZE - 28)) ||              // Buffer is almost full
        (serial_bridge_packet_counter > (SERIAL_BRIDGE_PACKET_MARKER_BUFFER_SIZE -1)) ||   // Or packet buffer marker is almost full
        (millis() > (serial_bridge_polling_window + (3 * SERIAL_POLLING))) {               // Or no data has been received in 4 polling intervals
      //serial_bridge_buffer[serial_bridge_in_byte_counter] = 0;                   // Serial data completed
      //AddLog_P2(LOG_LEVEL_INFO, PSTR("Parsing Serial buffer with  %d bytes and %u packets"), serial_bridge_in_byte_counter, serial_bridge_packet_counter);
      int parse_counter=0;
      while( parse_counter < (serial_bridge_in_byte_counter-4 ) ){
        while (serial_bridge_buffer[parse_counter]!=0x5a && parse_counter < ( serial_bridge_in_byte_counter-4 )){
          parse_counter++;
        }
        parse_counter++;
        uint8_t command = serial_bridge_buffer[parse_counter++];                      //parse_counter pointer is at first param byte  
        int copy_bytes=0;
        switch (command) {
          case 1:
            copy_bytes=22;
            prev_array=prev_cmd_01;
            break;
          case 2:
            copy_bytes=4;
            prev_array=prev_cmd_02;
            break;
          case 3:
            copy_bytes=5;
            prev_array=prev_cmd_03;
            break;
          case 4:
            copy_bytes=6;
            prev_array=prev_cmd_04;
            break;
          case 6:
            copy_bytes=8;
            prev_array=prev_cmd_06;
            break;
          default:
            copy_bytes=0;
        }
        if (copy_bytes && (copy_bytes + parse_counter <= serial_bridge_in_byte_counter)) {  //got a valid command and we have enough bytes left
          char hex_param[(copy_bytes-1)*2+1];
          if(memcmp(serial_bridge_buffer + parse_counter, prev_array, copy_bytes)){         //if current message different to previous (of same cmd) print and send
          //if(1){
            memcpy(prev_array,serial_bridge_buffer + parse_counter,copy_bytes);             //copy current  to prev_cmd array
            if(1==command){
              ToHex_P((unsigned char*)prev_array+2, copy_bytes-1-2, hex_param, sizeof(hex_param));
              uint16_t rpm = serial_bridge_buffer[parse_counter+5]*256 + serial_bridge_buffer[parse_counter+6];
              uint16_t level = serial_bridge_buffer[parse_counter+13]*256 + serial_bridge_buffer[parse_counter+14];
              Response_P(PSTR("{\"wmCmd%d\":{\"wmParam1\":\"%s\", \"door\":%u, \"rpm\":%u, \"temp1\":%u, \"temp2\":%u, \"temp3\":%u, \"hum1\":%u, \"levl1\":%u}}"),
                command,
                hex_param,
                serial_bridge_buffer[parse_counter+4],
                rpm,
                serial_bridge_buffer[parse_counter+11],
                serial_bridge_buffer[parse_counter+17],
                serial_bridge_buffer[parse_counter+18],
                serial_bridge_buffer[parse_counter+12],
                level);
            } else {
              ToHex_P((unsigned char*)prev_array, copy_bytes-2, hex_param, sizeof(hex_param));
              Response_P(PSTR("{\"wmCmd%d\":{\"wmParam1\":\"%s\"}}"),
                command, hex_param );
            }
            MqttPublishPrefixTopic_P(RESULT_OR_TELE, PSTR(D_JSON_SSERIALRECEIVED));
          }
          parse_counter += copy_bytes; 
        } else {                                                                          //Only in case of packet with wrong size.
          int i=0;
          while((serial_bridge_packet_marker_buffer[i] <  parse_counter) && (i <  serial_bridge_packet_counter)){   //walk packet start marker buffer to find next start
            i++;
          }
          if ( i <serial_bridge_packet_counter ){
            int junk_bytes = serial_bridge_packet_marker_buffer[i] - parse_counter;
            char hex_junk[junk_bytes*2+1];
            ToHex_P((unsigned char*)(serial_bridge_buffer + parse_counter), junk_bytes, hex_junk, sizeof(hex_junk));
            AddLog_P2(LOG_LEVEL_INFO, PSTR("Received sserial command %d with data: %s"),
              command, hex_junk );
            parse_counter = serial_bridge_packet_marker_buffer[i];
          } else {
            AddLog_P2(LOG_LEVEL_INFO, PSTR("Received sserial command %d and jumped to end of buffer"),command);
            parse_counter = serial_bridge_in_byte_counter;                              // Go to end of input buffer
          }
        }
      }

      // char hex_char[(serial_bridge_in_byte_counter * 2)+2];
      // char hex_count[(serial_bridge_packet_counter * 3)+2];
      // int num_chars=0;
      
      // for (int i=0; i < serial_bridge_packet_counter; i++){
      //   ToHex_P((unsigned char*)(serial_bridge_buffer + num_chars), serial_bridge_packet_marker_buffer[i], hex_char + num_chars, sizeof(hex_char) - 2*num_chars -1);
      //   num_chars+=serial_bridge_packet_marker_buffer[i]*2;                    //one extra space for the delimiter
      //   hex_char[num_chars++]=(char)Settings.serial_delimiter;
      // }
      // hex_char[num_chars]=0;

      // ToHex_P((unsigned char*)serial_bridge_buffer, serial_bridge_in_byte_counter, hex_char, sizeof(hex_char));
      // ToHex_P((unsigned char*)serial_bridge_packet_marker_buffer, serial_bridge_packet_counter, hex_count, sizeof(hex_count),'|');
            
      // Response_P(PSTR("{\"" D_JSON_SSERIALRECEIVED "\":%d - %d : %s - %s}"),
      //   serial_bridge_in_byte_counter,
      //   serial_bridge_packet_counter,
      //   hex_count,
      //   hex_char);

      serial_bridge_in_byte_counter = 0;
      serial_bridge_packet_counter = 0;
      previous_packet_size = 0;
      //XdrvRulesProcess();
    }
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
