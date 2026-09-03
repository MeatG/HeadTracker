#include "crsfout.h"
#include "auxserial.h"
#include "crc8.h"
#include "trackersettings.h"

#if defined(CONFIG_SOC_SERIES_NRF52X)
  #define BACKPACK_BAUD BAUD115200   // nRF52: rekisteriarvo
#else
  #define BACKPACK_BAUD 115200       // muut alustat: baudi suoraan
#endif

CRSF crsfout;

void CrsfOutInit()
{
  crsfout.Begin();
}

volatile bool CRSF::ignoreSerialData = false;
volatile bool CRSF::CRSFframeActive = false; //since we get a copy of the serial data use this flag to know when to ignore it

void inline CRSF::nullCallback(void) {};

void (*CRSF::RCdataCallback1)() = &nullCallback; // function is called whenever there is new RC data.
void (*CRSF::RCdataCallback2)() = &nullCallback; // function is called whenever there is new RC data.

void (*CRSF::disconnected)() = &nullCallback; // called when CRSF stream is lost
void (*CRSF::connected)() = &nullCallback;    // called when CRSF stream is regained

void (*CRSF::RecvParameterUpdate)() = &nullCallback; // called when recv parameter update req, ie from LUA

bool CRSF::firstboot = true;

bool CRSF::CRSFstate = false;

volatile uint8_t CRSF::SerialInPacketLen = 0;                        // length of the CRSF packet as measured
volatile uint8_t CRSF::SerialInPacketPtr = 0;                        // index where we are reading/writing
volatile uint8_t CRSF::SerialInBuffer[100] = {0};                    // max 64 bytes for CRSF packet
volatile uint8_t CRSF::CRSFoutBuffer[CRSF_MAX_PACKET_LEN + 1] = {0}; // max 64 bytes for CRSF packet
volatile uint16_t CRSF::ChannelDataIn[16] = {0};
volatile uint16_t CRSF::ChannelDataInPrev[16] = {0};

volatile uint8_t CRSF::ParameterUpdateData[2] = {0};

volatile crsf_channels_s CRSF::PackedRCdataOut;
volatile crsf_attitude_s CRSF::AttitudeDataOut;
volatile crsfPayloadLinkstatistics_s CRSF::LinkStatistics;

void CRSF::Begin()
{
  AuxSerial_Close();
  AuxSerial_Open(BACKPACK_BAUD, CONF8N1);   // ELRS VRX backpack (HDZero-target) = 115200 8N1
}

void CRSF::sendLinkStatisticsToFC()
{
  uint8_t outBuffer[LinkStatisticsFrameLength + 4] = {0};

  outBuffer[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  outBuffer[1] = LinkStatisticsFrameLength + 2;
  outBuffer[2] = CRSF_FRAMETYPE_LINK_STATISTICS;

  memcpy(outBuffer + 3, (void *)&LinkStatistics, LinkStatisticsFrameLength);

  Crc8 _crc(0xd5);
  uint8_t crc = _crc.calc(&outBuffer[2], LinkStatisticsFrameLength + 1);

  outBuffer[LinkStatisticsFrameLength + 3] = crc;

  AuxSerial_Write(outBuffer, LinkStatisticsFrameLength + 4);
}

void CRSF::sendRCFrameToFC()
{
  // ELRS Backpack MSPv2: MSP_ELRS_BACKPACK_SET_PTR (0x0383), payload 3 x int16 LE
  uint16_t c0 = PackedRCdataOut.ch0;   // GUI: kanava 1
  uint16_t c1 = PackedRCdataOut.ch1;   // GUI: kanava 2
  uint16_t c2 = PackedRCdataOut.ch2;   // GUI: kanava 3

  uint8_t buf[15];
  buf[0] = '$'; buf[1] = 'X'; buf[2] = '<';
  buf[3] = 0;                       // flags
  buf[4] = 0x83; buf[5] = 0x03;     // function 0x0383 (LE)
  buf[6] = 6;    buf[7] = 0;        // payload size (LE)
  buf[8]  = c0 & 0xFF; buf[9]  = c0 >> 8;
  buf[10] = c1 & 0xFF; buf[11] = c1 >> 8;
  buf[12] = c2 & 0xFF; buf[13] = c2 >> 8;
  Crc8 crc(0xD5);                   // MSPv2 CRC8-DVB-S2, sama polynomi kuin CRSF:ssä
  buf[14] = crc.calc(&buf[3], 11);  // flags..payload
  AuxSerial_Write(buf, sizeof(buf));
}

void CRSF::sendAttitideToFC()
{
  uint8_t outBuffer[CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + 4] = {0};

  outBuffer[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // ??
  outBuffer[1] = CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + 2;
  outBuffer[2] = CRSF_FRAMETYPE_ATTITUDE;

  memcpy(outBuffer + 3, (void *)&PackedRCdataOut, CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE);
  Crc8 _crc(CRSF_CRC_POLY);
  uint8_t crc = _crc.calc(&outBuffer[2], CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + 1);

  outBuffer[CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + 3] = crc;

  AuxSerial_Write(outBuffer, CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + 4);
}
