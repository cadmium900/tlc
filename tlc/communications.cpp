///
/// \file       communications.cpp
/// \brief      The Lung Carburetor Firmware communications module
///
/// \author     Frederic Lauzon
/// \ingroup    communications
#include "communications.h"
#include "datamodel.h"
#include "configuration.h"
#include "safeties.h"
#include "control.h"

static bool     gSerialConnected    = false;

/// \struct tRxBuffer
/// \brief Receive buffer state
struct tRxBuffer
{
    uint8_t     data[kCommBufferSize];  ///> Data receive on serial port
    uint8_t     rxSize;                 ///> Size of data buffer
    uint32_t    lastRxTick;             ///> Last reception tick
    uint32_t    lastCmdTick;            ///> Last valid command tick
};
static tRxBuffer    gRxBuffer;
static uint8_t      gTxBuffer[kCommBufferSize];

bool Communications_Init()
{
    gSerialConnected        = false;

    gRxBuffer.rxSize        = 0;
    gRxBuffer.lastRxTick    = millis();
    gRxBuffer.lastCmdTick   = millis();

    Serial.begin(kSerialBaudRate);

    return true;
}

static uint16_t CRC16(uint8_t* pBuffer, uint8_t len)
{
    const uint16_t kPolynomial = 0x1021;
    uint16_t crc = 0;
    for (uint8_t a = 0; a < len; ++a)
    {
        crc ^= pBuffer[a] << 8;
        for(int b = 0; b < 8; ++b)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ kPolynomial;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

static void SendControl(tPacketHeader* packet)
{
    Serial.write((uint8_t*)packet, packet->size);
}

static void SendControlNAck(tPacketHeader* pReplyHeader, tPacketHeader* packet)
{
    memcpy(pReplyHeader, packet, sizeof(tPacketHeader));
    pReplyHeader->cmd   |= kPacketCommand_MaskNAck;
    pReplyHeader->size   = sizeof(tPacketHeader);
    pReplyHeader->crc    = 0;
    pReplyHeader->crc    = CRC16((uint8_t*)pReplyHeader, pReplyHeader->size);
    SendControl(pReplyHeader);
}

static void SendControlAck(tPacketHeader* pReplyHeader, tPacketHeader* packet)
{
    memcpy(pReplyHeader, packet, sizeof(tPacketHeader));
    pReplyHeader->cmd   |= kPacketCommand_MaskAck;
    pReplyHeader->size   = sizeof(tPacketHeader);
    pReplyHeader->crc    = 0;
    pReplyHeader->crc    = CRC16((uint8_t*)pReplyHeader, pReplyHeader->size);
    SendControl(pReplyHeader);
}

#if 0
static void SendControlTest(tPacketHeader* pReplyHeader)
{
    tPacketReadData* payload = (tPacketReadData*)((uint8_t*)pReplyHeader + sizeof(tPacketHeader));
    pReplyHeader->id     = kPacketId;
    pReplyHeader->cmd    = kPacketCommand_ReadData;
    pReplyHeader->size   = sizeof(tPacketHeader) + sizeof(tPacketReadData);
    pReplyHeader->keyCfg = CFGPROTOCOL_KEY;
    pReplyHeader->keyDataModel = PROTOCOL_KEY;
    pReplyHeader->crc    = 0;
    payload->bytesToRead = 17;
    payload->offset      = 0;
    pReplyHeader->crc    = CRC16((uint8_t*)pReplyHeader, pReplyHeader->size);
    SendControl(pReplyHeader);
}
#endif

static void ParseCommand(tPacketHeader* packet)
{
    tPacketHeader*  pReplyHeader    = (tPacketHeader*)gTxBuffer;
    uint8_t*        pReplyPayload   = (uint8_t*)pReplyHeader + sizeof(tPacketHeader);
    switch(packet->cmd)
    {
    case kPacketCommand_ReadData:
        {
            tPacketReadData* payload = (tPacketReadData*)((uint8_t*)packet + sizeof(tPacketHeader));
            if ((payload->offset + payload->bytesToRead <= sizeof(tDataModel)) && (payload->bytesToRead <= kCommBufferSize))
            {
                memcpy(pReplyHeader, packet, sizeof(tPacketHeader));
                pReplyHeader->cmd   |= kPacketCommand_MaskAck;
                pReplyHeader->size   = sizeof(tPacketHeader) + sizeof(tPacketReadData) + payload->bytesToRead;
                pReplyHeader->crc    = 0;

                memcpy(pReplyPayload, payload, sizeof(tPacketReadData));
                uint8_t* pData = (uint8_t*)pReplyPayload + sizeof(tPacketReadData);
                uint8_t* pSrc  = (uint8_t*)&gDataModel;
                memcpy(pData, &pSrc[payload->offset], payload->bytesToRead);

                pReplyHeader->crc    = CRC16((uint8_t*)pReplyHeader, pReplyHeader->size);
                SendControl(pReplyHeader);
            }
            else
            {
                SendControlNAck(pReplyHeader, packet);
            }
        }
        break;

    case kPacketCommand_WriteData:
        {
            tPacketWriteData* payload = (tPacketWriteData*)((uint8_t*)packet + sizeof(tPacketHeader));
            if (payload->offset + payload->bytesToWrite <= sizeof(tDataModel))
            {
                uint8_t* pData = (uint8_t*)payload + sizeof(tPacketWriteData);
                uint8_t* pDst  = (uint8_t*)&gDataModel;

                // Protect nState variable, this variable is read-only for communications
                eState oemState = gDataModel.nState;

                memcpy(&pDst[payload->offset], pData, payload->bytesToWrite);

                gDataModel.nState = oemState;

                memcpy(pReplyHeader, packet, sizeof(tPacketHeader));
                pReplyHeader->cmd   |= kPacketCommand_MaskAck;
                pReplyHeader->size   = sizeof(tPacketHeader) + sizeof(tPacketWriteData);
                pReplyHeader->crc    = 0;

                memcpy(pReplyPayload, payload, sizeof(tPacketReadData));

                pReplyHeader->crc    = CRC16((uint8_t*)pReplyHeader, pReplyHeader->size);
                SendControl(pReplyHeader);
            }
            else
            {
                SendControlNAck(pReplyHeader, packet);
            }
        }
        break;

    case kPacketCommand_ReadCfg:
        {
            tPacketReadData* payload = (tPacketReadData*)((uint8_t*)packet + sizeof(tPacketHeader));
            if ((payload->offset + payload->bytesToRead <= sizeof(tConfiguration)) && (payload->bytesToRead <= kCommBufferSize))
            {
                memcpy(pReplyHeader, packet, sizeof(tPacketHeader));
                pReplyHeader->cmd   |= kPacketCommand_MaskAck;
                pReplyHeader->size   = sizeof(tPacketHeader) + sizeof(tPacketReadData) + payload->bytesToRead;
                pReplyHeader->crc    = 0;

                memcpy(pReplyPayload, payload, sizeof(tPacketReadData));
                uint8_t* pData = (uint8_t*)pReplyPayload + sizeof(tPacketReadData);
                uint8_t* pSrc  = (uint8_t*)&gConfiguration;
                memcpy(pData, &pSrc[payload->offset], payload->bytesToRead);

                pReplyHeader->crc    = CRC16((uint8_t*)pReplyHeader, pReplyHeader->size);
                SendControl(pReplyHeader);
            }
            else
            {
                SendControlNAck(pReplyHeader, packet);
            }
        }
        break;

    case kPacketCommand_WriteCfg:
        {
            tPacketWriteData* payload = (tPacketWriteData*)((uint8_t*)packet + sizeof(tPacketHeader));
            if (payload->offset + payload->bytesToWrite <= sizeof(tConfiguration))
            {
                uint8_t* pData = (uint8_t*)payload + sizeof(tPacketWriteData);
                uint8_t* pDst  = (uint8_t*)&gConfiguration;
                memcpy(&pDst[payload->offset], pData, payload->bytesToWrite);

                memcpy(pReplyHeader, packet, sizeof(tPacketHeader));
                pReplyHeader->cmd   |= kPacketCommand_MaskAck;
                pReplyHeader->size   = sizeof(tPacketHeader) + sizeof(tPacketWriteData);
                pReplyHeader->crc    = 0;

                memcpy(pReplyPayload, payload, sizeof(tPacketReadData));

                pReplyHeader->crc    = CRC16((uint8_t*)pReplyHeader, pReplyHeader->size);
                SendControl(pReplyHeader);
            }
            else
            {
                SendControlNAck(pReplyHeader, packet);
            }
        }
        break;

    case kPacketCommand_WriteCfgToEeprom:
        {
            bool success = Configuration_Write();
            if (success)
            {
                SendControlAck(pReplyHeader, packet);
            }
            else
            {
                SendControlNAck(pReplyHeader, packet);
            }
        }
        break;

    case kPacketCommand_LoadCfgFromEeprom:
        {
            bool success = Configuration_Read();
            if (success)
            {
                SendControlAck(pReplyHeader, packet);
            }
            else
            {
                SendControlNAck(pReplyHeader, packet);
            }
        }
        break;

    case kPacketCommand_SetDefaultCfg:
        {
            bool success = Configuration_SetDefaults();
            if (success)
            {
                SendControlAck(pReplyHeader, packet);
            }
            else
            {
                SendControlNAck(pReplyHeader, packet);
            }
        }
        break;

    case kPacketCommand_SetZeroPressure:
        {
            gConfiguration.nPressureSensorOffset[0] = gDataModel.nRawPressure[0];
            gConfiguration.nPressureSensorOffset[1] = gDataModel.nRawPressure[1];
            gConfiguration.nPressureSensorOffset[2] = gDataModel.nRawPressure[2];
            SendControlAck(pReplyHeader, packet);
        }
        break;

    case kPacketCommand_ClearSafeties:
        {
            Safeties_Clear();
            SendControlAck(pReplyHeader, packet);
        }
        break;

    case kPacketCommand_DisableSafeties:
        {
            Safeties_Disable();
            SendControlAck(pReplyHeader, packet);
        }
        break;

    case kPacketCommand_EnableSafeties:
        {
            Safeties_Enable();
            SendControlAck(pReplyHeader, packet);
        }
        break;

    case kPacketCommand_SetCurve:
        {
            tPacketCurve* payload = (tPacketCurve*)((uint8_t*)packet + sizeof(tPacketHeader));                       
            bool success = true;
            if (success)
            {
                gDataModel.nRespirationPerMinute        = payload->breatheRate;
                gDataModel.fInhalePressureTarget_mmH2O  = payload->inhaleMmH2O;
                gDataModel.fExhalePressureTarget_mmH2O  = payload->exhaleMmH2O;
                gDataModel.fInhaleTime                  = payload->inhaleTime;
                gDataModel.fExhaleTime                  = payload->exhaleTime;

                gDataModel.fExhaleCheckPeepTime         = payload->exhaleCheckPeepTime;
                gDataModel.fExhaleRampTime              = payload->exhaleRampTime;
                gDataModel.fInhaleRampTime              = payload->inhaleRampTime;
                gDataModel.fPeepHighLimit_mmH2O         = payload->peepHighLimit_mmH2O;
                gDataModel.fPeepLowLimit_mmH2O          = payload->peepLowLimit_mmH2O;

                success = Control_SetCurveFromDataModel();
                if (success)
                {
                    SendControlAck(pReplyHeader, packet);
                }
                else
                {
                    SendControlNAck(pReplyHeader, packet);
                }
            }
            else
            {
                SendControlNAck(pReplyHeader, packet);
            }
        }
        break;

    default:
        // Unknown command, ignore to prevent DDOS
        break;
    }
}

void Communications_Process()
{
    // Communications is allowed in all states
    if (!gSerialConnected)
    {
        if (Serial)
        {
            Serial.setTimeout(kSerialRxTimeOut);
            gSerialConnected = true;
        }
    }
    else
    {
        //SendControlTest((tPacketHeader*)gTxBuffer);

        if (Serial.available() > 0)
        {
            if ((millis() - gRxBuffer.lastRxTick) > kSerialDiscardTimeout)
            {
                gRxBuffer.lastRxTick = millis();
                gRxBuffer.rxSize = 0;
            }

            if (millis() - gRxBuffer.lastCmdTick > kSerialCommandTimeout)
            {
               gRxBuffer.lastCmdTick = millis();
               gRxBuffer.rxSize = 0;
            }

            uint8_t  ofs   = gRxBuffer.rxSize;
            uint16_t count = gRxBuffer.rxSize + Serial.available();
            if (count >= kCommBufferSize)
            {
                count = kCommBufferSize;
            }

            int nRead = Serial.readBytes(&gRxBuffer.data[ofs], count-ofs);
            count = gRxBuffer.rxSize + nRead;

            uint8_t cmdOfs = 0;

            // Scan for packet header
            if (count >= sizeof(tPacketHeader))
            {
                for (uint8_t a = 0; a < (uint8_t)count; ++a)
                {
                    tPacketHeader* header = (tPacketHeader*)&gRxBuffer.data[a];
                    if (header->id == kPacketId)
                    {
                        // Check for complete rx'ed size and crc
                        if (count-a >= header->size)
                        {
                            uint16_t oemCrc = header->crc;
                            header->crc = 0;
                            if (oemCrc == CRC16((uint8_t*)header, header->size))
                            {
                                // Check if keys are Valid
                                if (header->keyCfg       == CFGPROTOCOL_KEY &&
                                    header->keyDataModel == PROTOCOL_KEY)
                                {
                                    // Valid Packet Detected!
                                    ParseCommand(header);
                                    gRxBuffer.lastCmdTick   = millis();
                                }
                            }
                            header->crc = oemCrc;

                            cmdOfs = a + header->size;
                            a = cmdOfs;
                        }
                    }
                }
            }

            if (cmdOfs > 0 && cmdOfs <= count)
            {
                memmove(&gRxBuffer.data[0], &gRxBuffer.data[cmdOfs], cmdOfs);
                count = count - cmdOfs;
            }

            // Discard data if we have too much in bank
            if (count > kCommBufferSize - kRxBufferReserve)
            {
                count = 0;
            }

            gRxBuffer.rxSize = count;
            gRxBuffer.lastRxTick = millis();
        }
    }
}
