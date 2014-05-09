#ifndef ENERGYCAMMODBUS_H
#define ENERGYCAMMODBUS_H

#include "energycampi.h"



int EnergyCamOpen(modbus_t** ctx,unsigned int Connection, unsigned int dwPort, unsigned int Baud, unsigned int slave);
bool EnergyCamClose(modbus_t** ctx);
int EnergyCam_GetManufacturerIdentification(modbus_t* ctx, uint16_t* pData);
int EnergyCam_GetAppFirmwareBuildNumber(modbus_t* ctx,uint32_t* pBuildNumber,uint16_t InfoFlag);
int EnergyCam_UpdateMCU(modbus_t* ctx,char* file,uint16_t InfoFlag);
int EnergyCam_GetResultOCRInstallation(modbus_t* ctx,uint16_t* pData);
int EnergyCam_TriggerReading(modbus_t* ctx);
int EnergyCam_WaitforReadingDone(modbus_t* ctx);
int EnergyCam_TriggerInstallation(modbus_t* ctx);
int EnergyCam_BMPStart(modbus_t* ctx);
int EnergyCam_BMPFillBuffer(modbus_t* ctx, uint8_t *pBuffer, uint32_t BufferSize);
int EnergyCam_GetStatusEnergyCam(modbus_t* ctx,uint16_t* pStatus);
int EnergyCam_GetStatusReading(modbus_t* ctx,uint16_t* pStatus);
int EnergyCam_GetResultOCRInt(modbus_t* ctx, uint32_t* pInt, uint16_t* pFrac);
int EnergyCam_GetOCRPicDone(modbus_t* ctx,uint16_t* pCount);
int EnergyCam_GetOCRReadingPeriod(modbus_t* ctx,uint16_t* pData);
unsigned int  EnergyCam_Log2BMPFile(modbus_t* ctx,const char * path,uint32_t Build,uint16_t InfoFlag);


#endif
