#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <dirent.h>
#include <wiringPi.h>
#include <ctype.h>
#include "modbus.h"
#include "energycampi.h"
#include "bmp.h"




 int32_t getFileLength(char* f) {
    FILE *fp;
    if ((fp = fopen(f, ("rb"))) == NULL) {
        fprintf(stderr, "ERROR: Cannot open file >%s<\n", f);
        return -1;
    } else {
        int32_t length = 0;

        fseek(fp, 0, SEEK_END);
        length = ftell(fp);
        fseek(fp, 0, 0);       // set file ofs to beginning again
        fclose(fp);
        return length;
    }
}

 int32_t readFile(char* f, char *buf, int32_t len) {
    FILE *fp;
    if ((fp = fopen(f, ("rb"))) == NULL) {
        fprintf(stderr, "ERROR: Cannot open file >%s<\n", f);
        return -1;
    } else {
        int32_t indx      = 0;
        int32_t readBytes = 0;

        fseek(fp, 0, 0);
        
        while ((readBytes = fread(&buf[indx], 1, 1, fp)) == 1 && indx < len) {
            indx += readBytes;
        }
        fclose(fp);
    }
    return 0;
}


/* crc_tab[] -- this crcTable is being build by chksum_crc32GenTab().
*   so make sure, you call it before using the other
*   functions!
*/
uint32_t crc_tab[256];
bool crcTableInitialized = false;

/* chksum_crc32gentab() --      to a global crc_tab[256], this one will
*       calculate the crcTable for crc32-checksums.
*       it is generated to the polynom [..]
*/

void chksum_crc32gentab ()
{
    uint32_t crc, poly;
    uint32_t i, j;

    poly =  0xEDB88320L;    /* CRC-32 Polynom 0x04C11DB7, umgekehrte Bitfolge */

    for (i = 0; i < 256; i++) {
        crc = i;
        for (j = 8; j > 0; j--) {
            if (crc & 1)
                crc = (crc >> 1) ^ poly;
            else
                crc >>= 1;
        }
        crc_tab[i] = crc;
    }
    crcTableInitialized = true;
}

/*
 *  pBuf            buffer
 *  bufLen          Length of complete buffer even when startOffset is used
 *                    e.g. when buffer has 4 bytes and startOffset is len must still be given with 4
 *  omitAddr        When > 0 two byte starting at this address will be omitted, is relative to buffer start
 */
uint32_t crc32(const uint8_t *pBuf, int32_t omitAddr, uint32_t length) {
    uint32_t crc;
    int32_t i;

    if (!crcTableInitialized)
        chksum_crc32gentab();

    crc = 0xFFFFFFFF;
    for (i = 0; i < (int32_t)length; i++)
    {
        if (omitAddr >= 0 &&
            (i == omitAddr   ||
             i == omitAddr+1 ||
             i == omitAddr+2 ||
             i == omitAddr+3
            )) {
                continue;
        }
        crc = ((crc >> 8) & 0x00FFFFFF) ^ crc_tab[(crc ^ pBuf[i]) & 0xFF];
    }
    return (crc ^ 0xFFFFFFFF);
}

bool saIsImageHeaderValid(const tpsaImageHeader pImageHeader, saImageType_t expectedImageType) {
    bool magicOK = false;
    bool typeOK  = false;

    magicOK = pImageHeader->magic     == SA_IMAGE_HEADER_MAGIC;
    typeOK  = pImageHeader->imageType == expectedImageType;

    if (!magicOK)
  	    fprintf(stderr, "img magic wrong 0x%08X != 0x%08X\n", pImageHeader->magic, SA_IMAGE_HEADER_MAGIC);

    if (!typeOK)
  	    fprintf(stderr, "img type wrong 0x%04X != 0x%04X\n", pImageHeader->imageType, expectedImageType);
    
    return magicOK && typeOK;
}

bool saIsImageValid(const tpsaImage pImage, saImageType_t expectedImageType, bool checkCRC) {
    tpsaImageHeader pImageHeader = NULL;
    bool hdrOK = false;
    bool crcOK = true;

    pImageHeader = (tpsaImageHeader)pImage;
    hdrOK        = saIsImageHeaderValid(pImageHeader, expectedImageType);

    if (checkCRC) {
        if (hdrOK)
            crcOK = pImageHeader->crc32 == crc32((uint8_t*)pImage, 0, pImageHeader->length);
        else
            crcOK = false;
    }
    return hdrOK && crcOK;
}

//open serial port
int EnergyCamOpen(modbus_t** ctx,unsigned int Connection, unsigned int dwPort, unsigned int Baud, unsigned int slave) {
    char comDeviceName[100];
    
	
	if(EXPANSIONPORT == Connection)
			sprintf(comDeviceName, "/dev/ttyAMA%d", dwPort);  //Expansion connector  
	else 
			sprintf(comDeviceName, "/dev/ttyUSB%d", dwPort); //connected via USB
				
    *ctx = (modbus_t* )modbus_new_rtu(comDeviceName, Baud, 'E', 8, 1); // parity 'E'ven used by EnergyCam's freemodbus implementation
    
    if(NULL == *ctx)
        return MODBUSERROR;
    
    modbus_set_debug( *ctx, false);
    modbus_set_slave( *ctx, slave);
    
    if (modbus_connect(*ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        
        modbus_free(*ctx);
        *ctx = NULL;
        return MODBUSERROR; 
    } 

    return MODBUSOK;
}


//free resources
bool EnergyCamClose(modbus_t** ctx) {
    if (NULL != *ctx) {
        modbus_close(*ctx);
        modbus_free(*ctx);
        *ctx = NULL;
    }
    return true;    
}


//read manufacture ID
int EnergyCam_GetManufacturerIdentification(modbus_t* ctx, uint16_t* pData) {
    uint32_t readRegCnt;
    
    const uint32_t regCnt = 1;
    uint16_t inputRegs[regCnt];
    
    if(NULL == pData) 
        return MODBUSERROR;
    
    readRegCnt = modbus_read_input_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_COMMON_INPUTREG_MEMMAP_MANUFACTURERIDENTIFICATION), regCnt, &inputRegs[0]);
    if (readRegCnt != -1) {
        *pData= inputRegs[0];
        return MODBUSOK;
  }
  else {
        //fprintf(stderr,"EnergyCamHost_GetManufacturerIdentification  failed \r\n");
        return MODBUSERROR;
  }    
    return MODBUSERROR; 
} 

//read Buildnumber of Firmware
int EnergyCam_GetAppFirmwareBuildNumber(modbus_t* ctx,uint32_t* pBuildNumber,uint16_t InfoFlag) {
    uint32_t readRegCnt;
    const uint32_t regCnt = 2;
    uint16_t inputRegs[regCnt];

    if(NULL == pBuildNumber) 
        return MODBUSERROR;

    readRegCnt = modbus_read_input_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_COMMON_INPUTREG_MEMMAP_APPBUILDNUMBER1), regCnt, &inputRegs[0]);
    if (readRegCnt != -1) {
        uint32_t tmp = 0;
        tmp  = ((uint32_t)inputRegs[0]) << 16;
        tmp |=            inputRegs[1];
        *pBuildNumber = tmp;
        return MODBUSOK;
    } else {
        if(InfoFlag > SILENTMODE) fprintf(stderr,"EnergyCam_GetAppFirmwareBuildNumber ..\n");
        return MODBUSERROR;
    }    
    return MODBUSERROR; 
}

//update firmware
int EnergyCam_UpdateMCU(modbus_t* ctx,char* file,uint16_t InfoFlag) {
    uint32_t        writeRegCnt;
    uint32_t        regCnt;
    uint32_t        length     = getFileLength(file);
    tpsaImage       pImage     = (tpsaImage)malloc(length);
    uint32_t        imageChunkStartAddr;        // includes the image header in case of precedingHeader == ENERGYCAMHOST_UPDATEMCU_MODE_PRECEDINGHEADER
    uint32_t        imageBytes2Write;
    uint32_t        imageBytes2WriteLast;    
    uint32_t        curRegs2Write;
    tpsaImageHeader pImageHdr     = NULL;
    uint32_t        imageReadAddr = 0;
    uint32_t        chunkWriteTryCnt = 0;
    uint16_t        inputRegsCRCOK;
    uint32_t        reg;
    uint32_t        curBuild        = 0;
    uint16_t        curFirmwareType = 0;
    uint32_t        updBuild        = 0;
    uint16_t        updFirmwareType = 0;
    uint32_t        iRetry;
    
    struct timeval responseTimeoutStandard   = {0, (500*1000)};
    struct timeval responseTimeoutEraseFlash = {10, 0};             // on write of very first update image chunk which lead to a flash erase on EnergyCam which last long

    
    #define CHUNK_WRITE_TRY_COUNT 10

    if (pImage == NULL) {
        fprintf(stderr, "malloc failed\n");
        return MODBUSERROR;
    }

    if (readFile(file, (char*)pImage, length) < 0) {
        fprintf(stderr, "Cannot read file\n");
        return MODBUSERROR;
    }

    pImageHdr = (tpsaImageHeader)pImage; // header is at at beginning of update image

    if (saIsImageValid(pImage, SA_IMG_TYPE_APP_SENSOR, true)) {
        if(InfoFlag > SILENTMODE) printf("Update file is valid and CRC OK\n");
    } else {
        fprintf(stderr, "Update file is invalid or CRC wrong\n");
        return MODBUSERROR;
    }

    iRetry = 3;
    do {
        if(iRetry-- < 0) break;
    } while(MODBUSERROR == EnergyCam_GetAppFirmwareBuildNumber(ctx,&curBuild,InfoFlag));
  
    if (iRetry && MODBUSOK == EnergyCam_GetAppFirmwareType(ctx,&curFirmwareType)) {
        if(InfoFlag > SILENTMODE)  printf("Current: Build %d, firmwareType %d\n", curBuild, curFirmwareType);
    } else {
        fprintf(stderr, "Retrieving of properties of current app failed\n");
        return MODBUSERROR;
    }
    
    updBuild        = pImageHdr->version;
    updFirmwareType = pImageHdr->firmwareType;
    if(InfoFlag > SILENTMODE) printf("Update : Build %d, firmwareType %d\n", updBuild, updFirmwareType);
    
    if (updBuild > curBuild || (updBuild >= curBuild && curFirmwareType != updFirmwareType)) {
        if(InfoFlag > SILENTMODE) printf("Starting update...\n");
    } else {
        if (updBuild < curBuild)
            fprintf(stderr, "Downgrades not allowed, abort\n");
        if (updBuild == curBuild)
            fprintf(stderr, "Firmware already installed, abort\n");
        return MODBUSERROR;
    }

    tModbusUpdateChunk modbusUpdateChunk;

    imageBytes2Write = length;
    imageBytes2Write += sizeof(tsaImageHeader);
    imageBytes2WriteLast = imageBytes2Write;

    imageChunkStartAddr = 0;
    imageReadAddr       = imageChunkStartAddr;

    modbus_set_response_timeout(ctx, &responseTimeoutStandard);

    do {
        uint8_t* pImageAsBytes = NULL;

        if (imageChunkStartAddr == 0) {
            curRegs2Write = (sizeof(tsaImageHeader)+1) / 2; // /2 because from byte / uint16, +1 to round up to whole uint16 just in case the header is not a multiple of uint16
            pImageAsBytes = (uint8_t*)pImageHdr;

        } else {
            curRegs2Write = ((imageBytes2Write < UPDATE_IMAGE_DATA_BYTE_CNT)? imageBytes2Write : UPDATE_IMAGE_DATA_BYTE_CNT) / 2; // /2 because from byte / uint16
            pImageAsBytes = (uint8_t*)pImage;
        }

        modbusUpdateChunk.logical.chunkStartAddrHigh = (uint16_t)(imageChunkStartAddr>>16);  // modbus_write_registers() is converting internally to big endian, so keep then here in host byte order
        modbusUpdateChunk.logical.chunkStartAddrLow  = (uint16_t) imageChunkStartAddr;       // modbus_write_registers() is converting internally to big endian, so keep then here in host byte order

        for (reg=0; reg < curRegs2Write; reg++) {
            uint16_t data = 0;

            data  = ((uint16_t)*(pImageAsBytes+imageReadAddr+reg*2+1)) << 8;                // construct a uint16_t in host byte order (little endian)
            data |= ((uint16_t)*(pImageAsBytes+imageReadAddr+reg*2+0));            

            modbusUpdateChunk.logical.imageData[reg] = data;                                // modbus_write_registers() is converting internally to big endian, so keep them here in host byte order
        }

        if (imageChunkStartAddr == 0) {
            modbus_set_response_timeout(ctx, &responseTimeoutEraseFlash);
            fprintf(stderr, "modbus_set_response_timeout(%d s, %d ms)\n", (int)responseTimeoutEraseFlash.tv_sec, (int)responseTimeoutEraseFlash.tv_usec/1000);
        } else {
            struct timeval old_response_timeout;
            /* Save original timeout */
            modbus_get_response_timeout(ctx, &old_response_timeout);
            if (memcmp(&old_response_timeout, &responseTimeoutStandard, sizeof(struct timeval)) != 0) {
                fprintf(stderr, "modbus_get_response_timeout() = %d s, %d ms\n", (int)old_response_timeout.tv_sec, (int)old_response_timeout.tv_usec/1000);
                /* Define a new timeout! */
                modbus_set_response_timeout(ctx, &responseTimeoutStandard);
                fprintf(stderr, "modbus_set_response_timeout(%d s, %d ms)\n", (int)responseTimeoutStandard.tv_sec, (int)responseTimeoutStandard.tv_usec/1000);
            }
        }

        if (imageChunkStartAddr == 0) {
            fprintf(stderr, "wait for flash erase on EnergyCam....\n");
        }

        chunkWriteTryCnt++;
        writeRegCnt = modbus_write_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_COMMON_HOLDINGREG_VIRTUAL_UPDATECHUNKSTREAM), curRegs2Write+2, &modbusUpdateChunk.asModbusRegs[0]); // +2 regs more for address of current chunk (=sizeof(uint32_t)) 

        if (writeRegCnt == -1) {
            if (chunkWriteTryCnt < CHUNK_WRITE_TRY_COUNT) {
                 if(InfoFlag > SILENTMODE) {
					 fprintf(stderr, "modbus_write_registers() failed with '%s'\n", modbus_strerror(errno));
                     fprintf(stderr, "write of chunk to addr 0x%08x failed, try %d/%d\n", ((uint32_t)modbusUpdateChunk.logical.chunkStartAddrHigh << 16) | modbusUpdateChunk.logical.chunkStartAddrLow, chunkWriteTryCnt, CHUNK_WRITE_TRY_COUNT);
				 }
                continue;
            } else {
                if(InfoFlag > SILENTMODE) fprintf(stderr, "EnergyCamHost_UpdateMCU() failed\n");
                return MODBUSERROR;
            }
        } else
            chunkWriteTryCnt = 0;

        if (imageChunkStartAddr == 0) 
            imageReadAddr = 0;      // header which precedes image is taken from image and therefore restart read address
        else
            imageReadAddr += curRegs2Write*2;

        imageBytes2Write    -= curRegs2Write*2;
        imageChunkStartAddr += curRegs2Write*2;

        if (writeRegCnt != -1){
			if(imageBytes2WriteLast - imageBytes2Write > 16*1024){
				imageBytes2WriteLast = imageBytes2Write;
             if(InfoFlag > SILENTMODE)  printf("% 6d/%d bytes written, % 6d bytes remaining\n", length - imageBytes2Write, length, imageBytes2Write);        
		    }
		 }
    } while(imageBytes2Write > 0);

    regCnt = 1;
    writeRegCnt = modbus_read_input_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_COMMON_INPUTREG_VIRTUAL_UPDATECRCOK), regCnt, &inputRegsCRCOK);
    if (writeRegCnt == -1) {
        fprintf(stderr, "modbus_read_input_registers() failed with '%s'\n", modbus_strerror(errno));
        return MODBUSERROR;
    } else {
        bool crcOK = inputRegsCRCOK == 1;
        fprintf(stderr, "CRC of written update image in EnergyCam: %s\n", crcOK?"OK":"failed");
        if (crcOK)
            return MODBUSOK;
        else 
            return MODBUSERROR;        
    }
}


//read Apptype of Firmware
int EnergyCam_GetAppFirmwareType(modbus_t* ctx,uint16_t* pFirmwareType) {
    uint32_t readRegCnt;
    const uint32_t regCnt = 1;
    uint16_t inputRegs[regCnt];

    if(NULL == pFirmwareType) 
        return MODBUSERROR;

    readRegCnt = modbus_read_input_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_COMMON_INPUTREG_MEMMAP_APPFIRMWARETYPE), regCnt, &inputRegs[0]);
    if (readRegCnt != -1) {
        uint32_t tmp = 0;
        tmp  = inputRegs[0];
        *pFirmwareType = tmp;
        return MODBUSOK;
    } else {
        //fprintf(stderr,"EnergyCam_GetAppFirmwareType failed\n");
        return MODBUSERROR;
    }    
    return MODBUSERROR; 
}

//read Result of Installation
int EnergyCam_GetResultOCRInstallation(modbus_t* ctx,uint16_t* pData) {
    uint32_t readRegCnt;

    const uint32_t regCnt = 1;
    uint16_t inputRegs[regCnt];
    
    if(NULL == pData) 
        return MODBUSERROR;

    readRegCnt = modbus_read_input_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_INPUTREG_MEMMAP_RESULTINSTALLATION), regCnt, &inputRegs[0]);
    if (readRegCnt != -1) {
        *pData = inputRegs[0];         
         
        return MODBUSOK;
    } else {
	    //fprintf(stderr,"EnergyCam_GetResultOCRInstallation  failed \r\n");
        return MODBUSERROR;
    }    
    return MODBUSERROR; 
}

//Trigger a new Reading
int EnergyCam_TriggerReading(modbus_t* ctx) {
    uint32_t wroteRegCnt;
    const uint32_t regCnt = 1;
    uint16_t holdingRegs[1] = {1};

    wroteRegCnt = modbus_write_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_HOLDINGREG_MEMMAP_ACTIONOCR), regCnt, &holdingRegs[0]);
    if (wroteRegCnt == -1) {
        fprintf(stderr, "EnergyCam_TriggerReading failed with '%s'\n", modbus_strerror(errno));
        return MODBUSERROR;
    } else {
        fprintf(stdout, "TriggerReading\n");
        return MODBUSOK;
    }
    return MODBUSERROR;     
}

//Trigger a Installation
int EnergyCam_TriggerInstallation(modbus_t* ctx) {
    uint32_t wroteRegCnt;
    const uint32_t regCnt = 2;
    uint16_t holdingRegs[2] = {100,1};

    wroteRegCnt = modbus_write_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_HOLDINGREG_MEMMAP_ACTIONOCRINSTALLATIONTO), regCnt, &holdingRegs[0]);
    if (wroteRegCnt == -1) {
        fprintf(stderr, "TriggerInstallation failed with '%s'\n", modbus_strerror(errno));
        return MODBUSERROR;
    } else {
        fprintf(stdout, "TriggerInstallation\n");
        return MODBUSOK;
    }
    return MODBUSERROR;     
}


//Start saving picture
int EnergyCam_BMPStart(modbus_t* ctx) {
    uint32_t wroteRegCnt;
    const uint32_t regCnt = 1;
    uint16_t holdingRegs[1] = {1};

    wroteRegCnt = modbus_write_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_HOLDINGREG_MEMMAP_ACTIONBMP), regCnt, &holdingRegs[0]);
    if (wroteRegCnt == -1) {
        fprintf(stderr, "EnergyCam_TriggerReading failed with '%s'\n", modbus_strerror(errno));
        return MODBUSERROR;
    } else {
        //fprintf(stdout, "TriggerReading\n");
        return MODBUSOK;
    }
    return MODBUSERROR;     
}

//fill picture buffer
int EnergyCam_BMPFillBuffer(modbus_t* ctx, uint8_t *pBuffer, uint32_t BufferSize) 
{
	uint32_t curRegs2Read;
    uint32_t imageBytes2Read = BufferSize;
	uint32_t readRegCnt;
	uint8_t  *pB;


	curRegs2Read=64;
	pB=pBuffer;

	do{
		curRegs2Read = ((imageBytes2Read < BMP_IMAGE_DATA_BYTE_CNT)? imageBytes2Read : BMP_IMAGE_DATA_BYTE_CNT) / 2; // /2 because from byte / uint16


		readRegCnt = modbus_read_input_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_COMMON_INPUTREG_VIRTUAL_IMAGEREADFIRST), curRegs2Read, (unsigned short *)pB); // +2 regs more for address of current chunk (=sizeof(uint32_t)) 
		pB += curRegs2Read*2;

		imageBytes2Read -= curRegs2Read*2;
		if (readRegCnt == -1) {
			fprintf(stderr, "modbus_read_input_registers() failed with '%s'\n", modbus_strerror(errno));
			return MODBUSERROR;
		}

	}
	while(imageBytes2Read>0);


return MODBUSOK;

}



//Read the Status of the Reading
int EnergyCam_GetStatusReading(modbus_t* ctx,uint16_t* pStatus) {
    uint32_t readRegCnt;
    const uint32_t regCnt = 3;
    uint16_t inputRegs[regCnt];
    
    if(NULL == pStatus) 
        return MODBUSERROR;
    
    readRegCnt = modbus_read_input_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_INPUTREG_MEMMAP_RESULTOCRVALID), regCnt, &inputRegs[0]);
    if (readRegCnt != -1) {
        *pStatus = inputRegs[0];
        return MODBUSOK;
    } else {
        //fprintf(stderr,"EnergyCam_GetStatusReading failed\n");
        return MODBUSERROR;
    }    
    return MODBUSERROR; 
}

//Read OCR Result
int EnergyCam_GetResultOCRInt(modbus_t* ctx, uint32_t* pInt, uint16_t* pFrac) {
    uint32_t readRegCnt;
    const uint32_t regCnt = 3;
    uint16_t inputRegs[regCnt];
    
    if(NULL == pInt) 
        return MODBUSERROR;
    if(NULL == pFrac) 
        return MODBUSERROR;
    
    readRegCnt = modbus_read_input_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_INPUTREG_MEMMAP_RESULTOCRINTHIGH), regCnt, &inputRegs[0]);
    if (readRegCnt != -1) {
        uint32_t tmp = 0;
        tmp  = ((uint32_t)inputRegs[0]) << 16;
        tmp |=            inputRegs[1];
        *pInt  = tmp;
        *pFrac = inputRegs[2];
        return MODBUSOK;
    } else {
        //fprintf(stderr,"EnergyCam_GetResultOCRInt failed\n");
        return MODBUSERROR;
    }    
    return MODBUSERROR; 
}

//Read number of pictures done by OCR
int EnergyCam_GetOCRPicDone(modbus_t* ctx,uint16_t* pCount) {
    uint32_t readRegCnt;
    const uint32_t regCnt = 1;
    uint16_t inputRegs[regCnt];
    
    if(NULL == pCount) 
        return MODBUSERROR;
    
    readRegCnt = modbus_read_input_registers(ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_INPUTREG_MEMMAP_OCRPICDONE), regCnt, &inputRegs[0]);
    if (readRegCnt != -1) {
        *pCount = inputRegs[0];
        return MODBUSOK;
    } else {
        fprintf(stderr,"EnergyCam_GetOCRPicDone failed\n");
        return MODBUSERROR;
    }    
    return MODBUSERROR; 
}

unsigned int  EnergyCam_Log2BMPFile(modbus_t* ctx,const char * path,uint32_t Build,uint16_t InfoFlag)
{
	FILE    * hFileBMP;
	uint8_t *pBMP = NULL;
	unsigned int   dwSize=0;
	unsigned int   dwRW;
	unsigned int	FileCount = 0;
	int	Retry = 2;
	char  FilePath[_MAX_PATH];
	char  Temp[_MAX_PATH];
	int Result;
	

	int iX;
		
	DIR *d;
	struct dirent *dir;
	
	
	sprintf(FilePath,"%s/img",path);
	mode_t process_mask = umask(0);
	mkdir(FilePath,S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
	umask(process_mask);
		
	
	d = opendir(FilePath);
	if(d)
	{
		while((dir = readdir(d)) != NULL)
		   FileCount++;
		closedir(d);
	}
	FileCount--; //as . and .. are also counted
	
	sprintf(FilePath,"%s/Image_ecpi_%04d_%06d.bmp",FilePath,Build,FileCount);
	
	if(InfoFlag > SILENTMODE) printf("Log2BMPFile : %s \r\n",FilePath);
	dwSize=XSIZE*YSIZE;
	pBMP = (uint8_t *) malloc(dwSize+1024+sizeof(aryBmpHeadGrey));
	Result = MODBUSOK;

    do {

		if(ctx) {
			Result =  EnergyCam_BMPStart(ctx);
			if (MODBUSERROR == Result) {
				printf("ECHostService ActionBMP failed \r\n");
				return MODBUSERROR;
			}
		}


		memset(pBMP,0,XSIZE*YSIZE+sizeof(aryBmpHeadGrey));
		memcpy(pBMP,aryBmpHeadGrey,sizeof(aryBmpHeadGrey));

		BmpWriteHdrInfo(pBMP,XSIZE,YSIZE,8,sizeof(aryBmpHeadGrey));

		Result = EnergyCam_BMPFillBuffer(ctx,pBMP+sizeof(aryBmpHeadGrey),dwSize);

		if(Result == MODBUSOK){
			if ( (hFileBMP = fopen(FilePath, "wb")) != NULL ) {
					fwrite(pBMP, dwSize+sizeof(aryBmpHeadGrey), 1, hFileBMP);	
					fclose(hFileBMP);
					chmod(FilePath, 0755);
				}
		}
	}while ((Retry-- > 0) && (Result != MODBUSOK));

	if(pBMP) free( pBMP);

	return FileCount;
}
