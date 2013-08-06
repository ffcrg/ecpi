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
#include "modbus.h"
#include "energycampi.h"



modbus_t* m_ctx = NULL;

#ifndef TRUE
  #define TRUE  (1==1)
  #define FALSE (1==2)
#endif

#ifndef MAX
  #define MAX(x,y) ((x>y) ? x:y)
  #define MIN(x,y) ((x>y) ? y:x)
#endif 

#define _MAX_PATH 275
#define XSIZE     320
#define YSIZE     80

//colorcoding

#define PRINTF_BRIGHT  1
#define PRINTF_BLACK   0
#define PRINTF_RED     1
#define PRINTF_GREEN   2
#define PRINTF_YELLOW  3
#define PRINTF_BLUE    4
#define PRINTF_MAGENTA 5
#define PRINTF_CYAN    6
#define PRINTF_WHITE   7

struct timeval responseTimeoutStandard   = {0, (500*1000)};
struct timeval responseTimeoutEraseFlash = {10, 0};             // on write of very first update image chunk which lead to a flash erase on EnergyCam which last long

static int32_t getFileLength(char* f) {
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

static int32_t readFile(char* f, char *buf, int32_t len) {
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

void Colour(int8_t c, bool cr) {
  printf("%c[%dm",0x1B,(c>0) ? (30+c) : c);
  if(cr) 
    printf("\n");  
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
int EnergyCamOpen(unsigned int Connection, unsigned int dwPort, unsigned int Baud, unsigned int slave) {
    char comDeviceName[100];
    
	
	if(EXPANSIONPORT == Connection)
			sprintf(comDeviceName, "/dev/ttyAMA%d", dwPort);  //Expansion connector  
	else 
			sprintf(comDeviceName, "/dev/ttyUSB%d", dwPort); //connected via USB
				
    m_ctx = (modbus_t* )modbus_new_rtu(comDeviceName, Baud, 'E', 8, 1); // parity 'E'ven used by EnergyCam's freemodbus implementation
    
    if(NULL == m_ctx)
        return MODBUSERROR;
    
    modbus_set_debug( m_ctx, false);
    modbus_set_slave( m_ctx, slave);
    
    if (modbus_connect(m_ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        
        modbus_free(m_ctx);
        m_ctx = NULL;
        return MODBUSERROR; 
    } 

    return MODBUSOK;
}


//free resources
bool EnergyCamClose(void) {
    if (NULL != m_ctx) {
        modbus_close(m_ctx);
        modbus_free(m_ctx);
        m_ctx = NULL;
    }
    return true;    
}


//read manufacture ID
int EnergyCam_GetManufacturerIdentification(uint16_t* pData) {
    uint32_t readRegCnt;
    
    const uint32_t regCnt = 1;
    uint16_t inputRegs[regCnt];
    
    if(NULL == pData) 
        return MODBUSERROR;
    
    readRegCnt = modbus_read_input_registers(m_ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_COMMON_INPUTREG_MEMMAP_MANUFACTURERIDENTIFICATION), regCnt, &inputRegs[0]);
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

int EnergyCam_UpdateMCU(char* file) {
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
        printf("Update file is valid and CRC OK\n");
    } else {
        fprintf(stderr, "Update file is invalid or CRC wrong\n");
        return MODBUSERROR;
    }

    iRetry = 3;
    do {
        if(iRetry-- < 0) break;
    } while(MODBUSERROR == EnergyCam_GetAppFirmwareBuildNumber(&curBuild));
  
    if (iRetry && MODBUSOK == EnergyCam_GetAppFirmwareType(&curFirmwareType)) {
        printf("Current: Build %d, firmwareType %d\n", curBuild, curFirmwareType);
    } else {
        fprintf(stderr, "Retrieving of properties of current app failed\n");
        return MODBUSERROR;
    }
    
    updBuild        = pImageHdr->version;
    updFirmwareType = pImageHdr->firmwareType;
    printf("Update : Build %d, firmwareType %d\n", updBuild, updFirmwareType);
    
    if (updBuild > curBuild || (updBuild >= curBuild && curFirmwareType != updFirmwareType)) {
        printf("Starting update...\n");
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

    modbus_set_response_timeout(m_ctx, &responseTimeoutStandard);

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
            modbus_set_response_timeout(m_ctx, &responseTimeoutEraseFlash);
            fprintf(stderr, "modbus_set_response_timeout(%d s, %d ms)\n", (int)responseTimeoutEraseFlash.tv_sec, (int)responseTimeoutEraseFlash.tv_usec/1000);
        } else {
            struct timeval old_response_timeout;
            /* Save original timeout */
            modbus_get_response_timeout(m_ctx, &old_response_timeout);
            if (memcmp(&old_response_timeout, &responseTimeoutStandard, sizeof(struct timeval)) != 0) {
                fprintf(stderr, "modbus_get_response_timeout() = %d s, %d ms\n", (int)old_response_timeout.tv_sec, (int)old_response_timeout.tv_usec/1000);
                /* Define a new timeout! */
                modbus_set_response_timeout(m_ctx, &responseTimeoutStandard);
                fprintf(stderr, "modbus_set_response_timeout(%d s, %d ms)\n", (int)responseTimeoutStandard.tv_sec, (int)responseTimeoutStandard.tv_usec/1000);
            }
        }

        if (imageChunkStartAddr == 0) {
            fprintf(stderr, "wait for flash erase on EnergyCam....\n");
        }

        chunkWriteTryCnt++;
        writeRegCnt = modbus_write_registers(m_ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_COMMON_HOLDINGREG_VIRTUAL_UPDATECHUNKSTREAM), curRegs2Write+2, &modbusUpdateChunk.asModbusRegs[0]); // +2 regs more for address of current chunk (=sizeof(uint32_t)) 

        if (writeRegCnt == -1) {
            if (chunkWriteTryCnt < CHUNK_WRITE_TRY_COUNT) {
                fprintf(stderr, "modbus_write_registers() failed with '%s'\n", modbus_strerror(errno));
                fprintf(stderr, "write of chunk to addr 0x%08x failed, try %d/%d\n", ((uint32_t)modbusUpdateChunk.logical.chunkStartAddrHigh << 16) | modbusUpdateChunk.logical.chunkStartAddrLow, chunkWriteTryCnt, CHUNK_WRITE_TRY_COUNT);
                continue;
            } else {
                fprintf(stderr, "EnergyCamHost_UpdateMCU() failed\n");
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
             printf("% 6d/%d bytes written, % 6d bytes remaining\n", length - imageBytes2Write, length, imageBytes2Write);        
		    }
		 }
    } while(imageBytes2Write > 0);

    regCnt = 1;
    writeRegCnt = modbus_read_input_registers(m_ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_COMMON_INPUTREG_VIRTUAL_UPDATECRCOK), regCnt, &inputRegsCRCOK);
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

//read Buildnumber of Firmware
int EnergyCam_GetAppFirmwareBuildNumber(uint32_t* pBuildNumber) {
    uint32_t readRegCnt;
    const uint32_t regCnt = 2;
    uint16_t inputRegs[regCnt];

    if(NULL == pBuildNumber) 
        return MODBUSERROR;

    readRegCnt = modbus_read_input_registers(m_ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_COMMON_INPUTREG_MEMMAP_APPBUILDNUMBER1), regCnt, &inputRegs[0]);
    if (readRegCnt != -1) {
        uint32_t tmp = 0;
        tmp  = ((uint32_t)inputRegs[0]) << 16;
        tmp |=            inputRegs[1];
        *pBuildNumber = tmp;
        return MODBUSOK;
    } else {
        fprintf(stderr,"EnergyCam_GetAppFirmwareBuildNumber ..\n");
        return MODBUSERROR;
    }    
    return MODBUSERROR; 
}

//read Buildnumber of Firmware
int EnergyCam_GetAppFirmwareType(uint16_t* pFirmwareType) {
    uint32_t readRegCnt;
    const uint32_t regCnt = 1;
    uint16_t inputRegs[regCnt];

    if(NULL == pFirmwareType) 
        return MODBUSERROR;

    readRegCnt = modbus_read_input_registers(m_ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_COMMON_INPUTREG_MEMMAP_APPFIRMWARETYPE), regCnt, &inputRegs[0]);
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
int EnergyCam_GetResultOCRInstallation(uint16_t* pData) {
    uint32_t readRegCnt;

    const uint32_t regCnt = 1;
    uint16_t inputRegs[regCnt];
    
    if(NULL == pData) 
        return MODBUSERROR;

    readRegCnt = modbus_read_input_registers(m_ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_INPUTREG_MEMMAP_RESULTINSTALLATION), regCnt, &inputRegs[0]);
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
int EnergyCam_TriggerReading(void) {
    uint32_t wroteRegCnt;
    const uint32_t regCnt = 1;
    uint16_t holdingRegs[1] = {1};

    wroteRegCnt = modbus_write_registers(m_ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_HOLDINGREG_MEMMAP_ACTIONOCR), regCnt, &holdingRegs[0]);
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
int EnergyCam_TriggerInstallation(void) {
    uint32_t wroteRegCnt;
    const uint32_t regCnt = 2;
    uint16_t holdingRegs[2] = {100,1};

    wroteRegCnt = modbus_write_registers(m_ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_HOLDINGREG_MEMMAP_ACTIONOCRINSTALLATIONTO), regCnt, &holdingRegs[0]);
    if (wroteRegCnt == -1) {
        fprintf(stderr, "TriggerInstallation failed with '%s'\n", modbus_strerror(errno));
        return MODBUSERROR;
    } else {
        fprintf(stdout, "TriggerInstallation\n");
        return MODBUSOK;
    }
    return MODBUSERROR;     
}

//Read the Status of the Reading
int EnergyCam_GetStatusReading(uint16_t* pStatus) {
    uint32_t readRegCnt;
    const uint32_t regCnt = 3;
    uint16_t inputRegs[regCnt];
    
    if(NULL == pStatus) 
        return MODBUSERROR;
    
    readRegCnt = modbus_read_input_registers(m_ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_INPUTREG_MEMMAP_RESULTOCRVALID), regCnt, &inputRegs[0]);
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
int EnergyCam_GetResultOCRInt( uint32_t* pInt, uint16_t* pFrac) {
    uint32_t readRegCnt;
    const uint32_t regCnt = 3;
    uint16_t inputRegs[regCnt];
    
    if(NULL == pInt) 
        return MODBUSERROR;
    if(NULL == pFrac) 
        return MODBUSERROR;
    
    readRegCnt = modbus_read_input_registers(m_ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_INPUTREG_MEMMAP_RESULTOCRINTHIGH), regCnt, &inputRegs[0]);
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
int EnergyCam_GetOCRPicDone(uint16_t* pCount) {
    uint32_t readRegCnt;
    const uint32_t regCnt = 1;
    uint16_t inputRegs[regCnt];
    
    if(NULL == pCount) 
        return MODBUSERROR;
    
    readRegCnt = modbus_read_input_registers(m_ctx, MODBUS_GET_INTERNAL_ADDR_FROM_OFFICIAL(MODBUS_SLAVE_INPUTREG_MEMMAP_OCRPICDONE), regCnt, &inputRegs[0]);
    if (readRegCnt != -1) {
        *pCount = inputRegs[0];
        return MODBUSOK;
    } else {
        fprintf(stderr,"EnergyCam_GetOCRPicDone failed\n");
        return MODBUSERROR;
    }    
    return MODBUSERROR; 
}

//Log Reading with date info to CSV File
int EnergyCam_Log2CSVFile(const char *path,  uint32_t Int, uint16_t Frac) {
    FILE    *hFile;
    uint32_t FileSize = 0;
    
    char  CurrentTime[250];
    
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);  
    
    if ((hFile = fopen(path, "rb")) != NULL) {
        fseek(hFile, 0L, SEEK_END);
        FileSize = ftell(hFile);
        fseek(hFile, 0L, SEEK_SET);
        fclose(hFile);
    } else 
        return MODBUSERROR; 
    
    if ((hFile = fopen(path, "a")) != NULL) {
        if (FileSize == 0)  //start a new file with Header
            fprintf(hFile, "Date, Value \n");
        fprintf(hFile,"%d-%02d-%02d %02d:%02d, %d.%d\r\n",tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,tm.tm_hour,tm.tm_min, Int,Frac);
        fclose(hFile);
    } else 
        return MODBUSERROR; 
    
    return MODBUSOK; 
}

uint16_t DisplayInstallationStatus(int infoflag) {
    uint16_t Data = 0;  
    if (MODBUSOK == EnergyCam_GetResultOCRInstallation(&Data)) {
		if(infoflag > 0){
			switch(Data){
			case INSTALLATION_FAILED:    Colour(PRINTF_RED,false);    printf("Installation failed");     Colour(0,true);
						break;
			case INSTALLATION_NODIGITS:
			case INSTALLATION_NOTDONE:   Colour(PRINTF_RED,false);    printf("EnergyCAM not installed"); Colour(0,true); 
						break;
			case INSTALLATION_ONGOING:   Colour(PRINTF_YELLOW,false); printf("Installation ongoing");    Colour(0,true);
						break;
			default:                     Colour(PRINTF_GREEN,false);  printf("Installed with %d digits",(Data >>8)); Colour(0,true);
						break;    
			}
    	}
    }    
    return Data;
}

int getkey(void) {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN]  = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

static int iTime;
static int iMinute;

int IsNewSecond(int iS)
{
    int CurTime;
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    CurTime = tm.tm_hour*60*60+tm.tm_min*60+tm.tm_sec;
    if (iS > 0)
        CurTime = CurTime/iS;
        
    if (CurTime != iTime) {
        iTime = CurTime;
        return 1;
    }       
    return 0;
}

int IsNewMinute(void) {
    int CurTime;
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    CurTime = tm.tm_hour*60+tm.tm_min;
    
    if (CurTime != iMinute) {
        iMinute = CurTime;
        return 1;
    }      
    return 0;
}

int file_exist (char *filename) {
  struct stat   buffer;   
  return (stat (filename, &buffer) == 0);
}

void Intro(int Read)
{
    printf("   \n");
    Colour(62,false); 
    printf("############################################\n");
    printf("## ecpi - EnergyCam on raspberry Pi/Weezy ##\n"); 
    printf("############################################\n");
    
    Colour(0,true);
    printf("   Usage\n");
    printf("   q   : quit\n");
    printf("   R   : trigger OCR reading\n");
    printf("   r   : read OCR value\n");
    printf("   U   : update EnergyCam firmware\n");
    printf("   The meter reading is stored every %d minutes\n",Read); 
    printf("   \n");         
}


void IntroShowParam(void)
{
    printf("   \n");
    Colour(62,false); 
    printf("############################################\n");
    printf("## ecpi - EnergyCam on raspberry Pi/Weezy ##\n"); 
    printf("############################################\n");
    
    Colour(0,true);
    printf("   Commandline options:\n");
    printf("   ./ecpi -c USB -p 0 -s 1 -i  \n");
    printf("   -c USB : use USB connection\n");
    printf("   -p 0 : Portnumber 0 -> /dev/ttyUSB0\n");
    printf("   -s 1 : MODBUSSlaveAdress 1 \n\n");
    printf("   -i   : show detailed infos \n\n");    
    printf("   ./ecpi -c AMA -p 0 -s 1   \n");
    printf("   -c AMA : use Expansionport\n");
    printf("   -p 0 : Portnumber 0 -> /dev/ttyAMA0\n");
    printf("   -s 1 : MODBUSSlaveAdress 1 \n");    
    printf("   \n");         
}




void ErrorAndExit(const char *info) {
    Colour(PRINTF_RED,false);
    printf("%s",info);
    Colour(0,true); 
    
    EnergyCamClose();
      
    exit(0);
}

int check4Install(int infoflag) {
    uint16_t Data     = 0;
    int      iTimeout = 0;
    
    //Is EnergyCam installed
    Data = DisplayInstallationStatus(infoflag);
    
    //try to install the device if not installed
    if ((Data == INSTALLATION_NODIGITS) || (Data == INSTALLATION_NOTDONE)) {
        EnergyCam_TriggerInstallation();
        usleep(2000*1000);   //sleep 2000ms - wait for Installation
        printf("Installing OCR");
        iTimeout = 60;
        do {
            usleep(500*1000);   //sleep 500ms
            printf(".");
            if (MODBUSERROR == EnergyCam_GetResultOCRInstallation(&Data))
                Data = 0xFFFD; //retry if MODBUS returns with an Error          
        } while ((iTimeout-->0) && (Data == 0xFFFD));
        printf("\n");
        
        //Is EnergyCam installed
        Data = DisplayInstallationStatus(infoflag); 
    }
    
    if ((Data == INSTALLATION_NODIGITS) || (Data == INSTALLATION_NOTDONE) || (Data == INSTALLATION_FAILED) || (Data == INSTALLATION_ONGOING))
        ErrorAndExit("EnergyCAM OCR not installed\n");    
}

//support commandline 
int mainwithparam(int argc, char *argv[]) { 
	int c;

	unsigned int Connection = EXPANSIONPORT;
	unsigned int Port = 0;
	unsigned int Slave = DEFAULTSLAVEADRESS;
	int infoflag = 0;
	
	int iRetry = 3; 
    int iTimeout = 0;

	uint16_t Data           = 0; 
	uint32_t Build          = 0; 
	uint16_t FirmwareType   = 0; 
	uint32_t OCRData        = 0;
       
	opterr = 0;
	while ((c = getopt (argc, argv, "c:hip:s:")) != -1){
		switch (c){
			case 'c':
			case 'C':
			 if (NULL != optarg){
				 if(0 == strcmp("USB",optarg)) Connection=USBPORT;
				 if(0 == strcmp("AMA",optarg)) Connection=EXPANSIONPORT;
			 }
			 break;
			case 'i':
				 infoflag = 1;
				 break;             
			case 'p':
			case 'P':
				 if (NULL != optarg){
					 Port = atoi(optarg);
				 }
				 break;             
			case 's':
			case 'S':
				 if (NULL != optarg){
					 Slave = atoi(optarg);
				 }
				 break;        
			case 'h':
				IntroShowParam();
				return (0);
				break;                     
			 
			case '?':
				 if (optopt == 'c')
				   fprintf (stderr, "Option -%c requires an argument.\n", optopt);
				 else if (isprint (optopt))
				   fprintf (stderr, "Unknown option `-%c'.\n", optopt);
				 else
				   fprintf (stderr,"Unknown option character `\\x%x'.\n",optopt);
				 return 1;
			default:
				abort ();
		}
	}

    if(infoflag > 0) {		
	  printf("Connecting to /dev/tty%s%d Slaveaddress %d\n",(Connection==USBPORT) ? "USB" : "AMA",Port,Slave);
	}       
     
	if(MODBUSOK == EnergyCamOpen(Connection,Port,MODBUSBAUDRATE,Slave)){  //open serial port
	 
		//check EC device
		iRetry = 3; 
		do { 
			if(iRetry-- < 0) 
				break; 
		} while(MODBUSERROR == EnergyCam_GetManufacturerIdentification(&Data));

		if(Data == SAIDENTIFIER) {
			if(infoflag > 0) {			
				Colour(PRINTF_GREEN, false); 
				printf("EnergyCAM Sensor connected "); 
				Colour(0, true); 
				//Read Buildnumber   
				if (MODBUSOK == EnergyCam_GetAppFirmwareBuildNumber(&Build))
					printf("Build %d\n", Build); 
					 
				//Read FirmwareType 
				if (MODBUSOK == EnergyCam_GetAppFirmwareType(&FirmwareType))
					printf("FirmwareType %d\n", FirmwareType);         
			}
				
			check4Install(infoflag);
			
			//get last Reading 
			if (MODBUSOK == EnergyCam_GetResultOCRInt(&OCRData,&Data)) { 
				time_t t     = time(NULL); 
				struct tm tm = *localtime(&t);

				//Output to screen to grab in shell
				printf("%04d.%d\n",OCRData, Data); 
			}
						
		} else
			ErrorAndExit("EnergyCAM not found "); 
		 
		 EnergyCamClose();
 }
	
	


return(0);
}


//////////////////////////////////////////////
int main(int argc, char *argv[]) { 
    int  key=0; 
    int  Reading; 
    char OCR[20]; 
    char Version[20];    
    int  iReadRequest  = 0;
    int  ReadingPeriod = 10; 
    int  ReadingTimer  = ReadingPeriod+1;

    int iRetry = 3; 
    int iTimeout = 0;
    int Connection = EXPANSIONPORT;
    char cCurrentPath[FILENAME_MAX];
    
    uint16_t Data           = 0; 
    uint32_t Build          = 0; 
    uint16_t FirmwareType   = 0; 
    uint32_t OCRData        = 0;
    
    if(argc > 1)
      mainwithparam(argc,argv);
    else
    {
		
		
		getcwd(cCurrentPath, sizeof(cCurrentPath));
		Intro(ReadingPeriod);
		
		if (( EXPANSIONPORT == Connection) &&  (wiringPiSetup () == -1)) {     
			fprintf (stderr, "Not running on raspberry pi - now ending\n"); 
			exit(0); 
		}
		
		EnergyCamOpen(Connection,0,MODBUSBAUDRATE,DEFAULTSLAVEADRESS);  //open serial port
		
		//get Status & wakeup 
		iRetry = 3; 
		do { 
			if(iRetry-- < 0) 
				break; 
		} while(MODBUSERROR == EnergyCam_GetManufacturerIdentification(&Data));

		if(Data == SAIDENTIFIER) {
			Colour(PRINTF_GREEN, false); 
			printf("EnergyCAM Sensor connected "); 
			Colour(0, true); 
		} else
			ErrorAndExit("EnergyCAM not found "); 
			

		//Read Buildnumber   
		if (MODBUSOK == EnergyCam_GetAppFirmwareBuildNumber(&Build))
			printf("Build %d\n", Build); 
			 
		//Read FirmwareType 
		if (MODBUSOK == EnergyCam_GetAppFirmwareType(&FirmwareType))
			printf("FirmwareType %d\n", FirmwareType);         

		//Check Buildnumber, GetResultOCRInt requires Build 8374   
		if (Build < 8374) 
			ErrorAndExit("This App requires a Firmwareversion >= 8374. ");    

		check4Install(true);

		//get last Reading 
		if (MODBUSOK == EnergyCam_GetResultOCRInt(&OCRData,&Data)) { 
			time_t t     = time(NULL); 
			struct tm tm = *localtime(&t);

			printf("(%02d:%02d:%02d) Reading %04d.%d\n", tm.tm_hour, tm.tm_min, tm.tm_sec, OCRData, Data); 
			EnergyCam_Log2CSVFile("/var/www/ecpi/data/ecpi.csv", OCRData, Data); 
		}

		IsNewMinute();   
		ReadingTimer = ReadingPeriod + 1;
		
		while (!((key == 0x1B) || (key == 'q'))) {
			usleep(500*1000);   //sleep 500ms
			  
			key = getkey();
			
			if(key == 'r')   
				iReadRequest++; //Read now        
			
			if(key == 'R')   {
				ReadingTimer = 1;  //read in 1 minute
				
				//get Status & wakeup
				iRetry = 3;
				do {
					if (iRetry-- < 0 ) 
						break;
				} while (MODBUSERROR == EnergyCam_GetStatusReading(&Data));
				printf("GetStatusReading %04X \n", Data);  
				
				//trigger new reading
				EnergyCam_TriggerReading();     
			}     
			
			if(key == 'U') {
				char binaryFileName[1024];
				
				//wakeup
				iRetry = 3;
				do {
					if(iRetry-- < 0)
						break;
				} while (MODBUSERROR == EnergyCam_GetManufacturerIdentification(&Data));
				
				//
				sprintf(binaryFileName,"sensorT2.bin");
				if(file_exist(binaryFileName)){
					printf("Using: %s \n ",binaryFileName); 
				}else{				
					printf("Enter binaryFileName: "); 
					gets(binaryFileName);
					printf("binaryFileName >%s<\n", binaryFileName);
				}	
				  
				if (EnergyCam_UpdateMCU(binaryFileName) == MODBUSOK) {
					printf("transfer of firmware update succeeded, wait until EC reboots and firmware update installs\n");
					
					//wait for update being installed
					iRetry = 60;
					do {
						if (iRetry-- < 0) 
							break;
						usleep(500*1000);   //sleep 500ms
					} while(MODBUSERROR == EnergyCam_GetAppFirmwareBuildNumber(&Build));
					printf("Build %d\n",Build);
					
					if (MODBUSOK == EnergyCam_GetAppFirmwareType(&FirmwareType))
						printf("FirmwareType %d\n", FirmwareType);
					
					check4Install(true);
				} else
					fprintf(stderr, "firmware update failed\n");
			}     
					
			if (IsNewMinute()){
				if (--ReadingTimer <= 1)
					iReadRequest++;
				printf("%02d ", ReadingTimer);
			}
			
			if (iReadRequest > 0) {
				iReadRequest = 0;
				printf(" \n"); 
				ReadingTimer = ReadingPeriod+1;
				
				//get Status & wakeup
				iRetry = 3;
				do {
					if (iRetry-- < 0) 
						break;
				} while (MODBUSERROR == EnergyCam_GetStatusReading(&Data));
				printf("GetStatusReading %04X \n", Data);
				
				EnergyCam_GetOCRPicDone(&Data);
				printf("Pictures %04d\n", Data);
				
				if (MODBUSOK == EnergyCam_GetResultOCRInt(&OCRData, &Data)) {
					time_t t = time(NULL);
					struct tm tm = *localtime(&t);
						  
					printf("(%02d:%02d:%02d) Reading %04d.%d\n", tm.tm_hour, tm.tm_min, tm.tm_sec, OCRData, Data);
					EnergyCam_Log2CSVFile("/var/www/ecpi/data/ecpi.csv", OCRData, Data);
				} 
			}        
		} // end while
		
		EnergyCamClose();
	}
    
    return 0;
}
