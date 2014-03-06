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
#include "energycammodbus.h"

void Colour(int8_t c, bool cr) {
  printf("%c[%dm",0x1B,(c>0) ? (30+c) : c);
  if(cr)
    printf("\n");
}

//Log Reading with date info to CSV File
int EnergyCam_Log2CSVFile(const char *path,  uint32_t Int, uint16_t Frac) {
    FILE    *hFile;
    uint32_t FileSize = 0;

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


#define XMLBUFFER (1*1024*1024)
unsigned int Log2XMLFile(const char *path,double Reading)
{
	char szBuf[250];
	FILE    *hFile;
	unsigned char*   pXMLIN = NULL;
	unsigned char*   pXMLTop,*pXMLMem = NULL;
	unsigned char*  pXML;
	unsigned int   dwSize=0;
	unsigned int   dwSizeIn=0;
	char  CurrentTime[250];


	time_t t = time(NULL);
	struct tm tm = *localtime(&t);


	if ( (hFile = fopen(path, "rb")) != NULL ) {
		fseek(hFile, 0L, SEEK_END);
		dwSizeIn = ftell(hFile);
		fseek(hFile, 0L, SEEK_SET);

        pXMLIN = (unsigned char*) malloc(dwSizeIn+4096);
        memset(pXMLIN,0,sizeof(unsigned char)*(dwSizeIn+4096));
		if(NULL == pXMLIN) {
		    printf("Log2XMLFile - malloc failed \r\n");
		    return 0;
		}

		fread(pXMLIN, dwSizeIn, 1, hFile);
		fclose(hFile);


         //place on top
			pXMLTop = (unsigned char *)strstr((const char*)pXMLIN,"<ENERGYCAMOCR>\r\n"); //search on start
			if(pXMLTop){
				pXMLTop += strlen("<ENERGYCAMOCR>\r\n");
				pXMLMem = (unsigned char *) malloc(max(4*XMLBUFFER,XMLBUFFER+dwSizeIn));
				if(NULL == pXMLMem) {
					printf("Log2XMLFile - malloc %d failed \r\n",max(4*XMLBUFFER,XMLBUFFER+dwSizeIn));
					return 0;
				}
				memset(pXMLMem,0,sizeof(unsigned char)*max(4*XMLBUFFER,XMLBUFFER+dwSizeIn));
				pXML = pXMLMem;
				dwSize=0;
				dwSizeIn -= pXMLTop-pXMLIN;

	 			sprintf(szBuf,"<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\r\n");
				memcpy(pXML,szBuf,strlen(szBuf));dwSize+=strlen(szBuf);pXML+=strlen(szBuf);

				sprintf(szBuf,"<ENERGYCAMOCR>\r\n");
				memcpy(pXML,szBuf,strlen(szBuf));dwSize+=strlen(szBuf);pXML+=strlen(szBuf);
			}

	}
	else {
		//new File
		pXMLMem = (unsigned char *) malloc(XMLBUFFER);
		memset(pXMLMem,0,sizeof(unsigned char)*XMLBUFFER);
		pXML = pXMLMem;
		dwSize=0;

		sprintf(szBuf,"<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\r\n");
		memcpy(pXML,szBuf,strlen(szBuf));dwSize+=strlen(szBuf);pXML+=strlen(szBuf);

		sprintf(szBuf,"<ENERGYCAMOCR>\r\n");
		memcpy(pXML,szBuf,strlen(szBuf));dwSize+=strlen(szBuf);pXML+=strlen(szBuf);
	}

	if(pXMLMem) {

		sprintf(szBuf,"<OCR>\r\n");
		memcpy(pXML,szBuf,strlen(szBuf));dwSize+=strlen(szBuf);pXML+=strlen(szBuf);


		sprintf(CurrentTime,"%02d.%02d.%d %02d:%02d:%02d",tm.tm_mday,tm.tm_mon+1,tm.tm_year+1900,tm.tm_hour,tm.tm_min,tm.tm_sec);
		sprintf(szBuf,"<Date>%s</Date>\r\n",CurrentTime);memcpy(pXML,szBuf,strlen(szBuf));dwSize+=strlen(szBuf);pXML+=strlen(szBuf);

		sprintf(szBuf,"<Reading>%.1f</Reading>\r\n",Reading);memcpy(pXML,szBuf,strlen(szBuf));dwSize+=strlen(szBuf);pXML+=strlen(szBuf);
		sprintf(szBuf,"</OCR>\r\n");memcpy(pXML,szBuf,strlen(szBuf));dwSize+=strlen(szBuf);pXML+=strlen(szBuf);


		if(dwSizeIn>0){
			memcpy(pXML,pXMLTop,dwSizeIn);
		}
		else {
			sprintf(szBuf,"</ENERGYCAMOCR>\r\n");	memcpy(pXML,szBuf,strlen(szBuf));dwSize+=strlen(szBuf);pXML+=strlen(szBuf);
		}

		if ( (hFile = fopen(path, "wb")) != NULL ) {
			fwrite(pXMLMem, dwSizeIn+dwSize, 1, hFile);
			fclose(hFile);
			chmod(path, 0666);
		}
	}

	if(pXMLMem) free(pXMLMem);
	if(pXMLIN) free(pXMLIN);


	return true;
}


uint16_t DisplayInstallationStatus(modbus_t* ctx,int infoflag) {
    uint16_t Data = 0;
    if (MODBUSOK == EnergyCam_GetResultOCRInstallation(ctx,&Data)) {
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
    printf("   s   : save BMP File\n");
    printf("   I   : trigger installation\n");
    printf("   U   : update EnergyCam firmware\n");
    printf("   B   : switch to Black on White firmware\n");
    printf("   W   : switch to White on Black firmware\n");
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
    printf("   ./ecpi -c USB -p 0 -s 1 -t 5 -l VZ -o 1 -i  \n");
    printf("   -c USB : use USB connection\n");
    printf("   -p 0  : Portnumber 0 -> /dev/ttyUSB0\n");
    printf("   -s 1  : MODBUSSlaveAdress 1 \n");
    printf("   -t 5  : Readingperiod 5 min \n");
    printf("   -l VZ : log to (VZ)Volkszähler, (XML) XMLFile, (CSV) CSV File \n");
    printf("   -f /home/usr : folder to store XMLFile \n");
    printf("   -o 1  : (1) -> single run, (loop) -> continuous run\n");
    printf("   -i    : show detailed infos \n\n");

    printf("   ./ecpi -c AMA -p 0 -s 1 -l CSV  \n");
    printf("   -c AMA : use Expansionport\n");
    printf("   -p 0   : Portnumber 0 -> /dev/ttyAMA0\n");
    printf("   -s 1   : MODBUSSlaveAdress 1 \n");
    printf("   \n");
}




void ErrorAndExit(modbus_t** ctx,const char *info,bool bexit) {
    Colour(PRINTF_RED,false);
    printf("%s",info);
    Colour(0,true);

    if(true == bexit){

		EnergyCamClose(ctx);
		exit(0);
	}
}

int check4Install(modbus_t** ctx,bool force, int infoflag) {
    uint16_t Data     = 0;
    int      iTimeout = 0;

    //Is EnergyCam installed
    Data = DisplayInstallationStatus(*ctx,infoflag);

    //try to install the device if not installed
    if ((Data == INSTALLATION_NODIGITS) || (Data == INSTALLATION_NOTDONE) || force) {
        EnergyCam_TriggerInstallation(*ctx);
        usleep(2000*1000);   //sleep 2000ms - wait for Installation
        printf("Installing OCR");
        iTimeout = 60;
        do {
            usleep(500*1000);   //sleep 500ms
            printf(".");
            if (MODBUSERROR == EnergyCam_GetResultOCRInstallation(*ctx,&Data))
                Data = 0xFFFD; //retry if MODBUS returns with an Error
        } while ((iTimeout-->0) && (Data == 0xFFFD));
        printf("\n");

        //Is EnergyCam installed
        Data = DisplayInstallationStatus(*ctx,infoflag);
    }

    if ((Data == INSTALLATION_NODIGITS) || (Data == INSTALLATION_NOTDONE) || (Data == INSTALLATION_FAILED) || (Data == INSTALLATION_ONGOING))
        ErrorAndExit(ctx,"EnergyCAM OCR not installed",false);

  return Data;
}





//Log Reading with date info to CSV File
int Log2File(char *DataPath, uint16_t mode,uint16_t infoflag, uint32_t Int, uint16_t Frac) {

char   param[_MAX_PATH];
float  value;
time_t t;
struct tm curtime;

switch(mode){
	case LOGTOCSV : EnergyCam_Log2CSVFile("/var/www/ecpi/data/ecpi.csv", Int, Frac); return MODBUSOK; break;
	case LOGTOXML : if (strlen(DataPath) == 0)	{
					  getcwd(param, sizeof(param));
					  sprintf(param, "%s/ecpi.xml",param);
				    }else{
					  sprintf(param, "%s/ecpi.xml",DataPath);
				    }
					value = Int + Frac*1.0/10;
					Log2XMLFile(param,value);
					return MODBUSOK;

					break;
	case LOGTOVZ :  if( access( "add2vz.sh", F_OK ) != -1 ) {
							memset(param, '\0', sizeof(FILENAME_MAX));
							sprintf(param, "./add2vz.sh %ld ",(long int)Int*1000+Frac*100);
							int ret=system(param);
							t = time(NULL);
							curtime = *localtime(&t);

							if(0x100 == ret){
								 if(infoflag > SILENTMODE) printf("%02d:%02d Calling %s \n",curtime.tm_hour,curtime.tm_min,param);
							}else{
								 if(infoflag > SILENTMODE) printf("%02d:%02d Calling %s returned with 0x%X\n",curtime.tm_hour,curtime.tm_min,param,ret);
							}
						}
					return MODBUSOK;
					break;
				}
return MODBUSERROR;
}

//support commandline
int singlerun(unsigned int Connection,unsigned int Port,unsigned int Slave,int infoflag) {

	int iRetry = 3;

	uint16_t Data           = 0;
	uint32_t Build          = 0;
	uint16_t FirmwareType   = 0;
	uint32_t OCRData        = 0;

	modbus_t* ctx = NULL;


    if(infoflag > SILENTMODE) {
	  printf("Connecting to /dev/tty%s%d Slaveaddress %d\n",(Connection==USBPORT) ? "USB" : "AMA",Port,Slave);
	}

	if(MODBUSOK == EnergyCamOpen(&ctx,Connection,Port,MODBUSBAUDRATE,Slave)){  //open serial port

		//check EC device
		iRetry = 3;
		do {
			if(iRetry-- < 0)
				break;
		} while(MODBUSERROR == EnergyCam_GetManufacturerIdentification(ctx,&Data));

		if(Data == SAIDENTIFIER) {
			if(infoflag > SILENTMODE) {
				Colour(PRINTF_GREEN, false);
				printf("EnergyCAM Sensor connected ");
				Colour(0, true);
				//Read Buildnumber
				if (MODBUSOK == EnergyCam_GetAppFirmwareBuildNumber(ctx,&Build,infoflag))
					printf("Build %d\n", Build);

				//Read FirmwareType
				if (MODBUSOK == EnergyCam_GetAppFirmwareType(ctx,&FirmwareType))
					printf("FirmwareType %d\n", FirmwareType);
			}

			check4Install(&ctx,false,infoflag);

			//get last Reading
			if (MODBUSOK == EnergyCam_GetResultOCRInt(ctx,&OCRData,&Data)) {

				//Output to screen to grab in shell
				printf("%04d.%d\n",OCRData, Data);
			}

		} else
			ErrorAndExit(&ctx,"EnergyCAM not found ",true);

		 EnergyCamClose(&ctx);
 }

return(0);
}



//support commandline
int parseparam(int argc, char *argv[],char *filepath,uint16_t *infoflag,uint16_t *Connection,uint16_t *Port,uint16_t *Slave,uint16_t *Period,uint16_t *opMode,uint16_t *LogMode) {
	int c;

    if((NULL == LogMode) || (NULL == opMode) || (NULL == infoflag) || (NULL == Connection) || (NULL == Port) || (NULL == Slave) || (NULL == Period)) return 0;

	opterr = 0;
		while ((c = getopt (argc, argv, "c:f:hil:o:p:s:t:")) != -1){
		switch (c){
			case 'c':
			 if (NULL != optarg){
				 if(0 == strcmp("USB",optarg)) *Connection=USBPORT;
				 if(0 == strcmp("AMA",optarg)) *Connection=EXPANSIONPORT;
			 }
			 break;
			case 'f':
			 if (NULL != optarg){
				 strcpy(filepath,optarg);
			 }
			 break;
			case 'i':
				 *infoflag = SHOWDETAILS;
				 break;
			case 'l':
			 if (NULL != optarg){
				 if(0 == strcmp("VZ",optarg))  *LogMode=LOGTOVZ;
				 if(0 == strcmp("XML",optarg)) *LogMode=LOGTOXML;
			 }
			 break;
			case 'o':
			 if (NULL != optarg){
				 if(0 == strcmp("loop",optarg)) *opMode=LOOPOPERATION;
				 if(0 == strcmp("1",optarg))    *opMode=SINGLERUN;
			 }
			 break;
			case 'p':
				 if (NULL != optarg){
					 *Port = atoi(optarg);
				 }
				 break;
			case 's':
				 if (NULL != optarg){
					 *Slave = atoi(optarg);
				 }
				 break;
			case 't':
				 if (NULL != optarg){
					 *Period = atoi(optarg);
				 }
				 break;
			case 'h':
				IntroShowParam();
				exit (0);
				break;
			case '?':
				 if (optopt == 'f')
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
return 0;
}


//////////////////////////////////////////////
int main(int argc, char *argv[]) {
    int  key=0;
    int  iReadRequest  = 0;
    uint16_t  ReadingPeriod = 10;
    uint16_t  ReadingTimer  = ReadingPeriod+1;

    int iRetry = 3;

    uint16_t InfoFlag = SILENTMODE;
    uint16_t Connection = EXPANSIONPORT;
	uint16_t Port = 0;
	uint16_t Slave = DEFAULTSLAVEADRESS;
	uint16_t OpMode = LOOPOPERATION;
	uint16_t LogMode = LOGTOCSV;

    char cCurrentPath[FILENAME_MAX];
    char CommandlineDatPath[_MAX_PATH];

    uint16_t Data           = 0;
    uint32_t Build          = 0;
    uint16_t FWType         = 0;
    uint32_t OCRData        = 0;

    modbus_t* ctx = NULL;



    memset(CommandlineDatPath,0,_MAX_PATH*sizeof(char));

    //check commandling arguments
    if(argc > 1)
      parseparam(argc,argv,CommandlineDatPath,&InfoFlag, &Connection,&Port,&Slave,&ReadingPeriod,&OpMode,&LogMode);

    ReadingPeriod = max(2,min(15,ReadingPeriod)); //limit ReadingPeriod from 2...15

    if(OpMode == SINGLERUN)
        singlerun(Connection,Port,Slave,InfoFlag); //read only once
    else
    {
		//loop
		getcwd(cCurrentPath, sizeof(cCurrentPath));
		Intro(ReadingPeriod);

		//check running on raspberry pi
		if (( EXPANSIONPORT == Connection) &&  (wiringPiSetup () == -1)) {
			fprintf (stderr, "Not running on raspberry pi - now ending\n");
			exit(0);
		}

		EnergyCamOpen(&ctx,Connection,Port,MODBUSBAUDRATE,Slave);  //open serial port

		//get ManufactorID & wakeup device, power down can be enabled
		iRetry = 3;
		do {
			if(iRetry-- < 0)
				break;
		} while(MODBUSERROR == EnergyCam_GetManufacturerIdentification(ctx,&Data));



		if(Data == SAIDENTIFIER) {
			if(InfoFlag > SILENTMODE) {
				Colour(PRINTF_GREEN, false);
				printf("EnergyCAM Sensor connected ");
				Colour(0, true);
			}
		} else
			ErrorAndExit(&ctx,"EnergyCAM not found ",true);


		//Read Buildnumber
		if (MODBUSOK == EnergyCam_GetAppFirmwareBuildNumber(ctx,&Build,InfoFlag)){
			if(InfoFlag > SILENTMODE) printf("Build %d\n", Build);
		}


		//Check Buildnumber, GetResultOCRInt requires Build 8374
		if (Build < 8374)
			ErrorAndExit(&ctx,"This App requires a Firmwareversion >= 8374. ",true);

		//Read Firmwaretype
        if (MODBUSOK == EnergyCam_GetAppFirmwareType(ctx,&FWType)){
                    if(InfoFlag > SILENTMODE) printf("Firmware T2 %s\n",( SA_FIRMWARE_TYPE_APP_WMBUS_T2_OCR_BW == FWType) ? "Black on White" : "White on Black");
        }

		check4Install(&ctx,false,true); //check if device is installed

		//get last Reading
		if (MODBUSOK == EnergyCam_GetResultOCRInt(ctx,&OCRData,&Data)) {
			time_t t     = time(NULL);
			struct tm tm = *localtime(&t);

			printf("(%02d:%02d:%02d) Reading %04d.%d\n", tm.tm_hour, tm.tm_min, tm.tm_sec, OCRData, Data);
			Log2File(CommandlineDatPath,LogMode,InfoFlag, OCRData, Data);
		}

		IsNewMinute();
		ReadingTimer = ReadingPeriod + 1;

		while (!((key == 0x1B) || (key == 'q'))) {
			usleep(500*1000);   //sleep 500ms

			key = getkey();   //get key if one was pressed - not waiting

			if(key == 'r')
				iReadRequest++; //Read now

			if(key == 'R')   {
				ReadingTimer = 1;  //read in 1 minute

				//get Status & wakeup
				iRetry = 3;
				do {
					if (iRetry-- < 0 )
						break;
				} while (MODBUSERROR == EnergyCam_GetStatusReading(ctx,&Data));
				if(InfoFlag>SILENTMODE) printf("GetStatusReading %04X \n", Data);

				//trigger new reading
				EnergyCam_TriggerReading(ctx);
			}
			//save picture
			if(key == 's')   {
				EnergyCam_Log2BMPFile(ctx,cCurrentPath,Build,InfoFlag);
			}

			//save picture
			if(key == 'I')   {
				check4Install(&ctx,true,InfoFlag);
			}


			//update firmware
			if(('B' == key) || ('W' == key) || ('U' == key)) {
				char binaryFileName[1024];

				//wakeup
				iRetry = 3;
				do {
					if(iRetry-- < 0)
						break;
				} while (MODBUSERROR == EnergyCam_GetManufacturerIdentification(ctx,&Data));

				if('W' == key) sprintf(binaryFileName,"sensorT2WB.bin");
				if('B' == key) sprintf(binaryFileName,"sensorT2BW.bin");
				if('U' == key) {

                    EnergyCam_GetAppFirmwareType(ctx,&FWType);
                    sprintf(binaryFileName,( SA_FIRMWARE_TYPE_APP_WMBUS_T2_OCR_BW == FWType) ? "sensorT2BW.bin" : "sensorT2WB.bin");
                }

				if(file_exist(binaryFileName)){
					printf("Using: %s \n ",binaryFileName);
				}else{
					printf("Enter binaryFileName: ");
					fgets( binaryFileName, 1024, stdin );
					printf("binaryFileName >%s<\n", binaryFileName);
				}

				if (EnergyCam_UpdateMCU(ctx,binaryFileName,InfoFlag) == MODBUSOK) {
					if(InfoFlag>SILENTMODE) printf("transfer of firmware update succeeded, wait until EC reboots and firmware update installs\n");

					//wait for update being installed
					iRetry = 60;
					do {
						if (iRetry-- < 0)
							break;
						usleep(500*1000);   //sleep 500ms
					} while(MODBUSERROR == EnergyCam_GetAppFirmwareBuildNumber(ctx,&Build,InfoFlag));
					if(InfoFlag>SILENTMODE) printf("Build %d\n",Build);


					check4Install(&ctx,false,true);
				}
			}

			if (IsNewMinute()){
				if (--ReadingTimer <= 1)
					iReadRequest++;
				if(InfoFlag>SILENTMODE) printf("%02d ", ReadingTimer);
			}

			if (iReadRequest > 0) {
				iReadRequest = 0;
				if(InfoFlag>SILENTMODE) printf(" \n");
				ReadingTimer = ReadingPeriod+1;

				//get Status & wakeup
				iRetry = 3;
				do {
					if(iRetry-- < 0)
						break;
				} while (MODBUSERROR == EnergyCam_GetStatusReading(ctx,&Data));


				if(ReadingPeriod < 10){
					//trigger new reading
					EnergyCam_TriggerReading(ctx);

					iRetry = 10;
					do {
						EnergyCam_GetStatusReading(ctx,&Data);
						if(InfoFlag>SILENTMODE) printf("GetStatusReading %04X \n", Data);
						if (iRetry-- < 0)
							break;
						usleep(500*1000);   //sleep 500ms

					} while (( Data >=  OCR_ONGOING) || (Data == OCR_NODIGITS));

				}

				EnergyCam_GetOCRPicDone(ctx,&Data);
				if(InfoFlag>SILENTMODE) printf("Pictures %04d\n", Data);

				if (MODBUSOK == EnergyCam_GetResultOCRInt(ctx,&OCRData, &Data)) {
					time_t t = time(NULL);
					struct tm tm = *localtime(&t);

					printf("(%02d:%02d:%02d) Reading %04d.%d\n", tm.tm_hour, tm.tm_min, tm.tm_sec, OCRData, Data);
					Log2File(CommandlineDatPath,LogMode,InfoFlag, OCRData, Data);
				}
			}
		} // end while

		EnergyCamClose(&ctx);
	}

    return 0;
}
