#if defined(_WIN32)
#include <winsock2.h>
#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <conio.h>
#include <time.h>
#include <direct.h>

#else
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
#endif

#include "modbus.h"
#include "energycampi.h"
#include "energycammodbus.h"

void Colour(int8_t c, bool cr)
{
#if !defined(_WIN32)
    printf("%c[%dm",0x1B,(c>0) ? (30+c) : c);
#endif
    if(cr)
        printf("\n");
}

//Log Reading with date info to CSV File
int EnergyCam_Log2CSVFile(const char *path,  uint32_t Int, uint16_t Frac)
{
    FILE    *hFile;
    uint32_t FileSize = 0;

    time_t t = time(NULL);
    struct tm tm = *localtime(&t);

    if ((hFile = fopen(path, "rb")) != NULL){
        fseek(hFile, 0L, SEEK_END);
        FileSize = ftell(hFile);
        fseek(hFile, 0L, SEEK_SET);
        fclose(hFile);
    }


    if ((hFile = fopen(path, "a")) != NULL){
        if (FileSize == 0)  //start a new file with Header
            fprintf(hFile, "Date, Value \n");
        fprintf(hFile,"%d-%02d-%02d %02d:%02d, %d.%d\r\n",tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,tm.tm_hour,tm.tm_min, Int,Frac);
        fclose(hFile);
    }
    else
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


    if ( (hFile = fopen(path, "rb")) != NULL ){
        fseek(hFile, 0L, SEEK_END);
        dwSizeIn = ftell(hFile);
        fseek(hFile, 0L, SEEK_SET);

        pXMLIN = (unsigned char*) malloc(dwSizeIn+4096);
        memset(pXMLIN,0,sizeof(unsigned char)*(dwSizeIn+4096));
        if(NULL == pXMLIN){
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
            if(NULL == pXMLMem){
                printf("Log2XMLFile - malloc %d failed \r\n",max(4*XMLBUFFER,XMLBUFFER+dwSizeIn));
                return 0;
            }
            memset(pXMLMem,0,sizeof(unsigned char)*max(4*XMLBUFFER,XMLBUFFER+dwSizeIn));
            pXML = pXMLMem;
            dwSize=0;
            dwSizeIn -= pXMLTop-pXMLIN;

            sprintf(szBuf,"<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\r\n");
            memcpy(pXML,szBuf,strlen(szBuf));
            dwSize+=strlen(szBuf);
            pXML+=strlen(szBuf);

            sprintf(szBuf,"<ENERGYCAMOCR>\r\n");
            memcpy(pXML,szBuf,strlen(szBuf));
            dwSize+=strlen(szBuf);
            pXML+=strlen(szBuf);
        }

    }else{
        //new File
        pXMLMem = (unsigned char *) malloc(XMLBUFFER);
        memset(pXMLMem,0,sizeof(unsigned char)*XMLBUFFER);
        pXML = pXMLMem;
        dwSize=0;

        sprintf(szBuf,"<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\r\n");
        memcpy(pXML,szBuf,strlen(szBuf));
        dwSize+=strlen(szBuf);
        pXML+=strlen(szBuf);

        sprintf(szBuf,"<ENERGYCAMOCR>\r\n");
        memcpy(pXML,szBuf,strlen(szBuf));
        dwSize+=strlen(szBuf);
        pXML+=strlen(szBuf);
    }

    if(pXMLMem){

        sprintf(szBuf,"<OCR>\r\n");
        memcpy(pXML,szBuf,strlen(szBuf));
        dwSize+=strlen(szBuf);
        pXML+=strlen(szBuf);


        sprintf(CurrentTime,"%02d.%02d.%d %02d:%02d:%02d",tm.tm_mday,tm.tm_mon+1,tm.tm_year+1900,tm.tm_hour,tm.tm_min,tm.tm_sec);
        sprintf(szBuf,"<Date>%s</Date>\r\n",CurrentTime);
        memcpy(pXML,szBuf,strlen(szBuf));
        dwSize+=strlen(szBuf);
        pXML+=strlen(szBuf);

        sprintf(szBuf,"<Reading>%.1f</Reading>\r\n",Reading);
        memcpy(pXML,szBuf,strlen(szBuf));
        dwSize+=strlen(szBuf);
        pXML+=strlen(szBuf);
        sprintf(szBuf,"</OCR>\r\n");
        memcpy(pXML,szBuf,strlen(szBuf));
        dwSize+=strlen(szBuf);
        pXML+=strlen(szBuf);


        if(dwSizeIn>0){
            memcpy(pXML,pXMLTop,dwSizeIn);
        }else{
            sprintf(szBuf,"</ENERGYCAMOCR>\r\n");
            memcpy(pXML,szBuf,strlen(szBuf));
            dwSize+=strlen(szBuf);
            pXML+=strlen(szBuf);
        }

        if ( (hFile = fopen(path, "wb")) != NULL ){
            fwrite(pXMLMem, dwSizeIn+dwSize, 1, hFile);
            fclose(hFile);
			#if !defined(_WIN32)
            chmod(path, 0666);
			#endif
        }
    }

    if(pXMLMem) free(pXMLMem);
    if(pXMLIN) free(pXMLIN);


    return true;
}


uint16_t DisplayInstallationStatus(modbus_t* ctx,int infoflag)
{
    uint16_t Data = 0;
    if (MODBUSOK == EnergyCam_GetResultOCRInstallation(ctx,&Data)){
        if(infoflag > 0){
            switch(Data)
            {
            case INSTALLATION_FAILED:
                Colour(PRINTF_RED,false);
                printf("Installation failed");
                Colour(0,true);
                break;
            case INSTALLATION_NODIGITS:
            case INSTALLATION_NOTDONE:
                Colour(PRINTF_RED,false);
                printf("EnergyCAM not installed");
                Colour(0,true);
                break;
            case INSTALLATION_ONGOING:
                Colour(PRINTF_YELLOW,false);
                printf("Installation ongoing");
                Colour(0,true);
                break;
            default:
                Colour(PRINTF_GREEN,false);
                printf("Installed with %d digits",(Data >>8));
                Colour(0,true);
                break;
            }
        }
    }
    return Data;
}

int getkey(void)
{
    int character = 0;

#if defined(_WIN32)
	if(_kbhit()){
		character = _getch(); // Muss auf keine Eingabe warten, Taste ist bereits gedrückt
	}
#else
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
#endif
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

    if (CurTime != iTime){
        iTime = CurTime;
        return 1;
    }
    return 0;
}

int IsNewMinute(void)
{
    int CurTime;
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    CurTime = tm.tm_hour*60+tm.tm_min;

    if (CurTime != iMinute){
        iMinute = CurTime;
        return 1;
    }
    return 0;
}

int file_exist (char *filename)
{
#if defined(_WIN32)
	FILE *fp = fopen(filename,"r");
	if( fp ) {
		// exists
		fclose(fp);
		return 1;
	} else {
		return 0;
	}

#else
    struct stat   buffer;
    return (stat (filename, &buffer) == 0);
#endif
}

void Intro(void)
{

    printf("   \n");
    Colour(62,false);
#if defined(_WIN32)
	printf("#################################\n");
	printf("## ecpi - EnergyCam on Windows ##\n");
	printf("#################################\n");
#else
    printf("#################################################\n");
    printf("## ecpi - EnergyCam on raspberry Pi/cubieboard ##\n");
    printf("#################################################\n");
#endif

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
    printf("   \n");
}


void IntroShowParam(void)
{
#if defined(_WIN32)
    printf("   \n");
    Colour(62,false);
	printf("#################################\n");
    printf("## ecpi - EnergyCam on Windows ##\n");
	printf("#################################\n");

    Colour(0,true);
    printf("   Commandline options:\n");
    printf("   ecpi /p:3 /s:1 /o:1 /i  \n");
    printf("   /p:3  : Portnumber 3 -> COM3\n");
    printf("   /s:1  : MODBUSSlaveAdress 1 \n");
    printf("   /o:1  : (1) -> single run, (loop) -> continuous run\n");
    printf("   /i    : show detailed infos \n\n");
    printf("   \n");
#else
	printf("   \n");
	Colour(62,false);
	printf("#################################################\n");
	printf("## ecpi - EnergyCam on raspberry Pi/cubieboard##\n");
	printf("#################################################\n");

	Colour(0,true);
	printf("   Commandline options:\n");
	printf("   ./ecpi -c USB -p 0 -s 1 -l VZ -o 1 -i  \n");
	printf("   -c USB : use USB connection\n");
	printf("   -p 0  : Portnumber 0 -> /dev/ttyUSB0\n");
	printf("   -s 1  : MODBUSSlaveAdress 1 \n");
	printf("   -l VZ : log to (VZ)Volkszähler, (XML) XMLFile, (CSV) CSV File \n");
	printf("   -f /home/usr : folder to store files \n");
	printf("   -o 1  : (1) -> single run, (loop) -> continuous run\n");
	printf("   -i    : show detailed infos \n\n");

	printf("   ./ecpi -c AMA -p 0 -s 1 -l CSV  \n");
	printf("   -c AMA : use Expansionport\n");
	printf("   -p 0   : Portnumber 0 -> /dev/ttyAMA0\n");
	printf("   -s 1   : MODBUSSlaveAdress 1 \n");
	printf("   \n");
#endif

}




void ErrorAndExit(modbus_t** ctx,const char *info,bool bexit)
{
    Colour(PRINTF_RED,false);
    printf("%s",info);
    Colour(0,true);

    if((true == bexit) && (NULL != ctx)){
        EnergyCamClose(ctx);
        exit(0);
    }
}

uint16_t check4Install(modbus_t** ctx,bool force, int infoflag)
{
    uint16_t Data     = 0;
    int      iTimeout = 0;

    //Is EnergyCam installed
    Data = DisplayInstallationStatus(*ctx,infoflag);

    //try to install the device if not installed
    if ((Data == INSTALLATION_NODIGITS) || (Data == INSTALLATION_NOTDONE) || force){
        EnergyCam_TriggerInstallation(*ctx);
        printf("Installing OCR");
        MSSleep(2000);   //sleep 2000ms - wait for Installation
        iTimeout = 60;
        do{
            MSSleep(500);   //sleep 500ms
            printf(".");fflush(stdout);
            if (MODBUSERROR == EnergyCam_GetResultOCRInstallation(*ctx,&Data))
                Data = INSTALLATION_ONGOING; //retry if MODBUS returns with an Error
        }while ((iTimeout-->0) && (Data == INSTALLATION_ONGOING));
        printf("\n");

        //Is EnergyCam installed
        Data = DisplayInstallationStatus(*ctx,infoflag);
    }

    if ((Data == INSTALLATION_NODIGITS) || (Data == INSTALLATION_NOTDONE) || (Data == INSTALLATION_FAILED) || (Data == INSTALLATION_ONGOING))
        ErrorAndExit(ctx,"EnergyCAM OCR not installed",false);

    return Data;
}





//Log Reading with date info to CSV File
int Log2File(char *DataPath, uint16_t mode,uint16_t infoflag, uint32_t Int, uint16_t Frac)
{

    char   param[_MAX_PATH];
    float  value;
    time_t t;
    struct tm curtime;

    switch(mode)
    {
    case LOGTOCSV :
		#if defined(_WIN32)
        if (strlen(DataPath) == 0){
            _getcwd(param, sizeof(param));
            sprintf(param, "%s\\ecpi.csv",param);
        }else{
            sprintf(param, "%s\\ecpi.csv",DataPath);
        }
        EnergyCam_Log2CSVFile(param, Int, Frac);

        #else
        if (strlen(DataPath) == 0){
            sprintf(param, "/var/www/ecpi/data/ecpi.csv");
        }else{
            sprintf(param, "%s/ecpi.csv",DataPath);
        }
        EnergyCam_Log2CSVFile(param, Int, Frac);
        #endif
        return MODBUSOK;
        break;
    case LOGTOXML :
        if (strlen(DataPath) == 0){
			#if defined(_WIN32)
            _getcwd(param, sizeof(param));
			#else
            getcwd(param, sizeof(param));
			#endif
            sprintf(param, "%s/ecpi.xml",param);
        }else{
            sprintf(param, "%s/ecpi.xml",DataPath);
        }
        value = Int + Frac*1.0/10;
        Log2XMLFile(param,value);
        return MODBUSOK;

        break;
    case LOGTOVZ :
		#if !defined(_WIN32)
        if( access( "add2vz.sh", F_OK ) != -1 )
        {
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
		#endif
        return MODBUSOK;
        break;
    }
    return MODBUSERROR;
}

//support commandline
int singlerun(unsigned int Connection,unsigned int Port,unsigned int Slave,int infoflag)
{

    int iRetry = 3;

    uint16_t Data           = 0;
    uint32_t Build          = 0;
    uint16_t FirmwareType   = 0;
    uint32_t OCRData        = 0;

    modbus_t* ctx = NULL;

    if(infoflag > SILENTMODE){
        #if defined(_WIN32)
        printf("Connecting to COM%d Slaveaddress %d\n",Port,Slave);
        #else
        printf("Connecting to /dev/tty%s%d Slaveaddress %d\n",(Connection==USBPORT) ? "USB" : "AMA",Port,Slave);
        #endif
    }

    if(MODBUSOK == EnergyCamOpen(&ctx,Connection,Port,MODBUSBAUDRATE,Slave)) { //open serial port
        //check EC device
        iRetry = 3;
        do{
            if(iRetry-- < 0)
                break;
        }while(MODBUSERROR == EnergyCam_GetManufacturerIdentification(ctx,&Data));

        if(Data == SAIDENTIFIER){
            if(infoflag > SILENTMODE){
                Colour(PRINTF_GREEN, false);
                printf("EnergyCAM Sensor connected ");
                Colour(0, true);
                //Read Buildnumber
                if (MODBUSOK == EnergyCam_GetAppFirmwareBuildNumber(ctx,&Build,infoflag))
                    printf("Build %d\n", Build);

                //Read Firmwaretype
                if (MODBUSOK == EnergyCam_GetAppFirmwareType(ctx,&FirmwareType)){
                    printf("Firmware T2 %s\n",( SA_FIRMWARE_TYPE_APP_WMBUS_T2_OCR_BW == FirmwareType) ? "Black on White" : "White on Black");
                }


            }
            check4Install(&ctx,false,infoflag);

            //get last Reading
            if (MODBUSOK == EnergyCam_GetResultOCRInt(ctx,&OCRData,&Data)){
                //Output to screen to grab in shell
                printf("%04d.%d\n",OCRData, Data);
            }

        }
        else
            ErrorAndExit(&ctx,"EnergyCAM not found ",true);

        EnergyCamClose(&ctx);
    }

    return(0);
}

#if defined(_WIN32)
//support commandline
int parseparam(int argc, char *argv[],char *filepath,uint16_t *infoflag,uint16_t *Connection,uint16_t *Port,uint16_t *Slave,uint16_t *opMode,uint16_t *LogMode) {
	int i;
	char *key, *value;

	if((NULL == LogMode) || (NULL == opMode) || (NULL == infoflag) || (NULL == Connection) || (NULL == Port) || (NULL == Slave) ) return 0;

	*Connection=USBPORT;

	for( i = 1; i < argc; i++ ) {
		if( *argv[i] == '/' ) {
			key = argv[i] + 1;
			value = strchr(key, ':');
			if( value != NULL ) *value++ = 0;

			switch (*key){
			case 'i':
				*infoflag = SHOWDETAILS;
				break;
			case 'p':
				if( NULL != value)
					*Port = (uint16_t)atoi(value);
				break;
			case 's':
				if( NULL != value)
					*Slave = (uint16_t)atoi(value);
				break;
			case 'f':
				if( NULL != value)
					strcpy(filepath,value);
				break;
			case 'o':
				if( NULL != value){
					if(0 == strcmp("loop",value)) *opMode=LOOPOPERATION;
					if(0 == strcmp("1",value))    *opMode=SINGLERUN;
				}
				break;
			case 'h':
				IntroShowParam();
				exit (0);
				break;
			}
		}
	}
	return 0;
}
#else
//support commandline
int parseparam(int argc, char *argv[],char *filepath,uint16_t *infoflag,uint16_t *Connection,uint16_t *Port,uint16_t *Slave,uint16_t *opMode,uint16_t *LogMode)
{
    int c;

    if((NULL == LogMode) || (NULL == opMode) || (NULL == infoflag) || (NULL == Connection) || (NULL == Port) || (NULL == Slave)) return 0;

    opterr = 0;
    while ((c = getopt (argc, argv, "c:f:hil:o:p:s:t:")) != -1)
    {
        switch (c)
        {
        case 'c':
            if (NULL != optarg)
            {
                if(0 == strcmp("USB",optarg)) *Connection=USBPORT;
                if(0 == strcmp("AMA",optarg)) *Connection=EXPANSIONPORT;
            }
            break;
        case 'f':
            if (NULL != optarg)
            {
                strcpy(filepath,optarg);
            }
            break;
        case 'i':
            *infoflag = SHOWDETAILS;
            break;
        case 'l':
            if (NULL != optarg)
            {
                if(0 == strcmp("VZ",optarg))  *LogMode=LOGTOVZ;
                if(0 == strcmp("XML",optarg)) *LogMode=LOGTOXML;
            }
            break;
        case 'o':
            if (NULL != optarg)
            {
                if(0 == strcmp("loop",optarg)) *opMode=LOOPOPERATION;
                if(0 == strcmp("1",optarg))    *opMode=SINGLERUN;
            }
            break;
        case 'p':
            if (NULL != optarg)
            {
                *Port = atoi(optarg);
            }
            break;
        case 's':
            if (NULL != optarg)
            {
                *Slave = atoi(optarg);
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
#endif

//////////////////////////////////////////////
int main(int argc, char *argv[])
{
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
    uint16_t InstallStatus;

    modbus_t* ctx = NULL;



    memset(CommandlineDatPath,0,_MAX_PATH*sizeof(char));

    //check commandling arguments
    if(argc > 1)
        parseparam(argc,argv,CommandlineDatPath,&InfoFlag, &Connection,&Port,&Slave,&OpMode,&LogMode);

    if(OpMode == SINGLERUN)
        singlerun(Connection,Port,Slave,InfoFlag); //read only once
    else
    {
        //loop

		#if defined(_WIN32)
		_getcwd(cCurrentPath, sizeof(cCurrentPath));
		#else
		getcwd(cCurrentPath, sizeof(cCurrentPath));
		#endif
        Intro();

		#if !defined(_WIN32)
        //check running on raspberry pi
        if (( EXPANSIONPORT == Connection) &&  (wiringPiSetup () == -1))
            ErrorAndExit(&ctx,"Not running on raspberry pi - now ending",true);
		#endif


        if(MODBUSERROR == EnergyCamOpen(&ctx,Connection,Port,MODBUSBAUDRATE,Slave))  //open serial port
            ErrorAndExit(&ctx,"EnergyCAM not found ",true);

        //get ManufactorID & wakeup device, power down can be enabled
        iRetry = 3;
        do{
            if(iRetry-- < 0)
                break;
        }while(MODBUSERROR == EnergyCam_GetManufacturerIdentification(ctx,&Data));

        if(Data == SAIDENTIFIER){
            if(InfoFlag > SILENTMODE){
                Colour(PRINTF_GREEN, false);
                printf("EnergyCAM Sensor connected ");
                Colour(0, true);
            }
        }
        else
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

        //get ReadingPeriod
        if (MODBUSOK == EnergyCam_GetOCRReadingPeriod(ctx,&ReadingPeriod)){
            if(InfoFlag > SILENTMODE) printf("Reading is done every %d minutes\n",ReadingPeriod);
        }




        InstallStatus = check4Install(&ctx,false,true); //check if device is installed

        //get last Reading
        if ((INSTALLATION_FAILED != InstallStatus) && (MODBUSOK == EnergyCam_GetResultOCRInt(ctx,&OCRData,&Data))){
            time_t t     = time(NULL);
            struct tm tm = *localtime(&t);

            printf("(%02d:%02d:%02d) Reading %05d.%d\n", tm.tm_hour, tm.tm_min, tm.tm_sec, OCRData, Data);
            Log2File(CommandlineDatPath,LogMode,InfoFlag, OCRData, Data);
        }

        IsNewMinute();
        ReadingTimer = ReadingPeriod + 1;

        while (!((key == 0x1B) || (key == 'q')))
        {
            MSSleep(500);   //sleep 500ms

            key = getkey();   //get key if one was pressed - not waiting

            if(key == 'r')
                iReadRequest++; //Read now

            if(key == 'R'){
                ReadingTimer = 1;  //read in 1 minute

                //get Status & wakeup
                iRetry = 3;
                do{
                    if (iRetry-- < 0 )
                        break;
                }while (MODBUSERROR == EnergyCam_GetStatusReading(ctx,&Data));
                if(InfoFlag>SILENTMODE) printf("GetStatusReading %04X \n", Data);

                //trigger new reading
                EnergyCam_TriggerReading(ctx);

				//wait for reading done
				EnergyCam_WaitforReadingDone(ctx);

                iReadRequest++;
            }
            //save picture
            if(key == 's'){
				 //get Status & wakeup
                iRetry = 3;
                do{
                    if (iRetry-- < 0 )
                        break;
                }while (MODBUSERROR == EnergyCam_GetStatusReading(ctx,&Data));
                if(InfoFlag>SILENTMODE) printf("GetStatusReading %04X \n", Data);
                if (strlen(CommandlineDatPath) == 0)
                    EnergyCam_Log2BMPFile(ctx,cCurrentPath,Build,InfoFlag); //use current folder
                else
                    EnergyCam_Log2BMPFile(ctx,CommandlineDatPath,Build,InfoFlag); //use folder specified with -f
            }

            //install device
            if(key == 'I'){
                check4Install(&ctx,true,InfoFlag);
            }

            //update firmware
            if(('B' == key) || ('W' == key) || ('U' == key)){
                char binaryFileName[1024];

                //wakeup
                iRetry = 3;
                do{
                    if(iRetry-- < 0)
                        break;
                }while (MODBUSERROR == EnergyCam_GetManufacturerIdentification(ctx,&Data));

                if('W' == key) sprintf(binaryFileName,"sensorT2WB.bin");
                if('B' == key) sprintf(binaryFileName,"sensorT2BW.bin");
                if('U' == key){
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

                if (EnergyCam_UpdateMCU(ctx,binaryFileName,InfoFlag) == MODBUSOK){
                    if(InfoFlag>SILENTMODE) printf("transfer of firmware update succeeded, wait until EC reboots and firmware update installs\n");

                    //wait for update being installed
                    iRetry = 60;
                    do{
                        if (iRetry-- < 0)
                            break;
                        MSSleep(500);   //sleep 500ms
                    }while(MODBUSERROR == EnergyCam_GetAppFirmwareBuildNumber(ctx,&Build,InfoFlag));
                    if(InfoFlag>SILENTMODE) printf("Build %d\n",Build);
                    check4Install(&ctx,false,true);
                }
            }

            if (IsNewMinute()){
                if (--ReadingTimer <= 1)
                    iReadRequest++;
                if(InfoFlag>SILENTMODE) printf("%02d ", ReadingTimer);
            }

            if (iReadRequest > 0){
                iReadRequest = 0;
                if(InfoFlag>SILENTMODE) printf(" \n");
                ReadingTimer = ReadingPeriod+1;

				//wait for reading done
				EnergyCam_WaitforReadingDone(ctx);


                if( MODBUSOK == EnergyCam_GetOCRPicDone(ctx,&Data)){
                    if(InfoFlag>SILENTMODE) printf("Pictures %04d\n", Data);
                }

                if (MODBUSOK == EnergyCam_GetResultOCRInt(ctx,&OCRData, &Data)){
                    time_t t = time(NULL);
                    struct tm tm = *localtime(&t);

                    printf("(%02d:%02d:%02d) Reading %05d.%d\n", tm.tm_hour, tm.tm_min, tm.tm_sec, OCRData, Data);
                    Log2File(CommandlineDatPath,LogMode,InfoFlag, OCRData, Data);
                }
            }
        } // end while

        EnergyCamClose(&ctx);
    }

    return 0;
}
