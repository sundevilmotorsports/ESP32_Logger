

typedef struct
{
	uint8_t uniqueID[4];
	uint8_t uartWorkingBuffer[101];

	unsigned short year;
	uint8_t yearBytes[2];
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t fixType;

	signed long lon;
	uint8_t lonBytes[4];
	signed long lat;
	uint8_t latBytes[4];
	float fLon;
	float fLat;

	signed long height;
	signed long hMSL;
	uint8_t hMSLBytes[4];
	unsigned long hAcc;
	unsigned long vAcc;

	signed long gSpeed;
	uint8_t gSpeedBytes[4];
	signed long headMot;

}GNSS_StateHandle;

extern GNSS_StateHandle GNSS_Handle;