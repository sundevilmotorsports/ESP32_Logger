#ifndef INC_DTC_H_
#define INC_DTC_H_

#include <stdint.h>




// DTC Bitwise Macros for Updating the Code Status
#define SET_DTC(container, index)   ((container)->dtcCodes &= ~(1U << (index)))
#define CLEAR_DTC(container, index) ((container)->dtcCodes |= (1U << (index)))
#define CHECK_DTC(container, index) ((container)->dtcCodes & (1U << (index)))

#define DTC_THRESHOLD_MS 20
#define DTC_CHECK_INTERVAL_MS 1000 // 1 second interval for DTC checks
#define DTC_MEASURES 64 // Number of measures to calculate average response time
extern uint32_t DTC_PREV_CHECK_TIME;

typedef struct {
	uint8_t errState; //Error State flag (1 = Error, 0 = No Error)
	uint8_t DTC_Idx; // Index of the DTC Code in the Array
	
    //Corresponds to the 32 addresses in the DTC Code Handler
	uint8_t measures; // Goal Number of Measurements to Calculate Average Response Time (MAX: 256)
	uint8_t bufferIndex; // Index of the Current Measurement in the Buffer

	uint32_t totalTime; //Time (ms) from Last Average Response Time Calculation
	//This data can be received from the CAN_RDTxR register (I copied the data type hehe)
	uint32_t prevTime;
	uint16_t threshold; //Store the percentage of avg response time over allowed before throwing an error
	uint32_t *timeBuffer; //Buffer to Store the Last N Response Times (N = measures)

}can_dtc; //This name needs work I know... <- Have confidence, can_dtc is a great name!

#define DTC_DEVICES \
    X(frWheelBoard_DTC) \
    X(flWheelBoard_DTC) \
    X(rrWheelBoard_DTC) \
    X(rlWheelBoard_DTC) \
    X(fBrakePress_DTC) \
    X(rBrakePress_DTC) \
    X(steer_DTC) \
    X(flShock_DTC) \
    X(frShock_DTC) \
    X(rlShock_DTC) \
    X(rrShock_DTC) \
    X(flStrainGauge_DTC) \
    X(frStrainGauge_DTC) \
    X(rlStrainGauge_DTC) \
    X(rrStrainGauge_DTC) \
    X(imu_DTC) \
    X(brakeNthrottle_DTC) \
    X(gps_0_DTC) \
    X(gps_1_DTC) \
    X(shifter_DTC)



// DTC Index Enum
typedef enum {
    #define X(device) device,
    DTC_DEVICES
    #undef X
    DTC_COUNT
} DTC_Channel;

const char* dtc_device_names[] = {
    #define X(device) #device,
    DTC_DEVICES
    #undef X
};

extern can_dtc *dtc_devices[DTC_COUNT];


void DTC_CAN_Init_Device(can_dtc *dtc, uint8_t index, uint8_t measures, uint16_t threshold, uint32_t start_time);
void DTC_CAN_Update_Error_State(can_dtc *dtc, uint32_t current_time);
void DTC_CAN_Response_Measurement(can_dtc *dtc, uint32_t response_time);
void DTC_Init(uint32_t start_time);
void DTC_Error_Check(uint32_t current_time);
#endif