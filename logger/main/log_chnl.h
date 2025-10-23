#ifndef LOG_CHANNELS_H
#define LOG_CHANNELS_H

// Define all log channels with preprocessor macros for enum and file header generation
// Need an index for every byte each channel stores - indicate a reserved following byte
// by copying the name of the previous channel and adding the index of the byte 
// (e.g. TS is the channel name: TS1 TS2 and TS3 are storage bytes)
#define LOG_CHANNELS \
    X(TS) \
    X(TS1) \
    X(TS2) \
    X(TS3) \
    X(F_BRAKEPRESSURE) \
    X(F_BRAKEPRESSURE1) \
    X(R_BRAKEPRESSURE) \
    X(R_BRAKEPRESSURE1) \
    X(STEERING) \
    X(STEERING1) \
    X(FLSHOCK) \
    X(FLSHOCK1) \
    X(FRSHOCK) \
    X(FRSHOCK1) \
    X(RRSHOCK) \
    X(RRSHOCK1) \
    X(RLSHOCK) \
    X(RLSHOCK1) \
    X(CURRENT) \
    X(CURRENT1) \
    X(BATTERY) \
    X(BATTERY1) \
    X(IMU_X_ACCEL) \
    X(IMU_X_ACCEL1) \
    X(IMU_X_ACCEL2) \
    X(IMU_X_ACCEL3) \
    X(IMU_Y_ACCEL) \
    X(IMU_Y_ACCEL1) \
    X(IMU_Y_ACCEL2) \
    X(IMU_Y_ACCEL3) \
    X(IMU_Z_ACCEL) \
    X(IMU_Z_ACCEL1) \
    X(IMU_Z_ACCEL2) \
    X(IMU_Z_ACCEL3) \
    X(IMU_X_GYRO) \
    X(IMU_X_GYRO1) \
    X(IMU_X_GYRO2) \
    X(IMU_X_GYRO3) \
    X(IMU_Y_GYRO) \
    X(IMU_Y_GYRO1) \
    X(IMU_Y_GYRO2) \
    X(IMU_Y_GYRO3) \
    X(IMU_Z_GYRO) \
    X(IMU_Z_GYRO1) \
    X(IMU_Z_GYRO2) \
    X(IMU_Z_GYRO3) \
    X(FR_SG) \
    X(FR_SG1) \
    X(FL_SG) \
    X(FL_SG1) \
    X(RL_SG) \
    X(RL_SG1) \
    X(RR_SG) \
    X(RR_SG1) \
    X(FLW_AMB) \
    X(FLW_AMB1) \
    X(FLW_OBJ) \
    X(FLW_OBJ1) \
    X(FLW_RPM) \
    X(FLW_RPM1) \
    X(FRW_AMB) \
    X(FRW_AMB1) \
    X(FRW_OBJ) \
    X(FRW_OBJ1) \
    X(FRW_RPM) \
    X(FRW_RPM1) \
    X(RRW_AMB) \
    X(RRW_AMB1) \
    X(RRW_OBJ) \
    X(RRW_OBJ1) \
    X(RRW_RPM) \
    X(RRW_RPM1) \
    X(RLW_AMB) \
    X(RLW_AMB1) \
    X(RLW_OBJ) \
    X(RLW_OBJ1) \
    X(RLW_RPM) \
    X(RLW_RPM1) \
    X(BRAKE_FLUID) \
    X(BRAKE_FLUID1) \
    X(THROTTLE_LOAD) \
    X(THROTTLE_LOAD1) \
    X(BRAKE_LOAD) \
    X(BRAKE_LOAD1) \
    X(DRS) \
    X(GPS_LON) \
    X(GPS_LON1) \
    X(GPS_LON2) \
    X(GPS_LON3) \
    X(GPS_LAT) \
    X(GPS_LAT1) \
    X(GPS_LAT2) \
    X(GPS_LAT3) \
    X(GPS_SPD) \
    X(GPS_SPD1) \
    X(GPS_SPD2) \
    X(GPS_SPD3) \
    X(GPS_FIX) \
    X(ECT) \
    X(OIL_PSR) \
    X(OIL_PSR1) \
    X(TPS) \
    X(APS) \
    X(DRIVEN_WSPD) \
    X(DRIVEN_WSPD1) \
    X(TESTNO) \
    X(DTC_FLW) \
    X(DTC_FRW) \
    X(DTC_RLW) \
    X(DTC_RRW) \
    X(DTC_FLSG) \
    X(DTC_FRSG) \
    X(DTC_RLSG) \
    X(DTC_RRSG) \
    X(DTC_IMU) \
    X(GPS_0_) \
    X(GPS_1_) \
    X(CH_COUNT)

// Generate the enum using the macro
enum LogChannel {
    #define X(channel) channel,
    LOG_CHANNELS
    #undef X
};

extern uint8_t logBuffer[CH_COUNT];


// Optional: Generate string names for debugging/logging
#ifdef LOG_CHANNEL_NAMES
static const char* log_channel_names[] = {
    #define X(channel) {#channel ","},
    LOG_CHANNELS
    #undef X
};
#endif

#endif // LOG_CHANNELS_H