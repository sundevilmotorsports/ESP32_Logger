#ifndef LOG_CHANNELS_H
#define LOG_CHANNELS_H

// Define all log channels with preprocessor macros for enum and file header generation
// Need an index for every byte each channel stores - indicate a reserved following byte
// by copying the name of the previous channel and adding the index of the byte
// (e.g. TS is the channel name: TS1 TS2 and TS3 are storage bytes)
#define LOG_CHANNELS \
    X(TS,               "TS") \
    X(TS1,              "TS1") \
    X(TS2,              "TS2") \
    X(TS3,              "TS3") \
    X(TS4,              "TS4") \
    X(TS5,              "TS5") \
    X(TS6,              "TS6") \
    X(TS7,              "TS7") \
    X(F_BRAKEPRESSURE,  "p_F_brake") \
    X(F_BRAKEPRESSURE1, "p_F_brake1") \
    X(R_BRAKEPRESSURE,  "p_R_brake") \
    X(R_BRAKEPRESSURE1, "p_R_brake1") \
    X(STEERING,         "Steering") \
    X(STEERING1,        "Steering1") \
    X(FLSHOCK,          "l_FL_damper") \
    X(FLSHOCK1,         "l_FL_damper1") \
    X(FRSHOCK,          "l_FR_damper") \
    X(FRSHOCK1,         "l_FR_damper1") \
    X(RRSHOCK,          "l_RR_damper") \
    X(RRSHOCK1,         "l_RR_damper1") \
    X(RLSHOCK,          "l_RL_damper") \
    X(RLSHOCK1,         "l_RL_damper1") \
    X(CURRENT,          "amp_Batt") \
    X(CURRENT1,         "amp_Batt1") \
    X(BATTERY,          "v_batt") \
    X(BATTERY1,         "v_batt1") \
    X(IMU_X_ACCEL,      "a_Lat") \
    X(IMU_X_ACCEL1,     "a_Lat1") \
    X(IMU_Y_ACCEL,      "a_Long") \
    X(IMU_Y_ACCEL1,     "a_Long1") \
    X(IMU_Z_ACCEL,      "a_Vert") \
    X(IMU_Z_ACCEL1,     "a_Vert1") \
    X(IMU_X_GYRO,       "r_Pitch") \
    X(IMU_X_GYRO1,      "r_Pitch1") \
    X(IMU_Y_GYRO,       "r_Roll") \
    X(IMU_Y_GYRO1,      "r_Roll1") \
    X(IMU_Z_GYRO,       "r_Yaw") \
    X(IMU_Z_GYRO1,      "r_Yaw1") \
    X(FR_SG,            "FR_SG") \
    X(FR_SG1,           "FR_SG1") \
    X(FL_SG,            "FL_SG") \
    X(FL_SG1,           "FL_SG1") \
    X(RL_SG,            "RL_SG") \
    X(RL_SG1,           "RL_SG1") \
    X(RR_SG,            "RR_SG") \
    X(RR_SG1,           "RR_SG1") \
    X(FLW_AMB,          "t_FL_amb") \
    X(FLW_AMB1,         "t_FL_amb1") \
    X(FLW_OBJ,          "FLW_OBJ") \
    X(FLW_OBJ1,         "FLW_OBJ1") \
    X(FLW_RPM,          "r_FL_wheel") \
    X(FLW_RPM1,         "r_FL_wheel1") \
    X(FRW_AMB,          "t_FR_amb") \
    X(FRW_AMB1,         "t_FR_amb1") \
    X(FRW_OBJ,          "FRW_OBJ") \
    X(FRW_OBJ1,         "FRW_OBJ1") \
    X(FRW_RPM,          "r_FR_wheel") \
    X(FRW_RPM1,         "r_FR_wheel1") \
    X(RRW_AMB,          "t_RR_amb") \
    X(RRW_AMB1,         "t_RR_amb1") \
    X(RRW_OBJ,          "RRW_OBJ") \
    X(RRW_OBJ1,         "RRW_OBJ1") \
    X(RRW_RPM,          "r_RR_wheel") \
    X(RRW_RPM1,         "r_RR_wheel1") \
    X(RLW_AMB,          "t_RL_amb") \
    X(RLW_AMB1,         "t_RL_amb1") \
    X(RLW_OBJ,          "RLW_OBJ") \
    X(RLW_OBJ1,         "RLW_OBJ1") \
    X(RLW_RPM,          "r_RL_wheel") \
    X(RLW_RPM1,         "r_RL_wheel1") \
    X(BRAKE_FLUID,      "BRAKE_FLUID") \
    X(BRAKE_FLUID1,     "BRAKE_FLUID1") \
    X(THROTTLE_LOAD,    "THROTTLE_LOAD") \
    X(THROTTLE_LOAD1,   "THROTTLE_LOAD1") \
    X(BRAKE_LOAD,       "BRAKE_LOAD") \
    X(BRAKE_LOAD1,      "BRAKE_LOAD1") \
    X(DRS,              "DRS") \
    X(GPS_LON,          "gps_Long") \
    X(GPS_LON1,         "gps_Long1") \
    X(GPS_LON2,         "gps_Long2") \
    X(GPS_LON3,         "gps_Long3") \
    X(GPS_LAT,          "gps_Lat") \
    X(GPS_LAT1,         "gps_Lat1") \
    X(GPS_LAT2,         "gps_Lat2") \
    X(GPS_LAT3,         "gps_Lat3") \
    X(GPS_SPD,          "v_car_gps") \
    X(GPS_SPD1,         "v_car_gps1") \
    X(GPS_SPD2,         "v_car_gps2") \
    X(GPS_SPD3,         "v_car_gps3") \
    X(GPS_FIX,          "gps_fix") \
    X(ENGINE_SPEED,     "r_engine") \
    X(ENGINE_SPEED1,    "r_engine1") \
    X(ECT,              "t_eng_coolant") \
    X(OIL_TEMP,         "t_oil") \
    X(OIL_PRESS,        "p_oil") \
    X(OIL_PRESS1,       "p_oil1") \
    X(NEUTRAL_STAT,     "neutral") \
    X(LAMBDA,           "Lamb_1") \
    X(TPS,              "%_TPS") \
    X(GEAR,             "n-Gear") \
    X(GP_SPEED,         "v_trans_out") \
    X(GP_SPEED1,        "v_trans_out1") \
    X(APS_MAIN,         "%_APS_main") \
    X(APS_MAIN1,        "%_APS_main1") \
    X(FUEL_PRESS,       "p_Fuel") \
    X(FUEL_PRESS1,      "p_Fuel1") \
    X(KNOCK_COUNT,      "n_knock_count") \
    X(IGN_ANGLE,        "d_ign_angle") \
    X(IGN_CUT_PCT,      "%_ign_cut") \
    X(FUEL_CUT_PCT,     "%_fuel_cut") \
    X(IDLE_TARGET,      "r_idle_target") \
    X(LAMBDA_FUEL_CORR, "%_lambda_corr") \
    X(LAMBDA_TARGET_ERR,"Lambda_Target_Err") \
    X(IN_GEAR,          "in_gear") \
    X(UPSHIFT_ACT,      "upshift_act") \
    X(DOWNSHIFT_ACT,    "downshift_act") \
    X(LAUNCH_CTRL_STAT, "launch_ctrl_stat") \
    X(ENG_FAN_1,        "eng_fan_1") \
    X(FUEL_LEVEL,       "%_fuel_left") \
    X(ACCEL_FUEL,       "t_fuel_accel") \
    X(ACCEL_FUEL1,      "t_fuel_accel1") \
    X(ACCUM_DIST,       "acc_distance") \
    X(ACCUM_DIST1,      "acc_distance1") \
    X(MAP,              "p_MAP") \
    X(MAP1,             "p_MAP1") \
    X(AN_TEMP_3_,       "t_MAT") \
    X(ENG_IMU_X,        "a_Lat_ecu") \
    X(ENG_IMU_X1,       "a_Lat_ecu1") \
    X(ENG_IMU_Y,        "a_Long_ecu") \
    X(ENG_IMU_Y1,       "a_Long_ecu1") \
    X(ENG_IMU_Z,        "a_Vert_ecu") \
    X(ENG_IMU_Z1,       "a_Vert_ecu1") \
    X(TESTNO,           "TESTNO") \
    X(DTC_FLW,          "DTC_FLW") \
    X(DTC_FRW,          "DTC_FRW") \
    X(DTC_RLW,          "DTC_RLW") \
    X(DTC_RRW,          "DTC_RRW") \
    X(DTC_FLSG,         "DTC_FLSG") \
    X(DTC_FRSG,         "DTC_FRSG") \
    X(DTC_RLSG,         "DTC_RLSG") \
    X(DTC_RRSG,         "DTC_RRSG") \
    X(DTC_IMU,          "DTC_IMU") \
    X(GPS_0_,           "GPS_0_") \
    X(GPS_1_,           "GPS_1_") \
    X(FLT_TTA,          "FLT_TTA") \
    X(FLT_TTA1,         "FLT_TTA1") \
    X(FLT_TTA2,         "FLT_TTA2") \
    X(FLT_TTA3,         "FLT_TTA3") \
    X(FLT_TTA4,         "FLT_TTA4") \
    X(FLT_TTA5,         "FLT_TTA5") \
    X(FLT_TTA6,         "FLT_TTA6") \
    X(FLT_TTA7,         "FLT_TTA7") \
    X(FLT_TTB,          "FLT_TTB") \
    X(FLT_TTB1,         "FLT_TTB1") \
    X(FLT_TTB2,         "FLT_TTB2") \
    X(FLT_TTB3,         "FLT_TTB3") \
    X(FLT_TTB4,         "FLT_TTB4") \
    X(FLT_TTB5,         "FLT_TTB5") \
    X(FLT_TTB6,         "FLT_TTB6") \
    X(FLT_TTB7,         "FLT_TTB7") \
    X(FRT_TTA,          "FRT_TTA") \
    X(FRT_TTA1,         "FRT_TTA1") \
    X(FRT_TTA2,         "FRT_TTA2") \
    X(FRT_TTA3,         "FRT_TTA3") \
    X(FRT_TTA4,         "FRT_TTA4") \
    X(FRT_TTA5,         "FRT_TTA5") \
    X(FRT_TTA6,         "FRT_TTA6") \
    X(FRT_TTA7,         "FRT_TTA7") \
    X(FRT_TTB,          "FRT_TTB") \
    X(FRT_TTB1,         "FRT_TTB1") \
    X(FRT_TTB2,         "FRT_TTB2") \
    X(FRT_TTB3,         "FRT_TTB3") \
    X(FRT_TTB4,         "FRT_TTB4") \
    X(FRT_TTB5,         "FRT_TTB5") \
    X(FRT_TTB6,         "FRT_TTB6") \
    X(FRT_TTB7,         "FRT_TTB7") \
    X(RLT_TTA,          "RLT_TTA") \
    X(RLT_TTA1,         "RLT_TTA1") \
    X(RLT_TTA2,         "RLT_TTA2") \
    X(RLT_TTA3,         "RLT_TTA3") \
    X(RLT_TTA4,         "RLT_TTA4") \
    X(RLT_TTA5,         "RLT_TTA5") \
    X(RLT_TTA6,         "RLT_TTA6") \
    X(RLT_TTA7,         "RLT_TTA7") \
    X(RLT_TTB,          "RLT_TTB") \
    X(RLT_TTB1,         "RLT_TTB1") \
    X(RLT_TTB2,         "RLT_TTB2") \
    X(RLT_TTB3,         "RLT_TTB3") \
    X(RLT_TTB4,         "RLT_TTB4") \
    X(RLT_TTB5,         "RLT_TTB5") \
    X(RLT_TTB6,         "RLT_TTB6") \
    X(RLT_TTB7,         "RLT_TTB7") \
    X(RRT_TTA,          "RRT_TTA") \
    X(RRT_TTA1,         "RRT_TTA1") \
    X(RRT_TTA2,         "RRT_TTA2") \
    X(RRT_TTA3,         "RRT_TTA3") \
    X(RRT_TTA4,         "RRT_TTA4") \
    X(RRT_TTA5,         "RRT_TTA5") \
    X(RRT_TTA6,         "RRT_TTA6") \
    X(RRT_TTA7,         "RRT_TTA7") \
    X(RRT_TTB,          "RRT_TTB") \
    X(RRT_TTB1,         "RRT_TTB1") \
    X(RRT_TTB2,         "RRT_TTB2") \
    X(RRT_TTB3,         "RRT_TTB3") \
    X(RRT_TTB4,         "RRT_TTB4") \
    X(RRT_TTB5,         "RRT_TTB5") \
    X(RRT_TTB6,         "RRT_TTB6") \
    X(RRT_TTB7,         "RRT_TTB7") \
    X(SW_BUTTON,        "SW_BUTTON") \
    X(SW_MLP,           "SW_MLP") \
    X(SW_MRP,           "SW_MRP") \
    X(SW_BLP,           "SW_BLP") \
    X(SW_BMP,           "SW_BMP") \
    X(SW_BRP,           "SW_BRP") \
    X(CL_VOLA,          "CL_VOLA") \
    X(CL_VOLA1,         "CL_VOLA1") \
    X(CL_VOLA2,         "CL_VOLA2") \
    X(CL_VOLA3,         "CL_VOLA3") \
    X(CL_RATEA,         "CL_RATEA") \
    X(CL_RATEA1,        "CL_RATEA1") \
    X(CL_VOLB,          "CL_VOLB") \
    X(CL_VOLB1,         "CL_VOLB1") \
    X(CL_VOLB2,         "CL_VOLB2") \
    X(CL_VOLB3,         "CL_VOLB3") \
    X(CL_RATEB,         "CL_RATEB") \
    X(CL_RATEB1,        "CL_RATEB1") \
    X(CH_COUNT,         "CH_COUNT")

/*
*** OLD WFT HANDLERS ***
    X(SLIP_ANG_1_) \
    X(SLIP_ANG_1_1) \
    X(SLIP_ANG_2_) \
    X(SLIP_ANG_2_1) \
    X(SLIP_ANG_3_) \
    X(SLIP_ANG_3_1) \
    X(SLIP_ANG_4_) \
    X(SLIP_ANG_4_1) \
    X(SLIP_ANG_5_) \
    X(SLIP_ANG_5_1) \
    X(SLIP_ANG_6_) \
    X(SLIP_ANG_6_1) \
    X(WFT_FX_Force) \
    X(WFT_FX_Force1) \
    X(WFT_FY_Force) \
    X(WFT_FY_Force1) \
    X(WFT_FZ_Force) \
    X(WFT_FZ_Force1) \
    X(WFT_MX_Moment) \
    X(WFT_MX_Moment1) \
    X(WFT_MY_Force) \
    X(WFT_MY_Force1) \
    X(WFT_MZ_Force) \
    X(WFT_MZ_Force1) \
    X(WFT_Wheelspeed) \
    X(WFT_Wheelspeed1) \
    X(WFT_Position) \
    X(WFT_Position1) \
    X(WFT_X_Acceleration) \
    X(WFT_X_Acceleration1) \
    X(WFT_Y_Acceleration) \
    X(WFT_Y_Acceleration1) \
    X(WFT_Z_Acceleration) \
    X(WFT_Z_Acceleration1) \
*/


// Generate the enum using the macro
enum LogChannel {
    #define X(channel, name) channel,
    LOG_CHANNELS
    #undef X
};

extern uint8_t logBuffer[CH_COUNT];

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_accel_t;

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_gyro_t;


// Old WFT Handlers
// typedef struct{
//     int16_t Fx_Force;
//     int16_t Fy_Force;
//     int16_t Fz_Force;
//     int16_t Mx_Moment;
// } WFT_CAN1_t;

// typedef struct{
//     int16_t My_Moment;
//     int16_t Mz_Moment;
//     int16_t Wheelspeed;
//     int16_t Position;
// } WFT_CAN2_t;

// typedef struct{
//     int16_t X_Acceleration;
//     int16_t Y_Acceleration;
//     int16_t Z_Acceleration;
// } WFT_CAN3_t;

typedef struct{
    uint64_t tiretemp1;
    uint64_t tiretemp2;
} tiretemp_data;

// Old WFT Handlers
// typedef struct{
//     int16_t POS1;
//     int16_t POS2;
//     int16_t POS3;
//     int16_t POS4;
//     int16_t POS5;
//     int16_t POS6;
// } SLIP_t;

typedef struct{
    uint16_t ambTemp;
    uint16_t objTemp;
    uint16_t rpm;
} wheel_data_s_t;

typedef struct{
    // const uint8_t stream_id_2 = 1000;
    //Stream 2 Data:
    //Frame 1
    uint16_t engine_speed;
    uint8_t ect;
    uint8_t oil_temperature;
    uint16_t oil_pressure;
    uint8_t neutral_stat;
    //Frame 2
    uint8_t lambda1; // Multiplier of 100
    uint8_t tps;
    uint8_t gear;
    uint16_t gp_speed1;
    //Frame 3
    uint16_t aps_main;
    uint16_t fuel_pressure;
    //Frame 4
    uint8_t knock_count_global;
    uint8_t ign_angle;
    uint8_t ign_cut_pct;
    uint8_t fuel_cut_pct;
    uint8_t idle_target;
    uint8_t lambda_fuel_corr;
    uint8_t lambda_target_err;
    //Frame 5
    uint8_t in_gear;
    uint8_t upshift_act;
    uint8_t downshift_act;
    uint8_t launch_ctrl_stat;
    uint8_t eng_fan_1;
    uint8_t fuel_left;

    //Stream 3 Data:
    //Frame 1:
    uint16_t accel_fuel;
    uint16_t accumulated_dist;

    // const uint8_t stream_id_5 = 1001;
    //Stream 5 Data:
    //Frame 1
    //engine speed repeat
    //tps repeat
    //aps repeat
    //lambda1 repeat

    // const uint8_t stream_id_6 = 1002;
    //Stream 6 Data:
    //Frame 1
    //oil pressure repeat
    //fuel pressure repeat
    uint16_t map;

    // const uint8_t stream_id_7 = 1003;
    //Stream 7 Data:
    //Frame 1
    //ect repeat
    uint8_t an_temp_3; //Raw measurement of Engine Oil Temperature?

    //Stream 8 Data:
    int16_t imu_accel_x;
    int16_t imu_accel_y;
    int16_t imu_accel_z;


} engine_t;

extern engine_t engine;
extern imu_accel_t imu_accel;
extern imu_gyro_t  imu_gyro;
extern tiretemp_data frt, flt, rlt, rrt;

// Old WFT Handlers
// extern WFT_CAN1_t WFT_1;
// extern WFT_CAN2_t WFT_2;
// extern WFT_CAN3_t WFT_3;
// extern SLIP_t SLIP;

// Optional: Generate string names for debugging/logging
#ifdef LOG_CHANNEL_NAMES
static const char* log_channel_names[] = {
    #define X(channel, name) name ",",
    LOG_CHANNELS
    #undef X
};
#endif

#endif // LOG_CHANNELS_H
