//
// Created by vassilis on 02.02.16.
//

#ifndef SERIAL_TESTING_MIC488_H
#define SERIAL_TESTING_MIC488_H

#endif //SERIAL_TESTING_MIC488_H

#include <modbus/modbus.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define M1          0
#define M2          1
#define M3          2
#define M_ENABLE    1014
#define M_DISABLE   1015
#define M_VMAX      1052
#define M_POS_ACT   1020
#define M_POS_ABS   1084
#define M_STATUS    1010
#define ON          1
#define OFF         0

#define VELOCITY 15

int initController(modbus_t *ctx, int slaveAddress);
void initMotors(modbus_t *slave);
void motorsOnOff(modbus_t *ctx, bool OnOff);
void setMaxVelocity(modbus_t *slave, uint16_t M, float vel);
void setPosition(modbus_t *ctx, uint16_t M, float pos);
float getPosition(modbus_t *ctx, uint16_t M);
int getStatus(modbus_t *slave, uint16_t M);


#ifdef __cplusplus
}
#endif