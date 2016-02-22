//
// Created by vassilis on 02.02.16.
//

#include "mic488.h"
#ifdef __cplusplus
extern "C" {
#endif

/*  NAME:               int initController(modbus_t *slave, int slaveAddress)
 *
 *  DESCRIPTION:        Initialize a slave device
 *
 *  INPUTS:
 *  -modbus_t *slave    The slave device structure created before calling
 *                      the function
 *
 *  -int slaveAddress   The address of the slave device to be initialized
 *                      (1 - 247 in modbus)
 *
 *  OUTPUTS:            The error code:  0 for no errors, 1 for errors and
 *                      prints the error message
 *
 *
 */
int initController(modbus_t *slave, int slaveAddress) {
    if (slave == NULL) {
        fprintf(stderr, "Unable to create the libmodbus context on serial port\n");
        return -1;
    }

    if (modbus_set_slave(slave, slaveAddress)) {
        fprintf(stderr, "Failed to set modbus slave address\n");
        errno = -1;
        return -1;
    }


    if (modbus_connect(slave)) {
        fprintf(stderr, "Unable to connect to modbus server");
        errno = -1;
        return -1;
    }

    return 0;
}

/*  NAME:               initMotors(modbus_t *slave)
 *
 *  DESCRIPTION:        Initialize a slave device
 *
 *  INPUTS:
 *  -modbus_t *slave    The slave device structure created before calling
 *                      the function
 *
 *  OUTPUTS:            -
 *
 *
 *
 */
void initMotors(modbus_t *slave) {
    if (getStatus(slave, M1) == 0 || getStatus(slave, M2) == 0 || getStatus(slave, M3) == 0) {
        motorsOnOff(slave, ON);
        setMaxVelocity(slave, M1, VELOCITY);
        setMaxVelocity(slave, M2, VELOCITY);
        setMaxVelocity(slave, M3, VELOCITY);
    }

//    else
//    {
//        setPosition(slave,M1,0.0);
//        setPosition(slave,M2,0.0);
//        setPosition(slave,M3,0.0);
//
//        while(getStatus(slave,M1) !=4 && getStatus(slave,M2) !=4 && getStatus(slave,M3) !=4)
//            usleep(100000);
//        printf("Motors are at (0,0,0)\n");
//    }

}

/*  NAME:               motorsOnOff(modbus_t *slave, bool OnOff)
 *
 *  DESCRIPTION:        Switch ON or OFF the 3 motors on the selected
 *                      slave device
 *
 *  INPUTS:
 *  -modbus_t *slave    The slave device that will turn ON or OFF its
 *                      motors
 *
 *  -bool OnOff         1 for ON, 0 for OFF
 *
 *
 *  OUTPUTS:            -
 *
 *  NOTES:              Bitwise operation; 7d = 0b0111 ->
 *                      M1, M2, M3 - ON
 *
 *
 */
void motorsOnOff(modbus_t *slave, bool OnOff) {
    if (OnOff)
        modbus_write_register(slave, M_ENABLE, 7);
    else
        modbus_write_register(slave, M_DISABLE, 7);
}

/*  NAME:               void setVelocity(modbus_t *slave, uint16_t M, float vel)
 *
 *  DESCRIPTION:        Sets a maximum velocity for a motor
 *
 *  INPUTS:
 *  -modbus_t *slave    The target slave device
 *
 *
 *  -uint16_t M         The target motor
 *
 *  -float vel          The desired velocity
 *
 *
 *  OUTPUTS:            -
 *
 *  PROCESS:            [1] Sets the base register - 1068-1069
 *                          for M1, 1070-1071 for M2 and 1072-1073
 *                          for M3. M1 = 0 so the correct offset
 *                          is ensured by "baseRegister = M_VEL_ABS + M*2;"
 *                      [2] Convert the desired position to 2 bytes in
 *                          modbus format
 *                      [3] Write the converted value to the appropriate
 *                          register on the slave device
 */
void setMaxVelocity(modbus_t *slave, uint16_t M, float vel) {
    uint16_t *pos_HEX, baseRegister;
    baseRegister = M_VMAX + M * 2;

    modbus_set_float(vel, &pos_HEX);
    modbus_write_registers(slave, baseRegister, 2, &pos_HEX);


}

/*  NAME:               setPosition(modbus_t *slave, uint16_t M, float pos)
 *
 *  DESCRIPTION:        Sets a target position for a motor, which is immediately
 *                      set to motion towards it
 *
 *  INPUTS:
 *  -modbus_t *slave    The target slave device
 *
 *
 *  -uint16_t M         The target motor
 *
 *  -float pos          The target position
 *
 *
 *  OUTPUTS:            -
 *
 *  PROCESS:            [1] Sets the base register - 1084-1085
 *                          for M1, 1086-1087 for M2 and 1088-1089
 *                          for M3. M1 = 0 so the correct offset
 *                          is ensured by "baseRegister = M_POS_ABS + M*2;"
 *                      [2] Convert the desired position to 2 bytes in
 *                          modbus format
 *                      [3] Write the converted value to the appropriate
 *                          register on the slave device
 */
void setPosition(modbus_t *slave, uint16_t M, float pos) {
    uint16_t *pos_HEX, baseRegister;
    baseRegister = M_POS_ABS + M * 2;

    modbus_set_float(pos, &pos_HEX);
    modbus_write_registers(slave, baseRegister, 2, &pos_HEX);

}


/*  NAME:               getPosition(modbus_t *slave, uint16_t M)
 *
 *  DESCRIPTION:        Gets the current position of a certain motor
 *                      attached to a certain slave device
 *
 *
 *  INPUTS:
 *  -modbus_t *slave    The target slave device
 *
 *
 *  -uint16_t M         The target motor
 *
 *  OUTPUTS:
 *    RETURN:
 *    - float           The current position of the motor as a float
 *                      value
 *
 *  PROCESS:           [1]  Sets the base register - 1020-1021
 *                          for M1, 1022-1023 for M2 and 1024-1025
 *                          for M3. M1 = 0 so the correct offset
 *                          is ensured by "baseRegister = M_POS_ACT + M*2;"
 *                     [2]  The registers are read and the value is put into
 *                          the slaveRegister variable
 *                     [3]  The 2-byte value is converted to float and returned
 *                          by the function
 */
float getPosition(modbus_t *slave, uint16_t M) {
    int n;
    uint16_t slaveRegister[2], baseRegister;

    baseRegister = M_POS_ACT + M * 2;
    n = modbus_read_registers(slave, baseRegister, 2, slaveRegister);

    if (n <= 0) {
        fprintf(stderr, "Unable to read modbus registers\n");
        errno = -1;
    }

    return modbus_get_float(&slaveRegister);
}

/*  NAME:               getStatus(modbus_t *slave, uint16_t M)
 *
 *  DESCRIPTION:        Reads the status register of a motor
 *
 *
 *
 *  INPUTS:
 *  -modbus_t *slave    The target slave device
 *
 *
 *  -uint16_t M         The target motor
 *
 *  OUTPUTS:
 *    RETURN:
 *    - int             The current status of the motor:
 *                      0 – drive turned off (EN signal inactive)
 *                      1 – drive turned on, no motion (EN signal active)
 *                      2 – drive in set velocity mode
 *                      3 – drive in motion to set position mode
 *                      4 – drive achieved the set position
 *                      5 – error of achieving set position (for operation with
 *                      encoder)
 *                      6 – drive in homing mode
 *                      8 – drive in position correction mode (for operation
 *                      with encoder)
 *                      9 - drive achieved limit position L while motion towards
 *                      negative position value (by program or proximity
 *                      sensor signal KL)
 *                      10 - drive achieved limit position R while motion
 *                      towards positive position value (by program or
 *                      proximity sensor signal KR)
 *
 *
 *  PROCESS:           [1]  Sets the correct register - 1010 for M1,
 *                          1011 for M2 and 1012 for M3. M1 = 0 so the
 *                          correct offset is ensured by
 *                          baseRegister = M_STATUS + M;
 *                     [2]  The register is read and the value is put into
 *                          the slaveRegister variable
 *                     [3]  The byte value is returned by the function
 */
int getStatus(modbus_t *slave, uint16_t M) {
    uint16_t slaveRegister[2], baseRegister = M_STATUS + M;
    modbus_read_registers(slave, baseRegister, 1, slaveRegister);
    return slaveRegister[0];
}

#ifdef __cplusplus
}
#endif