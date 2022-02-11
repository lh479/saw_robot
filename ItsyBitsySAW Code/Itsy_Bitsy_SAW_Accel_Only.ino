#include <Wire.h>

// Accelerometer adresses and variables
#define IIS328DQ_REG_CTRL_REG1      0x20  // Set mode, ODR and enabled axes
#define IIS328DQ_REG_CTRL_REG3      0x22  // Interrupt control register
#define IIS328DQ_REG_CTRL_REG4      0x23  // Set scale
#define IIS328DQ_REG_OUT_X_L        0x28  // X-axis acceleration data in 2's complement. LSB
#define IIS328DQ_REG_INT1_CFG       0X30  // Configuration interrupt 1
#define IIS328DQ_REG_INT1_THS       0X32  // Threshold interrupt 1
#define IIS328DQ_REG_INT1_DURATION  0X33
#define IIS328DQ_PM_NORMAL          0x20
#define IIS328DQ_ODR_100_CF_74      0x08
#define IIS328DQ_X_ENABLE           0x01  // Enable X-axis
#define IIS328DQ_Y_ENABLE           0x02  // Enable Y-axis
#define IIS328DQ_Z_ENABLE           0x04
#define IIS328DQ_FS_2               0x00  // +/- 2g
#define IIS328DQ_FS_4               0x10  // +/- 4g
#define IIS328DQ_FS_8               0x30  // +/- 8g
#define IIS328DQ_BDU_ENABLE         0X80
#define IIS328DQ_SLAVE_ADDR         0X19
#define MULTIPLE_BYTES_MASK         0X80

byte X0,X1,Y0,Y1,Z0,Z1;
byte axis[6];
short X,Y,Z;

float pitch=0;
float roll=0;
float old_pitch_a=0;
float old_roll_a=0;
float alpha = 0.2;

void setup() {
    Serial.begin(19200);
    Wire.begin();
    delay(100);

    // Accelerometer configuration registers
    uint8_t ctrl_reg1 = (IIS328DQ_ODR_100_CF_74 | IIS328DQ_PM_NORMAL | IIS328DQ_Z_ENABLE | IIS328DQ_Y_ENABLE | IIS328DQ_X_ENABLE);
    writeToAccel(IIS328DQ_REG_CTRL_REG1,ctrl_reg1);
    
    //Scale to +/- 2g and BDU
    uint8_t ctrl_reg4 = IIS328DQ_FS_2 | IIS328DQ_BDU_ENABLE;
    writeToAccel(IIS328DQ_REG_CTRL_REG4,ctrl_reg4);

    //Interrupt control
    writeToAccel(IIS328DQ_REG_CTRL_REG3,0x00);

    //Scale threshold
    writeToAccel(IIS328DQ_REG_INT1_THS,0x1A);

    //Interrupt duration
    writeToAccel(IIS328DQ_REG_INT1_DURATION,0x64);

    //ZLIE, YLIE
    writeToAccel(IIS328DQ_REG_INT1_CFG,0x04);

}

void loop() {
    // Multiple bytes mask enables reg_address automatic increase
    Wire.beginTransmission(IIS328DQ_SLAVE_ADDR);
    Wire.write(IIS328DQ_REG_OUT_X_L  | MULTIPLE_BYTES_MASK);
    Wire.endTransmission();
    Wire.beginTransmission(IIS328DQ_SLAVE_ADDR);
    Wire.requestFrom(IIS328DQ_SLAVE_ADDR,6);

    for(int i=0; i<6; i++) {
        axis[i] = Wire.read();
    }

    Wire.endTransmission();

    X = (axis[1] << 8) | axis[0];
    Y = (axis[3] << 8) | axis[2];
    Z = (axis[5] << 8) | axis[4];

    /* CALIBRATION:
        2^16 = 65536 (dinamic range quantification)
        +/- 2g = 4g   ->    65536 / 4 = 16384    */
    X_prec = X * 0.981 / 16384;
    Y_prec = Y * 0.981 / 16384;
    Z_prec = Z * 0.981 / 16384;

    float roll_a = 180 * atan2(Y_prec*0.061, sqrt(X_prec*0.061*X_prec*0.061 + Z_prec*0.061*Z_prec*0.061))/M_PI;  
    float pitch_a = 180 * atan2(X_prec*0.061, sqrt(Y_prec*0.061*Y_prec*0.061 + Z_prec*0.061*Z_prec*0.061))/M_PI;

    roll = alpha * roll_a + ( 1 - alpha) * old_roll_a; 
    pitch = alpha * pitch_a + ( 1 - alpha) * old_pitch_a;           
    old_pitch_a = pitch;
    old_roll_a = roll;

//    Serial.print(" ");
//    Serial.print(X_prec);
//    Serial.print("\t");
//    Serial.print(" ");
//    Serial.print(Y_prec);
//    Serial.print("\t");
//    Serial.print(" ");
//    Serial.print(Z_prec);
//    Serial.println();

    Serial.print(" ");
    Serial.print(roll);
    Serial.print("\t");
    Serial.print(" ");
    Serial.print(pitch);
    Serial.println();
                          
    delay(100);
   
}

/* This should be added to a library (.h & .cpp) from here down to the end, not gonna waste time now */

/* WRITE TO ACCEL VIA I2C */
/* In future add IIS328DQ:: */
void writeToAccel(byte reg_address, byte value) {
    Wire.beginTransmission(IIS328DQ_SLAVE_ADDR);
    Wire.write(reg_address);
    Wire.write(value);
    Wire.endTransmission();
}
