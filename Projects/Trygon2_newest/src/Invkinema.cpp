#include <math.h>
#include <fastmath.h>
#include <Arduino.h>
#include <IcsHardSerialClass.h>
#include "Invkinema.hpp"
#include "standard.hpp"
// rR100,lR100 足首Rの傾き(deg)*100 raw
// rx : pitch direction (mm)
// ry : roll direction (mm)
// rz : height (mm)
void setPosInvLeg(int rR100, int lR100, float rx, float ry, float rz, float lx, float ly, float lz)
{
    float a = 60.0;    //リンク長
    float b = 34;      //腿Rの軸と腿P軸の距離(ｚ方向)
    float c = 34.5;    //足首R軸と足首P軸の距離(ｚ方向)
    float d = 38.454;  //膝の軸間距離
    float e = 226.954; //脚ピン長さ
    float rL = 0.0;
    float lL = 0.0;
    float thetaR1 = 0.0;
    float thetaR2 = 0.0;
    float thetaR3 = 0.0;
    float thetaL1 = 0.0;
    float thetaL2 = 0.0;
    float thetaL3 = 0.0;
    float ReductionRatioOfthighR = 1.0;
    float thighR_R;
    float thighR_L;
    float ankleR_R;
    float ankleR_L;
    float P1R;
    float P2R;
    float P1L;
    float P2L;
    float rR = rR100 / 100;
    float lR = lR100 / 100;
    float legdeg[8];

    // right
    rL = sqrt(pow(rx, 2) + pow(sqrt(pow(e - rz, 2) + pow(ry, 2)) - b - c - d, 2));
    thetaR3 = atan2(ry, e - rz) * 180.0 / M_PI;
    thighR_R = ReductionRatioOfthighR * thetaR3; //腿R
    ankleR_R = rR + thetaR3;                     //足首R
    thetaR1 = acos(rL / (2 * a)) * 180.0 / M_PI;
    thetaR2 = asin(rx / rL) * 180.0 / M_PI;
    P1R = thetaR1 + thetaR2; //腿P
    P2R = thetaR1 - thetaR2; //膝P
    // Serial.println(thighR_R);
    // Serial.println(abs(P1R));
    // Serial.println(abs(P2R));
    // Serial.println(ankleR_R);
    // Serial.println("");

    // left
    lL = sqrt(pow(lx, 2) + pow(sqrt(pow(e - lz, 2) + pow(ly, 2)) - b - c - d, 2));
    thetaL3 = atan2(ly, e - lz) * 180.0 / M_PI; //腿R
    thighR_L = -ReductionRatioOfthighR * thetaL3;
    ankleR_L = lR - thetaL3; //足首R
    thetaL1 = acos(lL / (2 * a)) * 180.0 / M_PI;
    thetaL2 = asin(lx / lL) * 180.0 / M_PI;
    P1L = thetaL1 + thetaL2; //腿P
    P2L = thetaL1 - thetaL2; //膝P

    legdeg[0] = thighR_R;
    legdeg[1] = P1R;
    legdeg[2] = -P2R;
    legdeg[3] = ankleR_R;
    legdeg[4] = thighR_L;
    legdeg[5] = -P1L;
    legdeg[6] = P2L;
    legdeg[7] = ankleR_L;
    // Serial.println(thighR_L);
    // Serial.println(abs(P1L));
    // Serial.println(abs(P2L));
    // Serial.println(ankleR_L);
    // Serial.println("");
    // //   setLegXBusServo(abs(thetaR1 + thetaR2), abs(thetaL1 + thetaL2),
    // //                   abs(thetaR1 - thetaR2), abs(thetaL1 - thetaL2));
    int pos_i = 1.0;
    for (int i = 1; i < 9; i++)
    {
        pos_i = krs.degPos(legdeg[i - 1]);
        krs.setPos(i , pos_i);
        // Serial.println(pos_i);
    }
}