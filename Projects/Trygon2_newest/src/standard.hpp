#ifndef _STANDARD_HPP
#define _STANDARD_HPP

#include <IcsHardSerialClass.h>
/*-----定数-----*/
extern const int EN_PIN;    //適当でよい
extern const long BAUDRATE; // 1250000はだめ
extern const int TIMEOUT;
extern const int SERVO_NUM; //サーボの個数（ID: 0 ～ SERVO_NUM - 1）

/*-----クラス-----*/
extern IcsHardSerialClass krs;

void Set_Pos_All(int id0, int id1, int id2, int id3, int id4, int id5, int id6, int id7, int id8, int id9, int id10, int id11, int id12);
void Set_Pos_Free();
void arm_free(int id0, int id1, int id2, int id3, int id4, int id5, int id6, int id7, int id8, int id9, int id11);
void set_arm(int id9, int id10, int id11, int id12);
void roll_waist(int id0);
#endif