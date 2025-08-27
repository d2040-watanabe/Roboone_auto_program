#include <Arduino.h>

#include "standard.hpp"

/*-----定数-----*/
const int TRANSFER_WAIT = 10;
const int FRAME_WAIT = 10;
const int EN_PIN = 2;          //適当でよい
const long BAUDRATE = 115200; // 1250000はだめ
const int TIMEOUT = 2;
const int SERVO_NUM = 13; //サーボの個数（ID: 0 ～ SERVO_NUM - 1）

/*-----クラス-----*/
IcsHardSerialClass krs(&Serial1, EN_PIN, BAUDRATE, TIMEOUT);

//補正をかけて、全サーボを動作
void Set_Pos_All(int id0, int id1, int id2, int id3, int id4, int id5, int id6, int id7, int id8, int id9, int id10, int id11, int id12)
{
  //  int pos = 1;
  int pos_i = 1;
  int crct[SERVO_NUM];
  //各サーボの補正値、crct[x] x:サーボ番号　補正値が小さくなるよう取り付ける
  crct[0] = id0 + 0;
  crct[1] = id1 + -500;
  crct[2] = id2 + 0;
  crct[3] = id3 + -500;
  crct[4] = id4 + 200;
  crct[5] = id5 + 300;
  crct[6] = id6 + 500;
  crct[7] = id7 + 0;
  crct[8] = id8 + 900;
  crct[9] = id9 + 200;
  crct[10] = id10 + 0;
  crct[11] = id11 + 500;
  crct[12] = id12 + 200;
  for (int i = 0; i < SERVO_NUM; i++)
  {
    pos_i = krs.degPos100(crct[i]);
    krs.setPos(i, pos_i);
  }
}

void set_arm(int id9, int id10, int id11, int id12)
{
  int pos_i = 1;
  int crct[SERVO_NUM];
  crct[9] = id9 + 0;
  crct[10] = id10 + 0;
  crct[11] = id11 + 0;
  crct[12] = id12 + 0;
  for (int i = 9; i < SERVO_NUM; i++)
  {
    pos_i = krs.degPos100(crct[i]);
    krs.setPos(i, pos_i);
  }
}

void roll_waist(int id0)
{
  int pos0 = krs.degPos100(id0);
  krs.setPos(0, pos0);
}

//脱力
void Set_Pos_Free()
{
  for (int i = 0; i < SERVO_NUM; i++)
  {
    krs.setFree(i);
    delay(20);
  }
}
void arm_free(int id0, int id1, int id2, int id3, int id4, int id5, int id6, int id7, int id8, int id9, int id11)
{
  int pos_i = 1;
  int pos11 = 1;
  int crct[SERVO_NUM];
  //各サーボの補正値、crct[x] x:サーボ番号
  crct[0] = id0 + 0;
  crct[1] = id1 + 0;
  crct[2] = id2 + 0;
  crct[3] = id3 - 0;
  crct[4] = id4 + 0;
  crct[5] = id5 + 0;
  crct[6] = id6 - 0;
  crct[7] = id7 + -3;
  crct[8] = id8 + -15;
  crct[9] = id9 + 0;
  crct[11] = id11 + 0;
  for (int i = 0; i < 9; i++)
  {
    pos_i = krs.degPos100(crct[i]);
    krs.setPos(i, pos_i);
  }
  pos11 = krs.degPos100(crct[11]);
  krs.setPos(11, pos11);
  krs.setFree(10);
  krs.setFree(12);
}