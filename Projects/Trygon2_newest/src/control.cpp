#include "control.hpp"

#include <Arduino.h>
#include <stdarg.h>

//ゲームパッドへ送る基本的なコマンド列
const uint8_t CMD[] = {0x01, 0x42, 0x00, 0x00, 0x00};
//コマンドのバイト数
const uint8_t CMD_BYTES = sizeof CMD;
//パッドから帰ってくるデータを格納する配列
uint8_t DAT[CMD_BYTES] = {0};
/*-----グローバル変数-----*/
//ヘッダファイルでは宣言のみ行う（この変数がプログラムにはあるよということだけ伝える）
//ここでは実際に変数を作成する実体定義を行っている．
const int TRANSFER_WAIT = 10;
const int FRAME_WAIT = 10;
int VS2_check;
int BIT_left;
int BIT_right;
int BIT_up;
int BIT_down;
int BIT_start;
int BIT_select;
int BIT_sikaku;
int BIT_batu;
int BIT_maru;
int BIT_sankaku;
int BIT_R1;
int BIT_L1;
int BIT_R2;
int BIT_L2;
int BIT_R3;
int BIT_L3;

//-------------------------------------------------------------------------
// Class
//-------------------------------------------------------------------------
typedef union
{
  //ビットフィールド構造体になっている．WORDとしてまとめて扱ったり，個別にアクセスも可能
  uint32_t WORD;
  struct
  {
    //一つのメンバ変数に一つのビットが割り当てられている
    unsigned char L2 : 1;
    unsigned char R2 : 1;
    unsigned char L1 : 1;
    unsigned char R1 : 1;

    unsigned char SANKAKU : 1;
    unsigned char MARU : 1;
    unsigned char BATU : 1;
    unsigned char SIKAKU : 1;

    unsigned char SELECT : 1;
    unsigned char L3 : 1;
    unsigned char R3 : 1;
    unsigned char START : 1;

    unsigned char UP : 1;
    unsigned char RIGHT : 1;
    unsigned char DOWN : 1;
    unsigned char LEFT : 1;
  };
} BUTTON_T;

class Pspad
{
public:
  //各ピン（データ線，コマンド線，セレクト線，クロック線）のピン番号を指定
  Pspad(int dat, int cmd, int sel, int clk);
  //ゲームパッドの状態を読み取るメイン処理
  int PsRead(void);

  BUTTON_T BUTTON;//現在のボタン状態
  int8_t right_x, right_y, left_x, left_y;//アナログスティック座標
  unsigned char VIBRATE;//振動モード制御フラグ（未使用）
  unsigned char ANALOG_MODE;//アナログモードフラグ

  //前回の状態保存用
  struct
  {
    BUTTON_T _BUTTON;
    int8_t right_x, right_y, left_x, left_y;
  } PREV;

  bool ERROR_FLAG;//通信失敗時のフラグ

protected:
  uint8_t PsComm(uint8_t send_data);//1バイト通信処理

  int datpin;
  int cmdpin;
  int selpin;
  int clkpin;
};

/*
   pspad.c

    Created on: 2012/12/21
        Author: KORA
*/

//#include "pspad.h"

//----------------------------------------------------------
// PS PAD
//----------------------------------------------------------
#define PS_DAT digitalRead(datpin) // 400Ωほどのプルアップ必要
#define PS_CMD_H digitalWrite(cmdpin, HIGH)
#define PS_CMD_L digitalWrite(cmdpin, LOW)
#define PS_SEL_H digitalWrite(selpin, HIGH)
#define PS_SEL_L digitalWrite(selpin, LOW)
#define PS_CLK_H digitalWrite(clkpin, HIGH)
#define PS_CLK_L digitalWrite(clkpin, LOW)

//----------------------------------------------------------
// 動作周波数
//----------------------------------------------------------
#define PS_FREQ 100 // 125         // パッドの動作周波数    (kHz)
// タイマはこの２倍の周波数で動かす
// 本来250kHzだが変更しても問題ない
// DAT線のプルアップ抵抗が高すぎる場合は速度を落とすこと

#define TM_PERI (500 / PS_FREQ) // タイマの１クロックに要する時間(us)
// タイマの２クロックをパッドの１クロックとしている

//----------------------------------------------------------
// constructor
//----------------------------------------------------------
Pspad::Pspad(int dat, int cmd, int sel, int clk) : datpin(dat), cmdpin(cmd), selpin(sel), clkpin(clk)
{

  pinMode(datpin, INPUT);
  pinMode(cmdpin, OUTPUT);
  pinMode(selpin, OUTPUT);
  pinMode(clkpin, OUTPUT);

  // GPIOの出力値を設定する
  PS_CMD_H; // CMD を1に
  PS_CLK_H; // CLK を1に
  PS_SEL_H; // SEL を1に
}

//----------------------------------------------------------
// プレステ用ゲームパッドとの１バイトの通信
// 引数
//  int send : 送信データ（0～255）
// 戻り値
//  int 受信データ（0～255）
//----------------------------------------------------------
uint8_t Pspad::PsComm(uint8_t send_data)
{
  uint8_t i;
  uint8_t recv_data = 0;

  for (i = 0; i < 8; i++)
  {

    // 送信データ
    if (send_data & 0x01)
    {
      PS_CMD_H;
    }
    else
    {
      PS_CMD_L;
    }
    // クロックをLOWに
    PS_CLK_L;
    // 待機
    //        while(tm.read_us()/TM_PERI <= 2*i);
    delayMicroseconds(TM_PERI);

    // クロックをHIGHに
    PS_CLK_H;
    // 受信データ
    recv_data |= (PS_DAT << i);
    // 待機
    //        while(tm.read_us()/TM_PERI <= 2*i+1);
    delayMicroseconds(TM_PERI);

    // 送信データを１ビットずらす
    send_data >>= 1;
  }

  // CMDをHIGHに戻す
  PS_CMD_H;

  // ５クロックほど待機
  //    while(tm.read_us()/TM_PERI <= 2*8+2*5);
  delayMicroseconds(TM_PERI * 5);

  return recv_data;
}

//----------------------------------------------------------
// プレステ用ゲームパッドからのデータの取得
// 変数
//  uint32_t key      ボタンの押下状態をbitmapで返す。
//  int8_t   right_x  -128～127 の値を返す。アナログスティック中立時は０
//  int8_t   right_y  -128～127 の値を返す。アナログスティック中立時は０
//  int8_t   left_x   -128～127 の値を返す。アナログスティック中立時は０
//  int8_t   left_y   -128～127 の値を返す。アナログスティック中立時は０
//----------------------------------------------------------
int Pspad::PsRead(void)
{
  uint8_t i, len;
  uint8_t rcv[20] = {0};

  // BUTTON.WORD = 0;
  right_x = 0; // アナログスティック中立
  right_y = 0; // アナログスティック中立
  left_x = 0;  // アナログスティック中立
  left_y = 0;  // アナログスティック中立

  // SELをLOWに
  PS_SEL_L;

  // 少し待機
  //    while(tm.read_us() <=  40);      // 40usでも動く模様
  delayMicroseconds(40);

  // 通信開始
  rcv[0] = PsComm(0x01);     // PADはコマンド'01h'を検出して動作を開始する
  rcv[1] = PsComm(0x42);     // 受信データの下位４ビットが転送バイト数の半分の数を表す
  len = (rcv[1] & 0x03) * 2; // 転送バイト数を求める
  unsigned char data = 0x00;
  if (VIBRATE)
    data = 0x41;
  for (i = 0; i < len + 1; i++)
  {
    rcv[i + 2] = PsComm(data); // PsComm(0x00);    // PADの状態を受信
  }

  // SELをHIGHに
  PS_SEL_H;

  // PADの状態を返す
  PREV._BUTTON.WORD = BUTTON.WORD;
  PREV.right_x = right_x;
  PREV.right_y = right_y;
  PREV.left_x = left_x;
  PREV.left_y = left_y;

  if (/*((rcv[1] >> 4)==0x04 || (rcv[1] >> 4)==0x07) &&*/ rcv[2] == 'Z')
  {
    ERROR_FLAG = 0;
    BUTTON.WORD = ((rcv[3] << 8) | rcv[4]) ^ 0xFFFF;
    if (len >= 6)
    { // Convert range to -128~127
      ANALOG_MODE = 1;
      right_x = rcv[5] - 128;
      right_y = -(rcv[6] - 128 + 1);
      left_x = rcv[7] - 128;
      left_y = -(rcv[8] - 128 + 1);
    }
    else
    {
      ANALOG_MODE = 0;
      right_x = 0;
      right_y = 0;
      left_x = 0;
      left_y = 0;
    }
  }
  else
  {
    ERROR_FLAG = 1;
    BUTTON.WORD = 0;
    ANALOG_MODE = 0;
    right_x = 0;
    right_y = 0;
    left_x = 0;
    left_y = 0;
    return 1;
  }

  return 0;
}

Pspad ps(12, 11, 10, 13);

void ReadVS2()
{
  //  digitalWrite(10, LOW);
  //  delayMicroseconds(TRANSFER_WAIT);
  //  for (uint8_t i = 0; i < CMD_BYTES; i++) {
  //    DAT[i] = SPI.transfer(CMD[i]);
  //    delayMicroseconds(TRANSFER_WAIT);
  //  }
  //  digitalWrite(10, HIGH);
  //  delayMicroseconds(FRAME_WAIT):
  VS2_check = 1;

  ps.PsRead();

  DAT[3] = (~(ps.BUTTON.WORD >> 8)) & 0xFF;
  DAT[4] = (~ps.BUTTON.WORD) & 0xFF;

  if (DAT[3] == 115)
    VS2_check = 1;
  BIT_left = 1, BIT_down = 1, BIT_right = 1, BIT_up = 1, BIT_start = 1,
  BIT_R3 = 1, BIT_L3 = 1, BIT_select = 1;
  if (DAT[3] > 127)
  {
    DAT[3] = DAT[3] - 128;
    BIT_left = 0;
  }
  if (DAT[3] > 63)
  {
    DAT[3] = DAT[3] - 64;
    BIT_down = 0;
  }
  if (DAT[3] > 31)
  {
    DAT[3] = DAT[3] - 32;
    BIT_right = 0;
  }
  if (DAT[3] > 15)
  {
    DAT[3] = DAT[3] - 16;
    BIT_up = 0;
  }
  if (DAT[3] > 7)
  {
    DAT[3] = DAT[3] - 8;
    BIT_start = 0;
  }
  if (DAT[3] > 3)
  {
    DAT[3] = DAT[3] - 4;
    BIT_R3 = 0;
  }
  if (DAT[3] > 1)
  {
    DAT[3] = DAT[3] - 2;
    BIT_L3 = 0;
  }
  if (DAT[3] > 0)
  {
    DAT[3] = DAT[3] - 1;
    BIT_select = 0;
  }
  BIT_sikaku = 1, BIT_batu = 1, BIT_maru = 1, BIT_sankaku = 1, BIT_R1 = 1,
  BIT_L1 = 1, BIT_R2 = 1, BIT_L2 = 1;
  if (DAT[4] > 127)
  {
    DAT[4] = DAT[4] - 128;
    BIT_sikaku = 0;
  }
  if (DAT[4] > 63)
  {
    DAT[4] = DAT[4] - 64;
    BIT_batu = 0;
  }
  if (DAT[4] > 31)
  {
    DAT[4] = DAT[4] - 32;
    BIT_maru = 0;
  }
  if (DAT[4] > 15)
  {
    DAT[4] = DAT[4] - 16;
    BIT_sankaku = 0;
  }
  if (DAT[4] > 7)
  {
    DAT[4] = DAT[4] - 8;
    BIT_R1 = 0;
  }
  if (DAT[4] > 3)
  {
    DAT[4] = DAT[4] - 4;
    BIT_L1 = 0;
  }
  if (DAT[4] > 1)
  {
    DAT[4] = DAT[4] - 2;
    BIT_R2 = 0;
  }
  if (DAT[4] > 0)
  {
    DAT[4] = DAT[4] - 1;
    BIT_L2 = 0;
  }
}