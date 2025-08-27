//多重インクルード（同じヘッダファイルが複数回読み込まれること）を防ぐための構文
#ifndef _CONTROL_HPP
#define _CONTROL_HPP

#include <cstdint>//標準変数型を使うためのインクルード

/*-----グローバル変数-----*/
extern const int TRANSFER_WAIT;
extern const int FRAME_WAIT;
// const uint8_t CMD[] = {0x01, 0x42, 0x00, 0x00, 0x00};
// const uint8_t CMD_BYTES = sizeof CMD;
// extern uint8_t DAT[CMD_BYTES] = {0};
extern int VS2_check;
extern int BIT_left; extern int BIT_right; extern int BIT_up; extern int BIT_down;
extern int BIT_start; extern int BIT_select;
extern int BIT_sikaku; extern int BIT_batu; extern int BIT_maru; extern int BIT_sankaku;
extern int BIT_R1; extern int BIT_L1; extern int BIT_R2; extern int BIT_L2; extern int BIT_R3; extern int BIT_L3;

void ReadVS2();

#endif