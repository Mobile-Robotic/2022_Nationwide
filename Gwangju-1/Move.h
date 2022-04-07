/************************************************************************************************************/
/*                                                                                                          */
/*                                                Move.h                                                    */
/*                                                                                                          */
/*                                                                                       2020. 1. 1.        */
/************************************************************************************************************/
#ifndef __MOVE_H
#define __MOVE_H

#define SW1		(~PINB&0x10)
#define SW2		(~PINB&0x20)
#define SW3		(~PINB&0x40)
#define SW4		(~PINB&0x80)

#define Hold()	{StopMotion(10); while(SW1); while(!SW1); while(SW1);} //로봇일시정지 (display 확인할 때 사용)

extern volatile int t_ga, t_gw, f_cnt, f_max;
extern volatile float speed, wspeed, acc, accw, Ga, Gw;
extern float STOP, WSTOP, ve_x[3], ve_y[3], ve_l[3], ve_d[3], p_err[4], delta[4], px[4], py[4], pw[4];
extern long A_DIS[3], A_RET[3], A_DEG[3][3], PSD[10];
extern int C_D[9][5], C_N, FLine, Getting;
extern char str[21];

extern char b_cnt, b_flg, b_val;

void Buzz(int cnt);
float Trans(int Min, int Max, int min, int max, float value);
float PD(int ch, float err, float kp, float kd);
int Mode(float len, int sp, int gg, int mode, int num, int value, int mm);
long READ_EN(int num);

void HD(float x, float y, float w, float filter);
void VT(float f_agl, float len, int ch);
void len_VT(float x, float y, int ch);
void DI(int value);
void reset(int sp, int wsp, int gg, int filter);

void ping(int set);
void LM(int ch, int sp, int wsp, int gg, int x, int y, int w, int filter);
void Hm(int f_agl, int sp, int gg, int x, int y, int mode, int num, int value, int mm, int filter);
void HM(int f_agl, int sp, int gg, int x, int y, int filter);
void TD(int f_agl, int sp, int gg, int x, int y, int w, int filter);
void Pm(int f_psd, int s_dir, int dir, int ta, int sp, int gg, int mode, int num, int value, int mm, int filter);
void FCC(int s_dir, int dir, int num, int value);
void RT(int f_agl, int dir, int sp, int gg, float rad, float w, int num, int mm, int filter);
void turn(int dir, int sp, int gg, int w, int num, int mm);

void V1(int mode, int first, int num, int color, int Xd, int Xu, int Yd, int Yu, int pointX, int pointY, int attack);

void Line(int f_agl, int sp, int gg, int mode, int num, int value, int mm, int filter);

void NH(float Fx, float Fy, float Fw);
void HolonomicW(float f_agl, float f_speed, float fw_speed);

#endif      // __MOVE_H
