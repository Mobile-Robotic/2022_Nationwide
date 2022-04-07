/************************************************************************************************************/
/*                                                                                                          */
/*                                              MobileRobot.c                                               */
/*                                                                                                          */
/*                                                                                       2020. 1. 1.        */
/************************************************************************************************************/
#include "Interface.h"

// 옵션에서(톱니바퀴)	-> libraries -> liba.a & libm.a 추가
// 옵션에서(톱니바퀴)	-> General -> Frequency 14745600 설정
// Interface.c 		 	-> Camera Baud Rate Setting : (USART1 Baud Rate: 115200)(통신속도)(UBRR1L=0x5F -> UBRR1L=0x07)
// Interface.c 			-> ISR Camera_V1 Setting
// Interface.c			-> lcd_clear_screen()을 cls()로 display_char()을 dc()로
// Interface.c 			-> Interface_init()에 putchar1(18); 추가 (카메라 V1 프로토콜 사용)
// Motor.c 			 	-> SetGain 10, 3, 1 Setting
// Sensor.c				-> Sensor_init()에 DDRD  &= ~0x50; 추가
// Sensor.h				-> READ_SENSOR()를 S로 변경
// Sensor.h				-> S에 | (~(PIND<<1)&0x20) | (~PIND&0x40) 추가

#define rev_stdir (-stdir)
#define rev_dir (!dir)
#define rev_num (9-num)
#define rev_ta (ta==135 ? 165:135)

int stdir, dir, num, ta;

void set(int m){
	if(m) stdir=1, dir=1, num=8, ta=135;
	else stdir=-1, dir=0, num=1, ta=165;
}

// <Node>
// ㅡㅡㅡ d ㅡㅡㅡ
// |			 |
// c			 c
// |			 |
// b			 b
// |			 |
// a ㅡ start ㅡ a
// |			 |
// ㅡㅡㅡ e ㅡㅡㅡ

// FLine 3번 사용
// int mode에 b_flg, b_cnt 추가

int A; // A구역 가변벽 위치
int T, E1, E2; // 티칭 색깔
int C; // C구역 위 아래 지나갈곳 (1~2)

//==========================================================
//구역별 이동함수

int check_gate(){ // 게이트 체크함수
	if(FLine==3) FLine=1;
	return FLine!=T;
}

int start_a(int flg){
	if(psd_value[2]<50) A=0;
	if(psd_value[3]<50) A=1;
	if(psd_value[7]<50) A=2;
	if(psd_value[6]<50) A=3;

	set(A>1);

	if(A%2){
		LM(4,500,100,11,-300,250*stdir,71*rev_stdir,0);
		if(dir){
			FCC(2,0,4,135);
			Pm(5,2,2,150,600,11,1,5,230,0,0);
		}
		else{
			FCC(2,0,5,165);
			Pm(4,2,2,150,600,11,1,4,230,0,0);
		}
	}
	else{
		ping(3);
		turn(rev_stdir,200,10,45,0,0);
		LM(3,500,200,111,200,250*stdir,110*rev_stdir,0);
		if(!dir){
			FCC(2,0,4,135);
			Pm(5,2,2,150,600,11,1,5,230,0,0);
		}
		else{
			FCC(2,0,5,165);
			Pm(4,2,2,150,600,11,1,4,230,0,0);
		}
	}

	V1(1,2,0,0,0,180,0,240,0,0,0);
	T=C_D[0][0];

	LM(3,400,100,11,px[3]-200,py[3],3*stdir,0);
	FCC(1,dir,0,0);
	return 0;
}
void do_b(){
	FCC(1,2,1,165);
	ping(1);
	TD(0,400,11,200,0,45,0);
	V1(1,0,0,0,0,180,0,240,0,0,0);
	if(C_D[0][0]==T){
		TD(-45,400,11,250,10,47,0);
		HM(0,300,11,50,0,0);
		FCC(1,2,0,0);
		HM(0,300,11,-50,-50,0);
		FCC(2,0,1,165);

		ping(3);
		Pm(0,0,0,165,300,11,0,0,170,0,0);
		LM(3,500,0,11,50,0,pw[3],0);
		FCC(1,2,1,165);
		HM(0,300,11,-50,0,0);
	}
	LM(1,500,0,11,10,0,0,0);
	FCC(1,2,1,165);
}
int a_b(int flg){
	FCC(1,dir,0,0);

	Pm(0,1,dir,0,650,10,3,9,0,0,0);

	if(check_gate()){
		Pm(0,1,dir,0,650,01,0,0,150,0,0);
		Pm(180,1,dir,0,650,11,0,0,1250,0,0);
		FCC(1,dir,4+dir,230);
		return 1;
	}

	if(dir){
		LM(4,500,100,01,150,0,45*rev_stdir,0);
		TD(45*stdir,400,11,250,50*rev_stdir,45*rev_stdir,0);
	}
	else{
		Pm(0,1,dir,0,650,00,0,0,400,0,0);
		TD(0,500,01,500,50*rev_stdir,92*rev_stdir,0);
	}
	if(flg) do_b();
	FCC(1,2,1,165);
	return 0;
}
void do_c(){
	FCC(1,2,rev_num,rev_ta);
	ping(1);
	LM(1,400,100,11,0,200*stdir,45*rev_stdir,0);
	V1(1,rev_dir,0,0,0,180,0,240,0,0,0);
	int tmp[3];
	for(int i=0;i<3;i++) tmp[i]=C_D[i][0];
	ping(3);
	A_DEG[0][2]=(long)3*416*45*rev_stdir;
	for(int i=0;i<3;i++){
		if(tmp[i]==T){
			LM(3,500,0,11,180+130*(i-1),0,45*rev_stdir,0);
			V1(0,2,0,T,0,150,0,240,130,130,3);
			HM(0,500,11,70,0,0);
			HM(0,500,11,-70,0,0);
		}
	}
	LM(1,500,0,11,10,0,0,0);
	FCC(1,2,rev_num,rev_ta);
}
int b_c(int flg){
	FCC(1,2,1,165);
	LM(4,400,100,11,0,200*rev_stdir,92*stdir,0);
	FCC(1,dir,0,0);	

	Pm(0,1,dir,0,650,10,3,9,0,0,0);
	
	if(check_gate()){
		Pm(0,1,dir,0,650,01,0,0,150,0,0);
		Pm(180,1,dir,0,650,11,0,0,2300,0,0);
		FCC(1,dir,4+dir,230);
		return 1;
	}

	LM(4,500,100,01,400,50*rev_stdir,92*rev_stdir,0);
	if(flg) do_c();
	FCC(1,2,rev_num,rev_ta);
	return 0;
}
int c_d(int flg){
	int B; // 가변벽이 0이면 맨 끝쪽, 1이면 중앙 안쪽

	FCC(1,2,rev_num,rev_ta);
	
	if((!dir && psd_value[2]<50) || (dir && psd_value[7]<50)){
		B=0;
		
		LM(4,400,100,11,-30,200*stdir,111*rev_stdir,0);
		if(dir){
			FCC(2,0,3,135);
			Pm(4,0,1,135,600,11,1,4,230,0,0);
		}
		else{
			FCC(2,0,6,165);
			Pm(5,0,0,165,600,11,1,5,230,0,0);
		}
		
		ping(3);
		A_DEG[0][2]=(long)3*416*20*rev_stdir;
		LM(3,400,100,11,-50,150*stdir,71*rev_stdir,0);

		b_cnt=0, b_flg=1;
		if(dir){
			b_val=0x40;
			FCC(2,0,4,135);
			Pm(5,0,1,135,600,10,0,0,200,0,0);
			E2=b_cnt;

			Pm(5,0,1,135,600,01,0,0,500,0,0);
		}
		else{
			b_val=0x20;
			FCC(2,0,5,165);
			Pm(4,0,0,165,600,10,0,0,200,0,0);
			E1=b_cnt;

			Pm(4,0,0,165,600,01,0,0,500,0,0);
		}
	}
	else{
		B=1;
		LM(4,500,80,11,450,200*stdir,92*stdir,0);
		FCC(2,0,num,ta);
		TD(0,500,11,450,0,92*stdir,0);

		FCC(1,rev_dir,0,0);
		FCC(2,0,rev_num,rev_ta);

		b_cnt=0, b_flg=1;
		b_val=0x60;
		Pm(0,0,rev_dir,rev_ta,600,11,1,0,110,0,0);
		if(dir) E2=b_cnt;
		else E1=b_cnt;

		turn(stdir,100,11,20,0,0);
		if(dir){
			FCC(2,0,4,135);
			Pm(5,0,1,135,600,11,0,0,700,0,0);
		}
		else{
			FCC(2,0,5,165);
			Pm(4,0,0,165,600,11,0,0,700,0,0);
		}
	}
	
	if(check_gate()){
		if(B){
			TD(20*rev_stdir,400,11,350,0,71*stdir,0);
			HM(0,300,11,50,0,0);
			FCC(1,2,rev_num,rev_ta);

			TD(0,400,10,300,0,20*stdir,0);
			TD(20*rev_stdir,500,01,300,500*rev_stdir,67*stdir,50);
			Hm(180,400,11,0,0,1,4,200,0,0);
		}
		else{
			turn(rev_stdir,100,11,20,0,0);
			FCC(2,0,rev_num,rev_ta);
			Pm(0,0,rev_dir,rev_ta,650,10,0,0,350,0,0);
			TD(0,500,01,500,0,92*stdir,0);
			HM(0,300,11,50,0,0);
			FCC(1,2,rev_num,rev_ta);
			
			Pm(0,0,rev_dir,rev_ta,600,10,0,0,150,0,0);
			TD(0,500,01,500,50*stdir,92*stdir,0);
		}
		FCC(1,2,rev_num,rev_ta);
		return 1;
	}
	return 0;
}
//역방향
int d_c(int flg){ // 리턴값 없음 (라인 체크 없음)
	if(dir){
		FCC(2,0,4,135);
		Pm(5,0,1,135,600,11,1,5,230,0,0);
	}
	else{
		FCC(2,0,5,165);
		Pm(4,0,0,165,600,11,1,4,230,0,0);
	}
	
	if((!dir && psd_value[2]<100) || (dir && psd_value[7]<100)){
		ping(3);
		A_DEG[0][2]=(long)3*416*20*stdir;
		LM(3,400,100,11,-50,200*stdir,71*rev_stdir,0);
		if(dir){
			FCC(2,0,4,135);
			Pm(5,0,1,135,600,11,0,0,500,0,0);
		}
		else{
			FCC(2,0,5,165);
			Pm(4,0,0,165,600,11,0,0,500,0,0);
		}
		TD(71*stdir,100,11,50,0,71*stdir,0);
	}
	else{
		TD(20*rev_stdir,500,11,500,0,71*stdir,0);
		HM(0,300,11,50,0,0);
		FCC(1,2,num,ta);

		TD(0,400,10,300,0,20*rev_stdir,0);
		TD(20*stdir,500,01,300,500*stdir,67*rev_stdir,50);
		Hm(180,400,11,0,0,1,4,200,0,0);
	}

	if(flg){
		set(rev_dir);
		do_c();
		set(rev_dir);
	}
	FCC(1,2,num,ta);
	return 0;
}
int c_b(int flg){
	FCC(1,2,num,ta);

	LM(4,400,100,11,0,200*rev_stdir,92*stdir,0);
	FCC(1,dir,0,0);

	Pm(0,1,dir,0,650,10,3,9,0,0,0);
	if(check_gate()){
		Pm(0,1,dir,0,650,01,0,0,150,0,0);
		Pm(180,1,dir,0,650,11,0,0,400,0,0);
		ping(3);
		turn(rev_stdir,200,10,45,0,0);
		LM(3,400,200,111,150,50*rev_stdir,92*rev_stdir,0);
		FCC(1,2,num,ta);
		return 1;
	}

	if(dir){
		LM(4,500,100,01,150,0,45*rev_stdir,0);
		TD(45*stdir,400,11,250,50*rev_stdir,45*rev_stdir,0);
	}
	else{
		Pm(0,1,dir,0,650,00,0,0,400,0,0);
		TD(0,500,01,500,50*rev_stdir,92*rev_stdir,0);
	}
	if(flg)	do_b();
	FCC(1,2,1,165);
	return 0;
}
int cb_a(int m){
	FCC(1,2,num,ta);
	int len;
	if(m==1) len=2050;
	else if(dir) len=1550;
	else len=1050;
	TD(0,500,10,-20,500*stdir,92*rev_stdir,0);
	Pm(180,1,rev_dir,0,650,01,0,0,len,0,70);
	FCC(1,rev_dir,rev_dir+4,230);
	return 0;
}
void do_e(){
	int t=dir?E1:E2;
	FCC(1,rev_dir,0,0);

	V1(1,dir,0,0,0,180,0,240,0,0,0);
	ping(3);
	if(C_D[0][0]==t){
		Hm(0,400,11,0,0,1,0,110,0,0);
		turn(stdir,200,11,30,0,0);
		HM(-pw[3],500,11,100,0,0);
		turn(rev_stdir,150,10,40,0,0);
	}
	else{
		Hm(0,400,11,0,0,1,0,120,0,0);
		turn(rev_stdir,100,11,20,0,0);
		if(dir) V1(0,rev_dir,0,0,0,180,0,240,0,140,2);
		else V1(0,rev_dir,0,0,0,180,0,240,0,120,2);
		HM(-pw[3],300,11,130,0,0);
		turn(rev_stdir,100,11,10,0,0);
	}
	LM(3,400,0,11,0,0,0,0);

	FCC(1,rev_dir,0,0);
}
int a_e(int flg){
	FCC(1,rev_dir,rev_dir+4,230);
	
	TD(0,400,10,-70,200*stdir,20*rev_stdir,0);
	if(!dir) Hm(20*stdir+180,400,01,0,0,1,5,230,0,50);
	else Hm(20*stdir+180,400,01,0,0,1,4,230,0,50);
	
	if(flg){
		TD(20*stdir,400,11,0,200*stdir,71*rev_stdir,0);

		if(dir) FCC(2,0,3,200);
		else FCC(2,0,6,230);
		do_e();

		turn(stdir,200,11,90,0,0);
		LM(4,400,100,11,0,180*rev_stdir,92*stdir,0);
	} 
	else turn(stdir,200,11,110,0,0);

	FCC(1,dir,dir+4,230);
	
	if(!C){
		if(dir) V1(1,0,0,0,50,180,20,240,0,0,0);
		else V1(1,0,0,0,50,180,0,220,0,0,0);
		if(C_N) C=1;
		else C=2;
	}

	if(C==1){
		HM(0,500,10,0,180*rev_stdir,0);
		Pm(0,2,2,150,600,00,0,0,550,0,100);
		HM(0,500,01,0,350*stdir,100);
	}
	else{
		FCC(2,0,num,ta);
		Pm(0,0,dir,ta,650,11,0,0,750,0,0);
		HM(0,300,11,0,30*rev_stdir,0);
	}

	if(!dir) FCC(2,0,3,200);
	else FCC(2,0,6,230);
	FCC(1,dir,0,0);

	if(flg){
		set(rev_dir);
		do_e();
		set(rev_dir);
	}

	ping(3);
	turn(rev_stdir,200,10,45,0,0);
	LM(3,400,200,111,200,30*rev_stdir,92*rev_stdir,0);

	if(dir) FCC(1,2,6,230);
	else FCC(1,2,3,230);

	if(flg){
		ping(3);
		if(C==1) LM(3,400,100,11,100,0,110*rev_stdir,0);
		else turn(rev_stdir,200,11,60,0,0);
		V1(1,2,0,0,0,180,0,240,0,0,0);
		if(C_D[0][0]==T){
			V1(0,2,0,0,0,180,0,240,130,130,3);
			HM(0,500,11,70,0,0);
		}
		LM(3,400,100,11,10,0,0,0);
	}

	if(dir) FCC(1,2,6,230);
	else FCC(1,2,3,230);
	return 0;
}
int e_a(int flg){
	if(dir) FCC(1,2,6,230);
	else FCC(1,2,3,230);

	TD(0,450,10,150,0,25*stdir,0);
	HM(20*rev_stdir,500,01,150,350*stdir,50);

	turn(rev_stdir,100,11,20,0,0);
	FCC(1,dir,4+dir,230);
	return 0;
}
void do_start(){
	ping(3);
	V1(0,2,0,T,0,180,0,240,130,130,3);
	HM(0,500,11,70,0,0);
	LM(3,500,0,11,0,0,pw[3],0);
}
//마지막 골인
int a_start(int flg){
	FCC(1,dir,4+dir,230);

	if(A%2) TD(0,500,11,450,30*rev_stdir,92*rev_stdir,0);
	else{
		Pm(0,1,dir,0,650,10,0,0,550,0,0);
		TD(0,500,01,500,30*rev_stdir,92*rev_stdir,0);
		set(!dir);
	}
	FCC(1,2,rev_num,rev_ta);

	Pm(0,0,rev_dir,rev_ta,600,10,0,0,450,0,0);
	RT(0,stdir,500,01,300,92,0,0,0);

	do_start();
	LM(3,0,200,11,px[3],py[3],180,0);
	do_start();
	return 0;
}

void M1()
{
	start_a(1);
	while(1){
		if(a_b(1)){
			set(!dir);
			a_e(0);
			e_a(0);

			a_b(1);
			b_c(1);
			c_d(1);
			d_c(1);
			c_b(1);
		
			set(!dir);
			b_c(0);
			c_d(0);
			d_c(0);
			cb_a(1);
			break;
		}
		if(b_c(1)){
			set(!dir);
			a_e(0);
			e_a(0);

			a_b(1);
			b_c(1);
			c_d(1);
			d_c(1);
		
			set(!dir);
			c_d(0);
			d_c(0);
			cb_a(1);
			break;
		}
		if(c_d(1)){
			set(!dir);
			cb_a(1);
			a_e(0);
			e_a(0);
			a_b(1);
			b_c(1);
			c_d(1);

			set(!dir);
			cb_a(1);
			break;
		}
		d_c(1);
		if(c_b(1)){
			set(!dir);
			c_d(0);
			d_c(0);
			cb_a(1);
			a_e(0);
			e_a(0);
			a_b(1);
		
			set(!dir);
			cb_a(2);
			break;
		}
		set(!dir);
		b_c(0);
		c_d(0);
		d_c(0);
		cb_a(1);
		a_e(0);
		e_a(0);
		
		set(!dir);
		break;
	}
	a_e(1);
	e_a(1);
	a_start(1);

	StopMotion(9);
	PORTB=0x0F;
	_delay_ms(2000);
	PORTB=0;
}
void M2()
{
	
}
void M3()
{
	
}
void M4()
{
	
}
void M5()
{
	
}
void M6()
{
	
}
void M7()
{
	
}
void M8()
{
	
}
void M9()
{

}

int main(void){
	unsigned char M_cnt=1;
    Interface_init();

	while(1)
	{
		if(SW1)
		{
			M_cnt++;
			PORTB=1;
			while(SW1);
			PORTB=0;
		}
		if(SW2)
		{
			M_cnt--;
			PORTB=2;
			while(SW2);
			PORTB=0;
		}
		if(SW3)
		{
			cls();
			WriteCommand(0,DFH);
			WriteCommand(1,DFH);
			WriteCommand(2,DFH);
			ping(0);

			if(!M_cnt)
			{
				M1(); M2(); M3(); M4(); M5(); M6(); M7(); M8(); M9();
				StopMotion(9);
				
				PORTB|=0x0F;
				_delay_ms(2000);
				PORTB&=~0x0F;
			}
			else if(M_cnt==1) M1();
			else if(M_cnt==2) M2();
			else if(M_cnt==3) M3();
			else if(M_cnt==4) M4();
			else if(M_cnt==5) M5();
			else if(M_cnt==6) M6();
			else if(M_cnt==7) M7();
			else if(M_cnt==8) M8();
			else if(M_cnt==9) M9();
			StopMotion(9);
		}
		if(SW4)
		{
			cls();
			WriteCommand(0,DFH);
			WriteCommand(1,DFH);
			WriteCommand(2,DFH);
			ping(0);

			Buzz(3);
			PORTB=1;
			while(SW4)
			{
				if(SW1) cls(), PORTB=1;
				if(SW2) cls(), PORTB=2;
				if(SW3) cls(), PORTB=4;

				if(PORTB&1)
				{	
					ping(0);
					sprintf(str,"    X = (%5d)",(int)px[0]); lcd_display_str(1,0,str);
					sprintf(str,"    Y = (%5d)",(int)py[0]); lcd_display_str(2,0,str);
					sprintf(str,"    W = (%5d)",(int)pw[0]); lcd_display_str(3,0,str);
				}
				if(PORTB&2)
				{
					dc(0,9,psd_value[0]);
					dc(0,4,psd_value[1]);dc(0,14,psd_value[8]);
					dc(1,0,psd_value[2]);dc(1,17,psd_value[7]);
					dc(2,0,psd_value[3]);dc(2,17,psd_value[6]);
					dc(3,4,psd_value[4]);dc(3,14,psd_value[5]);
				}
				if(PORTB&4)
				{
					V1(11,0,0,0,0,180,0,240,0,0,0);
					
					for(int i=0;i<5;i++){
						dc(0,i*4,C_D[i][0]);
						dc(1,i*4,C_D[i][1]);
						dc(2,i*4,C_D[i][2]);
					}

					dc(3,17,C_N);
				}
			}
			PORTB=0;
			cls();
		}
		dc(3,17,M_cnt);
	}
}


