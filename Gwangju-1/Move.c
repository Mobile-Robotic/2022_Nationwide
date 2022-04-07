/************************************************************************************************************/
/*                                                                                                          */
/*                                                Move.c                                                    */
/*                                                                                                          */
/*                                                                                       2020. 1. 1.        */
/************************************************************************************************************/
#include "Interface.h"

volatile int t_ga, t_gw, f_cnt, f_max;
volatile float speed, wspeed, acc, accw, Ga, Gw;
float STOP, WSTOP, ve_x[3], ve_y[3], ve_l[3], ve_d[3], p_err[4], delta[4], px[4], py[4], pw[4];
long A_DIS[3], A_RET[3], A_DEG[3][3], PSD[10];
int C_D[9][5], C_N, FLine, Getting;
char str[21];

char b_cnt, b_flg, b_val;

void Buzz(int cnt)
{
	for(int i=0; i<cnt; i++)
	{
		PORTB|=8;
		_delay_ms(30);
		PORTB&=~8;
		_delay_ms(20);
	}
}

float Trans(int Min, int Max, int min, int max, float value)//범위 변환 함수 Min~Max를 min~max로 변환(2차원 그래프)
{
	float rv;
	
	if(value<Min)value = Min;
	if(value>Max)value = Max;

	rv = (value - Min) / (Max - Min);
	rv = rv * (max - min);

	return rv+min;
}

float PD(int ch, float err, float kp, float kd)//비례 미분 제어기
{
	float P, D;

	P = kp * err;
	D = kd *((err - p_err[ch]) - delta[ch]);
	
	delta[ch]=err - p_err[ch];
	p_err[ch]=err;	

	return P+D;
}

long READ_EN(int num)//READ_Encoder
{		
	long EN=0;
	
	WriteCommand(num,0x0A); //(0x0A == RDRP)
	
	EN=(EN | ReadData(num))<<8;
	EN=(EN | ReadData(num))<<8;
	EN=(EN | ReadData(num))<<8;
	EN=(EN | ReadData(num));

	return EN;
}

int Mode(float len, int sp, int gg, int mode, int num, int value, int mm)
{
	//이동 거리 : ve_l[1]
	//남은 거리 : len
	int PSD_N=PD(3,psd_value[num],0,1);

	if(gg%10)
	{			
		if(mode==1) speed=Trans(PSD[9],PSD[9]+STOP,50,sp,PSD[num]);
		else		speed=Trans(len-STOP,len,sp,50,ve_l[1]);				
	}
	else speed=sp;

	if(ve_l[1]>=mm)
	{
		if(b_flg && (S&b_val)){
			b_flg=0;
			b_cnt++;
		}
		else if(!b_flg && !(S&b_val)) b_flg=1;

		if(S&1)FLine=1;
		if(S&8)FLine=2;
		if(S&2)FLine=3;

		if(mode==0 && ve_l[1]>=len) return 1;			//거리 조건
		if(mode==1 && psd_value[num]>=value) return 1;	//PSD_Y
		if(mode==2 && fabs(PSD_N)>=value) return 1;		//PSD_N, 센서의 순간변화량이 value보다 커지면 탈출(기존에 있던 조건은 값이 엄청튐)(value 30이 적당)
		if(mode==3 && (S & num)) return 1;				//PHT, 동시탈출 가능함
		if(mode==4 && (S & 0x7C)==num) return 1;		//PHT, 반드시 num이 모두 인식해야 탈출함(Line센서 탈때 필요)
	}
	return 0;
}

void HD(float x, float y, float w, float filter)//(Holonomic_Drive)
{	
	static float rx, ry;

	if(filter)
	{
		if(f_cnt)								//0.01 interrupt
		{
			f_cnt=0;		
						
				 if(x>rx+10)rx+=10;			
			else if(x<rx-10)rx-=10;
			else 			rx=x;
			
				 if(y>ry+10)ry+=10;
			else if(y<ry-10)ry-=10;
			else 			ry=y;
		}
	}
	else										//필터를 사용안할시에는 기억하고 있는 데이터를 현재 인자로 들어온 데이터로 세팅
	{
		rx=x;	
		ry=y; 
	}
	NH(rx,ry,w);
}

void VT(float f_agl, float len, int ch)//(Vector), 각도에 따라서 x, y 값 분해
{
	ve_x[ch] = len * cos(f_agl*0.017453); // 0.017453 == (M_PI/180)
	ve_y[ch] = len * sin(f_agl*0.017453); // 0.017453 == (M_PI/180)
}

void len_VT(float x, float y, int ch)//(len_vector), x, y를 알면 벡터의 길이와 각도 출력
{
	ve_l[ch] = hypot(x, y);				//남은 거리
	ve_d[ch] = atan2(y, x) * 180 / M_PI;//남은 각도
}

void DI(int value)//psd_value[x]의 데이터를 mm단위로 변환해서 PSD[x]변수에 저장하는 함수
{
	for(int i=0;i<10;i++)
	{
		if(i==9)PSD[9]=value;	
		else PSD[i]=psd_value[i];	

		if(PSD[i]<70) 		PSD[i]=Trans(60,70,400,300,PSD[i]);
		else if(PSD[i]<105) PSD[i]=Trans(70,105,300,200,PSD[i]);
		else if(PSD[i]<135) PSD[i]=Trans(105,135,200,150,PSD[i]);
		else if(PSD[i]<185) PSD[i]=Trans(135,185,150,100,PSD[i]);
		else 				PSD[i]=Trans(185,254,100,60,PSD[i]);
	}
}

//일의자리가 감속여부, 십의자리가 가속여부
//GG : 10 가속만
//	   01 감속만
//	   11 가감속
void reset(int sp, int wsp, int gg, int filter)
{
	acc=Trans(50,700,35,20,sp);		//속도에 따라 가속 세기 설정
	accw=Trans(10,300,40,20,wsp);	//속도에 따라 회전 가속 세기 설정

	STOP=Trans(50,700,0,160,sp);	//속도에 따라 감속 거리 설정
	WSTOP=Trans(10,300,0,65,wsp);	//속도에 따라 회전감속 각도 설정

	if(sp==999)						//속도가999이면 랜서컬렉터 퍽찌르는 모드
	{
		acc=13;
		STOP=150;

		//accw=30;
		//WSTOP=Trans(10,300,0,65,300);
	}

	f_cnt=0;
	f_max=filter;					//필터 시간 대입 

	t_ga=0;
	t_gw=0;
	Ga=0;
	Gw=0;

	if(gg/10==0)Ga=1,t_ga=5000;		//가속이 없을 때 가속 변수 1로세팅 가속에 사용하는 타이머 변수도 5000으로 셋
	if(gg/100)  Gw=1,t_gw=5000;		//백의자리수가 1이면 회전가속 안함

	for(int i=0;i<4;i++)
	{
		if(i<3)
		A_DIS[i]=0;					//엔코더 초기화(3개 채널)
		p_err[i]=0;					//	  PD 초기화(4개 채널)
		delta[i]=0;
	}
}

//set : 0 연산
//	  : 1이상 값의 거리변수값 초기화
void ping(int set)//(position)0일 경우연산, 1~3의 경우 리셋, 좌표계를 최대3개까지 설정가능(현재 좌표계3은 TD에서 항상 사용중)
{
	int i, j;
	double x, y, w[3];	

	static float P_PING[3];	//과거 엔코더 값(past_ping) 	
	static float N_PING[3];	//현재 엔코더 값( now_ping)

	if(set)					//해당되는 좌표계를 셋
	{
		px[set]=0;
		py[set]=0;
		pw[set]=0;
		A_DEG[0][set-1]=0;	
		A_DEG[1][set-1]=0;
		A_DEG[2][set-1]=0;
	}
	else
	{	
		for(i=0;i<3;i++)		
		{
			N_PING[i]=READ_EN(i);									//모션칩에서 엔코더값을 읽어옴 엔코더값을 현재값에저장
							A_DIS[i]   +=(N_PING[i]-P_PING[i]); 	//현재엔코더값에서 과거엔코더값을 빼서 프로그램 1회 진행동안 쌓인 엔코더값을 구함 (누적연산)
							A_RET[i]	=(N_PING[i]-P_PING[i]);		//현재엔코더값에서 과거엔코더값을 빼서 프로그램 1회 진행동안 쌓인 엔코더값을 구함 (한사이클 동한 변한값 연산)
			for(j=0;j<3;j++)A_DEG[i][j]+=(N_PING[i]-P_PING[i]);		//위와 동일, 하지만 좌표계가 3개라서 3개까지 연산
		}

		px[0]= (A_DIS[0]-A_DIS[2])/2.0/165.4;						//누적 엔코더값으로 x거리 연산
		py[0]=((A_DIS[0]+A_DIS[2])-(A_DIS[1])*2.0)/3.0/191.0;		//누적 엔코더값으로 y거리 연산
		pw[0]= (A_DIS[0]+A_DIS[1]+A_DIS[2])/3.0/416.0;				//누적 엔코더값으로  각도 연산

		x= (A_RET[0]-A_RET[2])/2.0/165.4;							//프로그램 1회 진행동안 쌓인 엔코더값으로 x거리 연산
		y=((A_RET[0]+A_RET[2])-(A_RET[1])*2.0)/3.0/191.0;			//프로그램 1회 진행동안 쌓인 엔코더값으로 y거리 연산

		for(i=0;i<3;i++)											//좌표계가 1~3(X, Y, W 분해)
		{
			w[i]=(A_DEG[0][i]+A_DEG[1][i]+A_DEG[2][i])/3.0/416.0;	//누적 엔코더값으로 각도 연산
			VT(w[i],x,0);											//1회 프로그램 진행동안 구한 x거리를 현재각도에 따라 x,y로 분리
			VT(w[i],y,1);											//1회 프로그램 진행동안 구한 y거리를 현재각도에 따라 x,y로 분리
			
			px[i+1]+=ve_x[0]-ve_y[1];								//분리된 x,y값을 절대좌표변수에 누적 연산
			py[i+1]+=ve_x[1]+ve_y[0];	
			pw[i+1]=w[i];

			P_PING[i]=N_PING[i];									//현재값을 과거값 변수에 저장
		}
	}
}
void LM(int ch, int sp, int wsp, int gg, int x, int y, int w, int filter)//(Location_move), 좌표이동
{
	double ox, oy, ow;	
	int b1=0, b2=0;

	if(Getting)wsp=50;									//길찾기 알고리즘 퍽 먹었을때 속도 늦추기
	if(!wsp)b2=1;
	reset(sp,wsp,gg,-filter);							//기본 설정
	
	if(ch==4) ping(3), ch=3; 							//수동 TD 편하게 사용
	while(1)
	{
		if(S&0x01)FLine=1;
		if(S&0x08)FLine=2;

		ping(0);										//좌표 연산
		ox=x-px[ch];									//목적좌표와 현재좌표의 남은 X거리
		oy=y-py[ch];									//목적좌표와 현재좌표의 남은 Y거리
		ow=w-pw[ch];									//목적좌표와 현재좌표의 남은 각도, 속도 직접대입이라서 5배증폭

		len_VT(ox,oy,0);								//남은x,y거리로 목적좌표까지 남은 거리와 각도 구함

		if(gg%10)speed=Trans(0,STOP+10,50,sp,ve_l[0]);	//감속이 있을경우에는 (LM은 10m전에 멈춤으로 감속에+10)
		else	 speed=sp;								//감속이 없을경우에는 속도 바로대입

		if(wsp)wspeed=Trans(-wsp,wsp,-wsp,wsp,ow*4);	//회전속도 제한(최대, 최소 스피드 제한), 남아있는각도(수동)
		else   wspeed=(speed*ow/ve_l[0]);				//현재 속도에따라 목적좌표에 동시에 도달하기위한 회전 속도를 구함(자동)

		if(ve_l[0]<10 || b1)b1=1, speed=0;				//목적 x,y좌표에 도달 하면 b1=1로 셋팅, 속도를 0으로 셋팅
		if(fabs(ow)<1)b2=1;								//목적 각도에 도달하면 b2를 1로 셋팅
		if(b1 && b2)break;								//목적좌표에 도달하면 종료
		if(gg%10==0 && ve_l[0]<150)break;				//감속이 없을 경우에는 목적좌표에서 150mm이내로 접근하면 정지(LM을 이용한 연결동작 스무스하기 위해서)

		VT(ve_d[0]-pw[ch],speed*Ga,0);					//감속을 거치면서 구해진 속도와 이동하고자하는 각도를 x,y로 변환
		HD(ve_x[0],ve_y[0],wspeed*Gw,f_max);
	}
}

void Hm(int f_agl, int sp, int gg, int x, int y, int mode, int num, int value, int mm, int filter)//(Holonomic_move)
{
	reset(sp,0,gg,-filter);
	len_VT(x,y,0);				//총 이동해야할 거리 계산

	while(1)
	{
		ping(0);
		DI(value);
			
		len_VT(px[0],py[0],1);	//현재 이동한 거리 계산
				
		if(Mode(ve_l[0],sp,gg,mode,num,value,mm))break;

		VT(ve_d[0]+f_agl,speed*Ga,0);
		HD(ve_x[0],ve_y[0],pw[0]*-4,f_max);
	}
}

void HM(int f_agl, int sp, int gg, int x, int y, int filter)//좌표이동만 가능한 Hm
{
	Hm(f_agl,sp,gg,x,y,0,0,0,0,filter);
}

void TD(int f_agl, int sp, int gg, int x, int y, int w, int filter)//(Turn_and_Drive)
{
	int flg=0;										//처음에 반드시 0처리
	double Auto_w, end_w=0;

	if(filter==0)	reset(sp,0,10+(gg%10),-filter);	//항상 가속 ON, W가감속을 위해서 일단은 항상 가속을 켜둠
	else			reset(sp,0,gg		 ,-filter);	//필터사용시 W가감속을 사용하면 W각도 손실이 너무 크므로 filter시에는 사용하지 않음

	len_VT(x,y,0);

	ping(3);
	while(1)
	{
		ping(0);

		len_VT(px[3],py[3],1);

		if(Mode(ve_l[0],sp,gg,0,0,0,0))break;
		
		Auto_w=(speed*w/ve_l[0]);				//XY거리에 비례해서 자동 W속도 추출

		if(gg/10==0 && gg%10==1 && filter==0)	//가속X, 감속0, 필터X
		{
			if(flg==0)
			{
				if(Auto_w * Ga == Auto_w)	
				{
					flg=1;						//계속 조건에 들어오지 못하도록 flg처리
					end_w=pw[3];				//W가속하는 동안 손실된 W값 추출
					Ga=1,t_ga=5000;				//W가감속을 위해서 킨 가속끄기
				}
			}
		}

		if(flg)	wspeed=speed*(w+end_w*acc*0.1)/ve_l[0];
		else 	wspeed=Auto_w;

		if(gg/10==0)VT((ve_d[0]-pw[3])+f_agl,speed   ,0);				//reset에서 항상 가속이 켜져 있을 수 있으니 가감속 조건 if문으로 분기
		else		VT((ve_d[0]-pw[3])+f_agl,speed*Ga,0);
		HD(ve_x[0],ve_y[0],wspeed*Ga,f_max);
	}
}

void Pm(int f_psd, int s_dir, int dir, int ta, int sp, int gg, int mode, int num, int value, int mm, int filter)//(Psd_move)
{
	int psd[2], ta1=ta-30;
	double errX=0, errY=0, errW=0;		//반드시 double(실수형 으로) 선언
	double d[3]={3, 3, 1};				//반드시 double(실수형 으로) 선언
	double diff[3]=						//반드시 double(실수형 으로) 선언
	{
		130.0/200,	//0번 방향 센서차
		110.0/230,	//1번 방향 센서차
		210.0/230.	//2번 방향 센서차
	};

	reset(sp,0,gg,-filter);

	psd[0]=f_psd+1;
	if(psd[0]>8)psd[0]=0;
	psd[1]=f_psd-1;
	if(psd[1]<0)psd[1]=8;

	while(1)
	{	
		ping(0);		
		DI(value);		

		len_VT(px[0],py[0],1);				//현재 이동한 거리 계산

		if(Mode(value,sp,gg,mode%10,num,value,mm))break;

		//보정종류
		if(s_dir==0)//(센서1개로 벽타기) -> (Y, W보정)(MS(Mid_SET) 꼭 하고 하기)
		{	
			errX=0;
			errY=psd_value[psd[dir]]-ta;
			if(dir) errY=-errY;
			errW=errY;
		}		
		if(s_dir==1)//(센서2개로 벽타기)(보정모드로 설정했을때 f_psd가 진행 f_agl이 됨), 센서의 변화량이 다름으로 일정값을 곱해줘서 잡아주기
		{
			if(dir==2)
			{
				errX=(psd_value[4]-230)*d[0];
				errW = (psd_value[4] * diff[2] - psd_value[5]) * d[2];

				if(ta)
				{
					if(num>4)d[1] = -3;
					errY = (psd_value[num] - ta) * d[1];
				}
				if(mode!=5)errY=0;
			} 
			else
			{		
				d[2] = 0.8;
				if(dir==0)
				{
					errY=(psd_value[1]-130)*d[1];
					errW = (psd_value[1] - psd_value[3] * diff[0]) * d[2];
				}
				if(dir==1)
				{
					errY=(110-psd_value[8])*d[1];
					errW = (psd_value[6] * diff[1] - psd_value[8]) * d[2];
				}
				if(ta)errX = (psd_value[num] - ta) * d[0];
				if(mode!=5)errX=0;
			}
		}
		if(s_dir==2)//MS(Mid_SET)
		{	
			errX=0;
			if(dir==2)//센서 2개(Avoid)
			{
				errY=0;
				errW=pw[0]*-4;
				if(psd_value[psd[0]]>ta) errY+=psd_value[psd[0]]-ta;
				if(psd_value[psd[1]]>ta1)errY-=psd_value[psd[1]]-ta1;
				if(3<=f_psd && f_psd<=6) errY=-errY;
			}
			else //센서 1개
			{
				errW = 0;
				if(num>4) d[1] = -3;
				errY = (psd_value[num] - ta) * d[1];
			}			
		}
		if(mode==5)
		{
			speed=0;
			if(fabs(errW)>10) errX = errY = 0;
			errX = Trans(-100,100,-100,100,errX);
			errY = Trans(-100,100,-100,100,errY);
			if(fabs(errX/d[0])<5 && fabs(errY/d[1])<5 && fabs(errW/d[2])<5)break;
		}

		if(mode==5)			VT(0,0,0);
		else if(s_dir==1)	VT(f_psd,speed*Ga,0);
		else 				VT(360-f_psd*40,speed*Ga,0);

		HD(ve_x[0] + errX,ve_y[0] + errY,errW,f_max);
	}
}

void FCC(int s_dir, int dir, int num, int value)//보정
{
	Pm(0,s_dir,dir,value,0,11,5,num,0,0,0);//필터 써서 가감속 조절(현재는 사용X)
	StopMotion(10);
}

void RT(int f_agl, int dir, int sp, int gg, float rad, float w, int num, int mm, int filter)
{   
	int mode=(num ? 3:0);
    float Dis=(rad*M_PI*(w/180.0));	//로봇이 회전할때 회전하는 원주의 길이

	reset(sp,sp,gg,-filter);
	
	while(1)
	{
		ping(0);

		len_VT(px[0],py[0],1); 						

		if(rad)
		{
			wspeed=(speed*w/Dis);
			if(Mode(Dis,sp,gg,mode,num,0,mm))break;
			
			VT(f_agl,speed*Ga,0);
			HD(ve_x[0],ve_y[0],wspeed*dir*Ga,f_max);
		}
		else
		{
			wspeed=Trans(w-WSTOP,w,sp,10,fabs(pw[0]));
			if(gg%10==0)wspeed=sp;	

			ve_l[1]=fabs(pw[0]);
			if(Mode(w,sp,gg,mode,num,0,mm))break;
			
			HD(0,0,wspeed*dir*Gw,f_max);
		}	
	}
}

void turn(int dir, int sp, int gg, int w, int num, int mm)//제자리 회전
{
	if(Getting)sp=50;				//길찾기 알고리즘 퍽 먹었을때 속도 늦추기
	if(gg/10==0)gg=100+(gg%10);		//gg에 01넣으면 101로 변경하는 작업, 가속을 안하면 백의자리에 1을 붙혀줌
	RT(0,dir,sp,gg,0,w,num,mm,0);
}

//mode = 십의자리(StopMotion(10) -> (0 : 동작) (1 : 미동작))
//mode = 일의자리					(0 : 추적) (1 : 저장)
void V1(int mode, int first, int num, int color, int Xd, int Xu, int Yd, int Yu, int pointX, int pointY, int attack)
{
	int i, j, k, sort=0, cnt=0, buff;
	int x=0, y=0, w=0, errX=0, errY=0;
	int C_buff_N=0;

	if(mode/10==0)
	{
		StopMotion(10);
		reset(500,0,11,0); //reset속도값이 클수록 가속량이 줄어듬(현재는 추적속도 리미트 500이라서 저렇게 맞춰놈)
	}
	else reset(500,0,01,0);
	
	for(i=0;i<9;i++)
	{
		for(j=0;j<5;j++)
		{
			C_D[i][j]=0;
		}
	}

	while(1)
	{
		C_N=0;

		C_EN=1;
		V1_flg=0;
		V1_cnt=0;

		while(!V1_flg && V1_cnt<20);
		while( V1_flg && V1_cnt<20);

		C_EN=0;

		C_buff_N=C_buff[1]-'0';
		
		if(V1_cnt>=20)C_buff_N=0;
		if(!C_buff_N)break;

		for(i=0;i<C_buff_N;i++)
		{
			C_D[C_N][0]=C_buff[i*15+3]-'0';

			C_D[C_N][2]=(C_buff[i*15+5]-'0')*100;
			C_D[C_N][2]+=(C_buff[i*15+6]-'0')*10;
			C_D[C_N][2]+=(C_buff[i*15+7]-'0');

			C_D[C_N][1]=(C_buff[i*15+8]-'0')*100;
			C_D[C_N][1]+=(C_buff[i*15+9]-'0')*10;
			C_D[C_N][1]+=(C_buff[i*15+10]-'0');

			C_D[C_N][4]=(C_buff[i*15+11]-'0')*100;
			C_D[C_N][4]+=(C_buff[i*15+12]-'0')*10;
			C_D[C_N][4]+=(C_buff[i*15+13]-'0');

			C_D[C_N][3]=(C_buff[i*15+14]-'0')*100;
			C_D[C_N][3]+=(C_buff[i*15+15]-'0')*10;
			C_D[C_N][3]+=(C_buff[i*15+16]-'0');

			if(C_D[C_N][0]<4 && C_D[C_N][1]<Xu && C_D[C_N][1]>Xd && C_D[C_N][2]<Yu && C_D[C_N][2]>Yd)
			{
				++C_N;
			}
		}

		for(i=0;i<5;i++) C_D[C_N][i]=0;

		sort = first < 2 ? 2 : 1;

		for(i=0;i<C_N-1;i++)
		{
			for(j=0;j<C_N-1;j++)
			{
				if(first==0 || first==3)
				{
					if(C_D[j][sort] > C_D[j+1][sort])//내림차순
					{
						for(k=0;k<5;k++)
						{
							buff=C_D[j][k];
							C_D[j][k]=C_D[j+1][k];
							C_D[j+1][k]=buff;
						}
					}
				}
				if(first==1 || first==2)
				{
					if(C_D[j][sort] < C_D[j+1][sort])//오름차순
					{
						for(k=0;k<5;k++)
						{
							buff=C_D[j][k];
							C_D[j][k]=C_D[j+1][k];
							C_D[j+1][k]=buff;
						}
					}
				}
			}
		}
		if(!C_N || mode%10) break;

		for(i=cnt=0;i<C_N;i++){
			if(!color ||(color && color==C_D[i][0]))
			{
				if(num==cnt)
				{
					errX=pointX-C_D[i][1];
					errY=C_D[i][2]-pointY;
					break;
				}
				cnt++;
			}
		}
		ping(0);
		if(attack==1)
		{
			x=PD(0,errX,11,1);
			y=PD(1,errY,5,1);
			w=pw[0]*-4;
		}
		if(attack==2)
		{					
			errX=0;
			w=PD(0,errY,0.7,0.5);
		}
		if(attack==3)
		{	
			x=PD(0,errX,11,1);
			w=PD(1,errY,0.5,0.2);
		}
		x=Trans(-500,500,-500,500,x);
		y=Trans(-500,500,-500,500,y);
		HD(x*Ga,y*Ga,w,f_max);
		if(fabs(errX)<3 && fabs(errY)<3)break;
	}
}

void Line(int f_agl, int sp, int gg, int mode, int num, int value, int mm, int filter)
{
	int y=0;
	
	reset(sp,0,gg,-filter);

	while(1)
	{
		ping(0);		
		DI(value);

		len_VT(px[0],py[0],1);

		// 5개 LINE
		if((S&0x7C)==0x20)y=-45; // 최좌
		else if((S&0x7C)==0x24)y=-30; // 좌 최좌
		else if((S&0x7C)==0x0C)y=-15; // 가 좌
		else if((S&0x7C)==0x08)y=  0; // 가
		else if((S&0x7C)==0x18)y= 15; // 가 우
		else if((S&0x7C)==0x50)y= 30; // 우 최우
		else if((S&0x7C)==0x40)y= 45; // 최우
		else y=0;

        // 3개 LINE
        /*
		if((S&0x7C)==0x04)y=-30; // 좌
		else if((S&0x7C)==0x0C)y=-15; // 가 좌
		else if((S&0x7C)==0x08)y=  0; // 가
		else if((S&0x7C)==0x18)y= 15; // 가 우
		else if((S&0x7C)==0x10)y= 30; // 우
		else y=0;
		*/
		
		if(Mode(value,sp,gg,mode,num,value,mm))break;	

		if(!f_agl)HD( speed*Ga,y/3, y,f_max);
		else	  HD(-speed*Ga,y*3,-y/2,f_max);
	}
}

ISR(TIMER1_OVF_vect){
	TCNT1H=0xFF; TCNT1L=0x70; //0.01초
	V1_cnt++;

	f_cnt++;
	f_max++;
	if(f_max>0)f_max=0;			//초기 reset 함수에서 필터의 지속시간 -데이터로 설정 f_max 카운팅이후 0보다크면 필터OFF

	t_ga++;
	t_gw++;

	Ga=t_ga/(1000.0/acc);		//거리가속
	if(Ga>1)Ga=1;

	Gw=t_gw/(1000.0/accw);		//회전가속
	if(Gw>1)Gw=1;
}

void NH(float Fx, float Fy, float Fw){

	float V[3]={0,0,0};

	V[0]=( 0.056*Fx)+(0.033*Fy)+(0.141*Fw);
	V[1]=(-0.065*Fy)+(0.141*Fw);
	V[2]=(-0.056*Fx)+(0.033*Fy)+(0.141*Fw);

	SetVelocity(0, V[0]*65536);
	SetVelocity(1, V[1]*65536);
	SetVelocity(2, V[2]*65536);

	if(SW1) Hold();

	StartMotion();
}

void HolonomicW(float f_agl, float f_speed, float fw_speed){
	long Fx=0, Fy=0, Fw=0; //속도

	Fx = f_speed * cos(f_agl*0.017453);
	Fy = f_speed * sin(f_agl*0.017453);
	Fw=fw_speed;
	if(f_agl>=360||f_agl<0){
		Fx=0;
		Fy=0;
		Fw=f_speed;
		if(f_agl<0)Fw=-f_speed;
	}	

	NH(Fx,Fy,Fw);
}

