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

float Trans(int Min, int Max, int min, int max, float value)//���� ��ȯ �Լ� Min~Max�� min~max�� ��ȯ(2���� �׷���)
{
	float rv;
	
	if(value<Min)value = Min;
	if(value>Max)value = Max;

	rv = (value - Min) / (Max - Min);
	rv = rv * (max - min);

	return rv+min;
}

float PD(int ch, float err, float kp, float kd)//��� �̺� �����
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
	//�̵� �Ÿ� : ve_l[1]
	//���� �Ÿ� : len
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

		if(mode==0 && ve_l[1]>=len) return 1;			//�Ÿ� ����
		if(mode==1 && psd_value[num]>=value) return 1;	//PSD_Y
		if(mode==2 && fabs(PSD_N)>=value) return 1;		//PSD_N, ������ ������ȭ���� value���� Ŀ���� Ż��(������ �ִ� ������ ���� ��ûƦ)(value 30�� ����)
		if(mode==3 && (S & num)) return 1;				//PHT, ����Ż�� ������
		if(mode==4 && (S & 0x7C)==num) return 1;		//PHT, �ݵ�� num�� ��� �ν��ؾ� Ż����(Line���� Ż�� �ʿ�)
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
	else										//���͸� �����ҽÿ��� ����ϰ� �ִ� �����͸� ���� ���ڷ� ���� �����ͷ� ����
	{
		rx=x;	
		ry=y; 
	}
	NH(rx,ry,w);
}

void VT(float f_agl, float len, int ch)//(Vector), ������ ���� x, y �� ����
{
	ve_x[ch] = len * cos(f_agl*0.017453); // 0.017453 == (M_PI/180)
	ve_y[ch] = len * sin(f_agl*0.017453); // 0.017453 == (M_PI/180)
}

void len_VT(float x, float y, int ch)//(len_vector), x, y�� �˸� ������ ���̿� ���� ���
{
	ve_l[ch] = hypot(x, y);				//���� �Ÿ�
	ve_d[ch] = atan2(y, x) * 180 / M_PI;//���� ����
}

void DI(int value)//psd_value[x]�� �����͸� mm������ ��ȯ�ؼ� PSD[x]������ �����ϴ� �Լ�
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

//�����ڸ��� ���ӿ���, �����ڸ��� ���ӿ���
//GG : 10 ���Ӹ�
//	   01 ���Ӹ�
//	   11 ������
void reset(int sp, int wsp, int gg, int filter)
{
	acc=Trans(50,700,35,20,sp);		//�ӵ��� ���� ���� ���� ����
	accw=Trans(10,300,40,20,wsp);	//�ӵ��� ���� ȸ�� ���� ���� ����

	STOP=Trans(50,700,0,160,sp);	//�ӵ��� ���� ���� �Ÿ� ����
	WSTOP=Trans(10,300,0,65,wsp);	//�ӵ��� ���� ȸ������ ���� ����

	if(sp==999)						//�ӵ���999�̸� �����÷��� ����� ���
	{
		acc=13;
		STOP=150;

		//accw=30;
		//WSTOP=Trans(10,300,0,65,300);
	}

	f_cnt=0;
	f_max=filter;					//���� �ð� ���� 

	t_ga=0;
	t_gw=0;
	Ga=0;
	Gw=0;

	if(gg/10==0)Ga=1,t_ga=5000;		//������ ���� �� ���� ���� 1�μ��� ���ӿ� ����ϴ� Ÿ�̸� ������ 5000���� ��
	if(gg/100)  Gw=1,t_gw=5000;		//�����ڸ����� 1�̸� ȸ������ ����

	for(int i=0;i<4;i++)
	{
		if(i<3)
		A_DIS[i]=0;					//���ڴ� �ʱ�ȭ(3�� ä��)
		p_err[i]=0;					//	  PD �ʱ�ȭ(4�� ä��)
		delta[i]=0;
	}
}

//set : 0 ����
//	  : 1�̻� ���� �Ÿ������� �ʱ�ȭ
void ping(int set)//(position)0�� ��쿬��, 1~3�� ��� ����, ��ǥ�踦 �ִ�3������ ��������(���� ��ǥ��3�� TD���� �׻� �����)
{
	int i, j;
	double x, y, w[3];	

	static float P_PING[3];	//���� ���ڴ� ��(past_ping) 	
	static float N_PING[3];	//���� ���ڴ� ��( now_ping)

	if(set)					//�ش�Ǵ� ��ǥ�踦 ��
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
			N_PING[i]=READ_EN(i);									//���Ĩ���� ���ڴ����� �о�� ���ڴ����� ���簪������
							A_DIS[i]   +=(N_PING[i]-P_PING[i]); 	//���翣�ڴ������� ���ſ��ڴ����� ���� ���α׷� 1ȸ ���ൿ�� ���� ���ڴ����� ���� (��������)
							A_RET[i]	=(N_PING[i]-P_PING[i]);		//���翣�ڴ������� ���ſ��ڴ����� ���� ���α׷� 1ȸ ���ൿ�� ���� ���ڴ����� ���� (�ѻ���Ŭ ���� ���Ѱ� ����)
			for(j=0;j<3;j++)A_DEG[i][j]+=(N_PING[i]-P_PING[i]);		//���� ����, ������ ��ǥ�谡 3���� 3������ ����
		}

		px[0]= (A_DIS[0]-A_DIS[2])/2.0/165.4;						//���� ���ڴ������� x�Ÿ� ����
		py[0]=((A_DIS[0]+A_DIS[2])-(A_DIS[1])*2.0)/3.0/191.0;		//���� ���ڴ������� y�Ÿ� ����
		pw[0]= (A_DIS[0]+A_DIS[1]+A_DIS[2])/3.0/416.0;				//���� ���ڴ�������  ���� ����

		x= (A_RET[0]-A_RET[2])/2.0/165.4;							//���α׷� 1ȸ ���ൿ�� ���� ���ڴ������� x�Ÿ� ����
		y=((A_RET[0]+A_RET[2])-(A_RET[1])*2.0)/3.0/191.0;			//���α׷� 1ȸ ���ൿ�� ���� ���ڴ������� y�Ÿ� ����

		for(i=0;i<3;i++)											//��ǥ�谡 1~3(X, Y, W ����)
		{
			w[i]=(A_DEG[0][i]+A_DEG[1][i]+A_DEG[2][i])/3.0/416.0;	//���� ���ڴ������� ���� ����
			VT(w[i],x,0);											//1ȸ ���α׷� ���ൿ�� ���� x�Ÿ��� ���簢���� ���� x,y�� �и�
			VT(w[i],y,1);											//1ȸ ���α׷� ���ൿ�� ���� y�Ÿ��� ���簢���� ���� x,y�� �и�
			
			px[i+1]+=ve_x[0]-ve_y[1];								//�и��� x,y���� ������ǥ������ ���� ����
			py[i+1]+=ve_x[1]+ve_y[0];	
			pw[i+1]=w[i];

			P_PING[i]=N_PING[i];									//���簪�� ���Ű� ������ ����
		}
	}
}
void LM(int ch, int sp, int wsp, int gg, int x, int y, int w, int filter)//(Location_move), ��ǥ�̵�
{
	double ox, oy, ow;	
	int b1=0, b2=0;

	if(Getting)wsp=50;									//��ã�� �˰��� �� �Ծ����� �ӵ� ���߱�
	if(!wsp)b2=1;
	reset(sp,wsp,gg,-filter);							//�⺻ ����
	
	if(ch==4) ping(3), ch=3; 							//���� TD ���ϰ� ���
	while(1)
	{
		if(S&0x01)FLine=1;
		if(S&0x08)FLine=2;

		ping(0);										//��ǥ ����
		ox=x-px[ch];									//������ǥ�� ������ǥ�� ���� X�Ÿ�
		oy=y-py[ch];									//������ǥ�� ������ǥ�� ���� Y�Ÿ�
		ow=w-pw[ch];									//������ǥ�� ������ǥ�� ���� ����, �ӵ� ���������̶� 5������

		len_VT(ox,oy,0);								//����x,y�Ÿ��� ������ǥ���� ���� �Ÿ��� ���� ����

		if(gg%10)speed=Trans(0,STOP+10,50,sp,ve_l[0]);	//������ ������쿡�� (LM�� 10m���� �������� ���ӿ�+10)
		else	 speed=sp;								//������ ������쿡�� �ӵ� �ٷδ���

		if(wsp)wspeed=Trans(-wsp,wsp,-wsp,wsp,ow*4);	//ȸ���ӵ� ����(�ִ�, �ּ� ���ǵ� ����), �����ִ°���(����)
		else   wspeed=(speed*ow/ve_l[0]);				//���� �ӵ������� ������ǥ�� ���ÿ� �����ϱ����� ȸ�� �ӵ��� ����(�ڵ�)

		if(ve_l[0]<10 || b1)b1=1, speed=0;				//���� x,y��ǥ�� ���� �ϸ� b1=1�� ����, �ӵ��� 0���� ����
		if(fabs(ow)<1)b2=1;								//���� ������ �����ϸ� b2�� 1�� ����
		if(b1 && b2)break;								//������ǥ�� �����ϸ� ����
		if(gg%10==0 && ve_l[0]<150)break;				//������ ���� ��쿡�� ������ǥ���� 150mm�̳��� �����ϸ� ����(LM�� �̿��� ���ᵿ�� �������ϱ� ���ؼ�)

		VT(ve_d[0]-pw[ch],speed*Ga,0);					//������ ��ġ�鼭 ������ �ӵ��� �̵��ϰ����ϴ� ������ x,y�� ��ȯ
		HD(ve_x[0],ve_y[0],wspeed*Gw,f_max);
	}
}

void Hm(int f_agl, int sp, int gg, int x, int y, int mode, int num, int value, int mm, int filter)//(Holonomic_move)
{
	reset(sp,0,gg,-filter);
	len_VT(x,y,0);				//�� �̵��ؾ��� �Ÿ� ���

	while(1)
	{
		ping(0);
		DI(value);
			
		len_VT(px[0],py[0],1);	//���� �̵��� �Ÿ� ���
				
		if(Mode(ve_l[0],sp,gg,mode,num,value,mm))break;

		VT(ve_d[0]+f_agl,speed*Ga,0);
		HD(ve_x[0],ve_y[0],pw[0]*-4,f_max);
	}
}

void HM(int f_agl, int sp, int gg, int x, int y, int filter)//��ǥ�̵��� ������ Hm
{
	Hm(f_agl,sp,gg,x,y,0,0,0,0,filter);
}

void TD(int f_agl, int sp, int gg, int x, int y, int w, int filter)//(Turn_and_Drive)
{
	int flg=0;										//ó���� �ݵ�� 0ó��
	double Auto_w, end_w=0;

	if(filter==0)	reset(sp,0,10+(gg%10),-filter);	//�׻� ���� ON, W�������� ���ؼ� �ϴ��� �׻� ������ �ѵ�
	else			reset(sp,0,gg		 ,-filter);	//���ͻ��� W�������� ����ϸ� W���� �ս��� �ʹ� ũ�Ƿ� filter�ÿ��� ������� ����

	len_VT(x,y,0);

	ping(3);
	while(1)
	{
		ping(0);

		len_VT(px[3],py[3],1);

		if(Mode(ve_l[0],sp,gg,0,0,0,0))break;
		
		Auto_w=(speed*w/ve_l[0]);				//XY�Ÿ��� ����ؼ� �ڵ� W�ӵ� ����

		if(gg/10==0 && gg%10==1 && filter==0)	//����X, ����0, ����X
		{
			if(flg==0)
			{
				if(Auto_w * Ga == Auto_w)	
				{
					flg=1;						//��� ���ǿ� ������ ���ϵ��� flgó��
					end_w=pw[3];				//W�����ϴ� ���� �սǵ� W�� ����
					Ga=1,t_ga=5000;				//W�������� ���ؼ� Ų ���Ӳ���
				}
			}
		}

		if(flg)	wspeed=speed*(w+end_w*acc*0.1)/ve_l[0];
		else 	wspeed=Auto_w;

		if(gg/10==0)VT((ve_d[0]-pw[3])+f_agl,speed   ,0);				//reset���� �׻� ������ ���� ���� �� ������ ������ ���� if������ �б�
		else		VT((ve_d[0]-pw[3])+f_agl,speed*Ga,0);
		HD(ve_x[0],ve_y[0],wspeed*Ga,f_max);
	}
}

void Pm(int f_psd, int s_dir, int dir, int ta, int sp, int gg, int mode, int num, int value, int mm, int filter)//(Psd_move)
{
	int psd[2], ta1=ta-30;
	double errX=0, errY=0, errW=0;		//�ݵ�� double(�Ǽ��� ����) ����
	double d[3]={3, 3, 1};				//�ݵ�� double(�Ǽ��� ����) ����
	double diff[3]=						//�ݵ�� double(�Ǽ��� ����) ����
	{
		130.0/200,	//0�� ���� ������
		110.0/230,	//1�� ���� ������
		210.0/230.	//2�� ���� ������
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

		len_VT(px[0],py[0],1);				//���� �̵��� �Ÿ� ���

		if(Mode(value,sp,gg,mode%10,num,value,mm))break;

		//��������
		if(s_dir==0)//(����1���� ��Ÿ��) -> (Y, W����)(MS(Mid_SET) �� �ϰ� �ϱ�)
		{	
			errX=0;
			errY=psd_value[psd[dir]]-ta;
			if(dir) errY=-errY;
			errW=errY;
		}		
		if(s_dir==1)//(����2���� ��Ÿ��)(�������� ���������� f_psd�� ���� f_agl�� ��), ������ ��ȭ���� �ٸ����� �������� �����༭ ����ֱ�
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
			if(dir==2)//���� 2��(Avoid)
			{
				errY=0;
				errW=pw[0]*-4;
				if(psd_value[psd[0]]>ta) errY+=psd_value[psd[0]]-ta;
				if(psd_value[psd[1]]>ta1)errY-=psd_value[psd[1]]-ta1;
				if(3<=f_psd && f_psd<=6) errY=-errY;
			}
			else //���� 1��
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

void FCC(int s_dir, int dir, int num, int value)//����
{
	Pm(0,s_dir,dir,value,0,11,5,num,0,0,0);//���� �Ἥ ������ ����(����� ���X)
	StopMotion(10);
}

void RT(int f_agl, int dir, int sp, int gg, float rad, float w, int num, int mm, int filter)
{   
	int mode=(num ? 3:0);
    float Dis=(rad*M_PI*(w/180.0));	//�κ��� ȸ���Ҷ� ȸ���ϴ� ������ ����

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

void turn(int dir, int sp, int gg, int w, int num, int mm)//���ڸ� ȸ��
{
	if(Getting)sp=50;				//��ã�� �˰��� �� �Ծ����� �ӵ� ���߱�
	if(gg/10==0)gg=100+(gg%10);		//gg�� 01������ 101�� �����ϴ� �۾�, ������ ���ϸ� �����ڸ��� 1�� ������
	RT(0,dir,sp,gg,0,w,num,mm,0);
}

//mode = �����ڸ�(StopMotion(10) -> (0 : ����) (1 : �̵���))
//mode = �����ڸ�					(0 : ����) (1 : ����)
void V1(int mode, int first, int num, int color, int Xd, int Xu, int Yd, int Yu, int pointX, int pointY, int attack)
{
	int i, j, k, sort=0, cnt=0, buff;
	int x=0, y=0, w=0, errX=0, errY=0;
	int C_buff_N=0;

	if(mode/10==0)
	{
		StopMotion(10);
		reset(500,0,11,0); //reset�ӵ����� Ŭ���� ���ӷ��� �پ��(����� �����ӵ� ����Ʈ 500�̶� ������ �����)
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
					if(C_D[j][sort] > C_D[j+1][sort])//��������
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
					if(C_D[j][sort] < C_D[j+1][sort])//��������
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

		// 5�� LINE
		if((S&0x7C)==0x20)y=-45; // ����
		else if((S&0x7C)==0x24)y=-30; // �� ����
		else if((S&0x7C)==0x0C)y=-15; // �� ��
		else if((S&0x7C)==0x08)y=  0; // ��
		else if((S&0x7C)==0x18)y= 15; // �� ��
		else if((S&0x7C)==0x50)y= 30; // �� �ֿ�
		else if((S&0x7C)==0x40)y= 45; // �ֿ�
		else y=0;

        // 3�� LINE
        /*
		if((S&0x7C)==0x04)y=-30; // ��
		else if((S&0x7C)==0x0C)y=-15; // �� ��
		else if((S&0x7C)==0x08)y=  0; // ��
		else if((S&0x7C)==0x18)y= 15; // �� ��
		else if((S&0x7C)==0x10)y= 30; // ��
		else y=0;
		*/
		
		if(Mode(value,sp,gg,mode,num,value,mm))break;	

		if(!f_agl)HD( speed*Ga,y/3, y,f_max);
		else	  HD(-speed*Ga,y*3,-y/2,f_max);
	}
}

ISR(TIMER1_OVF_vect){
	TCNT1H=0xFF; TCNT1L=0x70; //0.01��
	V1_cnt++;

	f_cnt++;
	f_max++;
	if(f_max>0)f_max=0;			//�ʱ� reset �Լ����� ������ ���ӽð� -�����ͷ� ���� f_max ī�������� 0����ũ�� ����OFF

	t_ga++;
	t_gw++;

	Ga=t_ga/(1000.0/acc);		//�Ÿ�����
	if(Ga>1)Ga=1;

	Gw=t_gw/(1000.0/accw);		//ȸ������
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
	long Fx=0, Fy=0, Fw=0; //�ӵ�

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

