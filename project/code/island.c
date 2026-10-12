#include "zf_common_headfile.h"
#include "Island.h"

//数字图像数组
const uint8 Image_Flags[][9][8]={
        {                                //0
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //1
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //2
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0X00,0X00,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //3
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0XFF,0XFF,0X00},
        {0X00,0X00,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //4
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0X00,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0X00,0XFF,0X00,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0XFF,0X00,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0XFF,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0X00,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0XFF,0X00,0X00,0X00},
},
{                                        //5
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //6
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //7
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0XFF,0X00,0X00,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //8
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
{                                        //9
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0XFF,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0X00},
        {0X00,0X00,0X00,0X00,0X00,0XFF,0X00,0X00},
        {0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00},
        {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00},
},
};

//常用基本变量
extern uint8 image_two_value[MT9V03X_H][MT9V03X_W];//二值化后的原数组
extern volatile int Left_Line[MT9V03X_H]; //左边线数组
extern volatile int Right_Line[MT9V03X_H];//右边线数组
extern volatile int Search_Stop_Line;     //搜索截止行
extern volatile int Boundry_Start_Left;   //左右边界起始点
extern volatile int Boundry_Start_Right;  //第一个非丢线点,常规边界起始点
extern volatile int Left_Lost_Time;       //边界丢线数
extern volatile int Right_Lost_Time; 
extern volatile int Both_Lost_Time;//两边同时丢线数
extern volatile float Err;//摄像头误差

//十字变量
extern volatile int Cross_Flag; 

//环岛变量
volatile int Island_State=0;     //环岛状态标志
volatile int Left_Island_Flag=0; //左右环岛标志
volatile int Right_Island_Flag=0;//左右环岛标志
volatile int Left_Up_Guai[2];    //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
volatile int Right_Up_Guai[2];   //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列

extern volatile int Mid_Line[MT9V03X_H];
volatile int monotonicity_change_line[2];//单调性改变点坐标，[0]寸某行，[1]寸某列


volatile int monotonicity_change_left_flag=0;//不转折是0
volatile int monotonicity_change_right_flag=0;//不转折是0
volatile int continuity_change_right_flag=0; //连续是0
volatile int continuity_change_left_flag=0;  //连续是0

volatile int left_down_guai[2]={0};//四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
volatile int right_down_guai[2]={0};//四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列

volatile float kk;

//斑马线
extern volatile int Zebra_State;

//坡道标志位
extern volatile int Ramp_Flag;//坡道标志

extern volatile float FJ_Angle;


/*-------------------------------------------------------------------------------------------------------------------
  @brief     环岛检测
  @param     null
  @return    null
  Sample     Island_Detect(void);
  @note      利用四个拐点判别函数，单调性改变函，连续性数撕裂点，分为8步
-------------------------------------------------------------------------------------------------------------------*/
void Island_Detect()
{ 
    static float k=0;//3和5状态的k
    static int island_state_5_down[2]={0};//状态5时即将离开环岛，左右边界边最低点，[0]存y，第某行，{1}存x，第某列
    static int island_state_3_up[2]={0};//状态3时即将进入环岛用，左右上面角点[0]存y，第某行，{1}存x，第某列
  
	int i;
	int j;
	
		
  continuity_change_left_flag=Continuity_Change_Left(MT9V03X_H-1-5,10);  //连续性
  continuity_change_right_flag=Continuity_Change_Right(MT9V03X_H-1-5,10);
  monotonicity_change_right_flag=Monotonicity_Change_Right(MT9V03X_H-1-10,10); //单调性
  monotonicity_change_left_flag=Monotonicity_Change_Left(MT9V03X_H-1-10,10);
	Right_Up_Guai[0]=Find_Right_Up_Point(0,MT9V03X_H-5);
	Left_Up_Guai[0]=Find_Left_Up_Point(0,MT9V03X_H-5);
	monotonicity_change_line[1]=Right_Line[monotonicity_change_right_flag];
	right_down_guai[0]=Find_Right_Down_Point(MT9V03X_H-1,20);
	left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
	
		/***********************************补线********************************************************/
		if(Island_State==3&&Right_Island_Flag==1)
		{
			Right_Up_Guai[0]=Find_Right_Up_Point(0,99);
			Right_Up_Guai[1]=Right_Line[Right_Up_Guai[0]];
			Left_Add_Line(10,99,Right_Up_Guai[1],Right_Up_Guai[0]);	
		}	
	
		if(Island_State==3&&Left_Island_Flag==1)
		{
			Left_Up_Guai[0]=Find_Left_Up_Point(0,99);
			Left_Up_Guai[1]=Left_Line[Left_Up_Guai[0]];
//			Right_Add_Line(158,99,Left_Up_Guai[1],Left_Up_Guai[0]);
			Right_Add_Line(158,99,Left_Up_Guai[1],Left_Up_Guai[0]);
		}
		if(Island_State==5&&Left_Island_Flag==1)
		{
			Left_Up_Guai[0]=Find_Left_Up_Point(30,99);
			Left_Up_Guai[1]=Left_Line[Left_Up_Guai[0]];
			Left_Add_Line(Left_Up_Guai[1],Left_Up_Guai[0],Left_Up_Guai[1]-10,99);
		}
	
		/********************************************************************************************/
//	if(Cross_Flag==0&&Island_State==0&&Ramp_Flag==0)
//    {
        continuity_change_left_flag=Continuity_Change_Left(MT9V03X_H-1-5,10);
        continuity_change_right_flag=Continuity_Change_Right(MT9V03X_H-1-5,10);

        if(Left_Island_Flag==0)//左环
        {
            if(monotonicity_change_right_flag==0&& 
               continuity_change_left_flag!=0&& 
               continuity_change_right_flag==0&&
							 Left_Up_Guai[0]!=0&&
               Left_Lost_Time>=10&& 
               Left_Lost_Time<=88&& 
               Right_Lost_Time<=40
               )
            {
				if(left_down_guai[0]>=25)   
                {
                    Island_State=1;
                    Left_Island_Flag=1;
										Clear_Gyroscope_Angle();
                }
                else//误判，归零
                {
                    Island_State=0;
                    Left_Island_Flag=0;
                }
            }
        }
        if(Right_Island_Flag==0)//右环
        {
            if(monotonicity_change_left_flag==0&&
               continuity_change_left_flag==0&& 
               continuity_change_right_flag!=0&& 
							 Search_Stop_Line>=MT9V03X_H*0.99&&
							 Right_Up_Guai[0]!=0&&
               Right_Lost_Time>=10&&           
               Right_Lost_Time<=88&&           
               Left_Lost_Time<=30         //这个根据实际情况改
               )
            {
                if(right_down_guai[0]>=20)    //这个根据实际情况改
                {
                    Island_State=1;
                    Right_Island_Flag=1;
										Clear_Gyroscope_Angle();
                }
                else
                {
                    Island_State=0;
                    Right_Island_Flag=0;
                }
            }
        }
//		}
	
		
	/**********************************************************************右环岛************************************************************************************/
//	if(FJ_Angle>=60&&(Island_State==3&&Right_Island_Flag==1))
		if(FJ_Angle>=30&&(Island_State==3&&Right_Island_Flag==1))
	{
		Island_State=4;
		Longest_White_Column();
	}
	/*****************************************************************************************************************************/
	if(Right_Island_Flag==1)
	{
		if(Island_State==1)
		{
			monotonicity_change_line[0]=Monotonicity_Change_Right(30,5);//单调性改变
			monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
			Right_Add_Line((int)(MT9V03X_W-1-(monotonicity_change_line[1]*0.15)),MT9V03X_H-1,monotonicity_change_line[1],monotonicity_change_line[0]);
			if(Boundry_Start_Right<=30)
			{
				Island_State=2;
			}
		}else if(Island_State==2)
		{
			monotonicity_change_line[0]=Monotonicity_Change_Right(70,5);//单调性改变
      monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
      Right_Add_Line((int)(MT9V03X_W-1-(monotonicity_change_line[1]*0.15)),MT9V03X_H-1,monotonicity_change_line[1],monotonicity_change_line[0]);
			if(Island_State==2&&(Boundry_Start_Right>=MT9V03X_H-25||monotonicity_change_line[0]>=40))//右下角再不丢线进3
//			if(Island_State==2&&monotonicity_change_line[0]>=45)//右下角再不丢线进3
      {
        Island_State=3;
				Longest_White_Column();	
      }
    }
		if(Island_State==4)
		{
			if(FJ_Angle>=240&&Left_Lost_Time!=0)
			{
				continuity_change_left_flag=Continuity_Change_Left(MT9V03X_H-1-5,10);
				Left_Add_Line(Left_Line[continuity_change_left_flag],continuity_change_left_flag,180,0);
			}
			if(Both_Lost_Time==0&&FJ_Angle>=330)
			{	
				Island_State=5;
			}
		}
		if(Island_State==5)
		{
			Right_Up_Guai[0]=Find_Right_Up_Point(0,99);
			Right_Up_Guai[1]=Right_Line[Right_Up_Guai[0]];
			Right_Add_Line(Right_Up_Guai[1],Right_Up_Guai[0],Right_Up_Guai[1],99);
			if(Right_Up_Guai[0]>=55||FJ_Angle>=358)
			{
				Island_State=0;
				Right_Island_Flag=2;
				Clear_Gyroscope_Angle();
			}
		}
		
	}

	/*********************************************************************左环岛************************************************************************************/
	
	if(FJ_Angle<=-40&&(Island_State==3&&Left_Island_Flag==1))
	{
		Island_State=4;
		Longest_White_Column();
	}
	/*****************************************************************************************************************/
	if(Left_Island_Flag==1)
	{
		if(Island_State==1)
		{
			monotonicity_change_line[0]=Monotonicity_Change_Left(30,5);//寻找单调性改变点
      monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]];
      Left_Add_Line((int)(monotonicity_change_line[1]*0.15),MT9V03X_H-1,monotonicity_change_line[1],monotonicity_change_line[0]);
			if((Island_State==1)&&(Boundry_Start_Left<30))//下方当丢线时候进2
      {
         Island_State=2;
      }
		}else if(Island_State==2)
		{
			 monotonicity_change_line[0]=Monotonicity_Change_Left(70,10);//寻找单调性改变点
       monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]];
       Left_Add_Line((int)(monotonicity_change_line[1]*0.1),MT9V03X_H-1,monotonicity_change_line[1],monotonicity_change_line[0]);
			if(Island_State==2&&(Boundry_Start_Left>=MT9V03X_H-5||monotonicity_change_line[0]>50))//当圆弧靠下时候，进3
      {
        Island_State=3;//最长白列寻找范围也要改，见camera.c
        Left_Island_Flag=1;
      }
		}
		if(Island_State==4)
		{
			if(FJ_Angle<=-260&&FJ_Angle>=-305)
			{
				continuity_change_right_flag=Continuity_Change_Right(MT9V03X_H-1,10);
//				if(continuity_change_right_flag!=0)
//				{
//					Right_Add_Line(Right_Line[continuity_change_right_flag],continuity_change_right_flag,0,0);
//				}
//				if(Both_Lost_Time!=0)
//				{
					Right_Add_Line(0,50,187,99);
//				}	
			}
				
			if(FJ_Angle<=-340&&Both_Lost_Time==0)
			{
				Island_State=5;
				Left_Up_Guai[0]=Find_Left_Up_Point(20,99);
				Left_Up_Guai[1]=Left_Line[Left_Up_Guai[0]];
			}
		}
		if(Island_State==5&&FJ_Angle<=-350)
		{
			Island_State=0;
			Left_Island_Flag=2;
			Clear_Gyroscope_Angle();
		}
	}
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     Continuity_Change_Left(int start,int end);
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Left(int start,int end)//连续性阈值设置为5
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
       return 1;
    if(Search_Stop_Line<=5)//搜所截止行很矮
       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
    
    for(i=start;i>=end;i--)
    {
       if(abs(Left_Line[i]-Left_Line[i-1])>=5)//连续判断阈值是5,可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Right(int start,int end)
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
       return 1;
    if(start>=MT9V03X_H-5)//数组越界保护
        start=MT9V03X_H-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }

    for(i=start;i>=end;i--)
    {
        if(abs(Right_Line[i]-Right_Line[i-1])>=5)//连续性阈值是5，可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Left_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int left_down_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(left_down_line==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&  
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=5&&
              (Left_Line[i]-Left_Line[i-3])>=10&&
              (Left_Line[i]-Left_Line[i-4])>=10)
        {
            left_down_line=i;//获取行数即可
            break;
        }
    }
    return left_down_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左上角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Left_Up_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Up_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int left_up_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_up_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=MT9V03X_H-Search_Stop_Line)//搜索截止行往上的全都不判
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(left_up_line==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i-1])<=5&&
           abs(Left_Line[i-1]-Left_Line[i-2])<=5&&  
           abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
              (Left_Line[i]-Left_Line[i+2])>=5&& 
              (Left_Line[i]-Left_Line[i+3])>=10&&
              (Left_Line[i]-Left_Line[i+4])>=15)
        {
            left_up_line=i;//获取行数即可
            break;
        }
    }
    return left_up_line;//如果是MT9V03X_H-1，说明没有这么个拐点
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int right_down_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return right_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
//        if(right_down_line==0&&//只找第一个符合条件的点
//           abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
//           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&  
//           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
//              (Right_Line[i]-Right_Line[i-2])<=-5&&
//              (Right_Line[i]-Right_Line[i-3])<=-10&&
//              (Right_Line[i]-Right_Line[i-4])<=-10)
			
			if(right_down_line==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i+1])<=3&&//角点的阈值可以更改
           abs(Right_Line[i+1]-Right_Line[i+2])<=3&&  
           abs(Right_Line[i+2]-Right_Line[i+3])<=3&&
              (Right_Line[i]-Right_Line[i-2])<=-3&&
              (Right_Line[i]-Right_Line[i-3])<=-5&&
              (Right_Line[i]-Right_Line[i-4])<=-5)
        {
            right_down_line=i;//获取行数即可
            break;
        }
    }
    return right_down_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右上角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Up_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int right_up_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return right_up_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }    
    if(end<=MT9V03X_H-Search_Stop_Line)//搜索截止行往上的全都不判
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
//        if(right_up_line==0&&//只找第一个符合条件的点
//           abs(Right_Line[i]-Right_Line[i-1])<=5&&//下面两行位置差不多
//           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&  
//           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
//              (Right_Line[i]-Right_Line[i+2])<=-8&&
//              (Right_Line[i]-Right_Line[i+3])<=-15&&
//              (Right_Line[i]-Right_Line[i+4])<=-15)
			
			if(right_up_line==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i-1])<=5&&//下面两行位置差不多
           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&  
           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
           abs(Right_Line[i]-Right_Line[i+2])>=5&&
           abs(Right_Line[i]-Right_Line[i+3])>=10&&
           abs(Right_Line[i]-Right_Line[i+4])>=15)
        {
            right_up_line=i;//获取行数即可
            break;
        }
    }
    return right_up_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Left(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
    int i;
    int monotonicity_change_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
       return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)//数组越界保护，在判断第i个点时
       start=MT9V03X_H-1-5; //要访问它前后5个点，数组两头的点要不能作为起点终点
    if(end<=5)
        end=5;
    if(start<=end)//递减计算，入口反了，直接返回0
      return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Left_Line[i]==Left_Line[i+5]&&Left_Line[i]==Left_Line[i-5]&&
        Left_Line[i]==Left_Line[i+4]&&Left_Line[i]==Left_Line[i-4]&&
        Left_Line[i]==Left_Line[i+3]&&Left_Line[i]==Left_Line[i-3]&&
        Left_Line[i]==Left_Line[i+2]&&Left_Line[i]==Left_Line[i-2]&&
        Left_Line[i]==Left_Line[i+1]&&Left_Line[i]==Left_Line[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Left_Line[i]>=Left_Line[i+5]&&Left_Line[i]>=Left_Line[i-5]&&
        Left_Line[i]>=Left_Line[i+4]&&Left_Line[i]>=Left_Line[i-4]&&
        Left_Line[i]>=Left_Line[i+3]&&Left_Line[i]>=Left_Line[i-3]&&
        Left_Line[i]>=Left_Line[i+2]&&Left_Line[i]>=Left_Line[i-2]&&
        Left_Line[i]>=Left_Line[i+1]&&Left_Line[i]>=Left_Line[i-1])
        {//就很暴力，这个数据是在前5，后5中最大的（可以取等），那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Right(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
    int i;
    int monotonicity_change_line=0;

    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
        return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
     if(end<=5)
        end=5;
    if(start<=end)
        return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Right_Line[i]==Right_Line[i+5]&&Right_Line[i]==Right_Line[i-5]&&
        Right_Line[i]==Right_Line[i+4]&&Right_Line[i]==Right_Line[i-4]&&
        Right_Line[i]==Right_Line[i+3]&&Right_Line[i]==Right_Line[i-3]&&
        Right_Line[i]==Right_Line[i+2]&&Right_Line[i]==Right_Line[i-2]&&
        Right_Line[i]==Right_Line[i+1]&&Right_Line[i]==Right_Line[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Right_Line[i]<=Right_Line[i+5]&&Right_Line[i]<=Right_Line[i-5]&&
        Right_Line[i]<=Right_Line[i+4]&&Right_Line[i]<=Right_Line[i-4]&&
        Right_Line[i]<=Right_Line[i+3]&&Right_Line[i]<=Right_Line[i-3]&&
        Right_Line[i]<=Right_Line[i+2]&&Right_Line[i]<=Right_Line[i-2]&&
        Right_Line[i]<=Right_Line[i+1]&&Right_Line[i]<=Right_Line[i-1])
        {//就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     元素标志位显示
  @param     二值化图片数组
  @return    null
  Sample     Image_Flag_Show(MT9V03X_W,image_two_value,Island_State);
  @note      要在图片显示前使用，传入标志位可更改
-------------------------------------------------------------------------------------------------------------------*/
void Image_Flag_Show(uint8 MT9V03XW,uint8(*InImg)[MT9V03XW],uint8 image_flag)
{
    for(uint8 H=1;H<8;H++)
    {
        for(uint8 W=1;W<7;W++)
        {
            switch (image_flag)
            {
               // case 0:     *(*(InImg+H)+W)=Image_Flags[0][H][W];    break;//环岛0~8
                case 1:     *(*(InImg+H)+W)=Image_Flags[1][H][W];    break;
                case 2:     *(*(InImg+H)+W)=Image_Flags[2][H][W];    break;
                case 3:     *(*(InImg+H)+W)=Image_Flags[3][H][W];    break;
                case 4:     *(*(InImg+H)+W)=Image_Flags[4][H][W];    break;
                case 5:     *(*(InImg+H)+W)=Image_Flags[5][H][W];    break;
                case 6:     *(*(InImg+H)+W)=Image_Flags[6][H][W];    break;
                case 7:     *(*(InImg+H)+W)=Image_Flags[7][H][W];    break;
                case 8:     *(*(InImg+H)+W)=Image_Flags[8][H][W];    break;
                case 9:     *(*(InImg+H)+W)=Image_Flags[9][H][W];    break;
                default:    break;
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     通过斜率，定点补线--
  @param     k       输入斜率
             startY  输入起始点纵坐标
             endY    结束点纵坐标
  @return    null
  Sample     K_Add_Boundry_Left(float k,int startY,int endY);
  @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Left(float k,int startX,int startY,int endY)
{
    int i,t;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    if(startY<endY)//--操作，start需要大
    {
        t=startY;
        startY=endY;
        endY=t;
    }
    for(i=startY;i>=endY;i--)
    {
        Left_Line[i]=(int)((i-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if(Left_Line[i]>=MT9V03X_W-1)
        {
            Left_Line[i]=MT9V03X_W-1;
        }
        else if(Left_Line[i]<=0)
        {
            Left_Line[i]=0;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     通过斜率，定点补线
  @param     k       输入斜率
             startY  输入起始点纵坐标
             endY    结束点纵坐标
  @return    null    直接补边线
  Sample     K_Add_Boundry_Right(float k,int startY,int endY);
  @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Right(float k,int startX,int startY,int endY)
{
    int i,t;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    if(startY<endY)
    {
        t=startY;
        startY=endY;
        endY=t;
    }
    for(i=startY;i>=endY;i--)
    {
        Right_Line[i]=(int)((i-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if(Right_Line[i]>=MT9V03X_W-1)
        {
            Right_Line[i]=MT9V03X_W-1;
        }
        else if(Right_Line[i]<=0)
        {
            Right_Line[i]=0;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     根据斜率划线
  @param     输入斜率，定点，画一条黑线
  @return    null
  Sample     K_Draw_Line(k, 20,MT9V03X_H-1 ,0)
  @note      补的就是一条线，需要重新扫线
-------------------------------------------------------------------------------------------------------------------*/
void K_Draw_Line(float k, int startX, int startY,int endY)
{
    int endX=0;

    if(startX>=MT9V03X_W-1)//限幅处理
        startX=MT9V03X_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    endX=(int)((endY-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
    Draw_Line(startX,startY,endX,endY);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     获取平均赛宽
  @param     int start_line,int end_line，起始行，中止行
  @return    这几行赛宽平均值
  Sample     road_wide=Get_Road_Wide(68,69);
  @note      ++运算，向下寻找，算出平均赛宽
-------------------------------------------------------------------------------------------------------------------*/
int Get_Road_Wide(int start_line,int end_line)
{
    if(start_line>=MT9V03X_H-1)
        start_line=MT9V03X_H-1;
    else if(start_line<=0)
        start_line=0;
    if(end_line>=MT9V03X_H-1)
        end_line=MT9V03X_H-1;
    else if(end_line<=0)
        end_line=0;
    int i=0,t=0;
    int road_wide=0;
    if(start_line>end_line)//++访问，坐标反了互换
    {
        t=start_line;
        start_line=end_line;
        end_line=t;
    }
    for(i=start_line;i<=end_line;i++)
    {
        road_wide+=Right_Line[i]-Left_Line[i];
    }
    road_wide=road_wide/(end_line-start_line+1);//平均赛宽
    return road_wide;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     获取左赛道边界斜率
  @param     int start_line,int end_line，起始行，中止行
  @return    两点之间的斜率
  Sample     k=Get_Left_K(68,69);
  @note      两点之间得出斜率，默认第一个参数小，第二个参数大
-------------------------------------------------------------------------------------------------------------------*/
float Get_Left_K(int start_line,int end_line)
{
    if(start_line>=MT9V03X_H-1)
        start_line=MT9V03X_H-1;
    else if(start_line<=0)
        start_line=0;
    if(end_line>=MT9V03X_H-1)
        end_line=MT9V03X_H-1;
    else if(end_line<=0)
        end_line=0;
    float k=0;
    int t=0;
    if(start_line>end_line)//++访问，坐标反了互换
    {
        t=start_line;
        start_line=end_line;
        end_line=t;
    }
    k=(float)(((float)Left_Line[start_line]-(float)Left_Line[end_line])/(end_line-start_line+1));
    return k;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     获取右赛道边界斜率
  @param     int start_line,int end_line，起始行，中止行
  @return    两点之间的斜率
  Sample     k=Get_Right_K(68,69);
  @note      两点之间得出斜率，默认第一个参数小，第二个参数大
-------------------------------------------------------------------------------------------------------------------*/
float Get_Right_K(int start_line,int end_line)
{
    if(start_line>=MT9V03X_H-1)
        start_line=MT9V03X_H-1;
    else if(start_line<=0)
        start_line=0;
    if(end_line>=MT9V03X_H-1)
        end_line=MT9V03X_H-1;
    else if(end_line<=0)
        end_line=0;
    float k=0;
    int t=0;
    if(start_line>end_line)//++访问，坐标反了互换
    {
        t=start_line;
        start_line=end_line;
        end_line=t;
    }
    k=(float)(((float)Right_Line[start_line]-(float)Right_Line[end_line])/(end_line-start_line+1));
    return k;
}
