#include "zf_common_headfile.h"
#include "Camera.h"
#include  <math.h>

//常用基本变量

//extern const uint8 Image_Flags[][9][8];     //放在图上的数字标记
uint8 image_two_value[MT9V03X_H][MT9V03X_W];//二值化后的原数组
volatile int Left_Line[MT9V03X_H]; //左边线数组
volatile int Right_Line[MT9V03X_H];//右边线数组
volatile int Mid_Line[MT9V03X_H];  //中线数组
volatile int Road_Wide[MT9V03X_H]; //赛宽数组
volatile int White_Column[MT9V03X_W];    //每列白列长度
volatile int Search_Stop_Line;     //搜索截止行,只记录长度，想要坐标需要用视野高度减去该值
volatile int Boundry_Start_Left;   //左右边界起始点
volatile int Boundry_Start_Right;  //第一个非丢线点,常规边界起始点
volatile int Left_Lost_Time;       //边界丢线数
volatile int Right_Lost_Time;
volatile int Both_Lost_Time;//两边同时丢线数
volatile int Longest_White_Column_Left[2]; //最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
volatile int Longest_White_Column_Right[2];//最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
int Left_Lost_Flag[MT9V03X_H] ; //左丢线数组，丢线置1，没丢线置0
int Right_Lost_Flag[MT9V03X_H]; //右丢线数组，丢线置1，没丢线置0

//十字
volatile int Cross_Flag=0;
volatile int Left_Down_Find=0;  //十字使用，找到被置行数，没找到就是0
volatile int Left_Up_Find=0;    //四个拐点标志
volatile int Right_Down_Find=0; //十字使用，找到被置行数，没找到就是0
volatile int Right_Up_Find=0;  

//控制标
volatile uint8 Obstacle_Dir=0; //0右拐，1左拐
volatile uint8 Island_Switch=1;//环岛识别开启标志位
volatile uint8 Straight_Flag=0;//长直道识别标
volatile uint8 Ramp_Switch=0;  //坡道识别标志位
extern volatile uint8 start_flag;

//舵机
extern volatile float Err;//摄像头误差

//横断路障标志位
volatile uint8 Barricade_Flag=0;     //横断，有1~4几个状态，0说明不在横断里

//坡道
volatile int Ramp_Flag=0;//坡道标志
volatile int Count=0;

//斑马线
volatile int Zebra_Stripes_Flag=0;//斑马线
volatile int change_count=0;//跳变计数
volatile int narrow_road_count=0;

//环岛
extern volatile int Island_State;     //环岛状态标志
extern volatile int Left_Island_Flag; //左环岛标志
extern volatile int Right_Island_Flag;//右环岛标志

//图像消失
volatile uint8 Img_Disappear_Flag=0; //图像消失标志位，1就说明丢图了
volatile uint8 Stop_Flag=0;          //停止标


volatile int sc=20;
volatile int ec=MT9V03X_W-20;
int z=1;
extern volatile float FJ_Angle;


//标准赛宽，将车子放在长直道上面实测，以下数值仅供参考
const uint8 Standard_Road_Wide[MT9V03X_H]=         //只取到104，因为后面都是全满
{ 37, 38, 40, 42, 43, 45, 46, 48, 50,		   //0~104一共105个数据
  49, 52, 53, 54, 56, 58, 59, 60, 61, 63,
  64, 65, 67, 68, 69, 71, 72, 73, 75, 76,
  77, 79, 80, 82, 83, 85, 87, 89, 91, 92,
  94, 96, 97, 98, 100, 102, 104, 106, 107, 109,
  110, 112, 114, 115,116,118,120,122,123,123,
  125,127,129,131,131,135,135,136,137,139,
  141,142,145,145,147,149,151,152,153,155,155,
  157,158,159,161,163,164,164,167,168,169,
  171,173,174,174,176,178,179,180,181,182,
};

  
//坡道赛宽，跟标准赛宽作比较，看看坡道赛宽有多少比标准赛宽大（稍小一点作为阈值）      几乎全比标准赛宽大，因此可以将40设为阈值
	/*
  Road_Wide[MT9V03X_H]= 
  {46, 48, 50, 52, 55, 57, 59, 61, 63, 65, 
   68, 70, 72, 74，75, 76, 78, 79, 80, 81, 
   82, 83, 85, 86, 87, 88, 89, 91, 92, 93, 
   94, 95, 96, 98, 99, 100,101,102,104,105,
   106,107,108,109,111,112,113,114,115,116,
   118,119,120,121,122,124,125,126,127,128,
   129,131,132,133,134,135,137,138,139,140,
   141,142,144,145,146,147,148,150,151,152,
   153,154,155,157,158,159,160,161,163,164,
   165,166,167,168,170,171,172,173,174,175,
   177,178,179,180,181}
	*/
  
  
  

//摄像头部分宏定义
#define MT9V03X_W               (188)                                           // 图像宽度     范围 [1-752]
#define MT9V03X_H               (100)                                           // 图像高度     范围 [1-480]
#define MT9V03X_IMAGE_SIZE      (MT9V03X_W * MT9V03X_H)                         // 图像尺寸     底乘高

/*-------------------------------------------------------------------------------------------------------------------
  @brief     快速大津求阈值，来自山威
  @param     image       图像数组
             col         列 ，宽度
             row         行，长度
  @return    null
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//山威快速大津
  @note      据说比传统大津法快一点，使用效果差不多
-------------------------------------------------------------------------------------------------------------------*/
uint8 Camera_GetOSTU(uint8 tmImage[MT9V03X_H][MT9V03X_W])
{ 
	int16 i,j; 
	uint32 Amount = 0; 
	uint32 PixelBack = 0; 
	uint32 PixelIntegralBack = 0; 
	uint32 PixelIntegral = 0; 
	int32 PixelIntegralFore = 0; 
	int32 PixelFore = 0; 
	double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差; 
	int16 MinValue, MaxValue; 
	uint8 Threshold = 0;
	uint8 HistoGram[256];              //  
 
	for (j = 0; j < 256; j++)  
		HistoGram[j] = 0; //初始化灰度直方图 
  
	for (j = 0; j < MT9V03X_H; j++) 
	{ 
		for (i = 0; i < MT9V03X_W; i++) 
		{ 
		HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
		} 
	} 
  
	for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
	for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值
      
	if (MaxValue == MinValue)     return MaxValue;         // 图像中只有一个颜色    
	if (MinValue + 1 == MaxValue)  return MinValue;        // 图像中只有二个颜色
    
	for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  像素总数
    
	PixelIntegral = 0;
	for (j = MinValue; j <= MaxValue; j++)
	{
		PixelIntegral += HistoGram[j] * j;//灰度值总数
	}
	SigmaB = -1;
	for (j = MinValue; j < MaxValue; j++)
	{
		PixelBack = PixelBack + HistoGram[j];   //前景像素点数
		PixelFore = Amount - PixelBack;         //背景像素点数
		OmegaBack = (double)PixelBack / Amount;//前景像素百分比
		OmegaFore = (double)PixelFore / Amount;//背景像素百分比
		PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
		PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
		MicroBack = (double)PixelIntegralBack / PixelBack;   //前景灰度百分比
		MicroFore = (double)PixelIntegralFore / PixelFore;   //背景灰度百分比
		Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
		if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
		{
		  SigmaB = Sigma;
		  Threshold = j;
		}
	}
	return Threshold;                        //返回最佳阈值;
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     图像二值化处理函数
  @param     原始图像数组，高度，宽度，二值化阈值
  @return    二值化之后的图像数组
  Sample     Image_Binarization(Threshold);//图像二值化
  @note      二值化处理，0x00黑，0xff是白，已经用宏定义替换过了
-------------------------------------------------------------------------------------------------------------------*/
void Image_Binarization(int threshold)//图像二值化
{
    uint16 i,j;
    for(i=0;i<MT9V03X_H;i++)
    {
        for(j=0;j<MT9V03X_W;j++)
        {
            if(mt9v03x_image[i][j]>=threshold)
                image_two_value[i][j]=IMG_WHITE;//白
            else
                image_two_value[i][j]=IMG_BLACK;//黑
        }
    }
}
	

/*-------------------------------------------------------------------------------------------------------------------
  @brief     丢图检测
  @param     null
  @return    null
  Sample     Stop_Detect(void)
  @note      最长白列很短，边界不连续，停车
-------------------------------------------------------------------------------------------------------------------*/
void Img_Disappear_Detect(void)
{
//    if(Barricade_Flag!=0||Stop_Flag!=0||Ramp_Flag!=0)//横断时候要关掉，停车之后也要关掉
//    {
//        return;
//    }
    int i=0;
    uint8 black_line_count=0;
    uint8 continuity_change_right_flag=0;
    uint8 continuity_change_left_flag=0;
    continuity_change_left_flag= Continuity_Change_Left (MT9V03X_H-1,MT9V03X_H-10);//连续性判断
    continuity_change_right_flag=Continuity_Change_Right(MT9V03X_H-1,MT9V03X_H-10);

    if(continuity_change_left_flag!=0&&continuity_change_right_flag!=0&&Search_Stop_Line<=10)
    {//不连续，且搜索截止行非常短
        Img_Disappear_Flag=1;//认为丢图
        return;
    }//下面这节专门为弯道入断路准备的，图像特征就是赛道的尖尖，有个三角形
    else if(Search_Stop_Line<=45&&//截止行比较短
            (abs(Longest_White_Column_Left[1]-Longest_White_Column_Right[1])<15)&&//左右最长白列位置近
            (Road_Wide[(MT9V03X_H-Search_Stop_Line)+1]<=30)&&
            (Road_Wide[(MT9V03X_H-Search_Stop_Line)+2]<=30)&&//截止行下面赛道宽很小
            (Standard_Road_Wide[(MT9V03X_H-Search_Stop_Line)+2]-Road_Wide[(MT9V03X_H-Search_Stop_Line)+2])>10&&
            (Standard_Road_Wide[(MT9V03X_H-Search_Stop_Line)+1]-Road_Wide[(MT9V03X_H-Search_Stop_Line)+1])>10&&//最长白列下面几行赛道宽度比正常短
            (Boundry_Start_Left +Left_Lost_Time ==MT9V03X_H-1)&&//丢线数+起始点==MT9V03X_H-1，说明丢的都是下面的点，上面没有丢点
            (Boundry_Start_Right+Right_Lost_Time==MT9V03X_H-1)&&//丢线数+起始点==MT9V03X_H-1，说明丢的都是下面的点
             Left_Lost_Time<10&&Right_Lost_Time<=10)//丢线数少，保证不能像90度弯一样，延伸出去
    {
        Img_Disappear_Flag=1;                                   //认为丢图
        return;
    }
    else
    {
        for(i= MT9V03X_H-1; i>MT9V03X_H-1-10; i--)//选定区域全黑，认为丢图了
        {
            if(image_two_value[i][4]==IMG_BLACK&&image_two_value[i][5]==IMG_BLACK&&
               image_two_value[i][10]==IMG_BLACK&&image_two_value[i][15]==IMG_BLACK&&
               image_two_value[i][20]==IMG_BLACK&&image_two_value[i][25]==IMG_BLACK&&
               image_two_value[i][64]==IMG_BLACK&&image_two_value[i][65]==IMG_BLACK&&
               image_two_value[i][66]==IMG_BLACK&&image_two_value[i][67]==IMG_BLACK&&
               image_two_value[i][68]==IMG_BLACK&&image_two_value[i][69]==IMG_BLACK&&
               image_two_value[i][70]==IMG_BLACK&&image_two_value[i][75]==IMG_BLACK&&
               image_two_value[i][66]==IMG_BLACK&&image_two_value[i][67]==IMG_BLACK&&
               image_two_value[i][80]==IMG_BLACK&&image_two_value[i][90]==IMG_BLACK&&
               image_two_value[i][100]==IMG_BLACK&&image_two_value[i][110]==IMG_BLACK&&
               image_two_value[i][115]==IMG_BLACK&&image_two_value[i][120]==IMG_BLACK&&
               image_two_value[i][125]==IMG_BLACK&&image_two_value[i][130]==IMG_BLACK&&
               image_two_value[i][135]==IMG_BLACK&&image_two_value[i][136]==IMG_BLACK&&
               image_two_value[i][137]==IMG_BLACK&&image_two_value[i][138]==IMG_BLACK
            )
                black_line_count++;
        }
        if(black_line_count>=5)
        {
            Img_Disappear_Flag=1;
        }
    }
}




/*-------------------------------------------------------------------------------------------------------------------
  @brief     双最长白列巡线
  @param     null
  @return    null
  Sample     Longest_White_Column_Left();
  @note      最长白列巡线，寻找初始边界，丢线，最长白列等基础元素，后续读取这些变量来进行赛道识别
-------------------------------------------------------------------------------------------------------------------*/


void Longest_White_Column()//最长白列巡线
{
    int i, j;
    int start_column=20;
	  int end_column=MT9V03X_W-20;
		
		start_column=20;
		end_column=MT9V03X_W-20;
	
    if(Right_Island_Flag==1&&Island_State==3)//右环
    {
            start_column=40;
            end_column=MT9V03X_W-20;
    }
    else if(Left_Island_Flag==1&&Island_State==3)//左环
    {
            start_column=20;
            end_column=MT9V03X_W-100;
    } 
		
//		if((Left_Island_Flag==1&&Island_State==4)&&FJ_Angle>=-65)
//		{
//			start_column=20;
//			end_column=MT9V03X_W-80;
//		}
		
		ec=end_column;
		sc=start_column;

	
	int left_border = 0, right_border = 0;//临时存储赛道位置
    Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Right_Lost_Time = 0;    //边界丢线数
    Left_Lost_Time  = 0;
    Boundry_Start_Left  = 0;//第一个非丢线点,常规边界起始点
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;//两边同时丢线数
	

    for (i = 0; i <=MT9V03X_H-1; i++)//数据清零
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        Left_Line[i] = 0;
        Right_Line[i] = MT9V03X_W-1;
    }
    for(i=0;i<=MT9V03X_W-1;i++)
    {
        White_Column[i] = 0;
    }
		
		

	

    //从左到右，从下往上，遍历全图记录范围内的每一列白点数量
    for (j =start_column; j<=end_column; j++)
    {
        for (i = MT9V03X_H - 1; i >= 0; i--)
        {
            if(image_two_value[i][j] == IMG_BLACK)
                break;
            else
                White_Column[j]++;
        }
    }

    //从左到右找左边最长白列
    Longest_White_Column_Left[0] =0;
    for(i=start_column;i<=end_column;i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
        }
    }
    //从右到左找右左边最长白列
    Longest_White_Column_Right[0] = 0;//【0】是白列长度
    for(i=end_column;i>=start_column;i--)//从右往左，注意条件，找到左边最长白列位置就可以停了
    {
        if (Longest_White_Column_Right[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Right[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
        }
    }

    Search_Stop_Line = Longest_White_Column_Left[0];//搜索截止行选取左或者右区别不大，他们两个理论上是一样的
    for (i = MT9V03X_H - 1; i >=MT9V03X_H-Search_Stop_Line; i--)//常规巡线
    {
        for (j = Longest_White_Column_Right[1]; j <= MT9V03X_W - 1 - 2; j++)
        {
            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j + 1] == IMG_BLACK && image_two_value[i][j + 2] == IMG_BLACK)//白黑黑，找到右边界
            {
                right_border = j;
                Right_Lost_Flag[i] = 0; //右丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(j>=MT9V03X_W-1-2)//没找到右边界，把屏幕最右赋值给右边界
            {
                right_border = j;
                Right_Lost_Flag[i] = 1; //右丢线数组，丢线置1，不丢线置0
                break;
            }
        }
        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--)//往左边扫描
        {
            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j - 1] == IMG_BLACK && image_two_value[i][j - 2] == IMG_BLACK)//黑黑白认为到达左边界
            {
                left_border = j;
                Left_Lost_Flag[i] = 0; //左丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(j<=0+2)
            {
                left_border = j;//找到头都没找到边，就把屏幕最左右当做边界
                Left_Lost_Flag[i] = 1; //左丢线数组，丢线置1，不丢线置0
                break;
            }
        }
        Left_Line [i] = left_border;       //左边线线数组
        Right_Line[i] = right_border;      //右边线线数组
    }

    for (i = MT9V03X_H - 1; i >= 0; i--)//赛道数据初步分析
    {
        if (Left_Lost_Flag[i]  == 1)//单边丢线数
            Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 1)
            Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)//双边丢线数
            Both_Lost_Time++;
        if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]  != 1)//记录第一个非丢线点，边界起始点
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
            Boundry_Start_Right = i;
        Road_Wide[i]=Right_Line[i]-Left_Line[i];
    }
    
    //debug使用，屏幕显示相关参数
//    ips200_showint16(0,0, Longest_White_Column_Right[0]);//【0】是白列长度
//    ips200_showint16(0,1, Longest_White_Column_Right[1]);//【1】是下标，第j列)
//    ips200_showint16(0,2, Longest_White_Column_Left[0]);//【0】是白列长度
//    ips200_showint16(0,3, Longest_White_Column_Left[1]);//【1】是下标，第j列)
}
   



/*-------------------------------------------------------------------------------------------------------------------
  @brief     横断检测
  @param     null
  @return    null
  Sample
  @note      截止行很低，边界起始点靠下，tof测到前面有东西
-------------------------------------------------------------------------------------------------------------------*/
void Barricade_Detect(void)
{
	int i;
    if(Barricade_Flag!=0||Ramp_Flag!=0)
    {//元素互斥  不是斜坡 
        return;
    }
    if(Boundry_Start_Left>=MT9V03X_H-10&&Boundry_Start_Right>=MT9V03X_H-10&&
       Left_Lost_Time<=10&&Right_Lost_Time<=10&&Search_Stop_Line<=60&&
       Left_Line[(MT9V03X_H-Search_Stop_Line)+1]>=10&&Right_Line[(MT9V03X_H-Search_Stop_Line)+1]<=(MT9V03X_W-10)&&//截止行往下不能丢线
       Left_Line[(MT9V03X_H-Search_Stop_Line)+2]>=10&&Right_Line[(MT9V03X_H-Search_Stop_Line)+2]<=(MT9V03X_W-10)&&
       Left_Line[(MT9V03X_H-Search_Stop_Line)+3]>=10&&Right_Line[(MT9V03X_H-Search_Stop_Line)+3]<=(MT9V03X_W-10)&&
       Left_Line[(MT9V03X_H-Search_Stop_Line)+4]>=10&&Right_Line[(MT9V03X_H-Search_Stop_Line)+4]<=(MT9V03X_W-10))
    {
		for(i=Search_Stop_Line;i<MT9V03X_H-10;i++)
		{
			if(abs(Mid_Line[i]-Mid_Line[i+3])>=5)
			{
				Barricade_Flag=1;
			}
		}	
    }
}




/*-------------------------------------------------------------------------------------------------------------------
  @brief     画线
  @param     输入起始点，终点坐标，补一条宽度为2的黑线
  @return    null
  Sample     Draw_Line(0, 0,MT9V03X_W-1,MT9V03X_H-1);
             Draw_Line(MT9V03X_W-1, 0,0,MT9V03X_H-1);
                                    画一个大×
  @note     补的就是一条线，需要重新扫线
-------------------------------------------------------------------------------------------------------------------*/
void Draw_Line(int startX, int startY, int endX, int endY)
{
    int i,x,y;
    int start=0,end=0;
    if(startX>=MT9V03X_W-1)//限幅处理
        startX=MT9V03X_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endX>=MT9V03X_W-1)
        endX=MT9V03X_W-1;
    else if(endX<=0)
        endX=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    if(startX==endX)//一条竖线
    {
        if (startY > endY)//互换
        {
            start=endY;
            end=startY;
        }
        for (i = start; i <= end; i++)
        {
            if(i<=1)
                i=1;
            image_two_value[i][startX]=IMG_BLACK;
            image_two_value[i-1][startX]=IMG_BLACK;
        }
    }
    else if(startY == endY)//补一条横线
    {
        if (startX > endX)//互换
        {
            start=endX;
            end=startX;
        }
        for (i = start; i <= end; i++)
        {
            if(startY<=1)
                startY=1;
            image_two_value[startY][i]=IMG_BLACK;
            image_two_value[startY-1][i]=IMG_BLACK;
        }
    }
    else //上面两个是水平，竖直特殊情况，下面是常见情况
    {
        if(startY>endY)//起始点矫正
        {
            start=endY;
            end=startY;
        }
        else
        {
            start=startY;
            end=endY;
        }
        for (i = start; i <= end; i++)//纵向补线，保证每一行都有黑点
        {
            x =(int)(startX+(endX-startX)*(i-startY)/(endY-startY));//两点式变形
            if(x>=MT9V03X_W-1)
                x=MT9V03X_W-1;
            else if (x<=1)
                x=1;
            image_two_value[i][x] = IMG_BLACK;
            image_two_value[i][x-1] = IMG_BLACK;
        }
        if(startX>endX)
        {
            start=endX;
            end=startX;
        }
        else
        {
            start=startX;
            end=endX;
        }
        for (i = start; i <= end; i++)//横向补线，保证每一列都有黑点
        {

            y =(int)(startY+(endY-startY)*(i-startX)/(endX-startX));//两点式变形
            if(y>=MT9V03X_H-1)
                y=MT9V03X_H-1;
            else if (y<=0)
                y=0;
            image_two_value[y][i] = IMG_BLACK;
        }
    }
}



/*-------------------------------------------------------------------------------------------------------------------
  @brief     左补线
  @param     补线的起点，终点
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的,不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Left_Add_Line(int x1,int y1,int x2,int y2)//左补线,补的是边界
{
    int i,max,a1,a2;
    int hx;
    if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
       x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
     if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
     else if(y1<=0)
        y1=0;
     if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
     else if(x2<=0)
             x2=0;
     if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
     else if(y2<=0)
             y2=0;
    a1=y1;
    a2=y2;
    if(a1>a2)//坐标互换
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//根据斜率补线即可
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=MT9V03X_W)
            hx=MT9V03X_W;
        else if(hx<=0)
            hx=0;
        Left_Line[i]=hx;
    }
}



/*-------------------------------------------------------------------------------------------------------------------
  @brief     右补线
  @param     补线的起点，终点
  @return    null
  Sample     Right_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的，不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Right_Add_Line(int x1,int y1,int x2,int y2)//右补线,补的是边界
{
    int i,max,a1,a2;
    int hx;
    if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
       x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
    if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
    else if(y1<=0)
        y1=0;
    if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
    else if(x2<=0)
        x2=0;
    if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
    else if(y2<=0)
         y2=0;
    a1=y1;
    a2=y2;
    if(a1>a2)//坐标互换
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//根据斜率补线即可
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=MT9V03X_W)
            hx=MT9V03X_W;
        else if(hx<=0)
            hx=0;
        Right_Line[i]=hx;
    }
}




/*-------------------------------------------------------------------------------------------------------------------
  @brief     找下面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int start,int end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Down_Point(int start,int end)
{
    int i,t;
    Right_Down_Find=0;
    Left_Down_Find=0;
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
        if(Left_Down_Find==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=8&&
              (Left_Line[i]-Left_Line[i-3])>=15&&
              (Left_Line[i]-Left_Line[i-4])>=15)
        {
            Left_Down_Find=i;//获取行数即可
        }
        if(Right_Down_Find==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&
           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
              (Right_Line[i]-Right_Line[i-2])<=-8&&
              (Right_Line[i]-Right_Line[i-3])<=-15&&
              (Right_Line[i]-Right_Line[i-4])<=-15)
        {
            Right_Down_Find=i;
        }
        if(Left_Down_Find!=0&&Right_Down_Find!=0)//两个找到就退出
        {
            break;
        }
    }
}



/*-------------------------------------------------------------------------------------------------------------------
  @brief     找上面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Left_Up_Find=0;
             Right_Up_Find=0;
  Sample     Find_Up_Point(int start,int end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Up_Point(int start,int end)
{
    int i,t;
    Left_Up_Find=0;
    Right_Up_Find=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(Left_Up_Find==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i-1])<=5&&
           abs(Left_Line[i-1]-Left_Line[i-2])<=5&&
           abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
              (Left_Line[i]-Left_Line[i+2])>=8&&
              (Left_Line[i]-Left_Line[i+3])>=15&&
              (Left_Line[i]-Left_Line[i+4])>=15)
        {
            Left_Up_Find=i;//获取行数即可
        }
        if(Right_Up_Find==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i-1])<=5&&//下面两行位置差不多
           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&
           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
              (Right_Line[i]-Right_Line[i+2])<=-8&&
              (Right_Line[i]-Right_Line[i+3])<=-15&&
              (Right_Line[i]-Right_Line[i+4])<=-15)
        {
            Right_Up_Find=i;//获取行数即可
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//下面两个找到就出去
        {
            break;
        }
    }
    if(abs(Right_Up_Find-Left_Up_Find)>=30)//纵向撕裂过大，视为误判
    {
        Right_Up_Find=0;
        Left_Up_Find=0;
    }
}




/*-------------------------------------------------------------------------------------------------------------------
  @brief     左边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Stop_Detect(void)
  @note      从起始点向上找5个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Left_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
    if(end<start)//++访问，坐标互换
    {
        t=end;
        end=start;
        start=t;
    }

    if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
         Left_Add_Line(Left_Line[start],start,Left_Line[end],end);
    }

    else
    {
        k=(float)(Left_Line[start]-Left_Line[start-4])/5.0;//这里的k是1/斜率
        for(i=start;i<=end;i++)
        {
            Left_Line[i]=(int)(i-start)*k+Left_Line[start];//(x=(y-y1)*k+x1),点斜式变形
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
}




/*-------------------------------------------------------------------------------------------------------------------
  @brief     右左边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Stop_Detect(void)
  @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
    if(end<start)//++访问，坐标互换
    {
        t=end;
        end=start;
        start=t;
    }

    if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Right_Add_Line(Right_Line[start],start,Right_Line[end],end);
    }
    else
    {
        k=(float)(Right_Line[start]-Right_Line[start-4])/5.0;//这里的k是1/斜率
        for(i=start;i<=end;i++)
        {
            Right_Line[i]=(int)(i-start)*k+Right_Line[start];//(x=(y-y1)*k+x1),点斜式变形
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
}





/*-------------------------------------------------------------------------------------------------------------------
  @brief     十字检测
  @param     null
  @return    null
  Sample     Cross_Detect(void);
  @note      利用四个拐点判别函数，查找四个角点，根据找到拐点的个数决定是否补线
-------------------------------------------------------------------------------------------------------------------*/
void Cross_Detect()
{
    int down_search_start=0;//下点搜索开始行
    Cross_Flag=0;
        Left_Up_Find=0;
        Right_Up_Find=0;
        if(Both_Lost_Time>=10)//十字必定有双边丢线，在有双边丢线的情况下再开始找角点
        {
            Find_Up_Point( MT9V03X_H-1, 0 );
            if(Left_Up_Find==0&&Right_Up_Find==0)//只要没有同时找到两个上点，直接结束
            {
                return;
            }
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//找到两个上点，就找到十字了
        {
            Cross_Flag=1;//对应标志位，便于各元素互斥掉
//					  Ramp_Flag=2;
						Clear_Gyroscope_Pitch();
            down_search_start=Left_Up_Find>Right_Up_Find?Left_Up_Find:Right_Up_Find;//用两个上拐点坐标靠下者作为下点的搜索上限
            Find_Down_Point(MT9V03X_H-5,down_search_start+2);//在上拐点下2行作为下点的截止行 （不能太窄）
            if(Left_Down_Find<=Left_Up_Find)
            {
                Left_Down_Find=0;//下点不可能比上点还靠上
            }
            if(Right_Down_Find<=Right_Up_Find)
            {
                Right_Down_Find=0;//下点不可能比上点还靠上
            }
            if(Left_Down_Find!=0&&Right_Down_Find!=0)
            {//四个点都在，无脑连线，这种情况显然很少
                Left_Add_Line (Left_Line [Left_Up_Find ],Left_Up_Find ,Left_Line [Left_Down_Find ] ,Left_Down_Find);
                Right_Add_Line(Right_Line[Right_Up_Find],Right_Up_Find,Right_Line[Right_Down_Find],Right_Down_Find);
            }
            else if(Left_Down_Find==0&&Right_Down_Find!=0)//11//这里使用的都是斜率补线
            {//三个点                                     //01
                Lengthen_Left_Boundry(Left_Up_Find-1,MT9V03X_H-1);
                Right_Add_Line(Right_Line[Right_Up_Find],Right_Up_Find,Right_Line[Right_Down_Find],Right_Down_Find);
            }
            else if(Left_Down_Find!=0&&Right_Down_Find==0)//11
            {//三个点                                     //10
                Left_Add_Line (Left_Line [Left_Up_Find ],Left_Up_Find ,Left_Line [Left_Down_Find ] ,Left_Down_Find);
                Lengthen_Right_Boundry(Right_Up_Find-1,MT9V03X_H-1);
            }
            else if(Left_Down_Find==0&&Right_Down_Find==0)//11
            {//就俩上点                                   //00
                Lengthen_Left_Boundry (Left_Up_Find-1,MT9V03X_H-1);
                Lengthen_Right_Boundry(Right_Up_Find-1,MT9V03X_H-1);
            }
        }
        else
        {
            Cross_Flag=0;
        }
    
}




/*-------------------------------------------------------------------------------------------------------------------
  @brief     直道检测
  @param     null
  @return    null
  Sample     Straight_Detect()；
  @note      利用最长白列，边界起始点，中线起始点，
-------------------------------------------------------------------------------------------------------------------*/
void Straight_Detect(void)
{
    Straight_Flag=0;
    if(Search_Stop_Line>=65)//截止行很远
    {
        if(Boundry_Start_Left>=68&&Boundry_Start_Right>=65)//起始点靠下
        {
            if(-5<=Err&&Err<=5)//误差很小
            {
                Straight_Flag=1;//认为是直道
            }
        }
    }
}




/*-------------------------------------------------------------------------------------------------------------------
  @brief     坡道检测
  @param     null
  @return    null
  Sample     Ramp_Detect();
  @note      赛宽变宽，测距前面有东西，丢线数少，截止行上面不丢线
-------------------------------------------------------------------------------------------------------------------*/
void Ramp_Detect(void)
{
	int i=0;
	int count=0;
//	
//	    if(count>=35&&Island_State==0&&Cross_Flag==0)//赛道过宽超过某一阈值
				if(count>=35)
    {
       if(Ramp_Flag==0&&FJ_Pitch>=1)//之前是0状态，说明是刚进坡道
			{
				Ramp_Flag=1;
			}
		}
    
	if(Search_Stop_Line>=66)//截止行长
    {
        for(i=MT9V03X_H-1;i>MT9V03X_H-Search_Stop_Line;i--)//赛宽过长计数
        {
            if(Road_Wide[i]-Standard_Road_Wide[i]>35)//图像赛宽比标准赛宽大
            {
                count++;//赛宽过宽行
            }
        }

    }
		if(FJ_Pitch<= -10)
		{
			Ramp_Flag=2;
		}
		
		
		Count=count;
        
    
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     斑马线检测
  @param     null
  @return    null
  Sample     Zebra_Stripes_Detect(void)
  @note      边界起始靠下，最长白列较长，赛道宽度过窄，且附近大量跳变
-------------------------------------------------------------------------------------------------------------------*/
void Zebra_Stripes_Detect(void)
{
  int j=0;
	int count=0;
	//前面的元素互斥各位自行补全
	if(Search_Stop_Line>=60)//视野很长
	{
		for(j=10;j<=MT9V03X_W-10;j++)
		{
			if(abs(White_Column[j]-White_Column[j+1])>=30)
			{
				count++;
			}
		}
	}
	if(count>=8)
	{
		Zebra_Stripes_Flag=1;
	}
	if(Zebra_Stripes_Flag==1)
		{
			Draw_Line(0,25,180,90);
			if(z==1)
			{
				Clear_Gyroscope_Angle();
				z--;
			}
			Longest_White_Column();
			if(FJ_Angle<=-60)
				Stop_Flag=1;
		}
}







/*-------------------------------------------------------------------------------------------------------------------
  @brief     断路检测
  @param     null
  @return    null
  Sample     Break_Road_Detect()；
  @note      利用最长白列，边界起始点，中线起始点，符合要求后切换电磁寻迹
-------------------------------------------------------------------------------------------------------------------*/
void Break_Road_Detect(void)
{
    if(Barricade_Flag!=0||Ramp_Flag!=0)//横断时候要关掉，停车之后也要关掉
    {
        return;
    }
    static uint8 break_road_state=0;

    //直道进入断路，提前判断
    if(Search_Stop_Line<=45&&//边界起始点靠下，截止行不长，左右丢线都少
       Boundry_Start_Left>=MT9V03X_H-10&&Boundry_Start_Right>=MT9V03X_H-10&&//边界起始行低
       Left_Lost_Time<=10&&Right_Lost_Time<=10&&
       (Boundry_Start_Left +Left_Lost_Time ==MT9V03X_H-1)&&//丢线数+起始点==MT9V03X_H-1，说明丢的都是下面的点，上面没有丢点
       (Boundry_Start_Right+Right_Lost_Time==MT9V03X_H-1)&&//丢线数+起始点==MT9V03X_H-1，说明丢的都是下面的点
       Left_Line[(MT9V03X_H-Search_Stop_Line)+1]>=10&&Right_Line[(MT9V03X_H-Search_Stop_Line)+1]<=(MT9V03X_W-10)&&//截止行往下不能丢线
       Left_Line[(MT9V03X_H-Search_Stop_Line)+2]>=10&&Right_Line[(MT9V03X_H-Search_Stop_Line)+2]<=(MT9V03X_W-10)&&
       Left_Line[(MT9V03X_H-Search_Stop_Line)+3]>=10&&Right_Line[(MT9V03X_H-Search_Stop_Line)+3]<=(MT9V03X_W-10)&&
       Left_Line[(MT9V03X_H-Search_Stop_Line)+4]>=10&&Right_Line[(MT9V03X_H-Search_Stop_Line)+4]<=(MT9V03X_W-10))
    {
        break_road_state=1;  //图像丢失+电磁数据正常，进入断路模式
    }
    //普通情况进断路
    if(break_road_state==0&&Img_Disappear_Flag==1)
    {//目前状态是摄像头跑，断路状态是0，图像丢了
        break_road_state=1;  //图像丢失+电磁数据正常，进入断路模式
    }

    //判断出断路
    if(break_road_state==1&&Search_Stop_Line>=50&&(Boundry_Start_Left>60||Boundry_Start_Right>60)&&(75<Longest_White_Column_Left[1]&&Longest_White_Column_Left[1]<105))
    {//在断路状态，搜索截止行高，边界起始点靠下,最长白列居中
        break_road_state=0;//截止行很长，边界起始点靠下，认为正常，换回摄像头
        Img_Disappear_Flag=0;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     边界显示，用于图传，显示到屏幕上，
  @param     null
  @return    null
  Sample     直接调用
  @note      显示左中右边界，中线，
                                           正常情况下不要用，因为直接在原图上写入了边界信息
                                           会对元素判断造成干扰的，调试时候调用
-------------------------------------------------------------------------------------------------------------------*/
void Show_Boundry(void)
{
    int16 i;
    for(i=MT9V03X_H-1;i>=MT9V03X_H-Search_Stop_Line;i--)//从最底下往上扫描
    {
        image_two_value[i][Left_Line[i]+1]=IMG_BLACK;
        image_two_value[i][(Left_Line[i]+Right_Line[i])>>1]=IMG_BLACK;
        image_two_value[i][Right_Line[i]-1]=IMG_BLACK;
    }
}


/*---------------------------------------------------------------
 【函    数】Bin_Image_Filter
 【功    能】过滤噪点
 【参    数】无
 【返 回 值】无
 【注意事项】
 ----------------------------------------------------------------*/
void Bin_Image_Filter (void)
{
    int16 nr; //行
    int16 nc; //列

    for (nr = 1; nr < 60 - 1; nr++)
    {
        for (nc = 1; nc < 94 - 1; nc = nc + 1)
        {
            if ((image_two_value[nr][nc] == 0)
                    && (image_two_value[nr - 1][nc] + image_two_value[nr + 1][nc] + image_two_value[nr][nc + 1] + image_two_value[nr][nc - 1] > 2))
            {
                image_two_value[nr][nc] = 1;
            }
            else if ((image_two_value[nr][nc] == 1)
                    && (image_two_value[nr - 1][nc] + image_two_value[nr + 1][nc] + image_two_value[nr][nc + 1] + image_two_value[nr][nc - 1] < 2))
            {
                image_two_value[nr][nc] = 0;
            }
        }
    }
}









