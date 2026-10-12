#ifndef __CAMERA_H
#define __CAMERA_H

/* 官方封装好的摄像头初始化函数 */

// 函数简介     MT9V03X 摄像头初始化
// 函数调用		uint8 mt9v03x_init (void)
// 参数说明     void
// 返回参数     uint8           1-失败 0-成功
// 使用示例     zf_log(mt9v03x_init(), "mt9v03x init error");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------

#define IMG_BLACK     0X00      //0x00是黑
#define IMG_WHITE     0Xff      //0xff为白


// 摄像头数据二维数组 ： mt9v03x_image[MT9V03X_H][MT9V03X_W]
// 访问图像数据直接访问这个指针变量就可以
// 访问方式非常简单，可以直接使用下标的方式访问
// 例如访问第10行 50列的点，mt9v03x_image[10][50]就可以了

// 摄像头采集完成标志位：mt9v03x_finish_flag ， 返回值为vuint8, 完成标志位置1，需要手动置0

int my_adapt_threshold(uint8 *image, uint16 col, uint16 row);
void Longest_White_Column(void);
void Image_Binarization(int threshold);//图像二值化
void Ramp_Detect(void);
void Stop_Detect(void);
void Zebra_Detect(void);
void Cross_Detect(void);
void Straight_Detect(void);
void Barricade_Detect(void);
void Break_Road_Detect(void);
void Zebra_Stripes_Detect(void);
void Img_Disappear_Detect(void);
void Show_Boundry(void);
void Find_Up_Point(int start,int end);
void Find_Down_Point(int start,int end);
void Lengthen_Left_Boundry(int start,int end);
void Lengthen_Right_Boundry(int start,int end);
void Left_Add_Line(int x1,int y1,int x2,int y2);
void Right_Add_Line(int x1,int y1,int x2,int y2);
void Draw_Line(int startX, int startY, int endX, int endY);

uint8 Camera_GetOSTU(uint8 tmImage[MT9V03X_H][MT9V03X_W]);
void Bin_Image_Filter (void);


#endif
