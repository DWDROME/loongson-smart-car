#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#ifndef MIN
    #define MIN(x, y) ((x) < (y) ? (x) : (y))
#endif
#ifndef MAX
    #define MAX(x, y) ((x) > (y) ? (x) : (y))
#endif

#define LCDH_0 120 // 原始图像
#define LCDW_0 160
#define LCDH_1  60// 压缩后的图像60
#define LCDW_1 94//94
#define FPS 120
#define white 255 // 白点像素值
#define black 0   // 黑点像素值
extern uint8_t Image_Zip[LCDH_1][LCDW_1] ; //压缩后的图像数组
extern uint8_t Image_Use[LCDH_1][LCDW_1] ;//(经过大津法，膨胀，腐蚀后的)二值化的图像
// 大津阈值下限（Otsu 结果低于此则抬到此值）；参数 threshold_static 可改
extern uint8 Threshold_static;
extern uint8_t Threshold; // 本帧实际使用的二值阈值（含边沿 -10 前）
extern unsigned char Image_IFS[LCDH_1][LCDW_1];
extern int Left_Sideline[LCDH_0],Right_Sideline[LCDH_0],zhangai_Rnum,zhangai_Lnum;    //左边线数组
extern int Mid_Line[LCDH_0];         //中线数组

 struct RedObject {
    cv::Point center;//质心坐标
    int area;//红色区域面积
 };
extern std::vector<RedObject> red_objects;
extern int red_area;
struct imageInformation
{
    // 图像相关信息
    uint16 bottom=0;  // 底部
    uint16 top=0;     // 截止行
    uint16 lastMid=0; //
    uint16 white_num=0;
    uint16 max_column=0;    // 截止列
    uint16 Control_Row=0;   // 打角行
    uint16 L_loselineSum=0; // 左丢边数
    uint16 R_loselineSum=0; // 右丢边数
    uint16 L_track_row=0;   // 左车轮轨迹线上的白点数
    uint16 R_track_row=0;   // 右车轮轨迹线上的白点数
    uint16 L_straight_flag=0;
    uint16 R_straight_flag=0;
    uint16 L_lose_r_num=0;
    uint16 R_lose_L_num=0;
    uint16 Both_lose=0;
    float err_sum=0;
};
struct Guaidian
{
    //图像相关信息
    uint8 row=0;
    uint8 column=0;
    uint8 flag=0;
};
extern struct Guaidian L_l_guai, L_h_guai, R_l_guai, R_h_guai,R_h_guai1, L_h_guai1;  //拐点信息

struct Wandian
{
    //图像相关信息
    uint8 row=0;
    uint8 column=0;
    uint8 flag=0;
};
extern struct Wandian L_l_wan, L_h_wan, R_l_wan, R_h_wan;  //拐点信息
struct YuanSu
{
    uint8 Buxian=0;

    uint8 Shizi=0;

    uint8 Huandao_L=0;
    uint8 Huandao_R=0;

    uint8 Zhangai=0;
    uint8 NO_Zhangai=0;

    uint8 Zebra_cross=0;

    uint8 ramp=0;
    uint8 small_rock=0;


    uint8 Redblock=0;
    uint8 picture = 0;
    uint8 picture_dir = 0;
    uint8 avoid=0;

    uint8 infer = 0;//推理开关
    uint8_t jiansu = 0;//减速标志位

    uint8 supply = 0;//物资
    uint8 weapon = 0;//武器
    uint8 vehicle = 0;//交通工具
};
extern struct YuanSu Flag;

extern struct imageInformation imgInfo;
extern cv::Mat frame, grayFrame, binaryFrame,resizedFrame,flippedFrame,translatedFrame,translationMatrix;
extern uint8_t Image_Zip[LCDH_1][LCDW_1];
extern uint8_t Image_Use[LCDH_1][LCDW_1];
extern PID steering_pid;
extern float Dir_err, Last_Dir_err, Dir_Err[60],D_ERR; // 图像误差
extern char txt[80];
extern float real_distance[60];
extern float curvature,distance,distance_picture,ramp_err,recognize_distance2;
extern uint16_t jump_point,maxkuan_line;
extern int avoid_state,x_err_red;
extern int forward,forward1, red_x_mid,red_y_mid;;
// 前瞻基准行：TRACK 中 forward1 = g_forward_base - Master_Speed/60（接上 forward_base 参数）
extern float g_forward_base;
extern unsigned char Left_Sideline_flag[LCDH_0] ;   //左边线标志位
extern unsigned char Right_Sideline_flag[LCDH_0];  //右边线标志位
// 圆环函数入口日志快照：HLg/HRg；实际视觉逻辑直接读取 Flag.Huandao_*。
extern uint8_t g_huandao_L_geom;
extern uint8_t g_huandao_R_geom;
extern float Yaw_Huandao,Yaw_Huandao_err,yaw_correct,distance_HUAN1,black_ratio,err_picture,real_picture_distance;
extern float g_huandao_l_state5_exit_yaw, g_huandao_r_state5_exit_yaw;
extern int right_num ,r_num,left_num ,l_num ,R_l_lsoe,L_l_lose,picture_first_num,picture_second_num,maxlong_colume,long_max,jump_point1,picture_white,picture_black,red_find_x,red_find_y,red_find_y1;;
extern cv::Point center;
extern cv::Mat lq_frame;
void ImageDeal(void);
// 0=相机打开成功；非 0=失败（TRACK 不得 armed）
int image_init(void);
// 图像新鲜度（审查：空帧/失败不得无限沿用旧 Image_out）
extern int g_camera_ok;
extern uint32_t g_image_frame_id;
extern float g_image_age_ms; // 由控制拍刷新：距上次成功 ImageDeal 的 ms
float Image_AgeMs_Now(void); // 即时年龄
void distance_judge(void);
 void DetectRedBlock(const cv::Mat &search_space,int roi_x,int roi_y,int width,int height);
bool InTrack(int x,int y);
bool GenerateROI(const cv::Point &center, cv::Rect &roi, const cv::Mat &src);
void RedBlockProcess(cv::Mat &src);
int ArgMax(int *arr,int n);
void avoid_process(void);
void Draw_BlackSideline(uint8_t Image_1[LCDH_1][LCDW_1]);
void Find_Sideline(uint8 Start_row, uint8 End_row);
// 64x64 三分类 NCNN 在 image_init() 中初始化；无运行时 fallback。

void debug_log_printf_callback(const char* s);
float Yaw_correct(float current_yaw,float target_yaw);
int real_distance_to_row(float distance) ;
extern float real_distance[60];
float actan_err(float err);

#endif
