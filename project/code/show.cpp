#include "zf_common_headfile.hpp"
uint8_t key1_state,key2_state,key3_state,key4_state,key5_state,key6_state,long_key;
uint16_t key7_state, key8_state,key7_last_state,key8_last_state;
uint8_t key1_last_state,key2_last_state,key3_last_state,key4_last_state,key5_last_state,key6_last_state;
int8_t oled_flag,last_oled_flag;
int16_t Parameter_flag = 1;
uint16_t esc_duty = 550;
void key_scan(void)
{
    key1_state=key_1.get_level();//下
    key2_state=key_2.get_level();//右
    key3_state=key_3.get_level();//上
    key4_state=key_4.get_level();//左
    key5_state =key_5.get_level();//右按键中键
    key6_state =key_6.get_level();//左按键中键
    key7_state = key_7.convert();//右按键 左
    key8_state = key_8.convert();//右按键 右

    if(key1_state==0&&key1_last_state==1)//下
    {   
         Parameter_flag++;
    }

    if(key2_state==0&&key2_last_state==1)//右
    {
        ips200.clear();
        oled_flag ++;
        if(oled_flag>4) oled_flag = 0;
        Parameter_flag=1;
    }

        if(key3_state==0&&key3_last_state==1)//上
    {  
        Parameter_flag--;
        if(Parameter_flag < 1){
            Parameter_flag = 1;
        }
    }

        if(key4_state==0&&key4_last_state==1) // 左
    {
        ips200.clear();
        Parameter_flag = 0;
        oled_flag--;
        if(oled_flag<0) oled_flag = 0;
    }

           if(key7_state<4000&&key7_last_state>=4000) // 右按键 左
    {   //printf("右按键左按下\n");

        if(Parameter_flag == 1&&oled_flag==4){Flash.debug_rgb_r_min-=5;}
        if(Parameter_flag == 2&&oled_flag==4){Flash.debug_rgb_rg_diff-=5;}
        if(Parameter_flag == 3&&oled_flag==4){Flash.debug_rgb_rb_diff-=5;}

        
        Param_SaveAll();

    }

        if(key8_state<4000&&key8_last_state>=4000) // 右按键 右
    {       
        //printf("右按键右按下\n");
        if(Parameter_flag == 1&&oled_flag==4){Flash.debug_rgb_r_min+=5;}
        if(Parameter_flag == 2&&oled_flag==4){Flash.debug_rgb_rg_diff+=5;}
        if(Parameter_flag == 3&&oled_flag==4){Flash.debug_rgb_rb_diff+=5;}

        Param_SaveAll();
    }

    


    key1_last_state=key1_state;
    key2_last_state=key2_state;
    key3_last_state=key3_state;
    key4_last_state=key4_state;
    key5_last_state=key5_state;
    key6_last_state=key6_state;
    key7_last_state=key7_state;
    key8_last_state=key8_state;          

                    //  if(Flag.Huandao_R==1||Flag.Huandao_R==3|| Flag.Huandao_L==1||Flag.Huandao_L==3||Flag.Huandao_L==5||Flag.Huandao_R==5
                    //  ||Flag.Zebra_cross==4
                    //  ||Flag.Zhangai==2||Flag.ramp==1||Flag.small_rock==1||Flag.small_rock==2)//|| Flag.Danbianqiao!=0 Flag.small_rock!=0||

}


void oled_show(void)
{
    //红色阈值调试界面
if(oled_flag == 4)
{
    if(!resizedFrame.empty())
    {
        cv::Mat debug_hsv, debug_mask1, debug_mask2, debug_mask;
        debug_mask.create(resizedFrame.size(), CV_8UC1);
        debug_mask.setTo(0); // 初始化为黑
        if(debug_mask.empty()) 
        {
            printf("debug_resized is empty\n");
            return;
        }

        for(int y = 0; y < resizedFrame.rows; y++)
        {
            const cv::Vec3b* ptr = resizedFrame.ptr<cv::Vec3b>(y);
            uchar* mask_ptr = debug_mask.ptr<uchar>(y);
            
            for(int x = 0; x < resizedFrame.cols; x++)
            {
                int b = ptr[x][0];
                int g = ptr[x][1];
                int r = ptr[x][2];

                // 这里必须与 DetectRedBlock 中的逻辑完全一致
                if (r > Flash.debug_rgb_r_min && (r - g) > Flash.debug_rgb_rg_diff && (r - b) > Flash.debug_rgb_rb_diff)
                {
                    mask_ptr[x] = 255; // 白色表示检测到红色
                }
            }
        }
            for(int i = 0; i < LCDH_1; i++)
            {
                // 直接拷贝一行数据，效率比逐像素遍历高
                memcpy(Image_IFS[i], debug_mask.ptr<uint8_t>(i), LCDW_1);
            }
            
            ips200.show_gray_image(0, 0, (const uint8_t*)Image_IFS, LCDW_1, LCDH_1, LCDW_1, LCDH_1, 1);
    }
    else{
        printf("resizedFrame is empty\n");
        return;
    }
    sprintf(txt, "R_Min : %d  ", Flash.debug_rgb_r_min);
    ips200.show_string(0, 60, txt);

    sprintf(txt, "R-G Diff : %d  ", Flash.debug_rgb_rg_diff);
    ips200.show_string(0, 80, txt);
        
    sprintf(txt, "R-B Diff: %d ", Flash.debug_rgb_rb_diff);
    ips200.show_string(0, 100, txt);
        

    sprintf(txt,"L_h_guai:%d",L_h_guai.flag);
    ips200.show_string(0,140,txt);

    sprintf(txt,"R_h_guai:%d",R_h_guai.flag);
    ips200.show_string(120,140,txt);

    sprintf(txt,"red_find_x:%d",red_find_x);
    ips200.show_string(0,160,txt);

    sprintf(txt,"Flag.picture:%d",Flag.picture);
    ips200.show_string(120,160,txt);

    sprintf(txt,"red_find_y:%d",red_find_y);
    ips200.show_string(0,180,txt);

    sprintf(txt,"Parameter:%d",Parameter_flag);
    ips200.show_string(0,300,txt);

    sprintf(txt,"oled_page:%d",oled_flag);
    ips200.show_string(130,300,txt);
}

if(oled_flag == 3)
{


    sprintf(txt,"Parameter:%d",Parameter_flag);
    ips200.show_string(0,300,txt);

    sprintf(txt,"oled_page:%d",oled_flag);
    ips200.show_string(130,300,txt);
}

if(oled_flag == 2)
{
    for (int i = 0; i < LCDH_1; i++)
     {
         for (int j = 0; j < LCDW_1; j++) {if (Image_Use[i][j] > 0)Image_IFS[i][j] = 255;else Image_IFS[i][j] = 0;}
     }
     for (int j = 0; j < LCDH_1; j++) { int lx=Left_Sideline[j]; if(lx>=0&&lx<LCDW_1) Image_IFS[j][lx]=0; }
     for (int j = 0; j < LCDH_1; j++) { int rx=Right_Sideline[j]; if(rx>=0&&rx<LCDW_1) Image_IFS[j][rx]=0; }
     for (int j = 0; j < LCDH_1; j++) { int mx=Mid_Line[j]; if(mx>=0&&mx<LCDW_1) Image_IFS[j][mx]=0; }
       ips200.show_gray_image(0,0,(const uint8_t*)Image_IFS,LCDW_1,LCDH_1,LCDW_1,LCDH_1,1);
    
        
        sprintf(txt,"Dir_err:%f  ",Dir_err);
        ips200.show_string(0,60,txt);

         sprintf(txt,"Flag.Zhangai:%d",Flag.Zhangai);
         ips200.show_string(0,80,txt);

        sprintf(txt,"Flag.Redblock:%d:",Flag.Redblock);
        ips200.show_string(0,100,txt);

         sprintf(txt,"Flag.picture:%d:",Flag.picture);
         ips200.show_string(0,120,txt);

         sprintf(txt,"encode_l_total:%d     ",encode_l_total);
         ips200.show_string(0,140,txt);

         sprintf(txt,"encode_r_total:%d     ",encode_r_total);
         ips200.show_string(0,160,txt);




        // sprintf(txt,"Image.column:%d  ",imgInfo.max_column);
        // ips200.show_string(0,160,txt);

        sprintf(txt,"encoder_abs:%d   ",encoder_abs);
        ips200.show_string(0,180,txt);

        sprintf(txt,"jump_point:%d  ",jump_point);
        ips200.show_string(0,200,txt);

        sprintf(txt,"icm_data.gyro_z:%.2f  ",icm_data.gyro_z );
        ips200.show_string(0,220,txt);
        sprintf(txt,"icm_data.gyro_y:%.2f  ",icm_data.gyro_y );
        ips200.show_string(0,240,txt);
        sprintf(txt,"Flag.Zebra_cross:%d ",Flag.Zebra_cross);
        ips200.show_string(0,260,txt);
        sprintf(txt,"dis:%.2f",distance);
        ips200.show_string(150,260,txt);

        sprintf(txt,"Parameter:%d",Parameter_flag);
        ips200.show_string(0,300,txt);

        sprintf(txt,"oled_page:%d",oled_flag);
        ips200.show_string(130,300,txt);
}
 
if(oled_flag ==1){
    for (int i = 0; i < LCDH_1; i++)
     {
         for (int j = 0; j < LCDW_1; j++) {if (Image_Use[i][j] > 0)Image_IFS[i][j] = 255;else Image_IFS[i][j] = 0;}
     }
     for (int j = 0; j < LCDH_1; j++) {
         int lx = Left_Sideline[j];
         if (lx >= 0 && lx < LCDW_1) Image_IFS[j][lx] = 0;
         int rx = Right_Sideline[j];
         if (rx >= 0 && rx < LCDW_1) Image_IFS[j][rx] = 0;
     }
     for (int j = 0; j < LCDH_1; j++) { int mx=Mid_Line[j]; if(mx>=0&&mx<LCDW_1) Image_IFS[j][mx]=0; }
       ips200.show_gray_image(0,0,(const uint8_t*)Image_IFS,LCDW_1,LCDH_1,LCDW_1,LCDH_1,1);
    
                //    sprintf(txt,"yaw:%.1f ",icm_data.yaw);
                //    ips200.show_string(100,0,txt);


                //    sprintf(txt,"err:%.1f ",Yaw_Huandao_err);
                //    ips200.show_string(100,20,txt);

                //    sprintf(txt,"dl1x:%d ",dl1x_distance_raw);
                //    ips200.show_string(100,40,txt);

                //    sprintf(txt,"jump_point:%d ",jump_point);
                //    ips200.show_string(100,60,txt);

                //    sprintf(txt,"Huandao_L:%d",Flag.Huandao_L);
                //    ips200.show_string(0,80,txt);

                //    sprintf(txt,"Huandao_R:%d",Flag.Huandao_R);
                //    ips200.show_string(110,80,txt);




                //    sprintf(txt,"Huandao_L:%d",Flag.Huandao_L);
                //    ips200.show_string(0,80,txt);

                //    sprintf(txt,"Huandao_R:%d",Flag.Huandao_R);
                //    ips200.show_string(110,80,txt);
// //2
//                    sprintf(txt,"maxlong_colume:%.d ",maxlong_colume);
//                    ips200.show_string(100,0,txt);


//                    sprintf(txt,"long_max:%.d",long_max);
//                    ips200.show_string(100,20,txt);

//                    sprintf(txt,"dl1x:%d ",dl1x_distance_raw);
//                    ips200.show_string(100,40,txt);

//                    sprintf(txt,"jump_point:%d ",jump_point);
//                    ips200.show_string(100,60,txt);

//2


                   sprintf(txt,"pic_white:%d  ",picture_white);
                   ips200.show_string(100,0,txt);


                   sprintf(txt,"pic_black:%d  ",picture_black);
                   ips200.show_string(100,20,txt);

                   sprintf(txt,"point1:%d  ",jump_point1);
                   ips200.show_string(100,40,txt);

                   sprintf(txt,"black_ratio:%.3f",black_ratio);
                   ips200.show_string(100,60,txt);


                   sprintf(txt,"Huandao_L:%d",Flag.Huandao_L);
                   ips200.show_string(0,80,txt);

                   sprintf(txt,"Huandao_R:%d",Flag.Huandao_R);
                   ips200.show_string(110,80,txt);




              sprintf(txt,"L_h_guai:%d",L_h_guai.flag);
              ips200.show_string(0,100,txt);

              sprintf(txt,"R_h_guai:%d",R_h_guai.flag);
              ips200.show_string(110,100,txt);

              sprintf(txt,"L_l_guai:%d",L_l_guai.flag);
              ips200.show_string(0,120,txt);

              sprintf(txt,"R_l_guai:%d",R_l_guai.flag);
              ips200.show_string(110,120,txt);


//2
                   sprintf(txt,"m_l_colume:%.d ",maxlong_colume);
                   ips200.show_string(0,140,txt);


                   sprintf(txt,"long_max:%.d",long_max);
                   ips200.show_string(120,140,txt);




            //   sprintf(txt,"L_lose:%d  ",imgInfo.L_loselineSum);
            //   ips200.show_string(0,140,txt);

            //   sprintf(txt,"R_lose:%d  ",imgInfo.R_loselineSum);
            //   ips200.show_string(80,140,txt);

              sprintf(txt,"distance:%.1f",distance);
              ips200.show_string(0,160,txt);

              sprintf(txt,"Dir_err:%.1f",Dir_err);
              ips200.show_string(120,160,txt);

                    //   sprintf(txt,"L_strai:%d",imgInfo.L_straight_flag);
                    //   ips200.show_string(0,180,txt);

                    //   sprintf(txt,"R_strai:%d",imgInfo.R_straight_flag);
                    //   ips200.show_string(120,180,txt);

                   sprintf(txt,"R_row+10:%d     ",real_distance_to_row(real_distance[R_h_guai.row]+10));
                   ips200.show_string(0,180,txt);



                   sprintf(txt,"Picture:%d",Flag.picture);
                   ips200.show_string(110,180,txt);


                      sprintf(txt,"Both_l:%d ",imgInfo.Both_lose);
                      ips200.show_string(0,200,txt);

                      sprintf(txt,"top:%d ",imgInfo.top);
                      ips200.show_string(110,200,txt);

                      sprintf(txt,"Rh.row1:%d ",R_h_guai1.row);
                      ips200.show_string(110,220,txt);

                      sprintf(txt,"Lh.row1:%d ",L_h_guai1.row);
                      ips200.show_string(0,220,txt);


                      sprintf(txt,"Rh.column1:%d ",R_h_guai1.column);
                      ips200.show_string(110,240,txt);

                      sprintf(txt,"Lh.column1:%d ",L_h_guai1.column);
                      ips200.show_string(0,240,txt);




                      sprintf(txt,"Rh.row:%d ",R_h_guai.row);
                      ips200.show_string(110,260,txt);

                      sprintf(txt,"Lh.row:%d ",L_h_guai.row);
                      ips200.show_string(0,260,txt);


                      sprintf(txt,"Rh.column:%d ",R_h_guai.column);
                      ips200.show_string(110,280,txt);

                      sprintf(txt,"Lh.column:%d ",L_h_guai.column);
                      ips200.show_string(0,280,txt);


                    //     sprintf(txt,"picture_num1:%d ",picture_second_num);
                    //   ips200.show_string(110,280,txt);

                    //   sprintf(txt,"picture_num2:%d ",picture_first_num);
                    //   ips200.show_string(0,280,txt);


                     sprintf(txt,"Parameter:%d",Parameter_flag);
                     ips200.show_string(0,300,txt);

                     sprintf(txt,"oled_page:%d",oled_flag);
                     ips200.show_string(130,300,txt);
}
if(oled_flag == 0){
    for (int i = 0; i < LCDH_1; i++)
     {
         for (int j = 0; j < LCDW_1; j++) {if (Image_Use[i][j] > 0)Image_IFS[i][j] = 255;else Image_IFS[i][j] = 0;}
     }
     for (int j = 0; j < LCDH_1; j++) {
         int lx = Left_Sideline[j];
         if (lx >= 0 && lx < LCDW_1) Image_IFS[j][lx] = 0;
         int rx = Right_Sideline[j];
         if (rx >= 0 && rx < LCDW_1) Image_IFS[j][rx] = 0;
     }
     for (int j = 0; j < LCDH_1; j++) { int mx=Mid_Line[j]; if(mx>=0&&mx<LCDW_1) Image_IFS[j][mx]=0; }
       ips200.show_gray_image(0,0,(const uint8_t*)Image_IFS,LCDW_1,LCDH_1,LCDW_1,LCDH_1,1);
    
                   sprintf(txt,"yaw:%.1f ",icm_data.yaw);
                   ips200.show_string(100,0,txt);


                   sprintf(txt,"err:%.1f ",Yaw_Huandao_err);
                   ips200.show_string(100,20,txt);

                   sprintf(txt,"dl1x:%d ",dl1x_distance_raw);
                   ips200.show_string(100,40,txt);

                   sprintf(txt,"jump_point:%d ",jump_point);
                   ips200.show_string(100,60,txt);

                //    sprintf(txt,"Huandao_L:%d",Flag.Huandao_L);
                //    ips200.show_string(0,80,txt);

                //    sprintf(txt,"Huandao_R:%d",Flag.Huandao_R);
                //    ips200.show_string(110,80,txt);




                //    sprintf(txt,"Huandao_L:%d",Flag.Huandao_L);
                //    ips200.show_string(0,80,txt);

                //    sprintf(txt,"Huandao_R:%d",Flag.Huandao_R);
                //    ips200.show_string(110,80,txt);
// //2
//                    sprintf(txt,"maxlong_colume:%.d ",maxlong_colume);
//                    ips200.show_string(100,0,txt);


//                    sprintf(txt,"long_max:%.d",long_max);
//                    ips200.show_string(100,20,txt);

//                    sprintf(txt,"dl1x:%d ",dl1x_distance_raw);
//                    ips200.show_string(100,40,txt);

//                    sprintf(txt,"jump_point:%d ",jump_point);
//                    ips200.show_string(100,60,txt);

//2


                //    sprintf(txt,"pic_white:%d  ",picture_white);
                //    ips200.show_string(100,0,txt);


                //    sprintf(txt,"pic_black:%d  ",picture_black);
                //    ips200.show_string(100,20,txt);

                //    sprintf(txt,"point1:%d  ",jump_point1);
                //    ips200.show_string(100,40,txt);

                //    sprintf(txt,"black_ratio:%.3f",black_ratio);
                //    ips200.show_string(100,60,txt);


                   sprintf(txt,"Huandao_L:%d",Flag.Huandao_L);
                   ips200.show_string(0,80,txt);

                   sprintf(txt,"Huandao_R:%d",Flag.Huandao_R);
                   ips200.show_string(110,80,txt);




              sprintf(txt,"L_h_guai:%d",L_h_guai.flag);
              ips200.show_string(0,100,txt);

              sprintf(txt,"R_h_guai:%d",R_h_guai.flag);
              ips200.show_string(110,100,txt);

              sprintf(txt,"L_l_guai:%d",L_l_guai.flag);
              ips200.show_string(0,120,txt);

              sprintf(txt,"R_l_guai:%d",R_l_guai.flag);
              ips200.show_string(110,120,txt);






              sprintf(txt,"L_lose:%d  ",imgInfo.L_loselineSum);
              ips200.show_string(0,140,txt);

              sprintf(txt,"R_lose:%d  ",imgInfo.R_loselineSum);
              ips200.show_string(80,140,txt);

              sprintf(txt,"distance:%.1f",distance);
              ips200.show_string(0,160,txt);

              sprintf(txt,"Dir_err:%.1f",Dir_err);
              ips200.show_string(120,160,txt);

                    //   sprintf(txt,"L_strai:%d",imgInfo.L_straight_flag);
                    //   ips200.show_string(0,180,txt);

                    //   sprintf(txt,"R_strai:%d",imgInfo.R_straight_flag);
                    //   ips200.show_string(120,180,txt);

                   sprintf(txt,"L_row+10:%d     ",real_distance_to_row(real_distance[L_h_guai.row]+10));
                   ips200.show_string(0,180,txt);



                   sprintf(txt,"Picture:%d",Flag.picture);
                   ips200.show_string(110,180,txt);


                      sprintf(txt,"Both_l:%d ",imgInfo.Both_lose);
                      ips200.show_string(0,200,txt);

                      sprintf(txt,"top:%d ",imgInfo.top);
                      ips200.show_string(110,200,txt);

                      sprintf(txt,"real_guai:%.1f ",real_distance[MAX(R_h_guai.row,L_h_guai.row)]);
                      ips200.show_string(110,220,txt);

                      sprintf(txt,"Lh.row1:%d ",L_h_guai1.row);
                      ips200.show_string(0,220,txt);


                      sprintf(txt,"Rh.column1:%d ",R_h_guai1.column);
                      ips200.show_string(110,240,txt);

                      sprintf(txt,"Lh.column1:%d ",L_h_guai1.column);
                      ips200.show_string(0,240,txt);




                      sprintf(txt,"Rh.row:%d ",R_h_guai.row);
                      ips200.show_string(110,260,txt);

                      sprintf(txt,"Lh.row:%d ",L_h_guai.row);
                      ips200.show_string(0,260,txt);


                      sprintf(txt,"Rh.column:%d ",R_h_guai.column);
                      ips200.show_string(110,280,txt);

                      sprintf(txt,"Lh.column:%d ",L_h_guai.column);
                      ips200.show_string(0,280,txt);


                    //     sprintf(txt,"picture_num1:%d ",picture_second_num);
                    //   ips200.show_string(110,280,txt);

                    //   sprintf(txt,"picture_num2:%d ",picture_first_num);
                    //   ips200.show_string(0,280,txt);


                     sprintf(txt,"Parameter:%d",Parameter_flag);
                     ips200.show_string(0,300,txt);

                     sprintf(txt,"oled_page:%d",oled_flag);
                     ips200.show_string(130,300,txt);

}
 
}
