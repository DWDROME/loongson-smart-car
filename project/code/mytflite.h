#ifndef _MY_TFLITE_H_
#define _MY_TFLITE_H_

#include <opencv2/core/mat.hpp>
using cv::Mat;

// 4m 口径：60x60 输入，3 类 materials/traffic/weapon
void tflite_init(void);
bool tflite_run(Mat img, int &class_idx, float &confidence, long &infer_us,
                float class_probs[3] = nullptr);
const char *tflite_get_label(int class_idx);

#endif
