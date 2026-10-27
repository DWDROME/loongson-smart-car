# 走马观碑智能车｜LS2K0300

面向“走马观碑”全元素赛道的智能车代码整理。系统基于龙芯 LS2K0300、摄像头、编码器和 IMU，将赛道图像处理、元素状态机与差速闭环控制连接起来；同时保留目标板识别和红砖绕行的相关流程。

本仓库整理自项目团队提供的工程快照，作为代码阅读与交接资料。赛事成绩、团队成员与个人分工待补充。

## 系统概览

```text
摄像头 → 图像缩放与二值化 → 边线、中线与元素状态机 → 方向偏差
                                                       ↓
                      电机 PWM ← 左右轮速度环 ← 差速控制 ← 图像方向环
                                  ↑ 编码器          ↑ IMU
```

- **赛道视觉**：在 `image.cpp` 中处理赛道边线、前瞻、圆环、斑马线和坡道等元素。
- **运动控制**：图像方向环、转向中环和左右独立速度环组成控制链，速度环采用前馈与 PI。
- **识别与绕行**：提供目标板识别接口、红色目标分流与红砖绕行状态机；分类模型另行准备。
- **调试工具**：包含参数管理、比赛操作、图像采集、硬件观察和部分逻辑测试。

## 目录

```text
project/
├── code/             视觉、控制、参数、图传与绕行代码
└── user/             程序入口、CMake 和调试工具
    └── tools/        参数、比赛操作、数据处理及测试工具
licenses/             第三方组件许可证副本
```

主要入口：`project/user/main.cpp`；视觉处理：`project/code/image.cpp`；控制：`project/code/control.cpp`；红砖相关逻辑：`project/code/detour/`。

## 环境与依赖

工程面向 Linux / LoongArch64，原有 CMake 配置使用龙芯交叉工具链、OpenCV 和 NCNN；外设接口依赖逐飞 LS2K0300 开源库。按原工程结构，逐飞库位于仓库根目录 `libraries/`，工具链与 OpenCV 路径见 `project/user/cross.cmake` 和 `project/user/CMakeLists.txt`。

本仓库仅发布项目代码：**分类模型、龙邱相机库及其头文件、授权待确认的龙邱 NCNN 封装，以及第三方预编译库均未包含。** 使用者需自行取得对应依赖并按接口接入。源码保留原有构建配置，用于说明工程结构；补齐依赖并适配工具链后方可构建。

`project/user/build.sh` 为原工程脚本，会清理 `project/out` 并向配置的车端上传程序与模型；运行前应先检查脚本中的环境、地址和目标设备。`project/user/tools/params_config.local.txt` 为提供的参数快照，实际使用需按车辆和场地重新标定。

## 模型迁移与部署

交接文档记录了目标板分类模型的迁移路径：使用车端相机采集的数据训练 PyTorch 模型，导出 ONNX，再转换为 NCNN 的 `.param` / `.bin`，最后在 LS2K0300 上接入并验证。导出和转换时应逐步核对输入形状与推理输出。

原工程的模型接口采用 `1×3×64×64` 输入和三类输出（`supplies`、`vehicle`、`weapon`）。车端从原图截取 80×80 BGR 区域，缩放至 64×64，再转 RGB 并按模型约定归一化。更换模型时，须同步模型接口、`image.cpp` 的裁剪位置、`ncnn_model_smoke` 的参考结果，以及部署脚本中的模型文件配置。

仓库公开了[数据整理脚本](project/user/tools/prepare_stele_dataset.py)、[板端模型自检](project/user/tools/ncnn_model_smoke.cpp)和[数据集评估入口](project/user/tools/ncnn_dataset_eval.cpp)。上述数据整理脚本针对官方六类数据，交接文档建议另用车端相机采集适合现场的数据；训练与格式转换脚本、模型文件以及所需厂商封装未随本仓库发布，板端验证须先补齐依赖。完整思路参见[交接文档第 8 节](https://feishu.doubao.com/docx/PInzdUMJloZlzaxRwN5cNY3Xn6g)。

## 交接与当前状态

[走马观碑交接文档](https://feishu.doubao.com/docx/PInzdUMJloZlzaxRwN5cNY3Xn6g)介绍了硬件、视觉、控制和调车流程。该文档描述了后续参数快照，具体行为以本仓库代码及实际运行配置为准。

随附参数快照中 `red_brick_enable=0`、`picture_roi_enable=0`。交接文档记录红砖分流和确认逻辑需要重新检查，目标板触发距离需按比赛现场重新标定；圆环出环与整条赛道连续运行亦有后续验证工作。文档还指出小石头处理入口存在问题，使用前需检查。

## 来源与许可

本项目基于逐飞 LS2K0300 相关库开发；相机及部分推理接口来自龙邱科技，原文件中的版权与授权说明应予保留。`project/code/ww_transmission.*` 标注为 Blockingsys 的 Wuwu 开源库组件，使用 GPL-3.0-or-later；完整许可证见 [licenses/GPL-3.0.txt](licenses/GPL-3.0.txt)。本仓库其余文件保留原有版权标识；未附带的第三方组件以各自许可为准。
