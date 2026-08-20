# EKF-PX4 / UAV Navigation Estimation Project

一个面向 PX4 估计器、姿态解算和传感器融合研究的工程项目，主要用于学习和验证 EKF（Extended Kalman Filter）在无人机导航与状态估计中的应用。

本项目参考了 PX4 相关状态估计的设计思路，适合用于：
- PX4 EKF 算法学习
- 无人机姿态估计研究
- IMU 与传感器融合实验
- 状态估计与导航算法调试
- 研究型开发与代码分析

---

## 项目简介

本仓库的目标是理解和验证 PX4 中的状态估计方法，重点关注：

- IMU 数据处理
- 姿态角估计
- 速度与位置融合
- 传感器噪声与漂移分析
- EKF 状态传播与更新
- 无人机导航相关基础实现

该项目适用于个人研究、课程实验、无人机算法调试以及 PX4 代码学习。

---

## 目录结构

```text
PX4_vs/
├── AlphaFilter/            # 过滤器相关实现
├── EKF/                    # EKF 核心模块
├── geo/                    # 地理坐标与位置相关计算
├── lib/                    # 通用库与数学库
├── px4_platform_common/     # 平台通用代码
├── ecl.h                   # 统一接口 / 头文件
├── ekf_wrapper.cpp         # EKF 封装实现
├── ekf_wrapper.h           # EKF 封装头文件
├── main.cpp                # 程序入口
├── CMakeLists.txt          # 构建配置
├── README.md               # 项目说明
└── ...
```

---

## 主要功能

- EKF 状态初始化
- IMU 模拟输入与更新
- 姿态输出（Roll / Pitch / Yaw）
- 基于 PX4 风格的估计器模块组织
- 适合实验性算法研究和调试
- 支持通过日志输出进行估计结果分析

---

## 环境要求

建议使用以下环境：

- C++17
- CMake
- GCC / Clang
- Linux / Windows / WSL
- Python（可选，用于辅助脚本或数据处理）

---

## 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/your-username/PX4_vs.git
cd PX4_vs
```

### 2. 创建构建目录

```bash
mkdir build
cd build
cmake ..
make -j4
```

如果当前工程在你的开发环境中使用的是其他编译方式，也可以按实际项目结构进行编译。

### 3. 运行程序

```bash
./main
```

如果项目中存在不同测试入口，可根据实际文件结构选择对应执行文件。

---

## 运行示例

程序中包含动态姿态解算测试逻辑，输出示例如下：

```text
--- 开始 EKF 动态姿态解算测试 ---
Time:   1.0 s | Roll: 0.0000, Pitch: 0.0000, Yaw: 0.0000
Time:   2.0 s | Roll: 0.0102, Pitch: -0.0041, Yaw: 0.2521
Time:   3.0 s | Roll: 0.0188, Pitch: -0.0093, Yaw: 0.4315
...
--- 测试运行结束 ---
```

这类输出可用于观察 EKF 对姿态变化的跟踪情况。

---

## 代码说明

### main.cpp
程序入口，负责：
- 初始化 EKF
- 模拟时间推进
- 调用状态更新函数
- 输出姿态角结果

### ekf_wrapper.h / ekf_wrapper.cpp
用于对底层 EKF 进行封装，便于在实验环境中直接调用。

### EKF/
包含与 PX4 估计器相关的核心实现代码，例如：
- 状态预测
- 观测更新
- 协方差处理
- 传感器融合相关模块

### geo/
包含地理坐标相关计算与导航辅助函数。

### lib/
包含通用数学库与依赖模块。

---

## 当前项目状态

本项目偏向研究与实验性质，主要用于：

- 学习 PX4 EKF 的实现思路
- 调试状态估计流程
- 验证姿态更新逻辑
- 作为后续无人机导航开发的基础代码

目前属于“开发中 / 研究型工程”，适合持续迭代与扩展。

---

## 后续规划

后续可进一步扩展，包括但不限于：

- 增加更完整的编译脚本
- 提供参数配置支持
- 增加更稳定的测试用例
- 改进输出日志和可视化能力
- 扩展真实传感器输入接口
- 与 PX4 仿真环境联合调试

---

## 参考与声明

本项目参考了 PX4 的 EKF 和状态估计相关设计思路，代码结构和实现方式受 PX4 ECL 设计影响较大。  
使用本项目时，请遵守相应开源代码协议与使用规范，尤其在二次开发和商业使用时需进一步确认许可要求。

---

## 许可证

本项目当前未明确声明许可证，若需要公开发布，建议在正式发布前确认是否需要加入：
- MIT License
- Apache 2.0
- GPL 等其他协议

如果你希望开源发布，建议尽早补充 LICENSE 文件。

---

## 贡献

欢迎提出问题、反馈建议或提交改进代码。  
如果你希望参与项目开发，建议先阅读 EKF 相关实现和接口设计，再进行调试与扩展。

---

## 联系方式

如需交流项目内容、代码问题或实验思路，可通过 GitHub Issue 或其他联系方式进行讨论。

---

## License

This project is currently without an explicit license declaration.  
If you intend to publish it publicly, please add an appropriate open-source license such as MIT or Apache 2.0 before release.
