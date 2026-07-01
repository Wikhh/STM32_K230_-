# STM32_K230_-激光追踪系统
项目的核心目标是设计并实现一套红绿色激光自动追随与靶标锁定的闭环控制系统。系统通过摄像头自动识别空间中的A4纸黑色矩形靶标，并在靶标范围内实时检测、捕捉红、绿两色激光点，利用两者的空间相对位置偏差进行解算，最终驱动二维云台运动，实现绿色激光点对红色激光目标点的精准动态追随与重合。

本项目选用K230作为视觉识别部分，STM32作为云台主控部分

<img width="684" height="706" alt="接线图" src="https://github.com/user-attachments/assets/13826597-fad4-4d13-8a9e-5e54383fc089" />




以上为硬件连接图
在STM32中设计了滤波算法和PID算法，K230中设计了低通滤波与串口坐标发送



<img width="351" height="220" alt="image" src="https://github.com/user-attachments/assets/e9803dc7-7037-4181-bb9e-e989124cc7ee" />



项目中STM32_3为STM32代码，zonghe333为K230代码
