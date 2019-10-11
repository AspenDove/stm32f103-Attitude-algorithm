# stm32f103-Attitude-algorithm

该项目使用了MahonyAHRS开源库。该库中使用的姿态融合算法以及向量外积互补滤波十分适用于嵌入式设备。

## 上手指南

以下指南帮助你在你自己的嵌入式设备上运行该项目，进行开发和测试。

### 需要的工具

- TDK MPU9250；
- 可以使用STM32 HAL库进行开发的芯片；
- 合适的开发工具。

### 移植步骤

注意到工程目录下的以下文件：

+ IMU.cpp;IMU.h
  用于初始化MPU9250，完成数据的读取与校准。

+ MahonyAHRS.cpp;MahonyAHRS.h
  使用IMU的数据进行姿态解算，姿态融合算法在这两个文件夹里。
+ main.cpp
  含有main函数，实现任务调度以及逻辑。

微控制器与IMU之间使用I2C通信，为了匹配端口，请打开IMU.h，关注其中的59-61行。将GPIO，SCL，SDA正确配置。

>~~~C++
>#define GPIO     GPIOB
>#define SCL      GPIO_PIN_6
>#define SDA      GPIO_PIN_7
>~~~

在正确连接端口之后，请关注IMU.cpp中的IMU_Init，该函数进行一系列的初始化，因此请保证在读取IMU之前调用该函数。
初始化之后，可以调用IMU_Calibrate进行校准，首先进行的是加速度计校准，采样数由samples指定。在此阶段，请尽量保证IMU静止。接下来进行的是地磁计校准，我将GPIOC，PIN13复位，以打开开发板上的LED，等待2秒后进行校准。

>```C++
>HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
>HAL_Delay(2000);
>```

请在此时将IMU处于尽可能多的方向，同样，采样数由samples指定。
校准完毕，我将LED关闭，以标志结束。

> ~~~C++
> HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
> ~~~

此时即可调用IMU_Measure读取IMU的数据。
请注意由于地磁计的返回频率相对较慢，因此IMU_Measure的返回值标志着地磁计与上次相比是否进行了更新。此标志可以用于MahonyAHRSupdate。

IMU.h引出了以下全局变量：

~~~C++
extern IMU_Accel accel;
extern IMU_Gyro  gyro;
extern IMU_Mag   mag;
extern IMU_Angle angle;
~~~

典型的调用示例为：

~~~C++
bool updated = IMU_Measure();
MahonyAHRSupdate(accel, gyro, mag, &angle, updated);
~~~

这样angle结构体中就得到了解算之后的**弧度制**角度。其中

需要注意，算法中的参数假定该解算的频率为125Hz，如真实的解算频率不同，有可能需要在MahonyAHRS.cpp中修改参数。

### 角度说明

IMU_Angle中包含欧拉角：

~~~C++
typedef struct
{
	float pitch, roll, yaw;
}IMU_Angle;
~~~

经过姿态解算之后，假设你的视线与IMU的x轴正方向同向，头顶沿z轴正方向，那么角度按如下规律变化：

+ pitch：在水平方向上为0，向下倾斜时递增，向上倾斜时递减，范围在$\pm \frac{\pi}{2}$；
+ roll：在水平方向上为0，向右倾斜时递增，向左倾斜时递减，范围在$\pm\frac{\pi}{2}$；
+ yaw：面向地磁南极方向为0，沿z轴负方向看去时，顺时针旋转时递增，逆时针旋转时递减，范围在$\pm\pi$。