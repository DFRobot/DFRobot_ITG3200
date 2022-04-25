# DFRobot_ITG3200

- [中文版](./README_CN.md)

DFrobot的高集成度低成本的10自由度传感器,集合了ADXL345加速度计、QMC5883L磁罗盘、ITG3205陀螺仪以及BMP280气压传感器和温度传感器。内置了低噪声的低压线性稳压器，还扩展了电源电压输入范围，支持3V-5V电源电压。同时，10自由度IMU也可以直接和Arduino控制板兼容。

![正反面svg效果图](./resources/images/SEN0140.png)

## Product Link (https://www.dfrobot.com/product-818.html)

    SKU: SEN0140

## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary

这个库提供了ITG3200陀螺仪获取X,Y,Z三轴的数据的库和例程

## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++

    /**
     * @fn begin
     * @param _SRateDiv Sample rate divider
     * @param _Range Gyro Full Scale Range
     * @param _filterBW Digital Low Pass Filter BandWidth and SampleRate
     * @param _ClockSrc Clock Source - user parameters
     * @param _ITGReady 使能中断寄存器
     * @param _INTRawDataReady 使能数据准备寄存器
     * @return bool类型，表示返回初始化的状态
     * @retval true 初始化成功
     * @retval false 初始化失败
     */
    bool begin(uint8_t _SRateDiv = NOSRDIVIDER, uint8_t _Range = RANGE2000, uint8_t _filterBW = BW256_SR8, uint8_t _ClockSrc = PLL_XGYRO_REF, bool _ITGReady = true, bool _INTRawDataReady = true);

    /**
     * @fn setSamplerateDiv
     * @brief Set the gyroscope sampling rate
     * @param _SampleRate
     * @n     Sample rate divider: 0 to 255
     * @n     calculate:Fsample = Finternal / (_SampleRate+1)
     * @n     Finternal is 1kHz or 8kHz
     */
    void setSamplerateDiv(uint8_t _SampleRate);

    /**
     * @fn getSamplerateDiv
     * @brief Get the gyroscope sampling rate
     * @return uint8_t
     */
    uint8_t getSamplerateDiv(void);

    /**
     * @fn setIntlogicLvl
     * @brief Interrupt Configuration
     * @param bool
     * @n ACTIVE_ONHIGH 中断高电平有效
     * @n ACTIVE_ONLOW 中断低电平有效
     */
    void setIntlogicLvl(bool _State);

    /**
     * @fn isIntactiveOnlow
     * @brief 模块是否是中断触发时为低电平
     * @return bool
     * @retval true 中断高电平有效
     * @retval false 中断低电平有效
     */
    bool isIntactiveOnlow(void);

    /**
     * @fn setIntdriveType
     * @brief 设置中断引脚的状态
     * @param _State 引脚的状态
     * @n OPEN_DRAIN 开漏输出
     * @n PUSH_PULL 推挽输出
     */
    void setIntdriveType(bool _State);

    /**
     * @fn isIntopenDrain
     * @brief 中断引脚是否是开漏输出
     * @return bool
     * @retval true 是
     * @retval false 否
     */
    bool isIntopenDrain(void);

    /**
     * @fn setItgready
     * @brief if enable interrupt when device is ready (PLL ready after changing clock source)
     * @param _State
     * @n     true   打开
     * @n     false  关闭
     */
    void setItgready(bool _State);

    /**
     * @fn isItgreadyOn
     * @brief if the interrupt switch has been turned on
     * @return true yes
     * @return false no
     */
    bool isItgreadyOn(void);

    /**
     * @fn isItgready
     * @brief 中断是否使能
     * @return bool
     * @retval true 使能
     * @retval false 不使能
     */
    bool isItgready(void);

    /**
     * @fn isRawdataReady
     * @brief 原始数据是否准备好
     * @return bool
     * @retval true 已经准备好
     * @retval false 没有准备好
     */
    bool isRawdataReady(void);

    /**
     * @fn readTemp
     * @brief 获取板子温度
     * @param _Temp 存储温度
     */
    void readTemp(float *_Temp);

    /**
     * @fn zeroCalibrate
     * @brief assuming gyroscope is stationary (updates XYZ offsets)
     * @param totSamples
     * @param sampleDelayMS
     */
    void zeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS);

    /**
     * @fn readGyro(float *_GyroX, float *_GyroY, float *_GyroZ)
     * @brief read deg/sec calibrated & ScaleFactor
     * @param _GyroX
     * @param _GyroY
     * @param _GyroZ
     */
    void readGyro(float *_GyroX, float *_GyroY, float *_GyroZ);

    /**
     * @fn readGyro(float *_GyroXYZ)
     * @brief read deg/sec calibrated & ScaleFactor
     * @param _GyroXYZ
     */
    void readGyro(float *_GyroXYZ);          

    /**
     * @fn reset
     * @brief after reset all registers have default values
     */
    void reset(void);

    /**
     * @fn isLowpower
     * @brief 查看传感器是否处于低功耗模式
     * @return bool
     * @retval true 是
     * @retval false 否
     */
    bool isLowpower(void);

    /**
     * @fn setPowerMode
     * @brief 设置功耗模式
     * @param _State 模式选择
     * @n     NORMAL
     * @n     STANDBY
     */
    void setPowermode(bool _State);

    /**
     * @fn isXgyroStandby
     * @brief 陀螺仪X方向上的数据是否已经准备好
     * @return bool
     * @retval true 准备好了
     * @retval false 没准备好
     */
    bool isXgyroStandby(void);

    /**
     * @fn isYgyroStandby
     * @brief 陀螺仪Y方向上的数据是否已经准备好
     * @return bool
     * @retval true 准备好了
     * @retval false 没准备好
     */
    bool isYgyroStandby(void);

    /**
     * @fn isZgyroStandby
     * @brief 陀螺仪Z方向上的数据是否已经准备好
     * @return bool
     * @retval true 没准备好
     * @retval false 没准备好
     */
    bool isZgyroStandby(void);

    /**
     * @fn setXgyroStandby
     * @brief 设置X方向待机模式，待机模式将不会获取X方向的数据
     * @param _Status
     * @n     NORMAL 使能
     * @n     STANDBY 不使能
     */
    void setXgyroStandby(bool _Status);

    /**
     * @fn setYgyroStandby
     * @brief 设置Y方向待机模式，待机模式将不会获取X方向的数据
     * @param _Status
     * @n     NORMAL 使能
     * @n     STANDBY 不使能
     */
    void setYgyroStandby(bool _Status);

    /**
     * @fn setZgyroStandby
     * @brief 设置Z方向待机模式，待机模式将不会获取X方向的数据
     * @param _Status
     * @n     NORMAL 使能
     * @n     STANDBY 不使能
     */
    void setZgyroStandby(bool _Status);

    /**
     * @fn setFilterBW
     * @brief 设置滤波器带宽
     * @param _BW 带宽
     */
    void setFilterBW(uint8_t _BW);

    /**
     * @fn getFilterBW
     * @brief 获取滤波器带宽
     * @return uint8_t
     */
    uint8_t getFilterBW(void);

    /**
     * @fn getFSrange
     * @brief 获取陀螺仪量程
     * @return uint8_t
     */
    uint8_t getFSrange(void);

    /**
     * @fn isLatchuntilCleared
     * @brief if Latch mode is latch until interrupt is cleared
     * @return true latch until interrupt is cleared
     * @return false 50us pulse
     */
    bool isLatchuntilCleared(void);

    /**
     * @fn isAnyregClrmode
     * @brief if Latch clear method is any register read
     * @return true any register read
     * @return false status register read only
     */
    bool isAnyregClrmode(void);

    /**
     * @fn getClocksource
     * @brief 获取时钟源
     * @return uint8_t
     */
    uint8_t getClocksource(void);

```

## Compatibility

主板               | 通过  | 未通过   | 未测试   | 备注
------------------ | :----------: | :----------: | :---------: | -----
Arduino uno        |      √       |              |             | 
Mega2560        |      √       |              |             | 
Leonardo        |      √       |              |             | 
ESP32           |      √       |              |             | 
ESP8266           |      √       |              |             | 
Micro:bit           |      √       |              |             | 

## History

- 2022/3/2 - 2.0.0 版本

## Credits

Written by Peng Kaixing(kaixing.peng@dfrobot.com), 2020. (Welcome to our [website](https://www.dfrobot.com/))