/*!
 * @file DFRobot_ITG3200.h
 * @brief 一个陀螺仪传感器库
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author PengKaixing(kaixing.peng@dfrobot.com)
 * @version  V2.0.0
 * @date  2022-2-28
 * @url https://github.com/DFRobot/DFRobot_ITG3200
 */

#ifndef ITG3200_h
#define ITG3200_h
#include <Arduino.h>
#include <Wire.h>

// 50ms from gyro startup + 20ms register r/w startup
#define GYROSTART_UP_DELAY 70 

/* ---- Registers ---- */
#define WHO_AM_I 0x00   // RW   SETUP: I2C address
#define SMPLRT_DIV 0x15 // RW   SETUP: Sample Rate Divider
#define DLPF_FS 0x16    // RW   SETUP: Digital Low Pass Filter/ Full Scale range
#define INT_CFG 0x17    // RW   Interrupt: Configuration
#define INT_STATUS 0x1A // R	Interrupt: Status
#define TEMP_OUT 0x1B   // R	SENSOR: Temperature 2bytes
#define GYRO_XOUT 0x1D  // R	SENSOR: Gyro X 2bytes
#define GYRO_YOUT 0x1F  // R	SENSOR: Gyro Y 2bytes
#define GYRO_ZOUT 0x21  // R	SENSOR: Gyro Z 2bytes
#define PWR_MGM 0x3E    // RW	Power Management

/* ---- bit maps ---- */
#define DLPFFS_FS_SEL 0x18           // 00011000
#define DLPFFS_DLPF_CFG 0x07         // 00000111
#define INTCFG_ACTL 0x80             // 10000000
#define INTCFG_OPEN 0x40             // 01000000
#define INTCFG_LATCH_INT_EN 0x20     // 00100000
#define INTCFG_INT_ANYRD_2CLEAR 0x10 // 00010000
#define INTCFG_ITG_RDY_EN 0x04       // 00000100
#define INTCFG_RAW_RDY_EN 0x01       // 00000001
#define INTSTATUS_ITG_RDY 0x04       // 00000100
#define INTSTATUS_RAW_DATA_RDY 0x01  // 00000001
#define PWRMGM_HRESET 0x80           // 10000000
#define PWRMGM_SLEEP 0x40            // 01000000
#define PWRMGM_STBY_XG 0x20          // 00100000
#define PWRMGM_STBY_YG 0x10          // 00010000
#define PWRMGM_STBY_ZG 0x08          // 00001000
#define PWRMGM_CLK_SEL 0x07          // 00000111

/************************************/
/*    REGISTERS PARAMETERS    */
/************************************/
// Sample Rate Divider
#define NOSRDIVIDER 0 // default    FsampleHz=SampleRateHz/(divider+1)
// Gyro Full Scale Range
#define RANGE2000 3 // default
// Digital Low Pass Filter BandWidth and SampleRate
#define BW256_SR8 0 // default    256Khz BW and 8Khz SR
#define BW188_SR1 1
#define BW098_SR1 2
#define BW042_SR1 3
#define BW020_SR1 4
#define BW010_SR1 5
#define BW005_SR1 6
// Interrupt Active logic lvl
#define ACTIVE_ONHIGH 0 // default
#define ACTIVE_ONLOW 1
// Interrupt drive type
#define PUSH_PULL 0 // default
#define OPEN_DRAIN 1
// Interrupt Latch mode
#define PULSE_50US 0 // default
#define UNTIL_INT_CLEARED 1
// Interrupt Latch clear method
#define READ_STATUSREG 0 // default
#define READ_ANYREG 1
// Power management
#define NORMAL 0 // default
#define STANDBY 1
// Clock Source - user parameters
#define INTERNALOSC 0 // default
#define PLL_XGYRO_REF 1
#define PLL_YGYRO_REF 2
#define PLL_ZGYRO_REF 3
#define PLL_EXTERNAL32 4 // 32.768 kHz
#define PLL_EXTERNAL19 5 // 19.2 Mhz

class DFRobot_ITG3200{
  public:
    DFRobot_ITG3200(TwoWire *pWire = &Wire, uint8_t I2C_addr = 0x68);

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
    float gains[3];
    int offsets[3];
    float polarities[3];
  private:
    void setClocksource(uint8_t _CLKsource);
    void setFSrange(uint8_t _Range);
    void readGyrorawCal(int *_GyroX, int *_GyroY, int *_GyroZ);
    void readGyrorawCal(int *_GyroXYZ);
    void readGyroraw(int *_GyroX, int *_GyroY, int *_GyroZ);
    void readGyroraw(int *_GyroXYZ);
    void setGains(float _Xgain, float _Ygain, float _Zgain);
    void setRevpolarity(bool _Xpol, bool _Ypol, bool _Zpol);
    void setOffsets(int _Xoffset, int _Yoffset, int _Zoffset);
    void setRawdataReady(bool _State);
    bool isRawdataReadyon(void);
    void writeData(uint8_t reg, void *pdata, uint8_t len);
    int16_t readData(uint8_t reg, uint8_t *data, uint8_t len);
    TwoWire *_pWire;
    uint8_t _I2C_addr;
    uint8_t _buff[6];
};
#endif