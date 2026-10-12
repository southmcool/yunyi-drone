#ifndef GPIO_CONFIG
#define GPIO_CONFIG
// title = # 引脚定义
// 传感器 gpio 统一配置

// ===== 电机 =====
#define MOTOR_FL GPIO_NUM_11
#define MOTOR_FR GPIO_NUM_38
#define MOTOR_BL GPIO_NUM_10
#define MOTOR_BR GPIO_NUM_39

// spi

// ===== IMU SPI =====
#define IMU_CSPIN GPIO_NUM_18
#define IMU_CLKPIN GPIO_NUM_15
#define IMU_MISOPIN GPIO_NUM_16
#define IMU_MOSIPIN GPIO_NUM_17
#define IMU_INT_GPIO GPIO_NUM_21

// ===== PMW3901 SPI =====
#define PWM3901_CSPIN GPIO_NUM_8
#define PWM3901_CLKPIN GPIO_NUM_4
#define PWM3901_MISOPIN GPIO_NUM_5
#define PWM3901_MOSIPIN GPIO_NUM_6

// iic
// ===== VL53L1X I2C =====
#define VL53L1X_SDA GPIO_NUM_2
#define VL53L1X_SCL GPIO_NUM_1

// ===== MAG / BARO I2C =====
#define ATK901_RX_PIN GPIO_NUM_9
#define ATK901_TX_PIN GPIO_NUM_42

#endif