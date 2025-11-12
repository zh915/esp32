/*
 * ESPRESSIF MIT 许可证
 *
 * 版权所有 (c) 2019 <乐鑫信息科技（上海）有限公司>
 *
 * 特此授予在所有 ESPRESSIF 系统产品上使用本软件的许可，在这种情况下，
 * 任何获得本软件及相关文档文件（以下简称“软件”）副本的人，都可以免费无限制地处理本软件，
 * 包括但不限于使用、复制、修改、合并、出版、发行、再许可和/或出售软件副本，
 * 以及允许获得软件的人这样做，但须遵守以下条件：
 *
 * 上述版权声明和本许可声明应包含在所有副本或软件的重要部分中。
 *
 * 本软件按“原样”提供，不提供任何形式的担保，无论是明示的还是暗示的，
 * 包括但不限于适销性、特定用途适用性和非侵权性的担保。在任何情况下，
 * 作者或版权持有人均不对任何索赔、损害或其他责任承担责任，无论是在合同诉讼、
 * 侵权行为或其他方面，由软件或软件的使用或其他交易引起的、与之相关的。
 *
 */

#ifndef _AUDIO_BOARD_DEFINITION_H_
#define _AUDIO_BOARD_DEFINITION_H_

#include "driver/touch_pad.h"

/**
 * SD卡配置（若未使用，禁用功能）
 */
#define FUNC_SDCARD_EN              (-1)  // 禁用SD卡
#define SDCARD_OPEN_FILE_NUM_MAX    (5)    // SD卡最大打开文件数
#define SDCARD_INTR_GPIO            (-1)   // SD卡中断GPIO（未使用）
#define SDCARD_PWR_CTRL             (-1)   // SD卡电源控制（未使用）
#define ESP_SD_PIN_CLK              (-1)   // SD卡时钟引脚（未使用）
#define ESP_SD_PIN_CMD              (-1)   // SD卡命令引脚（未使用）
#define ESP_SD_PIN_D0               (-1)   // SD卡数据0引脚（未使用）
#define ESP_SD_PIN_D1               (-1)   // SD卡数据1引脚（未使用）
#define ESP_SD_PIN_D2               (-1)   // SD卡数据2引脚（未使用）
#define ESP_SD_PIN_D3               (-1)   // SD卡数据3引脚（未使用）
#define ESP_SD_PIN_D4               (-1)   // SD卡数据4引脚（未使用）
#define ESP_SD_PIN_D5               (-1)   // SD卡数据5引脚（未使用）
#define ESP_SD_PIN_D6               (-1)   // SD卡数据6引脚（未使用）
#define ESP_SD_PIN_D7               (-1)   // SD卡数据7引脚（未使用）
#define ESP_SD_PIN_CD               (-1)   // SD卡检测引脚（未使用）
#define ESP_SD_PIN_WP               (-1)   // SD卡写保护引脚（未使用）

// #define ESP_SD_PIN_CLK            GPIO_NUM_14  // 注释：SD卡时钟引脚实际定义（当前禁用）
// #define ESP_SD_PIN_CMD            GPIO_NUM_15  // 注释：SD卡命令引脚实际定义（当前禁用）
// #define ESP_SD_PIN_D0             GPIO_NUM_2   // 注释：SD卡数据0引脚实际定义（当前禁用）
// #define ESP_SD_PIN_D1             GPIO_NUM_4   // 注释：SD卡数据1引脚实际定义（当前禁用）
// #define ESP_SD_PIN_D2             GPIO_NUM_12  // 注释：SD卡数据2引脚实际定义（当前禁用）
// #define ESP_SD_PIN_D3             GPIO_NUM_13  // 注释：SD卡数据3引脚实际定义（当前禁用）


/**
 * @brief LED功能定义
 */
#define FUNC_SYS_LEN_EN           (1)          // 启用系统LED功能
//#define GREEN_LED_GPIO            GPIO_NUM_22  // 注释：绿色LED引脚定义（当前未启用）


/**
 * @brief 音频编解码器芯片功能定义
 */
#define FUNC_AUDIO_CODEC_EN       (1)          // 启用音频编解码器功能
#define AUXIN_DETECT_GPIO         (-1)         // AUX输入检测引脚（未使用）
#define HEADPHONE_DETECT          (-1)         // 耳机检测引脚（未使用）
#define PA_ENABLE_GPIO            GPIO_NUM_NC  // 功率放大器使能引脚（未连接）
#define CODEC_ADC_I2S_PORT        ((i2s_port_t)0)                  // 编解码器ADC的I2S端口
#define CODEC_ADC_BITS_PER_SAMPLE ((i2s_data_bit_width_t)16)       // 16位采样
#define CODEC_ADC_SAMPLE_RATE     (16000)                          // 采样率16000Hz
#define RECORD_HARDWARE_AEC       (false)                          // 禁用硬件AEC（声学回声消除）
#define BOARD_PA_GAIN             (10)                             // 板载功率放大器增益（dB）

/**
 * @brief ADC输入数据格式
 */
#define AUDIO_ADC_INPUT_CH_FORMAT "N"  // N表示单声道（Mono）

extern audio_hal_func_t AUDIO_CODEC_ES8388_DEFAULT_HANDLE;  // ES8388编解码器默认句柄

// 音频编解码器默认配置
#define AUDIO_CODEC_DEFAULT_CONFIG(){                   \
        .adc_input  = AUDIO_HAL_ADC_INPUT_LINE1,        \  // ADC输入为LINE1
        .dac_output = AUDIO_HAL_DAC_OUTPUT_ALL,         \  // DAC输出为全部通道
        .codec_mode = AUDIO_HAL_CODEC_MODE_BOTH,        \  // 编解码器模式为同时支持输入输出
        .i2s_iface = {                                  \
            .mode = AUDIO_HAL_MODE_SLAVE,               \  // I2S工作在从模式
            .fmt = AUDIO_HAL_I2S_NORMAL,                \  // I2S格式为标准格式
            .samples = AUDIO_HAL_16K_SAMPLES,           \  // 采样率48K
            .bits = AUDIO_HAL_BIT_LENGTH_16BITS,        \  // 位长16位
        },                                              \
};


/**
 * @brief 按键功能定义
 */
#define FUNC_BUTTON_EN            (1)          // 启用按键功能
#define INPUT_KEY_NUM             6            // 输入按键数量
#define BUTTON_REC_ID             GPIO_NUM_36  // 录音按键GPIO
#define BUTTON_MODE_ID            GPIO_NUM_39  // 模式按键GPIO
#define BUTTON_SET_ID             TOUCH_PAD_NUM9  // 设置触摸按键
#define BUTTON_PLAY_ID            TOUCH_PAD_NUM8  // 播放触摸按键
#define BUTTON_VOLUP_ID           TOUCH_PAD_NUM7  // 音量加触摸按键
#define BUTTON_VOLDOWN_ID         TOUCH_PAD_NUM4  // 音量减触摸按键

// 输入按键默认信息配置
#define INPUT_KEY_DEFAULT_INFO() {                      \
     {                                                  \
        .type = PERIPH_ID_BUTTON,                       \  // 类型为物理按键
        .user_id = INPUT_KEY_USER_ID_REC,               \  // 用户ID为录音键
        .act_id = BUTTON_REC_ID,                        \  // 实际引脚为录音按键GPIO
    },                                                  \
    {                                                   \
        .type = PERIPH_ID_BUTTON,                       \  // 类型为物理按键
        .user_id = INPUT_KEY_USER_ID_MODE,              \  // 用户ID为模式键
        .act_id = BUTTON_MODE_ID,                       \  // 实际引脚为模式按键GPIO
    },                                                  \
    {                                                   \
        .type = PERIPH_ID_TOUCH,                        \  // 类型为触摸按键
        .user_id = INPUT_KEY_USER_ID_SET,               \  // 用户ID为设置键
        .act_id = BUTTON_SET_ID,                        \  // 实际触摸 pad 为设置键
    },                                                  \
    {                                                   \
        .type = PERIPH_ID_TOUCH,                        \  // 类型为触摸按键
        .user_id = INPUT_KEY_USER_ID_PLAY,              \  // 用户ID为播放键
        .act_id = BUTTON_PLAY_ID,                       \  // 实际触摸 pad 为播放键
    },                                                  \
    {                                                   \
        .type = PERIPH_ID_TOUCH,                        \  // 类型为触摸按键
        .user_id = INPUT_KEY_USER_ID_VOLUP,             \  // 用户ID为音量加键
        .act_id = BUTTON_VOLUP_ID,                      \  // 实际触摸 pad 为音量加键
    },                                                  \
    {                                                   \
        .type = PERIPH_ID_TOUCH,                        \  // 类型为触摸按键
        .user_id = INPUT_KEY_USER_ID_VOLDOWN,           \  // 用户ID为音量减键
        .act_id = BUTTON_VOLDOWN_ID,                    \  // 实际触摸 pad 为音量减键
    }                                                   \
}

#endif