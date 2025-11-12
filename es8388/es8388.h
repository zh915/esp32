/*
 * ESPRESSIF MIT 许可证
 *
 * 版权所有 (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * 特此授予在所有 ESPRESSIF 系统产品上使用的许可，在这种情况下，
 * 任何获得本软件及相关文档文件（以下简称"软件"）副本的人，都可以免费无限制地处理本软件，
 * 包括但不限于使用、复制、修改、合并、出版、发行、再许可和/或出售软件副本，
 * 以及允许获得软件的人这样做，但须遵守以下条件：
 *
 * 上述版权声明和本许可声明应包含在软件的所有副本或重要部分中。
 *
 * 本软件按"原样"提供，不提供任何形式的保证，无论是明示的还是暗示的，
 * 包括但不限于适销性、特定用途适用性和非侵权性的保证。在任何情况下，
 * 作者或版权持有人均不对任何索赔、损害或其他责任负责，无论是在合同诉讼、侵权行为或其他方面，
 * 无论是由于使用或与软件的使用或其他交易有关。
 *
 */

#ifndef __ES8388_H__
#define __ES8388_H__

#include "esp_types.h"
#include "audio_hal.h"
#include "driver/i2c.h"
#include "esxxx_common.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

i2c_master_bus_handle_t bus_handle;     /* 总线句柄 */

/* ES8388 地址 */
#define ES8388_ADDR 0x10  /*!< 0x22:CE=1;0x20:CE=0*/

/* ES8388 寄存器定义 */
#define ES8388_CONTROL1         0x00  /* 控制寄存器1 */
#define ES8388_CONTROL2         0x01  /* 控制寄存器2 */

#define ES8388_CHIPPOWER        0x02  /* 芯片电源寄存器 */

#define ES8388_ADCPOWER         0x03  /* ADC电源寄存器 */
#define ES8388_DACPOWER         0x04  /* DAC电源寄存器 */

#define ES8388_CHIPLOPOW1       0x05  /* 芯片低功耗寄存器1 */
#define ES8388_CHIPLOPOW2       0x06  /* 芯片低功耗寄存器2 */

#define ES8388_ANAVOLMANAG      0x07  /* 模拟音量管理寄存器 */

#define ES8388_MASTERMODE       0x08  /* 主模式寄存器 */

/* ADC相关寄存器 */
#define ES8388_ADCCONTROL1      0x09  /* ADC控制寄存器1 */
#define ES8388_ADCCONTROL2      0x0a  /* ADC控制寄存器2 */
#define ES8388_ADCCONTROL3      0x0b  /* ADC控制寄存器3 */
#define ES8388_ADCCONTROL4      0x0c  /* ADC控制寄存器4 */
#define ES8388_ADCCONTROL5      0x0d  /* ADC控制寄存器5 */
#define ES8388_ADCCONTROL6      0x0e  /* ADC控制寄存器6 */
#define ES8388_ADCCONTROL7      0x0f  /* ADC控制寄存器7 */
#define ES8388_ADCCONTROL8      0x10  /* ADC控制寄存器8 */
#define ES8388_ADCCONTROL9      0x11  /* ADC控制寄存器9 */
#define ES8388_ADCCONTROL10     0x12  /* ADC控制寄存器10 */
#define ES8388_ADCCONTROL11     0x13  /* ADC控制寄存器11 */
#define ES8388_ADCCONTROL12     0x14  /* ADC控制寄存器12 */
#define ES8388_ADCCONTROL13     0x15  /* ADC控制寄存器13 */
#define ES8388_ADCCONTROL14     0x16  /* ADC控制寄存器14 */

/* DAC相关寄存器 */
#define ES8388_DACCONTROL1      0x17  /* DAC控制寄存器1 */
#define ES8388_DACCONTROL2      0x18  /* DAC控制寄存器2 */
#define ES8388_DACCONTROL3      0x19  /* DAC控制寄存器3 */
#define ES8388_DACCONTROL4      0x1a  /* DAC控制寄存器4 */
#define ES8388_DACCONTROL5      0x1b  /* DAC控制寄存器5 */
#define ES8388_DACCONTROL6      0x1c  /* DAC控制寄存器6 */
#define ES8388_DACCONTROL7      0x1d  /* DAC控制寄存器7 */
#define ES8388_DACCONTROL8      0x1e  /* DAC控制寄存器8 */
#define ES8388_DACCONTROL9      0x1f  /* DAC控制寄存器9 */
#define ES8388_DACCONTROL10     0x20  /* DAC控制寄存器10 */
#define ES8388_DACCONTROL11     0x21  /* DAC控制寄存器11 */
#define ES8388_DACCONTROL12     0x22  /* DAC控制寄存器12 */
#define ES8388_DACCONTROL13     0x23  /* DAC控制寄存器13 */
#define ES8388_DACCONTROL14     0x24  /* DAC控制寄存器14 */
#define ES8388_DACCONTROL15     0x25  /* DAC控制寄存器15 */
#define ES8388_DACCONTROL16     0x26  /* DAC控制寄存器16 */
#define ES8388_DACCONTROL17     0x27  /* DAC控制寄存器17 */
#define ES8388_DACCONTROL18     0x28  /* DAC控制寄存器18 */
#define ES8388_DACCONTROL19     0x29  /* DAC控制寄存器19 */
#define ES8388_DACCONTROL20     0x2a  /* DAC控制寄存器20 */
#define ES8388_DACCONTROL21     0x2b  /* DAC控制寄存器21 */
#define ES8388_DACCONTROL22     0x2c  /* DAC控制寄存器22 */
#define ES8388_DACCONTROL23     0x2d  /* DAC控制寄存器23 */
#define ES8388_DACCONTROL24     0x2e  /* DAC控制寄存器24 */
#define ES8388_DACCONTROL25     0x2f  /* DAC控制寄存器25 */
#define ES8388_DACCONTROL26     0x30  /* DAC控制寄存器26 */
#define ES8388_DACCONTROL27     0x31  /* DAC控制寄存器27 */
#define ES8388_DACCONTROL28     0x32  /* DAC控制寄存器28 */
#define ES8388_DACCONTROL29     0x33  /* DAC控制寄存器29 */
#define ES8388_DACCONTROL30     0x34  /* DAC控制寄存器30 */

/**
 * @brief 初始化ES8388编解码器芯片
 *
 * @param cfg ES8388的配置参数
 *
 * @return
 *     - ESP_OK 成功
 *     - ESP_FAIL 失败
 */
esp_err_t es8388_init(audio_hal_codec_config_t *cfg);

/**
 * @brief 反初始化ES8388编解码器芯片
 *
 * @return
 *     - ESP_OK 成功
 *     - ESP_FAIL 失败
 */
esp_err_t es8388_deinit(void);

/**
 * @brief 配置ES8388的I2S格式
 *
 * @param mod 配置模块：ADC、DAC或两者
 * @param cfg ES8388的I2S格式
 *
 * @return
 *     - ESP_OK 成功
 *     - ESP_FAIL 失败
 */
esp_err_t es8388_config_fmt(es_module_t mod, es_i2s_fmt_t cfg);

/**
 * @brief 配置主模式下的I2S时钟
 *
 * @param cfg 时钟配置（位时钟和WS时钟）
 *
 * @return
 *     - ESP_OK 成功
 *     - ESP_FAIL 失败
 */
esp_err_t es8388_i2s_config_clock(es_i2s_clock_t cfg);

/**
 * @brief 配置ES8388的数据采样位数
 *
 * @param mode 配置模块：ADC、DAC或两者
 * @param bit_per_sample 每个样本的位数
 *
 * @return
 *     - ESP_OK 成功
 *     - ESP_FAIL 失败
 */
esp_err_t es8388_set_bits_per_sample(es_module_t mode, es_bits_length_t bit_per_sample);

/**
 * @brief 启动ES8388编解码器芯片
 *
 * @param mode 启动模块：ADC、DAC或两者
 *
 * @return
 *     - ESP_OK 成功
 *     - ESP_FAIL 失败
 */
esp_err_t es8388_start(es_module_t mode);

/**
 * @brief 停止ES8388编解码器芯片
 *
 * @param mode 停止模块：ADC、DAC或两者
 *
 * @return
 *     - ESP_OK 成功
 *     - ESP_FAIL 失败
 */
esp_err_t es8388_stop(es_module_t mode);

/**
 * @brief 设置音量
 *
 * @param volume 音量值（0~100）
 *
 * @return
 *     - ESP_OK 成功
 *     - ESP_FAIL 失败
 */
esp_err_t es8388_set_voice_volume(int volume);

/**
 * @brief 获取当前音量
 *
 * @param[out] *volume 音量值（0~100）
 *
 * @return
 *     - ESP_OK 成功
 *     - ESP_FAIL 失败
 */
esp_err_t es8388_get_voice_volume(int *volume);

/**
 * @brief 配置ES8388的DAC静音状态
 *
 * @param enable 1:使能静音, 0:禁用静音
 *
 * @return
 *     - ESP_FAIL 参数错误
 *     - ESP_OK 成功
 */
esp_err_t es8388_set_voice_mute(bool enable);

/**
 * @brief 获取ES8388的DAC静音状态
 *
 * @return
 *     - ESP_FAIL 参数错误
 *     - ESP_OK 成功（返回0或1表示状态）
 */
esp_err_t es8388_get_voice_mute(void);

/**
 * @brief 设置ES8388的麦克风增益
 *
 * @param gain 麦克风增益（分贝）
 *
 * @return
 *     - ESP_FAIL 参数错误
 *     - ESP_OK 成功
 */
esp_err_t es8388_set_mic_gain(es_mic_gain_t gain);

/**
 * @brief 设置ES8388的ADC输入模式
 *
 * @param input ADC输入模式
 *
 * @return
 *     - ESP_FAIL 参数错误
 *     - ESP_OK 成功
 */
esp_err_t es8388_config_adc_input(es_adc_input_t input);

/**
 * @brief 设置ES8388的DAC输出模式
 *
 * @param output DAC输出模式
 *
 * @return
 *     - ESP_FAIL 参数错误
 *     - ESP_OK 成功
 */
esp_err_t es8388_config_dac_output(es_dac_output_t output);

/**
 * @brief 向ES8388寄存器写入数据
 *
 * @param reg_add 寄存器地址
 * @param data 要写入的数据
 *
 * @return
 *     - ESP_FAIL 参数错误
 *     - ESP_OK 成功
 */
esp_err_t es8388_write_reg(uint8_t reg_add, uint8_t data);

/**
 * @brief 打印所有ES8388寄存器的值
 */
void es8388_read_all(void);

/**
 * @brief 配置ES8388的编解码器模式和I2S接口
 *
 * @param mode 编解码器模式
 * @param iface I2S配置
 *
 * @return
 *     - ESP_FAIL 参数错误
 *     - ESP_OK 成功
 */
esp_err_t es8388_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface);

/**
 * @brief 控制ES8388编解码器芯片
 *
 * @param mode 编解码器模式
 * @param ctrl_state 启动或停止解码/编码过程
 *
 * @return
 *     - ESP_FAIL 参数错误
 *     - ESP_OK 成功
 */
esp_err_t es8388_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state);

/**
 * @brief 设置ES8388的功率放大器
 *
 * @param enable true:使能PA电源, false:禁用PA电源
 *
 * @return
 *     - ESP_ERR_INVALID_ARG 参数错误
 *     - ESP_OK 成功
 */
esp_err_t es8388_pa_power(bool enable);

#ifdef __cplusplus
}
#endif

#endif //__ES8388_H__