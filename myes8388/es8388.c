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

#include <string.h>
#include "esp_log.h"
#include "i2c_bus.h"
#include "es8388.h"
#include "board.h"
#include "audio_volume.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef CONFIG_ESP_LYRAT_V4_3_BOARD
#include "headphone_detect.h"
#endif

static const char *ES_TAG = "ES8388_DRIVER";  // ES8388驱动的日志标签
static i2c_bus_handle_t i2c_handle;          // I2C总线句柄
static codec_dac_volume_config_t *dac_vol_handle;  // DAC音量配置句柄

// DAC音量配置默认值
#define ES8388_DAC_VOL_CFG_DEFAULT() {                      \
    .max_dac_volume = 0,                                    \
    .min_dac_volume = -96,                                  \
    .board_pa_gain = BOARD_PA_GAIN,                         \
    .volume_accuracy = 0.5,                                 \
    .dac_vol_symbol = -1,                                   \
    .zero_volume_reg = 0,                                   \
    .reg_value = 0,                                         \
    .user_volume = 0,                                       \
    .offset_conv_volume = NULL,                             \
}

// 自定义断言宏，用于错误处理
#define ES_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(ES_TAG, format, ##__VA_ARGS__); \
        return b;\
    }

// ES8388编解码器的默认函数句柄
audio_hal_func_t AUDIO_CODEC_ES8388_DEFAULT_HANDLE = {
    .audio_codec_initialize = es8388_init,               // 初始化函数
    .audio_codec_deinitialize = es8388_deinit,           // 反初始化函数
    .audio_codec_ctrl = es8388_ctrl_state,               // 状态控制函数
    .audio_codec_config_iface = es8388_config_i2s,       // I2S接口配置函数
    .audio_codec_set_mute = es8388_set_voice_mute,       // 静音设置函数
    .audio_codec_set_volume = es8388_set_voice_volume,   // 音量设置函数
    .audio_codec_get_volume = es8388_get_voice_volume,   // 音量获取函数
    .audio_codec_enable_pa = es8388_pa_power,            // 功率放大器控制函数
    .audio_hal_lock = NULL,                              // 锁函数
    .handle = NULL,                                      // 句柄
};

/**
 * @brief 向ES8388寄存器写入数据
 * 
 * @param slave_addr 从设备地址
 * @param reg_add 寄存器地址
 * @param data 要写入的数据
 * @return esp_err_t 操作结果
 */
static esp_err_t es_write_reg(uint8_t slave_addr, uint8_t reg_add, uint8_t data)
{
    return i2c_bus_write_bytes(i2c_handle, slave_addr, &reg_add, sizeof(reg_add), &data, sizeof(data));
}

/**
 * @brief 从ES8388寄存器读取数据
 * 
 * @param reg_add 寄存器地址
 * @param p_data 存储读取数据的指针
 * @return esp_err_t 操作结果
 */
static esp_err_t es_read_reg(uint8_t reg_add, uint8_t *p_data)
{
    return i2c_bus_read_bytes(i2c_handle, ES8388_ADDR, &reg_add, sizeof(reg_add), p_data, 1);
}

// /**
//  * @brief 初始化I2C总线
//  * 
//  * @return esp_err_t 操作结果
//  */
// esp_err_t i2c_init(void)
// {
//     i2c_master_bus_config_t i2c_bus_config = {
//         .clk_source                     = I2C_CLK_SRC_DEFAULT,  /* 时钟源 */
//         .i2c_port                       = I2C_NUM_0,         /* I2C端口 */
//         .scl_io_num                     = GPIO_NUM_40,     /* SCL管脚 */
//         .sda_io_num                     = GPIO_NUM_41,     /* SDA管脚 */
//         .glitch_ignore_cnt              = 7,                    /* 故障周期 */
//         .flags.enable_internal_pullup   = true,                 /* 内部上拉 */
//     };
//     /* 新建I2C总线 */
//     ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

//     return ESP_OK;
// }


static int i2c_init()
{
    int res;
    i2c_config_t es_i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    res = get_i2c_pins(I2C_NUM_0, &es_i2c_cfg);
    ES_ASSERT(res, "getting i2c pins error", -1);
    i2c_handle = i2c_bus_create(I2C_NUM_0, &es_i2c_cfg);
    return res;
}


/**
 * @brief 读取所有ES8388寄存器的值并打印
 */
void es8388_read_all()
{
    for (int i = 0; i < 50; i++) {
        uint8_t reg = 0;
        es_read_reg(i, &reg);
        ESP_LOGI(ES_TAG, "%x: %x", i, reg);
    }
}

/**
 * @brief 向ES8388指定寄存器写入数据（使用默认地址）
 * 
 * @param reg_add 寄存器地址
 * @param data 要写入的数据
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_write_reg(uint8_t reg_add, uint8_t data)
{
    return es_write_reg(ES8388_ADDR, reg_add, data);
}

/**
 * @brief 配置ES8388的ADC和DAC音量（可视为增益）
 * 
 * @param mode 配置模式：ADC、DAC或两者
 * @param volume 音量值（-96 ~ 0）
 * @param dot 是否包含0.5dB的精度
 * @return int 操作结果：0成功，-1参数错误
 */
static int es8388_set_adc_dac_volume(int mode, int volume, int dot)
{
    int res = 0;
    if ( volume < -96 || volume > 0 ) {  // 检查音量范围
        ESP_LOGW(ES_TAG, "警告: 音量 < -96 或 > 0!\n");
        if (volume < -96)
            volume = -96;
        else
            volume = 0;
    }
    dot = (dot >= 5 ? 1 : 0);  // 处理0.5dB精度
    volume = (-volume << 1) + dot;  // 转换为寄存器值
    
    // 根据模式配置相应寄存器
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, volume);  // ADC右声道音量=0dB
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, volume);
    }
    return res;
}


/**
 * @brief 电源管理（启动模块）
 * 
 * @param mode 要启动的模块
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_start(es_module_t mode)
{
    esp_err_t res = ESP_OK;
    uint8_t prev_data = 0, data = 0;
    es_read_reg(ES8388_DACCONTROL21, &prev_data);
    
    if (mode == ES_MODULE_LINE) {  // 线路输入模式
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x09); // 0x00:音频来自LIN1&RIN1, 0x09:LIN2&RIN2直通使能
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x50); // 左DAC到左混音器使能，LIN信号到左混音器使能（0dB）
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x50); // 右DAC到右混音器使能，LIN信号到右混音器使能（0dB）
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0xC0); // 使能ADC
    } else {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);   // 使能DAC
    }
    
    es_read_reg(ES8388_DACCONTROL21, &data);
    if (prev_data != data) {  // 寄存器值变化时启动状态机
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF0);   // 启动状态机
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);   // 启动状态机
    }
    
    // 根据模式启动相应模块电源
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);   // 上电ADC和线路输入
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);   // 上电DAC和线路输出
        res |= es8388_set_voice_mute(false);  // 取消静音
        ESP_LOGD(ES_TAG, "es8388启动，模式:%d", mode);
    }

    return res;
}

/**
 * @brief 电源管理（停止模块）
 * 
 * @param mode 要停止的模块
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_stop(es_module_t mode)
{
    esp_err_t res = ESP_OK;
    if (mode == ES_MODULE_LINE) {  // 线路输入模式停止
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80); // 使能DAC
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00); // 音频来自LIN1&RIN1
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90); // 仅左DAC到左混音器使能（0dB）
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90); // 仅右DAC到右混音器使能（0dB）
        return res;
    }
    
    // 根据模式停止相应模块电源
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x00);
        res |= es8388_set_voice_mute(true);  // 静音
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);  // 下电ADC和线路输入
    }
    if (mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x9C);  // 禁用主时钟
    }

    return res;
}


/**
 * @brief 配置主模式下的I2S时钟
 * 
 * @param cfg 时钟配置（包括SCLK和LCLK分频）
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_i2s_config_clock(es_i2s_clock_t cfg)
{
    esp_err_t res = ESP_OK;
    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, cfg.sclk_div);  // 配置SCLK分频
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, cfg.lclk_div);  // ADC时钟模式配置
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, cfg.lclk_div);  // DAC时钟模式配置
    return res;
}

/**
 * @brief 反初始化ES8388
 * 
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_deinit(void)
{
    int res = 0;
    res = es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xFF);  // 重置并停止ES8388
    i2c_bus_delete(i2c_handle);  // 删除I2C总线
    
#ifdef CONFIG_ESP_LYRAT_V4_3_BOARD
    headphone_detect_deinit();  // 耳机检测反初始化
#endif

    audio_codec_volume_deinit(dac_vol_handle);  // 音量控制反初始化
    return res;
}

/**
 * @brief 初始化ES8388编解码器
 * 
 * @param cfg 初始化配置
 * @return esp_err_t 操作结果：0成功，-1失败
 */
esp_err_t es8388_init(audio_hal_codec_config_t *cfg)
{
    int res = 0;
#ifdef CONFIG_ESP_LYRAT_V4_3_BOARD
    headphone_detect_init(get_headphone_detect_gpio());  // 耳机检测初始化
#endif  /* CONFIG_ESP_LYRAT_V4_3_BOARD */

    res = i2c_init();  // 初始化I2C（ESP32为主模式）

    // 初始化寄存器配置
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);  // 禁用DAC静音和软斜坡
    
    /* 芯片控制和电源管理 */
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
    res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);  // 正常模式，所有模块上电

    // 禁用内部DLL以改善8K采样率性能
    res |= es_write_reg(ES8388_ADDR, 0x35, 0xA0);
    res |= es_write_reg(ES8388_ADDR, 0x37, 0xD0);
    res |= es_write_reg(ES8388_ADDR, 0x39, 0xD0);

    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, cfg->i2s_iface.mode);  // CODEC工作在I2S从模式

    /* DAC配置 */
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xC0);  // 禁用DAC和输出
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);  // 播放和录制模式
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);   // 16位I2S格式
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);   // DAC单速模式，比率256
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00);  // 音频来自LIN1&RIN1
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90);  // 仅左DAC到左混音器使能（0dB）
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90);  // 仅右DAC到右混音器使能（0dB）
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);  // ADC和DAC使用相同的LRCK时钟
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);  // vroi=0

    // 设置输出音量（0dB）
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1E);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1E);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL26, 0);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL27, 0);
    
    // 根据配置设置DAC输出通道
    int tmp = 0;
    if (AUDIO_HAL_DAC_OUTPUT_LINE2 == cfg->dac_output) {
        tmp = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_ROUT1;
    } else if (AUDIO_HAL_DAC_OUTPUT_LINE1 == cfg->dac_output) {
        tmp = DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT2;
    } else {
        tmp = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
    }
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, tmp);  // 使能DAC和相应输出

    /* ADC配置 */
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);  // 初始下电ADC
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0xbb);  // MIC左右声道PGA增益
    
    // 根据配置设置ADC输入通道
    tmp = 0;
    if (AUDIO_HAL_ADC_INPUT_LINE1 == cfg->adc_input) {
        tmp = ADC_INPUT_LINPUT1_RINPUT1;
    } else if (AUDIO_HAL_ADC_INPUT_LINE2 == cfg->adc_input) {
        tmp = ADC_INPUT_LINPUT2_RINPUT2;
    } else {
        tmp = ADC_INPUT_DIFFERENCE;
    }
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, tmp);  // 设置ADC输入源
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x02);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0c);  // 16位I2S格式
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);  // ADC单速模式，比率256
    
    // 麦克风自动增益控制
    res |= es8388_set_adc_dac_volume(ES_MODULE_ADC, 0, 0);    // 0dB
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x09);  // 上电ADC，使能LIN&RIN

    /* 配置ES8388功率放大器GPIO */
    if (get_pa_enable_gpio() != -1) {
        gpio_config_t io_conf;
        memset(&io_conf, 0, sizeof(io_conf));
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = BIT64(get_pa_enable_gpio());
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);
        /* 使能ES8388功率放大器 */
        es8388_pa_power(true);
    }

    // 初始化DAC音量控制
    codec_dac_volume_config_t vol_cfg = ES8388_DAC_VOL_CFG_DEFAULT();
    dac_vol_handle = audio_codec_volume_init(&vol_cfg);
    ESP_LOGI(ES_TAG, "初始化完成,输出:%02x, 输入:%02x", cfg->dac_output, cfg->adc_input);
    return res;
}

/**
 * @brief 配置ES8388的I2S格式
 * 
 * @param mode 配置模式：ADC、DAC或两者
 * @param fmt I2S格式
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_config_fmt(es_module_t mode, es_i2s_fmt_t fmt)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    
    // 配置ADC的I2S格式
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xfc;  // 清除低2位
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | fmt);
    }
    
    // 配置DAC的I2S格式
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xf9;  // 清除特定位
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (fmt << 1));
    }
    return res;
}

/**
 * @brief 设置音量
 * 
 * @note 寄存器值：0xC0=-96dB, 0x64=-50dB, 0x00=0dB
 * @note 增益精度为0.5dB
 * @param volume 音量值（0~100）
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_set_voice_volume(int volume)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    reg = audio_codec_get_dac_reg_value(dac_vol_handle, volume);  // 获取寄存器值
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, reg);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, reg);
    ESP_LOGD(ES_TAG, "设置音量:%.2d 寄存器值:0x%.2x 分贝:%.1f", 
            (int)dac_vol_handle->user_volume, reg,
            audio_codec_cal_dac_volume(dac_vol_handle));
    return res;
}

/**
 * @brief 获取当前音量
 * 
 * @param[out] *volume 音量值（0~100）
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_get_voice_volume(int *volume)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL4, &reg);  // 读取寄存器值
    if (res == ESP_FAIL) {
        *volume = 0;
    } else {
        if (reg == dac_vol_handle->reg_value) {
            *volume = dac_vol_handle->user_volume;
        } else {
            *volume = 0;
            res = ESP_FAIL;
        }
    }
    ESP_LOGD(ES_TAG, "获取音量:%.2d 寄存器值:0x%.2x", *volume, reg);
    return res;
}

/**
 * @brief 配置ES8388的数据采样位数
 * 
 * @param mode 配置模式：ADC、DAC或两者
 * @param bits_length 采样位数
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_set_bits_per_sample(es_module_t mode, es_bits_length_t bits_length)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    int bits = (int)bits_length;

    // 配置ADC采样位数
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xe3;  // 清除相关位
        res |=  es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | (bits << 2));
    }
    
    // 配置DAC采样位数
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xc7;  // 清除相关位
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (bits << 3));
    }
    return res;
}

/**
 * @brief 配置ES8388的DAC静音状态
 * 
 * @param enable true:静音, false:取消静音
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_set_voice_mute(bool enable)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    reg = reg & 0xFB;  // 清除静音位
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, reg | (((int)enable) << 2));
    return res;
}

/**
 * @brief 获取ES8388的DAC静音状态
 * 
 * @return 0:未静音, 1:静音, 其他:错误
 */
esp_err_t es8388_get_voice_mute(void)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    if (res == ESP_OK) {
        reg = (reg & 0x04) >> 2;  // 提取静音位
    }
    return res == ESP_OK ? reg : res;
}

/**
 * @brief 配置DAC输出
 * 
 * @param output 输出模式
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_config_dac_output(es_dac_output_t output)
{
    esp_err_t res;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACPOWER, &reg);
    reg = reg & 0xc3;  // 清除输出配置位
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, reg | output);
    return res;
}

/**
 * @brief 配置ADC输入
 * 
 * @param input 输入模式
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_config_adc_input(es_adc_input_t input)
{
    esp_err_t res;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_ADCCONTROL2, &reg);
    reg = reg & 0x0f;  // 清除输入配置位
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, reg | input);
    return res;
}

/**
 * @brief 设置麦克风增益
 * 
 * @param gain 增益值（es_mic_gain_t类型）
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_set_mic_gain(es_mic_gain_t gain)
{
    esp_err_t res, gain_n;
    gain_n = (int)gain / 3;
    gain_n = (gain_n << 4) + gain_n;  // 转换为寄存器值
    res = es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, gain_n); // 设置MIC PGA增益
    return res;
}

/**
 * @brief 控制ES8388编解码器状态
 * 
 * @param mode 编解码器模式
 * @param ctrl_state 控制状态（启动/停止）
 * @return int 操作结果
 */
int es8388_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
    int res = 0;
    int es_mode_t = 0;
    
    // 转换编解码器模式
    switch (mode) {
        case AUDIO_HAL_CODEC_MODE_ENCODE:
            es_mode_t  = ES_MODULE_ADC;
            break;
        case AUDIO_HAL_CODEC_MODE_LINE_IN:
            es_mode_t  = ES_MODULE_LINE;
            break;
        case AUDIO_HAL_CODEC_MODE_DECODE:
            es_mode_t  = ES_MODULE_DAC;
            break;
        case AUDIO_HAL_CODEC_MODE_BOTH:
            es_mode_t  = ES_MODULE_ADC_DAC;
            break;
        default:
            es_mode_t = ES_MODULE_DAC;
            ESP_LOGW(ES_TAG, "不支持的编解码器模式，默认使用解码模式");
            break;
    }
    
    // 根据控制状态启动或停止
    if (AUDIO_HAL_CTRL_STOP == ctrl_state) {
        res = es8388_stop(es_mode_t);
    } else {
        res = es8388_start(es_mode_t);
        ESP_LOGD(ES_TAG, "启动默认解码模式:%d", es_mode_t);
    }
    return res;
}

/**
 * @brief 配置ES8388的I2S接口
 * 
 * @param mode 编解码器模式
 * @param iface I2S接口配置
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface)
{
    esp_err_t res = ESP_OK;
    int tmp = 0;
    res |= es8388_config_fmt(ES_MODULE_ADC_DAC, iface->fmt);  // 配置I2S格式
    
    // 配置采样位数
    if (iface->bits == AUDIO_HAL_BIT_LENGTH_16BITS) {
        tmp = BIT_LENGTH_16BITS;
    } else if (iface->bits == AUDIO_HAL_BIT_LENGTH_24BITS) {
        tmp = BIT_LENGTH_24BITS;
    } else {
        tmp = BIT_LENGTH_32BITS;
    }
    res |= es8388_set_bits_per_sample(ES_MODULE_ADC_DAC, tmp);
    return res;
}

/**
 * @brief 控制ES8388的功率放大器
 * 
 * @param enable true:使能PA, false:禁用PA
 * @return esp_err_t 操作结果
 */
esp_err_t es8388_pa_power(bool enable)
{
    esp_err_t res = ESP_OK;
    if (get_pa_enable_gpio() == -1) {
        return res;
    }
    if (enable) {
        res = gpio_set_level(get_pa_enable_gpio(), 1);  // 使能PA
    } else {
        res = gpio_set_level(get_pa_enable_gpio(), 0);  // 禁用PA
    }
    return res;
}