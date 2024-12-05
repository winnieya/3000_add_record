/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-23     Tery       the first version
 */
#include <rtdevice.h>

#include "om_1_data.h"
#include "om_1_sensor.h"
#include "om_1_power.h"
#include "om_1_session.h"
#include "om_1_ble.h"
#include "calibration.h"

#define DBG_TAG "om_1_data"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define OM_IGNITE_CMD   0x20
#define OM_RANGE_CMD   0x04
#define OM_INTEGRATION_CMD   0x0C
#define OM_PID_ONOFF_CMD   0x36
#define OM_LED_ONOFF_CMD   0x46
#define OM_H2_CMD_CMD   0x48
#define OM_READ_CMD   0x25
#define OM_FID_CALIBRATION_CMD   0x31
#define OM_PID_CALIBRATION_CMD   0x32

rt_uint8_t om_1_bt_calccrc(rt_uint8_t* buf, rt_uint8_t paramInt);

extern struct _fid_calibration_record fid_calibration_val;

/*指示点火线圈是否需要执行点火动作
 * RT_FALSE    表示点火线圈不需要工作
 * RT_TRUE     表示点火线圈需要工作
 * */
static rt_bool_t g_om_need_ignite = RT_FALSE; /*指示点火线圈是否需要执行点火动作*/
static uint8_t g_pid_onoff = 0;


/* 蓝牙上报数据结构体，包含所有需要上报的字段 */
struct om_1_report_data
{
    rt_uint8_t cmd_type;
    rt_uint8_t data_len;
    rt_uint8_t cmd_id;
    rt_uint8_t reserve0;
    rt_uint8_t dev_state;
    rt_uint8_t fid_calibrate_is_or_not;
    rt_uint8_t pid_calibrate_is_or_not;
    rt_uint8_t thermo_couple[2];
    rt_uint8_t battery_voltage[2];
    rt_uint8_t chamber_outer_temp[2];
    rt_uint8_t sample_pressure[2];
    rt_uint8_t air_pressure[2];
    rt_uint8_t tank_pressure[2];
    rt_uint8_t fid_range;
    rt_uint8_t reserve2[4];
    rt_uint8_t pico_amps[4];
    rt_uint8_t reserve3[4];
    rt_uint8_t fid_raw_ppm[4];
    rt_uint8_t system_current[2];
    rt_uint8_t pump_power;
    rt_uint8_t pid_value[4];
    rt_uint8_t pid_raw_ppm[2];
    rt_uint8_t troubleshooting_tips[2];
    rt_uint8_t checksum;
};


/* 蓝牙下发点火、关火数据结构体 */
struct om_1_ignite_data
{
    rt_uint8_t cmd_type;
    rt_uint8_t data_len;
    rt_uint8_t cmd_id;
    rt_uint8_t ignite;
    rt_uint8_t reserve0[20];
    rt_uint8_t ignite_type;
    rt_uint8_t reserve1;;
    rt_uint8_t checksum;
};

/* 蓝牙下发配置量程数据结构体 */
struct om_1_set_range_data
{
    rt_uint8_t cmd_type;
    rt_uint8_t data_len;
    rt_uint8_t cmd_id;
    rt_uint8_t range;
    rt_uint8_t reserve;
    rt_uint8_t checksum;
};

/* 蓝牙下发配置量程数据结构体 */
struct om_1_set_avg_data
{
    rt_uint8_t cmd_type;
    rt_uint8_t data_len;
    rt_uint8_t cmd_id;
    rt_uint8_t reserve0;
    rt_uint32_t reserve1;
    rt_uint8_t avg;
    rt_uint16_t reserve2;
    rt_uint8_t checksum;
};
/*
 * report test data protocol format
 * 0       固定（发0x5a,收0xa5）
 * 1       Len（发，收待确认）
 * 2       0x25 cmdid
 * 3       未知
 * 4       IsPumpAOn  [0]
 *         IsSolenoidAOn   [2]
 *         IsSolenoidBOn   [3]
 * 5~6     保留
 * 7~8     ThermoCouple，2个字节，ConvertKelvinToFahrenheit()*0.1f
 * 9~10    BatteryVoltage，2个字节，*0.001f
 * 11~12   ChamberOuterTemp，2个字节，ConvertKelvinToFahrenheit()*0.1f
 * 13~14   SamplePressure，2个字节，*0.01f
 * 15~16   AirPressure，2个字节，*0.01f
 * 17~18   TankPressure，2个字节
 * 19      FIDRange，1个字节
 * 20~23   保留
 * 24~27   PicoAmps，4个字节，*0.1f
 * 28~31   保留
 * 32~35   RawPpm，4个字节，*0.1f
 * 36~37   SystemCurrent，2个字节
 * 38      PumpPower，1个字节
 * 39      CRC校验码
 * */
void om_byte_order_conversion_uint32(rt_uint8_t* conv_buf,rt_uint32_t conv_value)
{
    conv_buf[0] = (rt_uint8_t)conv_value;
    conv_buf[1] = (rt_uint8_t)(conv_value >> 8);
    conv_buf[2] = (rt_uint8_t)(conv_value >> 16);
    conv_buf[3] = (rt_uint8_t)(conv_value >> 24);
}

void om_byte_order_conversion_uint16(rt_uint8_t* conv_buf,rt_uint16_t conv_value)
{
    conv_buf[0] = (rt_uint8_t)conv_value;
    conv_buf[1] = (rt_uint8_t)(conv_value >> 8);
}

uint16_t get_troubleshooting_tips(void)
{
    return 0;
}


uint8_t om_get_pid_state(void)
{
    return g_pid_onoff;
}
static uint8_t g_ignite_state = 0;
void om_set_ignite_state(uint8_t state)
{
    g_ignite_state = state;
}
uint8_t om_get_ignite_state(void)
{
    return g_ignite_state;
}


/*
 * 组包函数
 * 用于向APP端发送数据
 * */
 uint8_t zero_flag = 0;
void om_1_assemble_packet(void)
{
    uint16_t temp = 0;
//    uint8_t data_len = sizeof(struct om_1_report_data);
    struct om_1_report_data report_data = {0};//(struct om_1_report_data)om_report_collection_data;
    int32_t message_value = 0;

    report_data.cmd_type = 0xAA;
    report_data.data_len = 0x2E;
    report_data.cmd_id = 0x25;
    report_data.dev_state = om_get_pump_state() | 1 << 2;
    om_byte_order_conversion_uint16(report_data.thermo_couple,(rt_uint16_t)om_thermocouple_read());
    om_byte_order_conversion_uint16(report_data.battery_voltage,(rt_uint16_t)om_battery_level_read());
    om_byte_order_conversion_uint16(report_data.chamber_outer_temp,(rt_uint16_t)om_temperature_sensor_read());
    om_byte_order_conversion_uint16(report_data.sample_pressure,(rt_uint16_t)om_pressure5_sensor_read());
    om_byte_order_conversion_uint16(report_data.air_pressure,(rt_uint16_t)om_pressure9_sensor_read());
    om_byte_order_conversion_uint16(report_data.tank_pressure,0);

    report_data.reserve2[0] = om_get_ignite_state();
	
    report_data.fid_range = om_get_ddc112_range_value();
    message_value = om_get_ddc112_value();
	
	/*     add 2024.11.04          */
	//是否需要矫正
	if(om_is_need_correction() && fid_calibration_val.calibration_flag) 
	{
		electricity_correction_handle(&message_value);
	}
	/*  end  */
	
    om_byte_order_conversion_uint32(report_data.pico_amps,message_value);

    report_data.fid_calibrate_is_or_not = 0;
    om_byte_order_conversion_uint32(report_data.fid_raw_ppm,0);

    report_data.pid_calibrate_is_or_not = 0;
    om_byte_order_conversion_uint32(report_data.pid_raw_ppm,0);

    om_byte_order_conversion_uint16(report_data.system_current,(rt_uint16_t)om_system_current_read());//(rt_uint16_t)om_somtem_current_read());
    report_data.pump_power = 100 - (100*(om_get_pump_power_adjust()))/4096;
    temp = om_get_troubleshooting_tips();
    if(temp)
    {
        LOG_I("Fault detected %d\r\n",temp);
    }

    om_byte_order_conversion_uint16(report_data.troubleshooting_tips,temp);

    om_byte_order_conversion_uint32(report_data.pid_value,om_get_pid_value());
//    om_byte_order_conversion_uint32(report_data.pid_value,0);


    /*crc*/
    report_data.checksum = om_1_bt_calccrc((rt_uint8_t *)&report_data,sizeof(struct om_1_report_data) - 1);

    om_send_data((rt_uint8_t *)&report_data, sizeof(struct om_1_report_data));

    LOG_D("om_report_collection_data=");
}


/*蓝牙命令处理函数*/
void om_bt_data_handle(rt_uint8_t* bt_data,rt_uint8_t data_len)
{
    rt_uint8_t cmdid = 0;
    struct om_1_ignite_data* ignite_data = RT_NULL;
    uint16_t ajust_value = 0;

    /*get cmd id*/
    cmdid = bt_data[2];

    if(bt_data[data_len - 1] !=  om_1_bt_calccrc(bt_data,data_len - 1))
    {
        LOG_I("data=");
        for(int i=0;i<data_len;i++)
        {
            LOG_I("%x",bt_data[i]);
        }
        LOG_I("\r\n");
        LOG_E("crc check fail! \r\n");
        LOG_E("CRC %d",om_1_bt_calccrc(bt_data,data_len - 1));
        return;
    }
    switch(cmdid)
    {
        case OM_IGNITE_CMD:
            ignite_data  = (struct om_1_ignite_data*)bt_data;
            LOG_D("is ignite cmd %d! \r\n",ignite_data->ignite);
            if(ignite_data->ignite == 1)  /*ignite or close fire*/
            {
                LOG_D("open ignite 2 cmd! \r\n");
                om_ignite_set();

            }
            else
            {
                LOG_I("close fire cmd! \r\n");
                om_ignite_reset();
                om_set_ignite_state(0);
            }
            break;
        case OM_RANGE_CMD:
            LOG_D("is set range cmd! \r\n");
            break;
        case OM_INTEGRATION_CMD:
            LOG_D("is set integration cmd! \r\n");
            break;
        case OM_PID_ONOFF_CMD:
            LOG_I("is set OM_PID_ONOFF_CMD cmd! \r\n");
            set_pid_onoff(bt_data[3]);
            g_pid_onoff = bt_data[3];
            break;
        case OM_LED_ONOFF_CMD:
            LOG_I("is set OM_PLED_ONOFF_CMD cmd! \r\n");
            om_1_LED_onoff(bt_data[3]);
            break;
        case OM_READ_CMD:
//            om_1_assemble_packet();
            LOG_D("is read cmd! \r\n");
            break;
        case OM_FID_CALIBRATION_CMD:
            ajust_value = (uint16_t)(bt_data[3] + (bt_data[4] << 8));
            if(ajust_value == 0)
            {
                zero_flag = 1;
				calibration_record_reset();
            }
            else {
                zero_flag = 0;
            }
			zero_calib_updata(ajust_value,om_get_ddc112_value());
            LOG_I("is FID CALIBRATION  %d \r\n",ajust_value);
            break;

        default:
            LOG_D("is error cmd! \r\n");
            break;
    }
}

void rs485_recv_data_parsing(uint8_t* rs485_data,uint16_t data_len)
{

    if(rs485_data[data_len - 1] !=  om_1_bt_calccrc(rs485_data,data_len - 1))
    {
        LOG_D("crc error %d\r\n",om_1_bt_calccrc(rs485_data,data_len - 1));
        return;
    }
    if(rs485_data[2] == 1)
    {
        LOG_D("FRAME_NUM %x\r\n",rs485_data[1]);
    }
    switch(rs485_data[5])
    {

    case 0x01: //ACK命令
        break;
    case 0x02:  //采集信号值上报
        ddc112_value_get_post(rs485_data,data_len);
        break;

    default:
        break;
    }
}

/*指示点火线圈是否需要执行点火动作*/
rt_bool_t om_is_need_ignite(void)
{
    return g_om_need_ignite ;
}

/*表示点火线圈不需要执行点火动作*/
void om_ignite_reset(void)
{
    g_om_need_ignite = RT_FALSE;
}

/*表示点火线圈需要执行点火动作*/
void om_ignite_set(void)
{
    g_om_need_ignite = RT_TRUE;
}

/*CRC校验，用于APP端数据检查*/
rt_uint8_t om_1_bt_calccrc(rt_uint8_t* buf, rt_uint8_t paramInt)
{
    rt_uint8_t i = 213;   /* 手机端配置值为213*/
    for (rt_uint8_t b = 0; b < paramInt; b++)
    {
        rt_uint8_t b1 = buf[b];
        if (b1 < 0)
        b1 += 256;
        i = ((i << 1 | i >> 7) + b1) % 256;
    }
    return i;
}










