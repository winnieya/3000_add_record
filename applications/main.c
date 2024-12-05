/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-08     RT-Thread    first version
 */

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "om_1_sensor.h"
#include "om_1_power.h"
#include "om_1_session.h"
#include "om_1_ble.h"
#include "om_1_data.h"


#define OM_BT_DATA_REPORT_INTERVAL_TIME  300 /*ms*/
#define OM_BT_DATA_DELAY_INTERVAL_TIME  10 /*ms*/

static void om_init(void);
#define LED_POWER    GET_PIN(E,6)

int main(void)
{
    uint8_t count =0;
    om_init();
    om_start_monitor();

    while (1)
    {
        rs485_data_parsing();
        om_bt_uarts_data_parsing();
        /* 读取fid采集信号值 */
        om_get_fid_value();
        om_get_pid_signal();

        if(count == 25)
        {
            om_1_assemble_packet();
            count = 0;
        }
        count++;

        rt_thread_mdelay(OM_BT_DATA_DELAY_INTERVAL_TIME*2);
        LOG_D("bt data handle thread is running\n");
    }
    return RT_EOK;
}


static void om_init(void)
{
    om_1_SRC_init();
    om_1_LED_init(); /*照明辅助灯*/
    om_1_CAM_init();

    rt_pin_mode(LED_POWER, PIN_MODE_OUTPUT);
    rt_pin_write(LED_POWER, PIN_LOW);

    /*PID供电引脚*/
    rt_pin_mode(GET_PIN(E,0), PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(E,0), PIN_LOW);

    /*VD2 init*/
    rt_pin_mode(GET_PIN(E,3), PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(E,3), PIN_HIGH);

    om_1_ignite_init();
    om_adc_vol_init(); /*adc初始化*/

    om_rs485_init(); /*FID信号采集板rs485通信*/
    pid_spi_init();
    om_ble_uarts_init(); /*蓝牙模块*/

    set_pid_onoff(1);



    /*12V输出*/
    rt_pin_mode(GET_PIN(E, 1), PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(E, 1),PIN_HIGH);

}


