/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-15     Tery       the first version
 */
#ifndef OM_1_SESSION_H_
#define OM_1_SESSION_H_

#include <rtthread.h>

#define USEING_BT

#define RS485_RE_PIN     GET_PIN(B,8)
#define RS485_DE_PIN     GET_PIN(B,9)

typedef struct{
    rt_uint8_t resolve0;
    rt_uint8_t resolve1;
    rt_uint8_t resolve2;
    rt_uint8_t cmdid;
    rt_uint8_t IsPumpAOn ;
    rt_uint8_t IsSolenoidAOn;
    rt_uint8_t IsSolenoidBOn;
    rt_uint16_t ThermoCouple;
    rt_uint16_t BatteryVoltage;
    rt_uint16_t ChamberOuterTemp;
    rt_uint16_t SamplePressure;
    rt_uint16_t AirPressure;
    rt_uint16_t TankPressure;
    rt_uint8_t FIDRange;
    rt_uint32_t PicoAmps;
    rt_uint32_t RawPpm;
    rt_uint16_t SomtemCurrent;
    rt_uint8_t PumpPower;
}ble_data_report;

void om_sample(void);
int om_ble_uarts_init(void);
void om_bt_uarts_data_parsing(void);

void om_device_data_report_prepaer(rt_size_t size);
void om_send_data(void *buffer, rt_size_t size);
void om_bt_data_ctrl(void);

int om_rs485_init(void);
void rs485_data_parsing(void);
void esp32_init(void);
void hearbeat_reset(void);
void hearbeat_count(void);
uint8_t is_disconnect(void);
void om_get_fid_value(void);
uint8_t om_get_read_type(void);
#endif /* OM_1_SESSION_H_ */
