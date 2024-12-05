/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-23     Tery       the first version
 */
#ifndef OM_1_DATA_H_
#define OM_1_DATA_H_

#include <rtthread.h>

void om_bt_data_handle(rt_uint8_t* bt_data,rt_uint8_t data_len);

void om_1_assemble_packet(void);
rt_uint8_t om_get_ignite_type_and_state(void);
rt_bool_t om_is_need_ignite(void);
void om_ignite_reset(void);
void om_ignite_set(void);
void cal_process(uint8_t curve_type,uint16_t con_value);
void rs485_recv_data_parsing(uint8_t* rs485_data,uint16_t data_len);
rt_uint8_t om_1_bt_calccrc(rt_uint8_t* buf, rt_uint8_t paramInt);
uint8_t om_get_pid_state(void);

void om_set_ignite_state(uint8_t state);

#endif /* OM_1_DATA_H_ */
