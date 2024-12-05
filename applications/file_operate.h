/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-11-04     Winni      The first version.
 */
 
#ifndef file_operate_h
#define file_operate_h
#include <rtthread.h>
void file_sys_init(void);
int file_create(const char *file_name);
int file_read(const char *file_name,uint8_t *buffer,uint16_t read_len);
int file_write(const char *file_name,uint8_t *buffer,uint16_t write_len);
int file_is_exist(const char *file_name);
#endif