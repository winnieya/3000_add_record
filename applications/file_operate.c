/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-11-04     Winni      The first version.
 */

//#include <dfs_port.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <fal.h>
#include <dfs_fs.h>
#include <dfs_posix.h>
#include "file_operate.h"

#define DBG_TAG "file_operate"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "drv_spi.h"
#include "spi_flash_sfud.h"

rt_spi_flash_device_t norflash0 = NULL;
#define FS_PARTITION_NAME "filesystem"

int rt_hw_spi_flash_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    rt_hw_spi_device_attach("spi2", "spi20", GPIOB, GPIO_PIN_12);

    if(rt_sfud_flash_probe("norflash0","spi20"))
    {
        rt_kprintf("rt sfud flash probe seccess \r\n");
        return RT_EOK;

    }
    else {
        rt_kprintf("rt sfud flash probe fail \r\n");
        return -RT_ERROR;
    }
}

int dfs_mount_init(void)
{
    /* 在 spi flash 中名为 "filesystem" 的分区上创建一个块设备 */
    struct rt_device *flash_dev = fal_blk_device_create(FS_PARTITION_NAME);
    if (flash_dev == NULL)
    {
        LOG_E("Can't create a block device on '%s' partition.", FS_PARTITION_NAME);
    }
    else
    {
        LOG_D("Create a block device on the %s partition of flash successful.", FS_PARTITION_NAME);
    }

    /* 挂载 spi flash 中名为 "filesystem" 的分区上的文件系统 */
    if (dfs_mount(flash_dev->parent.name, "/", "elm", 0, 0) == 0)
    {
        LOG_I("Filesystem initialized!");
    }
    else /*如果挂载失败，格式化文件系统类型，重新挂载*/
    {
        dfs_mkfs("elm", flash_dev->parent.name);
        if (dfs_mount(flash_dev->parent.name, "/", "elm", 0, 0) == 0)
        {
            LOG_I("Filesystem initialized!");
        }
        else {
            LOG_E("Failed to initialize filesystem!");
            LOG_D("You should create a filesystem on the block device first!");
        }
    }
    return 0;
}

/* 创建文件文件，返回文件操作符，否则返回*/
int file_create(const char *file_name)
{
    int fd;
    /* 打开/text.txt 作写入，如果该文件不存在则建立该文件*/
    fd = open(file_name, O_CREAT);
    if (fd < 0)
    {
        LOG_I("file create fail");
    }
    close(fd);
    return fd;
}

/*从文件中读取特定长度数据*/
int file_read(const char *file_name,uint8_t *buffer,uint16_t read_len)
{
    int fd;
    int length;

    /* 只写 &  打开 */
    fd = open(file_name, O_RDONLY, 0);
    if (fd < 0)
    {
        rt_kprintf("open file for read  failed\n");
        return -1;
    }

    /* 读取数据 */
    length = read(fd, buffer, read_len);
    if (length != read_len)
    {
        rt_kprintf("check: read file failed %d %d\n",length,read_len);
        close(fd);
        return -1;
    }
    return length;
}

/*从文件中写入特定长度数据*/
int file_write(const char *file_name,uint8_t *buffer,uint16_t write_len)
{
    int fd;
    int length;

    /* 只写 & 打开 */
    fd = open(file_name, O_WRONLY  | O_TRUNC, 0);
    if (fd < 0)
    {
        rt_kprintf("open file for write failed\n");
        return -1;
    }


    /* 写入数据 */
    length = write(fd, buffer, write_len);
    if (length != write_len)
    {
        rt_kprintf("write data failed\n");
        close(fd);
        return -1;
    }

    /* 关闭文件 */
    close(fd);

    return 1;
}

int file_is_exist(const char *file_name)
{
    struct stat buf;

    stat(file_name, &buf);

    rt_kprintf("text.txt file size = %d\n", buf.st_size);

    if(buf.st_size > 0)
    {
        return 1;
    }
    return 0;
}


void file_sys_init(void){
	rt_hw_spi_flash_init();
    fal_init();
    dfs_mount_init();
}

