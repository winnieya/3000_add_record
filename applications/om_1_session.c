/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-15     Tery       the first version
 */

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include <string.h>

#define DBG_TAG "om_1_session"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "om_1_session.h"
#include "om_1_data.h"
#include "om_1_ble.h"
#include "om_1_power.h"

#define UART_RS485_NAME                 "uart2"
#define UART_BLE_NAME                 "uart5"
#define CARRIAGE_RETURN                0x0D       /* 结束位设置为 \r，即回车符 */
#define LINE_FEED                      0x0A       /* 结束位设置为 \n，即回车符 */
#define DATA_HEART                     0x55       /* 结束位设置为 \n，即回车符 */
#define DATA_HEART_21                     0x5A       /* 结束位设置为 \n，即回车符 */
#define DATA_END_LEN                   2

#define BLE_AT_INIT     "AT+BTINIT=1\r\n"
#define BLE_AT_SPINIT   "AT+BTSPPINIT=2\r\n"
#define BLE_AT_NAME     "AT+BTNAME=\"OM3000\"\r\n"
#define BLE_AT_MODE     "AT+BTSCANMODE=2\r\n"
#define BLE_AT_BT_POWER     "AT+BTPOWER=5,5\r\n"
#define BLE_AT_BTSECPARAM "AT+BTSECPARAM=3,0\r\n"
#define BLE_AT_SP_START  "AT+BTSPPSTART\r\n"
#define BLE_AT_SPSEND    "AT+BTSPPSEND\r\n"
#define BLE_AT_RST    "AT+RST\r\n"
#define BLE_AT_SYSMSG    "AT+SYSMSG=5\r\n"
#define ESP32_SLEEP1   "AT+SLEEP=2"

#define READ_DATA              1
#define READ_CARRIAGE_RETURN   2
#define READ_LINE_FEED         3
#define READ_HEART             4
#define READ_NONE              5

#define READ_FRAME             6
#define READ_ACK               7
#define READ_LEN               8
#define CHECKSUM_LEN           1

#define DATA_LEN_MAX           64

#define REC_DATA_LEN    10

#define IS_CONTACT(x,y)  ((strstr(x,y) != NULL) > 0)? 1:0

static char g_rs485_rev_data[DATA_LEN_MAX]={0};
static char *p_rs485_read;
uint16_t rs485_parket_len = 0;
static rt_uint8_t g_rs485_next_read_state = READ_NONE;
static rt_uint16_t g_rs485_cur_buf_len = 0;

static rt_device_t ble_serial3;
static rt_device_t rs485_serial2;

static char om_g_ble_rev_data[DATA_LEN_MAX]={0};
static char *p_read;
static rt_uint8_t g_om_read_state = READ_NONE;
static rt_uint8_t g_om_cur_buf_len = 0;


/* 蓝牙数据发送，使用串口透传到蓝牙模块 */
/* rs485数据发送，使用串口透传到蓝牙模块 */
void rs485_send_data(void *buffer,uint16_t length)
{
    rt_pin_write(RS485_RE_PIN, PIN_HIGH);
    rt_pin_write(RS485_DE_PIN, PIN_HIGH);
    rt_device_write(rs485_serial2, 0, buffer, length);
    rt_pin_write(RS485_RE_PIN, PIN_LOW);
    rt_pin_write(RS485_DE_PIN, PIN_LOW);
}

rt_err_t rs485_send_cb(rt_device_t dev, void *buffer)
{
    rt_pin_write(RS485_RE_PIN, PIN_LOW);
    rt_pin_write(RS485_DE_PIN, PIN_LOW);
    return RT_EOK;
}

void om_rs485_pin_init(void)
{
    rt_pin_mode(RS485_RE_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RS485_RE_PIN, PIN_LOW);

    rt_pin_mode(RS485_DE_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RS485_DE_PIN, PIN_LOW);
}

uint8_t bt_send_[18]={0x41,0x54,0x2b,0x42,0x54,0x53,0x50,0x50,0x53,0x45,0x4e,0x44,0x3d,0x30,0x2c,0x34,0x0D,0x0A};
/* 蓝牙数据发送，使用串口透传到蓝牙模块 */
void om_device_data_report_prepaer(rt_size_t size)
{
    bt_send_[15] = size;
    rt_device_write(ble_serial3, 0, bt_send_, 18);

}
void om_send_data(void *buffer, rt_size_t size)
{

    rt_device_write(ble_serial3, 0, buffer, size);
    LOG_D("send data %s and size %d!\n", buffer,size);
}

/* 数据解析，使用状态机流转操作
 * 当没有检测到数据头部时，是READ_NONE状态，当检测到头部时，变成READ_HEART，继续有字节检测时，将状态改为
 * READ_DATA，在READ_DATA状态时，如果检测字节是CARRIAGE_RETURN，将状态改为READ_CARRIAGE_RETURN，
 * 如果检测到LINE_FEED，并且当前状态时READ_CARRIAGE_RETURN，则表示单个命令接收完成，处理命令，处理完后，将状态改为
 * READ_NONE，如果检测到LINE_FEED，并且当前状态不是READ_CARRIAGE_RETURN，则将状态改为READ_DATA
 * */
rt_uint8_t bt_data_len = 0;
rt_uint8_t bt_data_len_tmp = 0;
#if 0
void om_bt_uarts_data_parsing(void)
{
    rt_uint8_t read_len = 0;
    rt_uint16_t check_len = 0;

    read_len = rt_device_read(ble_serial3, 0, &om_g_ble_rev_data[g_om_cur_buf_len], sizeof(om_g_ble_rev_data) - g_om_cur_buf_len);
    LOG_D("read_len size %x!\n", read_len);

    if(read_len == 0 || read_len > 255||read_len < 7)
    {
        return;
    }

    if(memcmp(om_g_ble_rev_data,"+BTDATA",7) != 0)
    {
        rt_memset(om_g_ble_rev_data, 0, sizeof(om_g_ble_rev_data));
        g_om_cur_buf_len = 0;
        bt_data_len = 0;
        bt_data_len_tmp = 0;
        g_om_read_state = READ_NONE;
        p_read = om_g_ble_rev_data;
        LOG_I("BLE_AT_SPSEND  \r\n");
        LOG_I("receive error  \r\n");
        return;
    }
    LOG_D("check_len size %d!\n", check_len);
    read_len = read_len - 7;
    rt_memmove(om_g_ble_rev_data,&om_g_ble_rev_data[7],read_len);
    for(int i=0;i<read_len;i++)
    {
        LOG_D("%x",*p_read);
        switch(g_om_read_state)
        {
        case READ_NONE:
            if(*p_read == DATA_HEART || *p_read == DATA_HEART_21)
            {
                g_om_read_state = READ_HEART;
                if(*p_read == DATA_HEART_21)
                {
                    is_21_cmd = 1;
                }
                else {
                    is_21_cmd = 0;
                }
            }
            p_read++;
            break;
        case READ_HEART:
            bt_data_len = *p_read;
            bt_data_len_tmp = *p_read;
            g_om_read_state = READ_DATA;
//            LOG_I("bt_data_len: %d\r\n",bt_data_len);
            if(bt_data_len > 2)
            {
                bt_data_len -= 2;
//                LOG_I("bt_data_len: %d\r\n",bt_data_len);
            }
            else {
                goto loop2;
            }
            p_read++;
            break;
        case READ_DATA:
            bt_data_len--;
//            LOG_I("bt_data_len: %d\r\n",bt_data_len);
            if(bt_data_len == 0)
            {
                bt_data_len_tmp = p_read - om_g_ble_rev_data + 1;
                om_bt_data_handle((rt_uint8_t *)om_g_ble_rev_data,bt_data_len_tmp);

                goto loop2;
            }
            else {
                p_read++;
            }
            break;
        default:
            break;
        }

    }
loop2:
    rt_memset(om_g_ble_rev_data, 0, sizeof(om_g_ble_rev_data));
    g_om_cur_buf_len = 0;

    bt_data_len = 0;
    bt_data_len_tmp = 0;
    g_om_read_state = READ_NONE;
    p_read = om_g_ble_rev_data;
}
#else
void om_bt_uarts_data_parsing(void)
{
    rt_uint8_t read_len = 0;

    read_len = rt_device_read(ble_serial3, 0, om_g_ble_rev_data, 64);

    if(read_len == 0 || read_len > 255||read_len < 4)
    {
        goto loop2;
        return;
    }

    for(int i=0;i<read_len;i++)
    {
        switch(g_om_read_state)
        {
        case READ_NONE:
            if(*p_read == DATA_HEART )
            {
                g_om_read_state = READ_HEART;
            }
            p_read++;
            break;
        case READ_HEART:
            bt_data_len_tmp = *p_read;
            om_bt_data_handle((rt_uint8_t *)om_g_ble_rev_data,bt_data_len_tmp);
            goto loop2;
            break;
        default:
            p_read++;
            break;
        }

    }
loop2:
    rt_memset(om_g_ble_rev_data, 0, sizeof(om_g_ble_rev_data));
    g_om_cur_buf_len = 0;

    bt_data_len = 0;
    bt_data_len_tmp = 0;
    g_om_read_state = READ_NONE;
    p_read = om_g_ble_rev_data;
}
#endif
/*蓝牙使用到的串口初始化*/
int om_ble_uarts_init(void)
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    //om_ble_pin_init();

    rt_strncpy(uart_name, UART_BLE_NAME, RT_NAME_MAX);

    /* 查找系统中的串口设备 */
    ble_serial3 = rt_device_find(uart_name);
    if (!ble_serial3)
    {
        LOG_D("find %s failed!\n", uart_name);
        return RT_ERROR;
    }

    /* step2：修改串口配置参数 */
    config.baud_rate = BAUD_RATE_115200;        //修改波特率为 115200
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 256;                   //修改缓冲区 buff size 为 256
    config.parity    = PARITY_NONE;           //无奇偶校验位

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(ble_serial3, RT_DEVICE_CTRL_CONFIG, &config);
    /* 以中断接收及轮询发送模式打开串口设备 */
    rt_device_open(ble_serial3, RT_DEVICE_FLAG_INT_RX);


    bt_data_len = 0;
    bt_data_len_tmp = 0;
    g_om_read_state = READ_NONE;
    p_read = om_g_ble_rev_data;
    return ret;
}



/* 数据解析，使用状态机流转操作
 * 当没有检测到数据头部时，是READ_NONE状态，当检测到头部时，变成READ_HEART，继续有字节检测时，将状态改为
 * READ_DATA，在READ_DATA状态时，如果检测字节是CARRIAGE_RETURN，将状态改为READ_CARRIAGE_RETURN，
 * 如果检测到LINE_FEED，并且当前状态时READ_CARRIAGE_RETURN，则表示单个命令接收完成，处理命令，处理完后，将状态改为
 * READ_NONE，如果检测到LINE_FEED，并且当前状态不是READ_CARRIAGE_RETURN，则将状态改为READ_DATA
 * */
#if 1
#define FID_REPORT_DATA_LEN  11
void rs485_data_parsing(void)
{
    rt_uint16_t read_len = 0;

    read_len = rt_device_read(rs485_serial2, 0, g_rs485_rev_data, 20);

    if(read_len == 0 || read_len > 512) return;
    LOG_D("read_len %d\r\n",read_len);

    for(int i=0;i<read_len;i++)
    {
        if(g_rs485_rev_data[i] == DATA_HEART)
        {
            rs485_recv_data_parsing((uint8_t *)&g_rs485_rev_data[i],FID_REPORT_DATA_LEN);
            break;
        }
    }

    rt_memset(g_rs485_rev_data, 0, DATA_LEN_MAX);
}
#else
void rs485_data_parsing(void)
{
    rt_uint16_t read_len = 0;
    rt_uint16_t data_len = 0;

    read_len = rt_device_read(rs485_serial2, 0, g_rs485_rev_data, 20);

    if(read_len == 0 || read_len > 512) return;
    LOG_D("read_len %d\r\n",read_len);
    g_rs485_cur_buf_len += read_len;

    for(int i=0;i<read_len;i++)
    {
        switch(g_rs485_next_read_state)
        {
        case READ_NONE:
            if(*p_rs485_read == DATA_HEART)
            {
                g_rs485_next_read_state = READ_FRAME;
            }
            p_rs485_read++;
            break;
        case READ_FRAME:
            g_rs485_next_read_state = READ_ACK;
            p_rs485_read++;
            break;
        case READ_ACK:
            g_rs485_next_read_state = READ_LEN;
            p_rs485_read++;
            break;
        case READ_LEN:
            rs485_parket_len = (*p_rs485_read) <<8;
            p_rs485_read++;
            rs485_parket_len += (*p_rs485_read) ;
            p_rs485_read++;
            i++;
            LOG_D("rs485_parket_len %d\r\n",rs485_parket_len);
            if(rs485_parket_len >= 7 && rs485_parket_len < 500)
            {
                rs485_parket_len -= 5;
                g_rs485_next_read_state = READ_DATA;
            }
            else {
                LOG_D("485 receive data error \r\n");
                goto loop1;
            }

            break;
        case READ_DATA:
            rs485_parket_len--;
            if(rs485_parket_len == 0)
            {
                g_rs485_next_read_state = READ_CARRIAGE_RETURN;
            }
            p_rs485_read++;
            break;
        case READ_CARRIAGE_RETURN:
            if(*p_rs485_read == CARRIAGE_RETURN)
            {
                g_rs485_next_read_state = READ_LINE_FEED;
            }
            else
            {
                LOG_I("error data %x\r\n",*p_rs485_read);
                goto loop1;
            }

            p_rs485_read++;
            break;
        case READ_LINE_FEED:
            if(*p_rs485_read == LINE_FEED)
            {
                data_len = p_rs485_read - g_rs485_rev_data - 1;
                if(data_len < DATA_LEN_MAX)
                {
                    rs485_recv_data_parsing((uint8_t *)g_rs485_rev_data,data_len);
                }
            }
            else {
                LOG_I("p_rs485_read %d\r\n",*p_rs485_read);
            }
            goto loop1;


                return;
            break;
        default:
            break;
        }
    }
    if(g_rs485_next_read_state == READ_NONE)
    {
        rt_memset(g_rs485_rev_data, 0, sizeof(g_rs485_rev_data));
        p_rs485_read = g_rs485_rev_data;
        g_rs485_cur_buf_len = 0;
    }
loop1:
    rt_memset(g_rs485_rev_data, 0, DATA_LEN_MAX);
    g_rs485_cur_buf_len = 0;
    g_rs485_next_read_state = READ_NONE;
    p_rs485_read = g_rs485_rev_data;
}
#endif
/*RS485使用到的串口初始化*/
int om_rs485_init(void)
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* åˆå§‹åŒ–é…ç½®å‚æ•° */

    om_rs485_pin_init();

    p_rs485_read = g_rs485_rev_data;

    rt_strncpy(uart_name, UART_RS485_NAME, RT_NAME_MAX);


    rs485_serial2 = rt_device_find(uart_name);
    if (!rs485_serial2)
    {
        LOG_D("find %s failed!\n", uart_name);
        return RT_ERROR;
    }

    config.baud_rate = BAUD_RATE_115200;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.bufsz     = 128;
    config.parity    = PARITY_NONE;

    rt_device_control(rs485_serial2, RT_DEVICE_CTRL_CONFIG, &config);

    rt_device_set_tx_complete(rs485_serial2,rs485_send_cb);

    rt_device_open(rs485_serial2, RT_DEVICE_FLAG_INT_RX|RT_DEVICE_FLAG_INT_TX);

    return ret;
}

#define CHECKSUM_INDEX    6
static uint8_t _get_buf[9] = {0xA5,0x01,0x00,0x00,0x07,0x04,0x00,0x0D,0x0A};
void om_get_fid_value(void)
{
    _get_buf[CHECKSUM_INDEX] = om_1_bt_calccrc(_get_buf,CHECKSUM_INDEX);

    rs485_send_data(_get_buf,9);
    LOG_D("rs485_send_data\n");
}

void bt_data_parsing(void)
{

}


