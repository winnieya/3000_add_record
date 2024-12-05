/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-23     Tery       the first version
 */
#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#define fid_calib_file "fid_calib_file"
#define OM_MAX_GAS_COUNT 16
#define OM_MIN_CAL_POINT 2
#define OM_CURR_RECORD_MAX   10

struct _Standard_gas{
	uint16_t calib_val;
	uint32_t microcurr_val;
};
struct _fid_calibration_record{
    uint8_t calibration_flag;
	uint8_t point_counts;
    struct _Standard_gas Standard_gas[OM_MAX_GAS_COUNT];
};

typedef struct _current_value_record{
	uint8_t count_p;
	uint32_t microcurr[10];
}curr_val_record;


//struct _zero_calibration_fid zero_calibration_fid = {0};


void zero_calib_updata(uint32_t _gas_value,uint32_t _signal_value);
void calibration_record_reset(void);
void half_hour_timeout(void);
void set_half_hour_flag(rt_bool_t flag);
rt_bool_t om_is_need_correction(void);
void om_correction_reset(void);
void om_correction_set(void);
void electricity_correction_handle(int32_t* pico_amps);

#endif
