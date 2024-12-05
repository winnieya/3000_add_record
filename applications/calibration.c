#include <file_operate.h>
#include <calibration.h>
#include <string.h>

#define DBG_TAG "calibration"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_bool_t Current_correction_flag = 0;
static rt_bool_t half_hour_later = RT_FALSE; /*指示在点火半小时后*/
int32_t g_fid_correction_vla = 0;

struct _fid_calibration_record fid_calibration_val = {0};
//curr_val_record g_rt_signal_record = {0};
curr_val_record g_hestory_corr_val = {0};

void calibration_record_reset(void)
{
    memset((void *)(&fid_calibration_val) ,0,sizeof(struct _fid_calibration_record));
}

void half_hour_timeout(void)
{
	half_hour_later = RT_TRUE;
}

void set_half_hour_flag(rt_bool_t flag)
{
	half_hour_later = flag;
}

/*指示是否需要浓度矫正*/
rt_bool_t om_is_need_correction(void)
{
    return Current_correction_flag;
}

/*表示不需要浓度矫正*/
void om_correction_reset(void)
{
    Current_correction_flag = RT_FALSE;
}

/*表示需要浓度矫正*/
void om_correction_set(void)
{
    Current_correction_flag = RT_TRUE;
}

uint8_t record_count = 0;
int32_t om_averag_func(curr_val_record* data,int32_t pico_amps){
	uint8_t* tmp_point = &(data->count_p);
	int32_t ave_pico_amps = 0;
	uint8_t i = 0;
	
	data->microcurr[*tmp_point] = pico_amps;
	if(record_count < OM_CURR_RECORD_MAX){
		record_count++;
	}
	if(*tmp_point < OM_CURR_RECORD_MAX-1){
		(*tmp_point)++;
	}else{
		*tmp_point = 0;
	}
	for(; i < OM_CURR_RECORD_MAX && data->microcurr[i]; i++){
		ave_pico_amps += (data->microcurr[i]/(record_count));
	}
	return ave_pico_amps;
}

/*
 * 电流矫正函数
 * 用于长时间不使用设备，点火后电流值偏高
 * */
void electricity_correction_handle(int32_t* pico_amps){
	//int32_t ave_rt_signal = om_averag_func(&g_rt_signal_record,*pico_amps);
	
	/*半小时内*/
    if(!half_hour_later){

		/*当前平均微电流值小于校准电流*/
		if(*pico_amps < fid_calibration_val.Standard_gas[0].microcurr_val){
			g_fid_correction_vla = om_averag_func(&g_hestory_corr_val,0);
		}else{
			int32_t tmp_val = *pico_amps - fid_calibration_val.Standard_gas[0].microcurr_val;
			g_fid_correction_vla = om_averag_func(&g_hestory_corr_val,tmp_val);
		}
		
	}
	else//半小时后
	{
		if(g_fid_correction_vla == 0){
			om_correction_reset(); //矫正完成
			return;
		}
		g_fid_correction_vla = g_fid_correction_vla >= 100?g_fid_correction_vla-100:0;
	}
	(*pico_amps) = *pico_amps-g_fid_correction_vla;
}



/*校准更新*/
void zero_calib_updata(uint32_t _gas_value,uint32_t _signal_value)
{
    //struct _zero_calibration_fid *temp_zero_calibration_fid = NULL;

    for(int i=0;i<OM_MAX_GAS_COUNT;i++)
    {
        if(_gas_value == fid_calibration_val.Standard_gas[i].calib_val)
        {
            if(i==0 && fid_calibration_val.point_counts > 0 )
            {
                /*只能有一个零点，当出现第二个零点时，不进行更新*/
                LOG_I("repeat calibration_point 0 \r\n");
                return;
            }
            if(i!=0)
            {
                /*当新校准值已经存在时，电流值更新*/
                fid_calibration_val.Standard_gas[i].calib_val = _gas_value;
                fid_calibration_val.Standard_gas[i].microcurr_val = _signal_value;

                file_write(fid_calib_file,(uint8_t*)&fid_calibration_val,sizeof(fid_calibration_val));
                return;
            }
            break;
        }
    }
    fid_calibration_val.Standard_gas[fid_calibration_val.point_counts].calib_val = _gas_value;
    fid_calibration_val.Standard_gas[fid_calibration_val.point_counts].microcurr_val = _signal_value;
    if(fid_calibration_val.point_counts < OM_MAX_GAS_COUNT)
    {
        fid_calibration_val.point_counts+=1;
    }
    if(fid_calibration_val.point_counts >= OM_MIN_CAL_POINT)
    {
        fid_calibration_val.calibration_flag = 1;
    }
    
    file_write(fid_calib_file,(uint8_t*)&fid_calibration_val,sizeof(fid_calibration_val));
    
}


int fid_calibration_init(void)
{
    /*文件系统初始化，SPI flash挂载*/
    file_sys_init();

    /*创建文件*/
    file_create(fid_calib_file);

    file_read(fid_calib_file,(uint8_t*)&fid_calibration_val,sizeof(fid_calibration_val));
    LOG_I("file read\r\n");

    return 0;
}

INIT_ENV_EXPORT(fid_calibration_init);


