#ifndef _DJI_API_H__
#define _DJI_API_H__



#include <stdint.h>

/*

	head	dev_id     cmd_id      len      data       check
	0xaa    (2byte)   (2byte)    (2byte)  (len byte)  (1byte)
*/

//--------user define type------------------

// dev_id
#define MY_DEV_ID         0x00

// cmd_id
#define API_STATUS_REQUEST    0x00
#define API_CTRL_REQUEST      0x01

typedef float f_t;
typedef struct _api_up_struct
{
	f_t q0;
	f_t q1;
	f_t q2;
	f_t q3;

	f_t vgx;
	f_t vgy;
	f_t vgz;

	f_t agx;
	f_t agy;
	f_t agz;

	f_t wx;
	f_t wy;
	f_t wz;


	double lati;
	double longti;
	f_t height;

    uint8_t status;
}fmu_api_sensor_data;


typedef struct
{
	uint16_t full_charge_capacity;
	uint16_t remaining_capacity;
	uint16_t pack_voltage;
	int16_t current;
	int16_t average_current;
	int16_t temperature;
	uint8_t capacity_percentage;
	uint8_t right;
}fmu_api_battery_data;

typedef struct
{
	uint16_t ack_command;
    uint8_t ack_result;
}fmu_api_ack_data;

typedef struct
{
    int16_t status_request;
}fmu_api_status_request_data;

typedef struct
{
	uint8_t ctrl_flag;
    f_t roll_or_x;
	f_t	pitch_or_y;
	f_t	thr_z;
	f_t	yaw;
	uint8_t sensor_flag;
	f_t	pos_z;
	f_t	vel_x;
	f_t	vel_y;
    f_t	vel_z;
}api_ctrl_data_t;

typedef struct
{
	uint8_t ctrl_flag;
	f_t 	roll_or_x;
	f_t	pitch_or_y;
	f_t	thr_z;
	f_t	yaw;
}api_ctrl_without_sensor_data_t;

//-------------end of user define type------------------

#define FMU_API_VERT_VEL 0
#define FMU_API_VERT_POS 1

#define FMU_API_HORI_ATTI_TILT_ANG 0
#define FMU_API_HORI_VEL 	   1
#define FMU_API_HORI_POS 	   2

#define FMU_API_YAW_ANG  0
#define FMU_API_YAW_RATE 1

#define GROUND_LEVEL 0
#define BODY_LEVEL   1
#define REF_LEVEL    2

#define GROUND_TORSION 0
#define BODY_TORSION   1


typedef enum
{
        API_UNPACK_HEADER = 0   ,
        API_UNPACK_DEVID    	,
        API_UNPACK_CMDID        ,
        API_UNPACK_LEN          ,
        API_UNPACK_DATA         ,
        API_UNPACK_CHECK        ,
//--------------------------------------
        API_UNPACK_ERR = -1
}api_unpack_step_u;


typedef enum
{
        API_UNPACK_RESULT_OK = 0    ,
        API_UNPACK_RESULT_ERR       ,
        API_UNPACK_RESULT_ING       ,
        API_UNPACK_RESULT_HEADER    ,
        API_UNPACK_FIFO_FULL
}api_unpack_result_u;


#define API_HEADER      (0xaa)
#define API_INTERFACE
#define DATA_MAX_SIZE   (200u)
#define ERR_INDEX       (0xffff)

typedef struct
{
        uint16_t    dev_id;
        uint16_t    cmd_id;
        uint8_t     len;
        uint8_t     data_buf[DATA_MAX_SIZE];
}dji_api_msg_t;


typedef uint8_t (*pfunc_write)(uint8_t* pbuf,uint8_t len);
typedef int16_t (*func_cmd_handler)(uint16_t cmd_id,uint8_t* pbuf,uint16_t len);

typedef struct _cmd_tab
{
    uint16_t            cmd_id;
    func_cmd_handler    pf_cmd_handler;
}cmd_handler_table_t;
typedef struct _dev_tab
{
    uint16_t                dev_id;
    cmd_handler_table_t*    p_cmd_handler_table;
}dev_handler_table_t;

typedef struct
{
    uint8_t     cur_pack_step;
    uint8_t     cur_pack_data_len;
    uint8_t     cur_pack_data_index;
    uint32_t    num_pack_per_second;
    uint32_t    num_send_pack_err_count;
    uint32_t    num_send_pack_all_count;
    uint32_t    num_recv_pack_err_count;
    uint32_t    num_recv_pack_all_count;
    pfunc_write pf_write;
}dji_protocol_state_t;

API_INTERFACE int16_t init_dji_api_protocol(pfunc_write pf_drc_drite);
API_INTERFACE int16_t dji_api_send_data(uint16_t dev_id,uint16_t cmd_id, uint8_t* pbuf,uint16_t len);
API_INTERFACE void dji_api_recv_ontick(uint8_t char_data,dev_handler_table_t* p_handler_tab);
int16_t dji_api_unpack_by_char(uint8_t char_data,dev_handler_table_t* p_handler_tab);
int16_t dji_api_pack_data2(uint16_t dev_id,uint16_t cmd_id,uint8_t*pbuf,uint8_t len);
#endif //DJI_API_H__
