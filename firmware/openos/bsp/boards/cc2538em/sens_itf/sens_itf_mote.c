#include <string.h>
#include "openwsn.h"
#include "sens_itf.h"
#include "opentimers.h"
#include "scheduler.h"

#define SENS_ITF_DBG_FRAME  0
#define SENS_ITF_OUTPUT     1
#define SENS_ITF_SM_TICK_MS 250
// minimum tick time is 1
#define MS2TICK(ms) (ms) > SENS_ITF_SM_TICK_MS ? (ms) / SENS_ITF_SM_TICK_MS : 1

enum {
    SENS_ITF_STATE_INIT = 0,

    SENS_ITF_STATE_SEND_ITF_VER,
    SENS_ITF_STATE_WAIT_ITF_VER_ANS,
    SENS_ITF_STATE_PROC_ITF_VER,

    SENS_ITF_STATE_SEND_BRD_ID,
    SENS_ITF_STATE_WAIT_BRD_ID_ANS,
    SENS_ITF_STATE_PROC_BRD_ID,

    SENS_ITF_STATE_SEND_PT_DESC,
    SENS_ITF_STATE_WAIT_PT_DESC_ANS,
    SENS_ITF_STATE_PROC_PT_DESC,

    SENS_ITF_STATE_BUILD_SCH,
    SENS_ITF_STATE_RUN_SCH,
    SENS_ITF_STATE_SEND_POINT_VAL,
    SENS_ITF_STATE_PROC_POINT_VAL
};

enum {
	SENS_ITF_STATE_EXEC_OK = 0,
	SENS_ITF_STATE_EXEC_WAIT_OK,
	SENS_ITF_STATE_EXEC_WAIT_ABORTED,
	SENS_ITF_STATE_EXEC_WAIT_TIMEOUT,
	SENS_ITF_STATE_EXEC_ERROR
};

typedef struct sens_itf_mote_sm_state_s
{
	volatile uint16_t trmout_counter;
	volatile uint8_t point_index;
	volatile uint8_t frame_arrived;
	volatile uint8_t curr_state;
	volatile uint8_t last_state;
} sens_itf_mote_sm_state_t;

typedef uint8_t (*sens_itf_mote_sm_func_t)(sens_itf_mote_sm_state_t *st);

typedef struct sens_itf_mote_sm_table_s
{
	sens_itf_mote_sm_func_t func;
	uint16_t timeout;
	uint8_t next_state;
	uint8_t timeout_state; // for indicating timeout or end of cyclic operation
	uint8_t error_state;
} sens_itf_mote_sm_table_t;


typedef struct sens_itf_acq_schedule_s
{
	uint8_t num_of_points;
	struct
	{
		uint8_t index;
		uint32_t sampling_time_x250ms;
        uint32_t counter;
	} points[SENS_ITF_MAX_POINTS];
} sens_itf_acq_schedule_t;


static sens_itf_mote_sm_state_t sm_state;

static sens_itf_point_ctrl_t sensor_points;
static sens_itf_cmd_brd_id_t board_info;
static const uint8_t datatype_sizes[] = { 1, 1, 2, 2, 4, 4, 8, 8, 4, 8 }; // check sens_itf_datatypes_e order
static sens_itf_acq_schedule_t acquisition_schedule;
//static os_timer_t acquistion_timer = 0;
//static os_serial_t serial = 0;
static uint8_t frame[SENS_ITF_MAX_FRAME_SIZE];
static sens_itf_cmd_req_t cmd;
static sens_itf_cmd_res_t ans;

#if 0





static uint8_t sens_itf_mote_recv_frame(uint8_t *frame, uint8_t size)
{
    int16_t recv;
    return (recv < 0 ? 0 : (uint8_t) recv); // CHECK AGAIN
}

static uint8_t sens_itf_mote_get_point_type(uint8_t point)
{
    return sensor_points.points[point].desc.type;
}



static uint8_t sens_itf_mote_get_points_desc(uint8_t point)
{

}

static uint8_t sens_itf_mote_get_points_value(uint8_t point)
{
	uint8_t size;
    uint8_t cmd_size = 4;
    uint8_t ans_size = 6 + datatype_sizes[sensor_points.points[point].desc.type];

    cmd.hdr.addr = SENS_ITF_REGMAP_READ_POINT_DATA_1 + point;

    size = sens_itf_pack_cmd_req(&cmd, frame);
    if (size != cmd_size)
        return 0;

    if (sens_itf_mote_send_frame(frame, cmd_size) != cmd_size)
        return 0;

    os_kernel_sleep(2000);

    size = sens_itf_mote_recv_frame(frame, ans_size);
    if (size != ans_size)
        return 0;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);
    if (size != ans_size)
        return 0;

    return 1;
}

static uint8_t sens_itf_mote_set_points_value(uint8_t point)
{
	uint8_t size;
    uint8_t cmd_size = 5 + datatype_sizes[sensor_points.points[point].desc.type];
    uint8_t ans_size = 5;

    cmd.hdr.addr = SENS_ITF_REGMAP_WRITE_POINT_DATA_1 + point;

    size = sens_itf_pack_cmd_req(&cmd, frame);
    if (size != cmd_size)
        return 0;

    if (sens_itf_mote_send_frame(frame, cmd_size) != cmd_size)
        return 0;

    os_kernel_sleep(2000);

    size = sens_itf_mote_recv_frame(frame, ans_size);
    if (size != ans_size)
        return 0;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);
    if (size != ans_size)
        return 0;

    return 1;
}

static void sens_itf_mote_build_acquisition_schedule(void)
{
}

static void sens_itf_mote_read_point(uint8_t index)
{
    /*
    NEXT STEP: JUST ADD A NEW TASK TO SCHEDULER, DO NOT READ THE POINT
    */
    uint8_t ret;
    OS_UTIL_LOG(1,("Reading point %d\n", index));
    ret = sens_itf_mote_get_points_value(index);
    if (ret)
    {
        // update
        memcpy(&sensor_points.points[index].value, &ans.payload.point_value_cmd, sizeof(sens_itf_cmd_point_t));
    }

}

static void sens_itf_mote_acquisition_timer_func(void)
{
    uint8_t n;

    for (n = 0; n < acquisition_schedule.num_of_points; n++)
    {
        acquisition_schedule.points[n].counter--;
        if (acquisition_schedule.points[n].counter == 0)
        {
            sens_itf_mote_read_point(acquisition_schedule.points[n].index);
            acquisition_schedule.points[n].counter = acquisition_schedule.points[n].sampling_time_x250ms;
        }
    }
    
}


static uint8_t sens_itf_mote_check_version(void)
{

}

static uint8_t sens_itf_mote_request_board_id(void)
{
}

static uint8_t sens_itf_mote_check_board_id(void)
{

}

void sens_itf_mote_sm(void)
{
#ifdef TRACE_ON
    printf("sens_itf_mote_main\n");
#endif

    switch(sens_itf_state)
    {
        case SENS_ITF_STATE_NO_BRD:

            break;
        case SENS_ITF_STATE_ITF_VER:

            
            break;
        case SENS_ITF_STATE_BRD_ID:
            if(sens_itf_frame_received)
            {
                if(sens_itf_mote_check_board_id())
                {
            
            break;
        case SENS_ITF_STATE_GET_POINT_DESC:
            if(sens_itf_frame_received)
            {
                if(sens_itf_mote_check_point_desc())
                {
                    sens_itf_timeout = 0;
                    sens_itf_frame_received = 0;
                    sens_itf_point_desc++;
                    if(sens_itf_point_desc >= board_info.num_of_points)
                    {
                        sens_itf_mote_build_acquisition_schedule();
                        if (acquisition_schedule.num_of_points > 0)
                        {
                            sens_itf_state = SENS_ITF_STATE_DATA_POLLING;
                        }
                        else
                        {
                            sens_itf_state = SENS_ITF_STATE_IDLE;
                        }
                    }
                    else
                    {
                        sens_itf_mote_request_point_desc(sens_itf_point_desc);
                    }
                }
            }        
            else
            {
                sens_itf_timeout++;
                if(sens_itf_timeout >= 3)
                {
                    sens_itf_state = SENS_ITF_STATE_NO_BRD;
                }
            }
            
            break;
        case SENS_ITF_STATE_DATA_POLLING:
            break;
        case SENS_ITF_STATE_IDLE:
        case SENS_ITF_STATE_ERROR:
        default:
            break;
    }
}


/*
// Serial or SPI interrupt, called when a new byte is received
static void sens_itf_sensor_rx_byte(void)
{
    uint8_t value;
    
    // DISABLE INTERRUPTS
    if (frame_timeout)
    {   
        // empty register
        return;
    }

	value = 0;// PUT CHANNEL HERE (uint8_t) pcSerial.getc();
	
    if (num_rx_bytes < SENS_ITF_MAX_FRAME_SIZE)
        rx_frame[num_rx_bytes] = value;
    
    num_rx_bytes++;
    if (num_rx_bytes >= SENS_ITF_MAX_FRAME_SIZE)
        num_rx_bytes = 0;

    // ENABLE INTERRUPTS
}
*/

#endif

extern sens_itf_mote_sm_table_t sens_itf_mote_sm_table[];

static uint8_t sens_itf_mote_send_frame(uint8_t *frame, uint8_t size)
{
    //int16_t sent;
   // return (sent < 0 ? 0 : (uint8_t) sent); // CHECK AGAIN
	return 0;
}


static uint8_t sens_itf_mote_sm_func_build_sch(sens_itf_mote_sm_state_t *st)
{
	uint8_t n, m;

    acquisition_schedule.num_of_points = 0;

    for (n = 0, m = 0; n < board_info.num_of_points; n++)
    {
        if ((sensor_points.points[n].desc.access_rights & SENS_ITF_ACCESS_READ_ONLY) &&
            (sensor_points.points[n].desc.sampling_time_x250ms > 0))
        {
            acquisition_schedule.points[m].index = n;
            acquisition_schedule.points[m].counter = sensor_points.points[n].desc.sampling_time_x250ms;;
            acquisition_schedule.points[m].sampling_time_x250ms = sensor_points.points[n].desc.sampling_time_x250ms;

            m++;
            acquisition_schedule.num_of_points++;
        }
    }

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_pt_desc_ans(sens_itf_mote_sm_state_t *st)
{
	uint8_t size;
    uint8_t ans_size = 20;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);

    if (size != ans_size)
        return SENS_ITF_STATE_EXEC_ERROR;

    memcpy(&sensor_points.points[st->point_index].desc,&ans.payload.point_desc_cmd,sizeof(sens_itf_cmd_point_desc_t));

    st->point_index++;
    sensor_points.num_of_points = st->point_index;

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_req_pt_desc(sens_itf_mote_sm_state_t *st)
{
	uint8_t size;
    uint8_t cmd_size = 4;

    if(st->point_index >= board_info.num_of_points)
    	return SENS_ITF_STATE_EXEC_WAIT_TIMEOUT;

    cmd.hdr.addr = SENS_ITF_REGMAP_POINT_DESC_1 + st->point_index;

    size = sens_itf_pack_cmd_req(&cmd, frame);

    if (size != cmd_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    if (sens_itf_mote_send_frame(frame, cmd_size) != cmd_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_proc_brd_id_ans(sens_itf_mote_sm_state_t *st)
{
	uint8_t size;
    uint8_t ans_size = 28;

    st->point_index = 0;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);

    if (size != ans_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    memcpy(&board_info, &ans.payload.brd_id_cmd, sizeof(sens_itf_cmd_brd_id_t));

    if ((board_info.num_of_points == 0) || (board_info.num_of_points > SENS_ITF_MAX_POINTS))
    	return SENS_ITF_STATE_EXEC_ERROR;

    sensor_points.num_of_points = 0;

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_req_brd_id(sens_itf_mote_sm_state_t *st)
{
	uint8_t size;
    uint8_t cmd_size = 4;

	cmd.hdr.size = cmd_size;
	cmd.hdr.addr = SENS_ITF_REGMAP_BRD_ID;

    size = sens_itf_pack_cmd_req(&cmd, frame);

    if (size != cmd_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    if (sens_itf_mote_send_frame(frame, cmd_size) != cmd_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_proc_itf_ver_ans(sens_itf_mote_sm_state_t *st)
{
    uint8_t size;
    uint8_t ans_size = 6;

    cmd.hdr.addr = SENS_ITF_REGMAP_ITF_VERSION;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);

    if (size != ans_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    if ((SENS_ITF_ANS_OK != ans.hdr.status) || (SENS_ITF_LATEST_VERSION != ans.payload.itf_version_cmd.version))
    	return SENS_ITF_STATE_EXEC_ERROR;

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_wait_ans(sens_itf_mote_sm_state_t *st)
{
	if(st->frame_arrived)
		return SENS_ITF_STATE_EXEC_WAIT_ABORTED;

	sm_state.trmout_counter++;

	if(sm_state.trmout_counter > sens_itf_mote_sm_table[sm_state.last_state].timeout)
		return SENS_ITF_STATE_EXEC_WAIT_TIMEOUT;

	return SENS_ITF_STATE_EXEC_WAIT_OK;
}

static uint8_t sens_itf_mote_sm_func_req_ver(sens_itf_mote_sm_state_t *st)
{
    uint8_t size;
    uint8_t cmd_size = 4;

    cmd.hdr.addr = SENS_ITF_REGMAP_ITF_VERSION;

    size = sens_itf_pack_cmd_req(&cmd, frame);

    if (size != cmd_size)
        return SENS_ITF_STATE_EXEC_ERROR;

    if (sens_itf_mote_send_frame(frame, cmd_size) != cmd_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    return SENS_ITF_STATE_EXEC_OK;
}


static uint8_t sens_itf_mote_sm_func_init(sens_itf_mote_sm_state_t *st)
{
	uint8_t ret = SENS_ITF_STATE_EXEC_OK;

	memset(&cmd, 0, sizeof(cmd));
	memset(&ans, 0, sizeof(ans));
    memset(&sensor_points, 0, sizeof(sensor_points));
	memset(&board_info, 0, sizeof(board_info));
	memset(&acquisition_schedule, 0, sizeof(acquisition_schedule));
	memset(st, 0, sizeof(sens_itf_mote_sm_state_t));

	return ret;
}

void sens_itf_mote_sm(void)
{
	uint8_t ret;

	sm_state.last_state = sm_state.curr_state;
	ret = sens_itf_mote_sm_table[sm_state.curr_state].func(&sm_state);

	switch(ret)
	{
		case SENS_ITF_STATE_EXEC_OK:
		case SENS_ITF_STATE_EXEC_WAIT_ABORTED:
			sm_state.curr_state = sens_itf_mote_sm_table[sm_state.curr_state].next_state;
			sm_state.trmout_counter = 0;
			break;
		case SENS_ITF_STATE_EXEC_WAIT_OK:
			// still waiting
			break;
		case SENS_ITF_STATE_EXEC_WAIT_TIMEOUT:
			// wait timeout
			sm_state.curr_state = sens_itf_mote_sm_table[sm_state.curr_state].timeout_state;
			break;
		case SENS_ITF_STATE_EXEC_ERROR:
		default:
			sm_state.curr_state = sens_itf_mote_sm_table[sm_state.curr_state].error_state;
			break;
	}

	#ifdef TRACE_ON
    printf("sens_itf_mote_sm %d -> %d\n",sm_state.last_state,sm_state.curr_state);
	#endif
}

sens_itf_mote_sm_table_t sens_itf_mote_sm_table[] =
{     //{ func,                                 tmrout_val, next_state,                 tmrout_state/end_state,         error_state         }
		{ sens_itf_mote_sm_func_init,                     0, SENS_ITF_STATE_SEND_ITF_VER,     SENS_ITF_STATE_INIT,      SENS_ITF_STATE_INIT },
		{ sens_itf_mote_sm_func_req_ver,      MS2TICK(5000), SENS_ITF_STATE_WAIT_ITF_VER_ANS, SENS_ITF_STATE_INIT,      SENS_ITF_STATE_INIT },
		{ sens_itf_mote_sm_func_wait_ans,                 0, SENS_ITF_STATE_PROC_ITF_VER,     SENS_ITF_STATE_INIT,      SENS_ITF_STATE_INIT },
		{ sens_itf_mote_sm_func_proc_itf_ver_ans,         0, SENS_ITF_STATE_SEND_BRD_ID,      SENS_ITF_STATE_INIT,      SENS_ITF_STATE_INIT },
		{ sens_itf_mote_sm_func_req_brd_id,               0, SENS_ITF_STATE_WAIT_BRD_ID_ANS,  SENS_ITF_STATE_INIT,      SENS_ITF_STATE_INIT },
		{ sens_itf_mote_sm_func_wait_ans,     MS2TICK(5000), SENS_ITF_STATE_PROC_BRD_ID,      SENS_ITF_STATE_INIT,      SENS_ITF_STATE_INIT },
		{ sens_itf_mote_sm_func_proc_brd_id_ans,          0, SENS_ITF_STATE_SEND_PT_DESC,     SENS_ITF_STATE_INIT,      SENS_ITF_STATE_INIT },
		{ sens_itf_mote_sm_func_req_pt_desc,  MS2TICK(5000), SENS_ITF_STATE_WAIT_PT_DESC_ANS, SENS_ITF_STATE_BUILD_SCH, SENS_ITF_STATE_INIT },
		{ sens_itf_mote_sm_func_wait_ans,     MS2TICK(5000), SENS_ITF_STATE_PROC_PT_DESC,     SENS_ITF_STATE_INIT,      SENS_ITF_STATE_INIT },
		{ sens_itf_mote_sm_func_pt_desc_ans,              0, SENS_ITF_STATE_SEND_PT_DESC,     SENS_ITF_STATE_INIT,      SENS_ITF_STATE_INIT },
		{ sens_itf_mote_sm_func_build_sch,                0, SENS_ITF_STATE_RUN_SCH,          SENS_ITF_STATE_INIT,      SENS_ITF_STATE_INIT },
};

static void sens_itf_mote_tick(void)
{
    scheduler_push_task((task_cbt) sens_itf_mote_sm, TASKPRIO_SENS_ITF_MAIN);
}

uint8_t sens_itf_mote_init(void)
{

	memset(&sm_state, 0, sizeof(sens_itf_mote_sm_state_t));
	sm_state.curr_state = SENS_ITF_STATE_INIT;

    opentimers_start(SENS_ITF_SM_TICK_MS, TIMER_PERIODIC, TIME_MS, (opentimers_cbt) sens_itf_mote_tick);

    return 0;
}


