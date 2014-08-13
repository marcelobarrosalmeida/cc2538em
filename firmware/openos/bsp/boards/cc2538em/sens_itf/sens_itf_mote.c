#include "board.h"
#include "stdio.h"
#include <string.h>
#include "hw_ioc.h"             // Access to IOC register defines
#include "hw_ssi.h"             // Access to SSI register defines
#include "hw_sys_ctrl.h"        // Clocking control
#include "hw_memmap.h"
#include "ioc.h"                // Access to driverlib ioc fns
#include "gpio.h"               // Access to driverlib gpio fns
#include "openwsn.h"
#include "sens_itf.h"
#include "opentimers.h"
#include "scheduler.h"
#include "board.h"
#include "sys_ctrl.h"
#include "sens_itf_mote.h"
#include "opencoap.h"
#include "uarthal.h"
#include "leds.h"
#include "uart.h"
#include "packetfunctions.h"
#include "openqueue.h"

// minimum tick time is 1
#define MS2TICK(ms) (ms) > SENS_ITF_SM_TICK_MS ? (ms) / SENS_ITF_SM_TICK_MS : 1

enum {
    SENS_ITF_STATE_INIT = 0,
    SENS_ITF_STATE_SEND_ITF_VER = 1,
    SENS_ITF_STATE_WAIT_ITF_VER_ANS = 2,
    SENS_ITF_STATE_PROC_ITF_VER = 3,
    SENS_ITF_STATE_SEND_BRD_ID = 4,
    SENS_ITF_STATE_WAIT_BRD_ID_ANS = 5,
    SENS_ITF_STATE_PROC_BRD_ID = 6,
    SENS_ITF_STATE_SEND_PT_DESC = 7,
    SENS_ITF_STATE_WAIT_PT_DESC_ANS = 8,
    SENS_ITF_STATE_PROC_PT_DESC = 9,
    SENS_ITF_STATE_BUILD_SCH = 10,
    SENS_ITF_STATE_RUN_SCH = 11,
    SENS_ITF_STATE_SEND_PT_VAL = 12,
    SENS_ITF_STATE_WAIT_PT_VAL_ANS = 13,
    SENS_ITF_STATE_PROC_PT_VAL = 14
};

enum {
	SENS_ITF_STATE_EXEC_OK = 0,
	SENS_ITF_STATE_EXEC_WAIT_OK,
	SENS_ITF_STATE_EXEC_WAIT_STOP,
	SENS_ITF_STATE_EXEC_WAIT_ABORT,
	SENS_ITF_STATE_EXEC_ERROR
};

typedef struct sens_itf_mote_sm_state_s
{
	volatile uint16_t trmout_counter;
	volatile uint16_t trmout;
	volatile uint8_t point_index;
	volatile uint8_t frame_arrived;
	volatile uint8_t state;
	volatile uint8_t retries;
} sens_itf_mote_sm_state_t;

typedef uint8_t (*sens_itf_mote_sm_func_t)(sens_itf_mote_sm_state_t *st);

typedef struct sens_itf_mote_sm_table_s
{
	sens_itf_mote_sm_func_t func;
	uint8_t next_state;
	uint8_t abort_state; // for indicating timeout or end of cyclic operation
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
	} points[OSENS_MAX_POINTS];

	struct
	{
		uint8_t num_of_points;
		uint8_t index[OSENS_MAX_POINTS];
	} scan;
} sens_itf_acq_schedule_t;



static uint8_t sens_itf_mote_sm_func_build_sch(sens_itf_mote_sm_state_t *st);
static uint8_t sens_itf_mote_sm_func_pt_desc_ans(sens_itf_mote_sm_state_t *st);
static uint8_t sens_itf_mote_sm_func_req_pt_desc(sens_itf_mote_sm_state_t *st);
static uint8_t sens_itf_mote_sm_func_run_sch(sens_itf_mote_sm_state_t *st);

//=========================== variables =======================================

typedef struct {
   coap_resource_desc_t desc;
   opentimer_id_t  timerId;
} sensor_vars_t;

const uint8_t sensor_path0[]        = "h";

const sens_itf_mote_sm_table_t sens_itf_mote_sm_table[];

const uint8_t datatype_sizes[] = { 1, 1, 2, 2, 4, 4, 8, 8, 4, 8 }; // check sens_itf_datatypes_e order

sensor_vars_t sensor_vars;

uint8_t num_rx_bytes;
uint8_t tx_data_len;
uint32_t flagErrorOccurred;

uint8_t frame[SENS_ITF_MAX_FRAME_SIZE];

sens_itf_cmd_req_t cmd;
sens_itf_cmd_res_t ans;

sens_itf_mote_sm_state_t sm_state;
sens_itf_point_ctrl_t sensor_points;
osens_brd_id_t board_info;
sens_itf_acq_schedule_t acquisition_schedule;


//=========================== prototypes =======================================
//=========================== public ==========================================
uint8_t sensor_receive(OpenQueueEntry_t* msg, coap_header_iht*  coap_header, coap_option_iht* coap_options);
void sensor_timer(void);
static void sens_itf_mote_tick(void);
static void buBufFlush(void);
void bspLedToggle(uint8_t ui8Leds);
void sensor_sendDone(OpenQueueEntry_t* msg, owerror_t error);

uint8_t sens_itf_mote_init(void) {

#if (ENABLE_UART0_DAG == 0)
	memset(&sm_state, 0, sizeof(sens_itf_mote_sm_state_t));
	sm_state.state = SENS_ITF_STATE_INIT;

	//buBufFlush();
    opentimers_start(SENS_ITF_SM_TICK_MS, TIMER_PERIODIC, TIME_MS, (opentimers_cbt) sens_itf_mote_tick);


    // prepare the resource descriptor
/*

    sensor_vars.desc.path0len            = sizeof(sensor_path0)-1;
    sensor_vars.desc.path0val            = (uint8_t*)(&sensor_path0);
    sensor_vars.desc.path1len            = 0;
    sensor_vars.desc.path1val            = NULL;
    sensor_vars.desc.componentID         = COMPONENT_RHELI;    //MUDAR
    sensor_vars.desc.callbackRx          = &sensor_receive;
    sensor_vars.desc.callbackSendDone    = &sensor_sendDone;

    opencoap_register(&sensor_vars.desc);
    sensor_vars.timerId    = opentimers_start(1000,
                                             TIMER_PERIODIC,TIME_MS,
                                             sensor_timer);

*/
#endif

    return 0;
}

//=========================== private ==========================================

owerror_t sensor_receive(OpenQueueEntry_t* msg,
                      coap_header_iht*  coap_header,
                      coap_option_iht*  coap_options) {
   owerror_t outcome;
   uint8_t rawdata[127];

   if (coap_header->Code==COAP_CODE_REQ_POST)
   {
	   //TODO!!!! INCLUDE WRITE SENSOR HERE

      // start/stop the data generation to data server
      if (msg->payload[0]=='1')
      {
         //restart timer
        // opentimers_restart(rxl1_vars.timerId);

      } else {
         //stop timer
         //opentimers_stop(rxl1_vars.timerId);
      }

      // reset packet payload
      msg->payload                     = &(msg->packet[127]);
      msg->length                      = 0;

      // CoAP header
      coap_header->Code                = COAP_CODE_RESP_VALID;

      outcome = E_SUCCESS;

   }
   else if (coap_header->Code==COAP_CODE_REQ_GET)
   {
      // return current sensor value

      // reset packet payload
      msg->payload                     = &(msg->packet[127]);
      msg->length                      = 0;

      // CoAP payload (8 bytes of XL data)
      packetfunctions_reserveHeaderSize(msg,9);

      //sensitive_accel_temperature_get_measurement(&rawdata[0]);

      msg->payload[0] = COAP_PAYLOAD_MARKER;

      memcpy(&msg->payload[1],&rawdata[8],8);

      // set the CoAP header
      coap_header->Code                = COAP_CODE_RESP_CONTENT;

      outcome                          = E_SUCCESS;

   }
   else {
      // return an error message
      outcome = E_FAIL;
   }

   return outcome;
}

void sensor_timer() {
   OpenQueueEntry_t* pkt;
   owerror_t           outcome;
   uint8_t           numOptions;
   uint8_t           rawdata[127];


   // create a CoAP RD packet
   pkt = openqueue_getFreePacketBuffer(COMPONENT_RXL1);
   if (pkt==NULL) {
      openserial_printError(COMPONENT_RXL1,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      openqueue_freePacketBuffer(pkt);
      return;
   }
   // take ownership over that packet
   pkt->creator    = COMPONENT_RXL1;
   pkt->owner      = COMPONENT_RXL1;
   // CoAP payload (2 bytes of temperature data)
   packetfunctions_reserveHeaderSize(pkt,2);
   //sensitive_accel_temperature_get_measurement(&rawdata[0]);
   pkt->payload[0] = rawdata[8];
   pkt->payload[1] = rawdata[9];
   numOptions = 0;
   // location-path option
   packetfunctions_reserveHeaderSize(pkt,sizeof(sensor_path0)-1);
   memcpy(&pkt->payload[0],&sensor_path0,sizeof(sensor_path0)-1);
   packetfunctions_reserveHeaderSize(pkt,1);
   //pkt->payload[0]                  = ((COAP_OPTION_NUM_URIPATH << (4 | sizeof(sensor_path0)-1);
   numOptions++;
   // content-type option
   packetfunctions_reserveHeaderSize(pkt,2);
   //pkt->payload[0]                  = COAP_OPTION_NUM_CONTENTFORMAT << 4 | 1;
   pkt->payload[1]                  = COAP_MEDTYPE_APPOCTETSTREAM;
   numOptions++;
   // metadata
   pkt->l4_destination_port         = WKP_UDP_COAP;
   pkt->l3_destinationAdd.type = ADDR_128B;
   memcpy(&pkt->l3_destinationAdd.addr_128b[0],&ipAddr_motesEecs,16);
   // send
   outcome = opencoap_send(pkt,
                           COAP_TYPE_NON,
                           COAP_CODE_REQ_PUT,
                           numOptions,
                           &sensor_vars.desc);
   // avoid overflowing the queue if fails
   if (outcome==E_FAIL) {
      openqueue_freePacketBuffer(pkt);
   }


   return;
}

void sensor_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}

static uint8_t sens_itf_mote_pack_send_frame(sens_itf_cmd_req_t *cmd, uint8_t cmd_size)
{
    uint8_t size;

    size = sens_itf_pack_cmd_req(cmd, frame);

    if (size != cmd_size)
        return SENS_ITF_STATE_EXEC_ERROR;

    if (sens_itf_mote_send_frame(frame, cmd_size) != cmd_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_pt_val_ans(sens_itf_mote_sm_state_t *st)
{
	uint8_t point,ui8Len;
	uint8_t size,ret;
    uint8_t ans_size;
    uint8_t *pui8Buf;
    uint32_t ui32Data;

    point = acquisition_schedule.scan.index[st->point_index];
    ans_size = 6 + datatype_sizes[sensor_points.points[point].desc.type];

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);

    // retry ?
    if (size != ans_size || ans.hdr.addr != (SENS_ITF_REGMAP_READ_POINT_DATA_1 + point))
        return SENS_ITF_STATE_EXEC_OK;

    // ok, save and go to the next
    memcpy(&sensor_points.points[point].value, &ans.payload.point_value_cmd, sizeof(osens_point_t));

    st->retries = 0;
    st->point_index++;

	leds_debug_toggle();  //LED3

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_req_pt_val(sens_itf_mote_sm_state_t *st)
{
    uint8_t point;

	// end of point reading
    if(st->point_index >= acquisition_schedule.scan.num_of_points)
    	return SENS_ITF_STATE_EXEC_WAIT_ABORT;

    // error condition after 3 retries
	st->retries++;
	if(st->retries > 3)
		return SENS_ITF_STATE_EXEC_ERROR;

	// normal point reading
	point = acquisition_schedule.scan.index[st->point_index];
    cmd.hdr.addr = SENS_ITF_REGMAP_READ_POINT_DATA_1 + point;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return sens_itf_mote_pack_send_frame(&cmd, 4);

}

static uint8_t sens_itf_mote_sm_func_run_sch(sens_itf_mote_sm_state_t *st)
{
    uint8_t n;

    acquisition_schedule.scan.num_of_points = 0;

    for (n = 0; n < acquisition_schedule.num_of_points; n++)
    {

    	if(acquisition_schedule.points[n].counter > 0)
    		acquisition_schedule.points[n].counter--;

        if (acquisition_schedule.points[n].counter == 0)
        {
        	acquisition_schedule.scan.index[acquisition_schedule.scan.num_of_points] = n;
        	acquisition_schedule.scan.num_of_points++;
        }

    }

    if(acquisition_schedule.scan.num_of_points > 0)
    {
    	st->point_index = 0;
    	st->retries = 0;
    	return SENS_ITF_STATE_EXEC_WAIT_ABORT;
    }
    else
    	return SENS_ITF_STATE_EXEC_WAIT_OK;
}

/*

{
    sens_itf_mote_read_point(acquisition_schedule.points[n].index);
    acquisition_schedule.points[n].counter = acquisition_schedule.points[n].sampling_time_x250ms;
}
*/

static uint8_t sens_itf_mote_sm_func_build_sch(sens_itf_mote_sm_state_t *st)
{
	uint8_t n, m;

    acquisition_schedule.num_of_points = 0;

    for (n = 0, m = 0; n < board_info.num_of_points; n++)
    {
        if ((sensor_points.points[n].desc.access_rights & OSENS_ACCESS_READ_ONLY) &&
            (sensor_points.points[n].desc.sampling_time_x250ms > 0))
        {
            acquisition_schedule.points[m].index = n;
            acquisition_schedule.points[m].counter = sensor_points.points[n].desc.sampling_time_x250ms;
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

    if (size != ans_size || (ans.hdr.addr != SENS_ITF_REGMAP_POINT_DESC_1 + st->point_index))
        return SENS_ITF_STATE_EXEC_ERROR;

    memcpy(&sensor_points.points[st->point_index].desc,&ans.payload.point_desc_cmd,sizeof(osens_point_desc_t));

    st->point_index++;
    sensor_points.num_of_points = st->point_index;

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_req_pt_desc(sens_itf_mote_sm_state_t *st)
{
	// end of cyclic point description reading
	board_info.num_of_points = 1;  //teste rff

    if(st->point_index >= board_info.num_of_points)
    	return SENS_ITF_STATE_EXEC_WAIT_ABORT;

    cmd.hdr.addr = SENS_ITF_REGMAP_POINT_DESC_1 + st->point_index;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return sens_itf_mote_pack_send_frame(&cmd, 4);
}

static uint8_t sens_itf_mote_sm_func_proc_brd_id_ans(sens_itf_mote_sm_state_t *st)
{
	uint8_t size;
    uint8_t ans_size = 28;

    st->point_index = 0;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);

    if (size != ans_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    memcpy(&board_info, &ans.payload.brd_id_cmd, sizeof(osens_brd_id_t));

    if ((board_info.num_of_points == 0) || (board_info.num_of_points > OSENS_MAX_POINTS))
    	return SENS_ITF_STATE_EXEC_ERROR;

    sensor_points.num_of_points = 0;

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_req_brd_id(sens_itf_mote_sm_state_t *st)
{
	cmd.hdr.size = 4;
	cmd.hdr.addr = SENS_ITF_REGMAP_BRD_ID;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return sens_itf_mote_pack_send_frame(&cmd, 4);
}


static uint8_t sens_itf_mote_sm_func_proc_itf_ver_ans(sens_itf_mote_sm_state_t *st)
{
    uint8_t size;
    uint8_t ans_size = 6;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);

	if (size != ans_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    if ((SENS_ITF_ANS_OK != ans.hdr.status) || (OSENS_LATEST_VERSION != ans.payload.itf_version_cmd.version))
    	return SENS_ITF_STATE_EXEC_ERROR;


    return SENS_ITF_STATE_EXEC_OK;
}


static uint8_t sens_itf_mote_sm_func_wait_ans(sens_itf_mote_sm_state_t *st)
{

	/* TODO!!!! MELHORAR O Tratamento do final de frame
	 * Devido ao problema da UART nao estar reconhecendo o final do frame
	 * a rotina esperar um ciclo de scan (250ms) entao se recebeu algum dado ele diz
	 * que recebeu o frame. Posteriormente sera tratado o frame e verificado se recebeu tudo
	 */
	if ((st->trmout_counter > 0) && (num_rx_bytes > 3))
	{
		st->frame_arrived = true;
		num_rx_bytes = 0;
	}

	if(st->frame_arrived)
	{
		st->frame_arrived = false;
		return SENS_ITF_STATE_EXEC_WAIT_STOP;
	}

	st->trmout_counter++;

	if(st->trmout_counter > st->trmout)
	{
		//se ocorreu timeout e existe bytes no buffer rx considero que recebeu msg
		if (num_rx_bytes > 0)
		{
			return SENS_ITF_STATE_EXEC_WAIT_STOP;
		}
		else
		{
			return SENS_ITF_STATE_EXEC_WAIT_ABORT;
		}
	}

	return SENS_ITF_STATE_EXEC_WAIT_OK;
}


static uint8_t sens_itf_mote_sm_func_req_ver(sens_itf_mote_sm_state_t *st)
{
    cmd.hdr.addr = SENS_ITF_REGMAP_ITF_VERSION;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(500);
    return sens_itf_mote_pack_send_frame(&cmd, 4);
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

	num_rx_bytes=0;


	return ret;
}

void sens_itf_mote_sm(void)
{
	uint8_t ret;

	#ifdef TRACE_ON
	uint8_t ls = sm_state.state;
	#endif

	ret = sens_itf_mote_sm_table[sm_state.state].func(&sm_state);

	if (flagErrorOccurred)
	{
      //reset uart
	  uart1_clearTxInterrupts();
	  uart1_clearRxInterrupts();      // clear possible pending interrupts
	  uart1_enableInterrupts();       // Enable USCI_A1 TX & RX interrupt

	  flagErrorOccurred = 0;
	}

	switch(ret)
	{
		case SENS_ITF_STATE_EXEC_OK:
		case SENS_ITF_STATE_EXEC_WAIT_STOP:
			sm_state.state = sens_itf_mote_sm_table[sm_state.state].next_state;
			break;
		case SENS_ITF_STATE_EXEC_WAIT_OK:
			// still waiting
			break;
		case SENS_ITF_STATE_EXEC_WAIT_ABORT:
			// wait timeout
			sm_state.state = sens_itf_mote_sm_table[sm_state.state].abort_state;
			break;
		case SENS_ITF_STATE_EXEC_ERROR:
		default:
			sm_state.state = sens_itf_mote_sm_table[sm_state.state].error_state;
			break;
	}

	#ifdef TRACE_ON
    printf("sens_itf_mote_sm %d -> %d\n",ls,sm_state.state);
	#endif

}

static void sens_itf_mote_tick(void)
{
    scheduler_push_task((task_cbt) sens_itf_mote_sm, TASKPRIO_SENS_ITF_MAIN);
}


const sens_itf_mote_sm_table_t sens_itf_mote_sm_table[] =
{     //{ func,                                   next_state,                      abort_state,                error_state         }
		{ sens_itf_mote_sm_func_init,             SENS_ITF_STATE_SEND_ITF_VER,     SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_INIT
		{ sens_itf_mote_sm_func_req_ver,          SENS_ITF_STATE_WAIT_ITF_VER_ANS, SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_SEND_ITF_VER
		{ sens_itf_mote_sm_func_wait_ans,         SENS_ITF_STATE_PROC_ITF_VER,     SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_WAIT_ITF_VER_ANS
		{ sens_itf_mote_sm_func_proc_itf_ver_ans, SENS_ITF_STATE_SEND_BRD_ID,      SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_PROC_ITF_VER
		{ sens_itf_mote_sm_func_req_brd_id,       SENS_ITF_STATE_WAIT_BRD_ID_ANS,  SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_SEND_BRD_ID
		{ sens_itf_mote_sm_func_wait_ans,         SENS_ITF_STATE_PROC_BRD_ID,      SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_WAIT_BRD_ID_ANS
		{ sens_itf_mote_sm_func_proc_brd_id_ans,  SENS_ITF_STATE_SEND_PT_DESC,     SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_PROC_BRD_ID
		{ sens_itf_mote_sm_func_req_pt_desc,      SENS_ITF_STATE_WAIT_PT_DESC_ANS, SENS_ITF_STATE_BUILD_SCH,   SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_SEND_PT_DESC
		{ sens_itf_mote_sm_func_wait_ans,         SENS_ITF_STATE_PROC_PT_DESC,     SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_WAIT_PT_DESC_ANS
		{ sens_itf_mote_sm_func_pt_desc_ans,      SENS_ITF_STATE_SEND_PT_DESC,     SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_PROC_PT_DESC
		{ sens_itf_mote_sm_func_build_sch,        SENS_ITF_STATE_RUN_SCH,          SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_BUILD_SCH
		{ sens_itf_mote_sm_func_run_sch,          SENS_ITF_STATE_RUN_SCH,          SENS_ITF_STATE_SEND_PT_VAL, SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_RUN_SCH
		{ sens_itf_mote_sm_func_req_pt_val,       SENS_ITF_STATE_WAIT_PT_VAL_ANS,  SENS_ITF_STATE_RUN_SCH,     SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_SEND_PT_VAL
		{ sens_itf_mote_sm_func_wait_ans,         SENS_ITF_STATE_PROC_PT_VAL,      SENS_ITF_STATE_SEND_PT_VAL, SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_WAIT_PT_VAL_ANS
		{ sens_itf_mote_sm_func_pt_val_ans,       SENS_ITF_STATE_SEND_PT_VAL,      SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_PROC_PT_VAL
};


/**************************************************************************//**
* @brief    This function puts up to \e size bytes into the BSP UART TX
*           buffer and starts to transfer data over UART.
*
*           If \b BSP_UART_ALL_OR_NOTHING is defined, data is put into the
*           TX buffer only if there is room for all \e size bytes.
*
* @param    frame            is a pointer to the source buffer.
* @param    size            is the number of bytes to transfer.
*
* @return   Returns the number of bytes actually copied to the TX buffer.
******************************************************************************/

uint8_t sens_itf_mote_send_frame(uint8_t *frame, uint8_t size)
{
 	uint16_t ui16Length=size;
    register uint16_t ui16Idx = 0;

    DISABLE_INTERRUPTS();
    while((ui16Idx < ui16Length))
    {
        UARTCharPut(BSP_UART_BASE, frame[ui16Idx++]);
    }
    ENABLE_INTERRUPTS();

    return ((uint8_t) ui16Length);
}

/**************************************************************************//**
* @brief    This function pushes a byte from the UART RX FIFO to the BSP UART
*           RX buffer. The function handles RX buffer wrap-around, but does not
*           handle RX buffer overflow. The function should only be called if
*           there is data in the UART RX FIFO. It modifies volatile variables
*           and should only be called when interrupts are disabled.
*
* @brief    None
******************************************************************************/
static void buBufPushByte(void)
{
    uint32_t value;

    // Push byte from RX FIFO to buffer
    value = UARTCharGetNonBlocking(SENS_ITF_UART_BASE);

    if (num_rx_bytes < SENS_ITF_MAX_FRAME_SIZE)
    	frame[num_rx_bytes] = (uint8_t) value;

    num_rx_bytes++;
    if (num_rx_bytes >= SENS_ITF_MAX_FRAME_SIZE)
        num_rx_bytes = 0;
}


/**************************************************************************//**
* @brief    This function flushes the ringbuffer control structure specified
*           by \e psBuf.
*
* @param    psBuf       is a pointer to a \e tBuBuf ringbuffer structure.
*
* @return   None
******************************************************************************/
static void buBufFlush(void)
{
    // Start of critical section
    bool bIntDisabled = IntMasterDisable();

    num_rx_bytes = 0;

    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
}


void uart1_isr_private(void)
{
    uint32_t ui32IntBm = UARTIntStatus(SENS_ITF_UART_BASE, 1);
    uint8_t dummy=0;

    UARTIntClear(SENS_ITF_UART_BASE, (ui32IntBm & 0xF0));

    //ERROR LINE
    if (ui32IntBm & (UART_INT_BE | UART_INT_PE | UART_INT_FE))
	{
    	UARTRxErrorClear(SENS_ITF_UART_BASE);
    	buBufFlush();
    	flagErrorOccurred = 1;
        //while(UARTCharsAvail(SENS_ITF_UART_BASE))
        //{
        //	dummy = UARTCharGetNonBlocking(SENS_ITF_UART_BASE);
        //}
	}
    else if(ui32IntBm & (UART_INT_OE))
	{    //OVERRUN
    	UARTRxErrorClear(SENS_ITF_UART_BASE);
        buBufFlush();
    	flagErrorOccurred = 1;
    	//dummy = UARTCharGetNonBlocking(SENS_ITF_UART_BASE);
        //while(UARTCharsAvail(SENS_ITF_UART_BASE))
        //{
        //	dummy = UARTCharGetNonBlocking(SENS_ITF_UART_BASE);
        //}
	}
    else if(ui32IntBm & (UART_INT_RX | UART_INT_RT))
    {
        //
        // Put received bytes into buffer
        //
        //while(UARTCharsAvail(BSP_UART_BASE) && !BU_IS_BUF_FULL(&sBuBufRx))
        while(UARTCharsAvail(SENS_ITF_UART_BASE))
        {
            buBufPushByte();
        }


        //if (ui32IntBm & UART_INT_RT)
        //{
        //	sm_state.frame_arrived = true;
        //}

    }
    else if(ui32IntBm & UART_INT_TX)
    {
        //while(UARTSpaceAvail(BSP_UART_BASE) && (!BU_IS_BUF_EMPTY(&sBuBufTx)))
        //{
        //    buBufPopByte();
        //}
    }
}

