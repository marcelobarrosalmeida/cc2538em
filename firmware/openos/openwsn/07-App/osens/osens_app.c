#include <string.h>
#include <stdint.h>
#include "openwsn.h"
#include "opencoap.h"
#include "openqueue.h"
#include "osens.h"
#include "osens_itf.h"
#include "packetfunctions.h"
#include "opentimers.h"
#include "scheduler.h"

#define TRACE_ON 0

owerror_t osens_desc_receive(OpenQueueEntry_t* msg, coap_header_iht*  coap_header, coap_option_iht*  coap_options);
void osens_desc_sendDone(OpenQueueEntry_t* msg, owerror_t error);
owerror_t osens_val_receive(OpenQueueEntry_t* msg, coap_header_iht*  coap_header, coap_option_iht*  coap_options);
void osens_val_sendDone(OpenQueueEntry_t* msg, owerror_t error);

const uint8_t osens_desc_path0 [] = "d";
coap_resource_desc_t osens_desc_vars;
const uint8_t osens_val_path0 [] = "s";
coap_resource_desc_t osens_val_vars;

static osens_point_ctrl_t sensor_points;

void sensors_init(void) {

    //osens_init_point_db();
    //osens_mote_init();

    // prepare the resource descriptor for the /d and /s paths
    osens_desc_vars.path0len = sizeof(osens_desc_path0) -1;
    osens_desc_vars.path0val = (uint8_t*) (&osens_desc_path0);
    osens_desc_vars.path1len = 0;
    osens_desc_vars.path1val = NULL;
    osens_desc_vars.componentID = COMPONENT_SENSORS;
    osens_desc_vars.callbackRx = &osens_desc_receive;
    osens_desc_vars.callbackSendDone = &osens_desc_sendDone;

    osens_val_vars.path0len = sizeof(osens_val_path0) -1;
    osens_val_vars.path0val = (uint8_t*) (&osens_val_path0);
    osens_val_vars.path1len = 0;
    osens_val_vars.path1val = NULL;
    osens_val_vars.componentID = COMPONENT_SENSORS;
    osens_val_vars.callbackRx = &osens_val_receive;
    osens_val_vars.callbackSendDone = &osens_val_sendDone;

    // register with the CoAP modules
    opencoap_register(&osens_desc_vars);
    opencoap_register(&osens_val_vars);
}

static uint8_t * insert_str(uint8_t *buffer, uint8_t *str, uint32_t len)
{
	uint8_t *p = buffer;
	memcpy(buffer,str,len);
	return (p+len);
}

// based on http://stackoverflow.com/questions/9655202/how-to-convert-integer-to-string-in-c
static uint8_t * insert_uint32(uint8_t *buffer, uint32_t val)
{
    uint8_t digit[] = "0123456789";
    uint8_t* p = buffer;
    uint8_t *pf;
    uint32_t shifter;

    /*
    if(val < 0)
    {
        *p++ = '-';
        val *= -1;
        n++;
    }
    */

    // count the number of digits
    shifter = val;
    do
    {
        ++p;
        shifter = shifter / 10;
    } while(shifter);
    //*p = '\0';
    pf = p;

    // now fill the digits
    do
    {
        *--p = digit[val % 10];
        val = val/10;
    } while(val);

    return pf;
}

owerror_t osens_desc_receive(OpenQueueEntry_t* msg, coap_header_iht*  coap_header, coap_option_iht*  coap_options)
{
	osens_brd_id_t board_info;
    owerror_t outcome = E_FAIL;
    uint8_t n;
    uint8_t buf[128];
    uint8_t *pbuf = &buf[0];

    switch (coap_header->Code) {
    case COAP_CODE_REQ_GET:
        // reset packet payload
        msg->payload = &(msg->packet[127]);
        msg->length = 0;

        if(sens_get_brd_desc(&board_info))
        {
            if (coap_options[1].length == 0)
            {
            	pbuf = insert_str(pbuf,(uint8_t*)"{\"ver\":",7);
            	pbuf = insert_uint32(pbuf,board_info.hardware_revision);
            	pbuf = insert_str(pbuf,(uint8_t*)"{\"model\":",9);
            	pbuf = insert_str(pbuf,board_info.model,strlen((char *)board_info.model));
            	pbuf = insert_str(pbuf,(uint8_t*)"{\"id\":",6);
            	pbuf = insert_uint32(pbuf,board_info.sensor_id);
            	pbuf = insert_str(pbuf,(uint8_t*)"{\"npts\":",8);
            	pbuf = insert_uint32(pbuf,board_info.num_of_points);
            	pbuf = insert_str(pbuf,(uint8_t*)"}",1);

            	n = ((uint32_t)pbuf - (uint32_t)buf);

            	packetfunctions_reserveHeaderSize(msg, 1 + n);
                msg->payload[0] = COAP_PAYLOAD_MARKER;

                memcpy(&msg->payload[1], buf, n);
                coap_header->Code = COAP_CODE_RESP_CONTENT;
                outcome = E_SUCCESS;
            }

        }
        break;

    case COAP_CODE_REQ_PUT:
        // reset packet payload
        msg->payload = &(msg->packet[127]);
        msg->length = 0;

        // set the CoAP header
        coap_header->Code = COAP_CODE_RESP_CHANGED;

        outcome = E_SUCCESS;
        break;

    default:
        outcome = E_FAIL;
        break;
    }

    return outcome;
}

owerror_t osens_val_receive(
    OpenQueueEntry_t* msg,
    coap_header_iht*  coap_header,
    coap_option_iht*  coap_options
    ) {
    owerror_t outcome = E_FAIL;
    uint8_t n, nb;
    uint8_t buf[128];
    uint8_t *pbuf = &buf[0];

    switch (coap_header->Code) {
    case COAP_CODE_REQ_GET:
        // reset packet payload
        msg->payload = &(msg->packet[127]);
        msg->length = 0;

        /*
        if (coap_options[1].length == 0) {
            n = snprintf(buf, 80, "[{\"temp\":%u},{\"hum\":\"%u%%\"},{\"alarm\":%u}]",
                ieee154e_vars.asn.bytes0and1 & 0xff,
                (ieee154e_vars.asn.bytes0and1 >> 8) & 0xff,
                ieee154e_vars.asn.bytes2and3 & 0xff);

            packetfunctions_reserveHeaderSize(msg, 1 + n);
            msg->payload[0] = COAP_PAYLOAD_MARKER;

            memcpy(&msg->payload[1], buf, n );
            coap_header->Code = COAP_CODE_RESP_CONTENT;
            outcome = E_SUCCESS;
        }*/

        break;

    case COAP_CODE_REQ_PUT:
        // reset packet payload
        msg->payload = &(msg->packet[127]);
        msg->length = 0;

        // set the CoAP header
        coap_header->Code = COAP_CODE_RESP_CHANGED;

        outcome = E_SUCCESS;
        break;

    default:
        outcome = E_FAIL;
        break;
    }

    return outcome;
}

void osens_desc_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
    openqueue_freePacketBuffer(msg);
}

void osens_val_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
    openqueue_freePacketBuffer(msg);
}
