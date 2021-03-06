/*
 * can.c
 *
 *  Created on: Oct 19, 2017
 *      Author: Oliver Douglas
 */

#include "stdinclude.h"
#include "system.h"
#include "can.h"

#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "inc/hw_can.h"


#include <string.h>

// File index for ASSERT() macro
FILENUM(7)


QueueHandle_t can_tx_queue;
QueueHandle_t can_rx_queue;

extern TaskHandle_t can_task_handle;


/* CAN ID bitfields
 *
 *  10 ... 6 | 5 ... 0
 *   command   node id
 */
#define RX_BUFFER_INDEX     1
#define TX_BUFFER_INDEX     2

#define CAN_CMD_MASK        0x7C0
#define CAN_NODE_ID_MASK    0x03F

#define CAN_CMD_OFFSET      6
#define CAN_NODE_ID_OFFSET  0

#define CAN_BASE            CAN1_BASE
#define CAN_SYSCTL          SYSCTL_PERIPH_CAN1
#define CAN_INT             INT_CAN1


/******************************************************************************
 * CAN ID bitfield manipulation
 *****************************************************************************/
static int can_id_get_cmd(int id)
{
    return (id & CAN_CMD_MASK) >> CAN_CMD_OFFSET;
}

static int can_id_set_cmd(int id, int cmd)
{
    return (id & CAN_NODE_ID_MASK) | ((cmd << CAN_CMD_OFFSET) & CAN_CMD_MASK);
}

static int can_id_get_node_id(int id)
{
    return (id & CAN_NODE_ID_MASK) >> CAN_NODE_ID_OFFSET;
}

static int can_id_set_node_id(int id, int node_id)
{
    return (id & CAN_CMD_MASK) | (node_id & CAN_NODE_ID_MASK);
}


/******************************************************************************
 * CAN thread and helpers
 *****************************************************************************/

static uint32_t can_ctrl_status;

void can_ISR(void)
{
    BaseType_t task_woken = false;
    uint32_t status = CANIntStatus(CAN_BASE, CAN_INT_STS_CAUSE);

    if (status == RX_BUFFER_INDEX) {
        tCANMsgObject msgObj;
        struct can_msg msg;
        msgObj.pui8MsgData = msg.data;

        CANMessageGet(CAN_BASE, RX_BUFFER_INDEX, &msgObj, true);

        msg.can_cmd = can_id_get_cmd(msgObj.ui32MsgID);
        msg.data_len = msgObj.ui32MsgLen;
        xQueueSendFromISR(can_rx_queue, &msg, &task_woken);
    }
    else if (status == TX_BUFFER_INDEX) {
        CANIntClear(CAN_BASE, TX_BUFFER_INDEX);
        vTaskNotifyGiveFromISR(can_task_handle, &task_woken);
    }
    else if (status == CAN_INT_INTID_STATUS) {
        can_ctrl_status = CANStatusGet(CAN_BASE, CAN_STS_CONTROL);
    }

    portYIELD_FROM_ISR(task_woken);
}


void can_init(void)
{
    uint32_t bit_rate = system_read_config_reg(REG_CAN_BAUD_RATE).i;
    SysCtlPeripheralEnable(CAN_SYSCTL);
    CANInit(CAN_BASE);
    CANBitRateSet(CAN_BASE, system_get_sysclk_freq(), bit_rate);

    tCANMsgObject msg;
    msg.ui32MsgIDMask = CAN_NODE_ID_MASK;
    msg.ui32MsgID = system_read_config_reg(REG_NODE_ID).i;
    msg.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    msg.ui32MsgLen = 8;
    CANMessageSet(CAN1_BASE, RX_BUFFER_INDEX, &msg, MSG_OBJ_TYPE_RX);

    // used as a binary semaphore to manage tx buffer: must start as free
    xTaskNotifyGive(can_task_handle);

    CANIntEnable(CAN1_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(CAN_INT);
    CANEnable(CAN_BASE);
}

void can_task_code(void ** arg)
{
    can_init();

    while (1) {
        struct can_msg msg;
        tCANMsgObject msgObj;

        xQueueReceive(can_tx_queue, &msg, portMAX_DELAY);

        int id = can_id_set_node_id(0, system_read_config_reg(REG_NODE_ID).i);
        msgObj.ui32MsgID = can_id_set_cmd(id, msg.can_cmd);
        msgObj.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
        msgObj.ui32MsgLen = msg.data_len;
        msgObj.pui8MsgData = msg.data;

        ulTaskNotifyTake(true, portMAX_DELAY);
        CANMessageSet(CAN_BASE, TX_BUFFER_INDEX, &msgObj, MSG_OBJ_TYPE_TX);
    }
}



/******************************************************************************
 * Public interface
 *****************************************************************************/

/*  Non-blocking transmit request. Returns true if the message was
 *  queued successfully, false otherwise.
 *
 *  NOTE: This function should NOT be used in an ISR!
 */
bool can_send(struct can_msg * msg)
{
    if (msg->data_len > 8)
        msg->data_len = 8;

    return xQueueSend(can_tx_queue, msg, 0);
}

/*  CAN receive request. Returns true if a message was
 *  written into the provided buffer, false otherwise.
 *
 *  NOTE: This function should NOT be used in an ISR!
 */
bool can_recv(struct can_msg * msg, bool blocking)
{
    return xQueueReceive(can_rx_queue, msg, blocking ? portMAX_DELAY : 0);
}

