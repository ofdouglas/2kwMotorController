
/* CAN Driver */

circularBuffer transmitQueues[3];

#enum { RT_QUEUE_INDEX, HI_QUEUE_INDEX, LO_QUEUE_INDEX, NUM_QUEUES };


void transmitMsg_CAN(int msgEnum, int arg1, int arg2)
{
  tCANMsgObject msgObj;

  populateMsgObj(&msgObj, msgEnum, arg1, arg2);
  circularBuffer * queue = getQueueFromMsgEnum(msgEnum);
  circularBuffer_put(queue, msgObj);
  enableCANTxInt();
}


void CANTxISR(void)
{
  for (int i = RT_QUEUE_INDEX; i < NUM_QUEUES; i++) {
    if (CANTxSlotIsOpen(i) && (queueIsEmpty(i) == false))
      CANTxSlotPut(i, queueGet(transmitQueues[i]));
  }

  // if all queues empty, disable CANTxISR
}

void CANRxISR(void)
{
  tCANMsgObject msgObj;
  msgObj = CANMsgGet(rxSlot);

  struct message msg;
  msg = msgFromCANMsgObj(msgObj);

  QueueSend(destQueueLookup(msg), msg);
}

/* When the CAN enters a failure mode, the motor controller does too.
 * When the CAN recovers, the motor controller does too (assuming that
 * all other modules are operating safely).
 */
void CANStatusISR(void)
{
  if (CANbusFailure)
    system_raise_fault(FAULT_CAN_BUS);

  if (errorRecovery)
    system_lower_fault(FAULT_CAN_BUS);
}
