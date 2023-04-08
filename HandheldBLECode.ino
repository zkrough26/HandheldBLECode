#include <ArduinoBLE_P.h>
# include "defines.h"

///////////////////////////////////////////////////////
// Program defines
///////////////////////////////////////////////////////
#define MAGICNUM            0XCF      // should be byte 0 in new message
#define RETRYCOUNT          5         // number of retry for better block handling

// Commands
#define NO_COMMAND          0         // No action Pending
#define REQ_NEW_MESSAGE     1         // central : new message req
#define CANCEL_MESSAGE      2         // Peripheral / Central : cancel and disgard message
#define NEW_BLOCK_AVAILABLE 3         // peripheral: to indicate new block is ready
#define RECEIVED_OK         4         // Peripheral / Central : acknowledge on good receive latest command
#define REQ_NEW_BLOCK       5         // Central : request new block
#define RCD_CMPLT_MSG       6         // !!central : complete message received
// errors
#define TIMEOUT_BLOCK       7         // !!central : timeout, req to resend
#define INVALID_BLOCK       8         // central : invalid blocknumber
#define CRC_BLOCK           9         // central : CRC error, req to resend
#define REQ_INVALID        10         // Peripheral / Central : Did not request / out of sequence
#define MAGIC_INVALID      11         // Central : Invalid first block
#define ERR_INTERNAL       12         // Central : internal error

///////////////////////////////////////////////////////
// Program variables
///////////////////////////////////////////////////////
uint8_t BlockCounter = 0;             // keep track of current block to send
int16_t TotalMessageLength = 0;       // keep track of total message bytes left to send
uint8_t *MessageBuf = NULL;           // holds the complete received message
uint16_t MessageWriteOffset = 0;      // current write offset for message
uint8_t *BlockBuf= NULL;              // Data buffer from Bluetooth to user level
uint8_t PeripheralCmd = NO_COMMAND;   // latest command received from peripheral
uint8_t PendingCommand = NO_COMMAND;  // latest command send to peripheral
uint8_t SetCommandNext = NO_COMMAND;  // Command PENDING to send in near future
uint8_t RetryErrorCnt = 0;            // keep track on repeated block request
bool    MessageComplete= false;       // if true complete message has been received. Ready to action / display
uint8_t CurrentMTUSize = 0;           // This will hold the agreed MTU size (set after connect in BLE_Comm)

///////////////////////////////////////////////////////
// NO NEED FOR CHANGES BEYOND THIS POINT
///////////////////////////////////////////////////////

unsigned long lastMessage = 0;

void setup() {
  Serial.begin(57600);

  Start_BLE();
}

void loop() {

  static uint8_t loopcnt = 0;

  if (!CheckConnect()) {

    // clean up
    SetCommandNext = NO_COMMAND;
    PeripheralCmd = NO_COMMAND;
    PendingCommand = NO_COMMAND;
    TotalMessageLength = 0;

    // if old blockbuffer pending release it (maybe MTU is changed on next connection)
    if (BlockBuf) {
      free(BlockBuf);
      BlockBuf = NULL;
    }

    Start_BLE();
  }

  // Handle (potential) input received from peripheral
  HandlePeripheralCmd();

  // Any pending command to send
  if (SetCommandNext != NO_COMMAND){
    if (loopcnt++ > 1) {
      SendCommand(SetCommandNext);
      SetCommandNext = NO_COMMAND;
      loopcnt = 0;
    }
  }

  // Do we have the complete message
  if (MessageComplete){
    DisplayMessage();
    TotalMessageLength = 0;
    MessageComplete = false;
  }

  unsigned long now = millis();
  if(now - lastMessage >= 5000)
  {
    SendCommand(REQ_NEW_MESSAGE);
    lastBlink = now;
  }
}

/**
 * take action on command received from peripheral
 */
void HandlePeripheralCmd()
{
  // if No command received from peripheral
  if (PeripheralCmd == NO_COMMAND) return;

  switch (PeripheralCmd){

    case NEW_BLOCK_AVAILABLE:
        PeripheralCmd = NO_COMMAND;
        ReadDataBlock();            // will be acknowledged if block is good or bad).
      break;

    case RECEIVED_OK:
      switch (PendingCommand) {     // this was previous command sent by us

        case CRC_BLOCK:
        case INVALID_BLOCK:
        case ERR_INTERNAL:
          RetryBlock();
          break;

        case CANCEL_MESSAGE:
          TotalMessageLength = 0;
          // fall through

        default:
          PeripheralCmd = NO_COMMAND;
          PendingCommand = NO_COMMAND;
      }
      break;

    case CANCEL_MESSAGE:
      SendCommand(RECEIVED_OK);
      PeripheralCmd = NO_COMMAND;
      PendingCommand = NO_COMMAND;
      SetCommandNext = NO_COMMAND;
      TotalMessageLength = 0;
      break;

    case REQ_INVALID:
      SendCommand(RECEIVED_OK);
      PendingCommand = NO_COMMAND;
      PeripheralCmd = NO_COMMAND;
      break;
  }
}

/**
 *  Retry new block after error only for RETRYCOUNT
 */
void RetryBlock()
{
  if (RetryErrorCnt++ < RETRYCOUNT)
    SendCommand(REQ_NEW_BLOCK);
  else {
    SendCommand(CANCEL_MESSAGE);
  }
}

/**
 * send command on Write characteristic
 * Byte[0] = MAGICNUM
 * Byte[1] = command
 * Byte[2] = optional parameter
 */
void SendCommand(uint8_t cmd){
  uint8_t s[3];
  s[0] = MAGICNUM;
  s[1] = cmd;
  s[2] = 0x0;  // optional parameter
  TXData(s,3);

  // set pending command
  PendingCommand = cmd;
}

void ReadDataBlock()
{
  uint16_t i = 0;                // offset to start copy data

  // obtain memory for a block (first time after connect only)
  if (BlockBuf == NULL) {

    BlockBuf = (uint8_t *) malloc(CurrentMTUSize);
    if (! BlockBuf) {
      while(1);
    }
  }

  // read Block
  int ret = RXData(BlockBuf, CurrentMTUSize);

  if (ret < 1) {
    SendCommand(ERR_INTERNAL);
    return;
  }

  // start of new message ?
  if (TotalMessageLength == 0) {

    // check for correct packet start
    if (BlockBuf[i++] != MAGICNUM)  {
      SendCommand(MAGIC_INVALID);
      return;
    }

    // get total message size
    TotalMessageLength = BlockBuf[i++] << 8;
    TotalMessageLength = TotalMessageLength |BlockBuf[i++];

    // get blocksize based on MTU
    if ( CurrentMTUSize !=  BlockBuf[i++] ) {
      while(1);
    }

    // if previous message pending release. The new message could have different length
    if (MessageBuf)  free(MessageBuf);

    // allocate space.
    MessageBuf = (uint8_t *) malloc(TotalMessageLength);

    if (! MessageBuf) {
      SendCommand(ERR_INTERNAL);
      return;
    }

    BlockCounter = 0;
    MessageWriteOffset = 0;
  }  // end header new message

  // for any block
  if ( BlockCounter++ != BlockBuf[i++]){
    SendCommand(INVALID_BLOCK);
    return;
  }

  uint16_t ReceiveBufLen = BlockBuf[i++] << 8;     // block data length (excluding CRC)
  ReceiveBufLen = ReceiveBufLen |BlockBuf[i++];

  // the total length is content length + CRC
  if (ret != ReceiveBufLen + 2 ){
    SendCommand(INVALID_BLOCK);
    return;
  }

  // save current message location in case of CRC error
  uint16_t SaveMessageWriteOffset = MessageWriteOffset;

  // copy data of this packet in the receive buffer
  for (; i < ReceiveBufLen ; i++) {
    // prevent buffer overrun
    if (MessageWriteOffset < TotalMessageLength)
        MessageBuf[MessageWriteOffset++] = BlockBuf[i];
  }

  // get CRC of message
  uint16_t RcvCRC = BlockBuf[i++] << 8;
  RcvCRC |= BlockBuf[i++];

  if (RcvCRC != calc_crc(BlockBuf,i-2)) {
    SendCommand(CRC_BLOCK);

    // restore write offset
    MessageWriteOffset = SaveMessageWriteOffset;
    return;
  }

  // we had a good block and communication
  SendCommand(RECEIVED_OK);
  RetryErrorCnt = 0;

  // did we receive the complete message.
  if (MessageWriteOffset >= TotalMessageLength) {
    MessageComplete = true;       // indicate message received, ready to display
    SendCommand(RCD_CMPLT_MSG);
  }
  else {
    // first allow RECEIVED_OK to be send
    SetCommandNext = REQ_NEW_BLOCK;
  }
}

/**
 * Hand-off the notify received from the peripheral
 *
 * Hand-off the data and let the "stack" clear with all the returns pending
 * to ArduinoBLE functions to prevent stack-overrun.
 * The handling of the command triggered once returned in loop()
 */
void HandlePeripheralNotify(const uint8_t *buf, uint16_t len)
{
  if (buf[0] != MAGICNUM) {
    SendCommand(MAGIC_INVALID);
    return;
  }

  PeripheralCmd = buf[1];
}

// Transmit Message via UART to MSP432
void DisplayMessage() {
  for(int i = 0; i < TotalMessageLength; i++)
  {
    Serial.print(MessageBuf[i]);
  }
}

/**
 * called when disconnect was performed
 * will wait for <enter>
 *
 * Reconnect will be triggered in loop()
 */
void WaitToReconnect()
{
  bool temp = false;
  while(!temp)
  {
    if(CheckConnect())
    {
      temp = true;
    }
    delay(50);
  }
}