/*
 * This file contains the routines Bluetooth Routines
 *
 * Version 1.0 / November  2022 / paulvha
 * - initial version
 */

# include "defines.h"

///////////////////////////////////////////////////////
// BLE defines
///////////////////////////////////////////////////////
const char BLE_PERIPHERAL_NAME[] = "Mobo";

#define SERVICE "9e400001-b5a3-f393-e0a9-e14e24dcca9e"

//  characteristic read, notify and write
#define CHARACTERISTIC_R_UUID "9e400002-b5a3-f393-e0a9-e14e24dcca9e" // receive feedback
#define CHARACTERISTIC_W_UUID "9e400003-b5a3-f393-e0a9-e14e24dcca9e" // Send data characteristic
#define CHARACTERISTIC_N_UUID "9e400004-b5a3-f393-e0a9-e14e24dcca9e" // Notify characteristic

// handles
BLECharacteristic RX_Char;
BLECharacteristic TX_Char;
BLECharacteristic N_Char;
BLEDevice peripheral;

// how many Ms to wait for peripheral
#define ScanTimeOut 10000

////////////////
// machine state
////////////////
#define AMD_IDLE      0
#define AMD_SCANNED   1
#define AMD_CONNECTED 2

// keep track of connection status
uint8_t AMD_stat = AMD_IDLE;

/**
 * Receive handle function to read data
 * 
 * @param value : pointer to buffer to store
 * @param vlen  : max length to store
 * 
 * @ return     : length of bytes read
 * 
 */
int RXData(uint8_t *value, int vlen) {
  
  int lenc = RX_Char.readValue(value, vlen);

  return(lenc);
}


/**
 * send handle function
 * 
 * @param value : pointer to buffer to send
 * @param vlen  : max length to send
 * 
 * @ return     : length of bytes read
 * 
 */
bool TXData(uint8_t *value, int vlen) {

  TX_Char.writeValue(value,  vlen);
  return(true);
}

/**
 * Receive notifications call back
 */
void N_Char_Received(BLEDevice central, BLECharacteristic characteristic) {

  // in sketch
  HandlePeripheralNotify(characteristic.value(),characteristic.valueSize());
}
 


///////////////////////////////////////////////////////
// BLE functions
///////////////////////////////////////////////////////

/**
 * perform poll for any BLE events and check for connected
 */
bool CheckConnect()
{
  // poll the central for events
  BLE.poll();

  if ( !peripheral.connected()){
    AMD_stat = AMD_IDLE;
    return(false);
  }

  return(true);
}

/**
 * start Bluetooth and look for peripheral information
 */
void Start_BLE()
{
  AMD_stat = AMD_IDLE;

  if (!BLE.begin()){
    while(1);
  }

  // find Peripheral by service name
  if (!PerformScan()){
    Start_BLE();
  }

  // connect and find handles
  if (!get_handles()){
    Start_BLE();
  }

  // set peripheral disconnect handler
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
}

/**
 * force a disconnect
 */
void perform_disconnect()
{
  if (AMD_stat == AMD_CONNECTED) BLE.disconnect();
  while (CheckConnect());
}

/**
 * scan for peripheral by service UUID
 */
bool PerformScan()
{
  if (AMD_stat != AMD_IDLE) return false;

  // prevent deadlock
  unsigned long st = millis();

  // start scanning for peripheral
  BLE.scanForUuid(SERVICE);

  while(1) {

    // check if a peripheral has been discovered
    peripheral = BLE.available();

    if (peripheral) {
      AMD_stat = AMD_SCANNED;
      BLE.stopScan();
      return true;
    }

    // Try for max ScanTimeOut Ms scan
    if (millis() - st > ScanTimeOut) {
      BLE.stopScan();
      return false;
    }
  }
}

/**
 * connect and obtain handles
 */
bool get_handles()
{
  if (AMD_stat != AMD_SCANNED) return false;

  peripheral.connect();

  AMD_stat = AMD_CONNECTED;

  // discover peripheral attributes
  if (peripheral.discoverAttributes()) {
    // needed for blocksize
    CurrentMTUSize = (uint8_t) peripheral.readMTU();
  }
  else {
    perform_disconnect();
    return false;
  }
  
  // retrieve the characteristics handles
  RX_Char = peripheral.characteristic(CHARACTERISTIC_W_UUID); // read data from peripheral
  TX_Char = peripheral.characteristic(CHARACTERISTIC_R_UUID); // send commands to peripheral
  N_Char = peripheral.characteristic(CHARACTERISTIC_N_UUID);  // Receive commands from peripheral

  if (!RX_Char || !TX_Char || !N_Char ) {
    perform_disconnect();
    return false;
  }
  else if (!TX_Char.canWrite() ) {
    perform_disconnect();
    return false;
  }

  if (!RX_Char.canRead() ) {
    peripheral.disconnect();
    return false;
  }

  if (!N_Char.subscribe()) {
    peripheral.disconnect();
    return false;
  }

  // assign event handlers for notify
  N_Char.setEventHandler(BLEUpdated, N_Char_Received);

  return true;
}

/**
 * called when peripheral disconnects
 */
void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  WaitToReconnect();    // in sketch
}
