# ScaleCon_firmware_03

## Scale BLE converter

## Scope : 

  >1. test current consumption of the ble device while advertising only.

  >2. device naming with mac address

  >3. When the ble connection is not established for 10 seconds, the device will enter power-down mode and wake up for uart receiving.
  >>- Wakeup source should be gpio level changes.
  
  >4. When the ble connection is established, uart peripheral should be enabled and work as a uart-ble converter.

## Result :

  >
