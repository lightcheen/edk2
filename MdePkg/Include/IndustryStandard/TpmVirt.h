
#ifndef _TPM_VIRTIO_H_
#define _TPM_VIRTIO_H_

// 
typedef struct {
  ///
  /// Used to determine current state of Locality of the TPM.
  ///
  UINT32    LocalityState;                                     // 0
  UINT8     Reserved1[4];                                      // 4
  ///
  /// Used to gain control of the TPM by this Locality.
  ///
  UINT32    LocalityControl;                                   // 8
  ///
  /// Used to determine whether Locality has been granted or Seized.
  ///
  UINT32    LocalityStatus;                                    // 0ch
  UINT8     Reserved2[0x20];                                   // 10h
  ///
  /// Used to identify the Interface types supported by the TPM.
  ///
  UINT32    InterfaceId;                                       // 30h
  ///
  /// Vendor ID
  ///
  UINT16    Vid;                                               // 34h
  ///
  /// Device ID
  ///
  UINT16    Did;                                               // 36h
  ///
  /// Optional Register used in low memory environments prior to CRB_DATA_BUFFER availability.
  ///
  UINT64    CrbControlExtension;                               // 38h
  ///
  /// Register used to initiate transactions for the CRB interface.
  ///
  UINT32    CrbControlRequest;                                 // 40h
  ///
  /// Register used by the TPM to provide status of the CRB interface.
  ///
  UINT32    CrbControlStatus;                                  // 44h
  ///
  /// Register used by software to cancel command processing.
  ///
  UINT32    CrbControlCancel;                                  // 48h
  ///
  /// Register used to indicate presence of command or response data in the CRB buffer.
  ///
  UINT32    CrbControlStart;                                   // 4Ch
  ///
  /// Register used to configure and respond to interrupts.
  ///
  UINT32    CrbInterruptEnable;                                // 50h
  UINT32    CrbInterruptStatus;                                // 54h
  ///
  /// Size of the Command buffer.
  ///
  UINT32    CrbControlCommandSize;                             // 58h
  ///
  /// Command buffer start address
  ///
  UINT32    CrbControlCommandAddressLow;                           // 5Ch
  UINT32    CrbControlCommandAddressHigh;                          // 60h
  ///
  /// Size of the Response buffer
  ///
  UINT32    CrbControlResponseSize;                            // 64h
  ///
  /// Address of the start of the Response buffer
  ///
  UINT64    CrbControlResponseAddrss;                          // 68h
  UINT8     Reserved4[0x10];                                   // 70h
  ///
  /// Command/Response Data may be defined as large as 3968 (0xF80).
  ///
  UINT8     CrbDataBuffer[0xF80];                              // 80h
} VIRTIO_REGISTERS;

//
// Define pointer types used to access CRB registers on PTP
//
typedef VIRTIO_REGISTERS* VIRTIO_REGISTERS_PTR;


#define VIRTIO_LOCALITY_CONTROL_REQUEST_ACCESS  BIT0

#define VIRTIO_LOCALITY_STATUS_GRANTED  BIT0



#define VIRTIO_TIMEOUT_A  (750 * 1000)                // 750ms





#endif