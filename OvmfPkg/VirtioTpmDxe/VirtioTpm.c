/** @file

  This driver produces EFI_TCG2_PROTOCOL instances for virtio-tpm devices.

  The implementation is based on OvmfPkg/VirtioTpmgDxe/VirtioTpm.c

**/

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/VirtioLib.h>
#include <Library/PcdLib.h>
#include <Library/IoLib.h>
#include <Guid/HobList.h>
#include <Uefi/UefiSpec.h>
#include <Pi/PiBootMode.h>
#include <Pi/PiHob.h>
#include <Uefi.h>
#include <Library/HobLib.h>

#include <IndustryStandard/TpmPtp.h>
#include <IndustryStandard/TpmVirt.h>

#include "VirtioTpm.h"


EFI_STATUS
Tpm2VirtioTpmCommandDXE( // SecurityPkg 需要，但是它位于 OvmfPkg
  IN     PTP_CRB_REGISTERS_PTR  CrbReg,
  IN     UINT8                 *BufferIn,
  IN     UINT32                SizeIn,
  IN OUT UINT8                 *BufferOut,
  IN OUT UINT32                *SizeOut
) {
  DEBUG ((EFI_D_INFO, "[Tpm2VirtioTpmCommandDXE]\n" ));
  
  VIRTIO_TPM_DEV        *Dev;
  DESC_INDICES          Indices;
  UINTN                 Index;
  EFI_STATUS            Status;
  // volatile UINT8        *Buffer;
  UINT32                Len;
  UINT32                TpmOutSize;
  UINT16                Data16;
  UINT32                Data32;
  EFI_PHYSICAL_ADDRESS  DeviceAddress;
  VOID                  *Mapping;
  EFI_TCG2_PROTOCOL  *mTcg2Protocol;

  
  Status = gBS->LocateProtocol(&gEfiTcg2ProtocolGuid, NULL, (VOID**)&mTcg2Protocol);
  if (EFI_ERROR (Status)) {
    //
    // Tcg2 protocol is not installed. So, TPM2 is not present.
    //
    DEBUG ((DEBUG_ERROR, "Tpm2VirtioTpmCommand - Tcg2 - %r\n", Status));
    return EFI_NOT_FOUND;
  }
  
  Dev = VIRTIO_TRUSTED_PLATFORM_MODULE_FROM_TPM (mTcg2Protocol); // 利用结构体成员获取整个结构体
  if (!Dev->Ready) {
    DEBUG ((DEBUG_INFO, "%a: not ready\n", __func__));
    return EFI_DEVICE_ERROR;
  }

  // 参考：PtpCrbTpmCommand
  // Step.1 Wait IDLE，已经删除 READY/IDLE 属性。（不需要考虑）
  // Step.2 将要发送的 TPM 命令的长度描述等信息写入到寄存器空间中。（不需要考虑）
  // Step.3 Ring 1 到 CrbReg->CrbControlStart 处，触发后端 TPM 命令发射。发射后进行 Poll，待 CrbControlStart 处寄存器设置为 0 时，命令处理完成，出现 Response。
  //（改 Step.3）
  TpmOutSize = 0;
  
  // Map Buffer's system physical address to device address
  Status = VirtioMapAllBytesInSharedBuffer (
             Dev->VirtIo,
             VirtioOperationBusMasterCommonBuffer,
             (VOID *)BufferIn,
             SizeIn,
             &DeviceAddress,
             &Mapping
             );
  if (EFI_ERROR (Status)) {
    Status = EFI_DEVICE_ERROR;
    return Status;
  }

  // 发送后，再接收。
  VirtioPrepare (&Dev->Ring, &Indices);
  VirtioAppendDesc (
    &Dev->Ring,
    DeviceAddress,
    SizeIn,
    VRING_DESC_F_WRITE, // 先写入！
    &Indices
  );

  DEBUG ((DEBUG_INFO, "Tpm2VirtioTpmCommand Send - "));
  if (VirtioFlush (Dev->VirtIo, 0, &Dev->Ring, &Indices, &Len) !=
      EFI_SUCCESS)
  {
    Status = EFI_DEVICE_ERROR;
    goto UnmapBuffer;
  }

  ASSERT (Len == SizeIn);
  
  VirtioPrepare (&Dev->Ring, &Indices);
  VirtioAppendDesc (
    &Dev->Ring,
    DeviceAddress,
    sizeof (TPM2_RESPONSE_HEADER),
    0,
    &Indices
  );

  if (VirtioFlush (Dev->VirtIo, 0, &Dev->Ring, &Indices, &Len) != EFI_SUCCESS) {
      Status = EFI_DEVICE_ERROR;
      goto UnmapBuffer;
  }
  
  ASSERT (Len == sizeof(TPM2_RESPONSE_HEADER));
  
  DEBUG ((DEBUG_INFO, "Tpm2VirtioTpmCommand ReceiveHeader - "));
  for (Index = 0; Index < sizeof (TPM2_RESPONSE_HEADER); Index++) {
    DEBUG ((DEBUG_INFO, "%02x ", BufferOut[Index]));
  }
  
  DEBUG ((DEBUG_INFO, "\n"));

  CopyMem(&Data16, BufferOut, sizeof(UINT16));
  // TPM2 should not use this RSP_COMMAND
  if (SwapBytes16 (Data16) == TPM_ST_RSP_COMMAND) {
    DEBUG ((DEBUG_INFO, "TPM2: TPM_ST_RSP error - %x\n", TPM_ST_RSP_COMMAND));
    Status = EFI_UNSUPPORTED;
    goto UnmapBuffer;
  }

  CopyMem (&Data32, (BufferOut + 2), sizeof (UINT32));
  TpmOutSize = SwapBytes32 (Data32);
  if (*SizeOut < TpmOutSize) {
    //
    // Command completed, but buffer is not enough
    //
    Status = EFI_BUFFER_TOO_SMALL;
    goto UnmapBuffer;
  }

  *SizeOut = TpmOutSize;

  VirtioPrepare (&Dev->Ring, &Indices);
  VirtioAppendDesc (
    &Dev->Ring,
    DeviceAddress + sizeof (TPM2_RESPONSE_HEADER),
    TpmOutSize,
    0,
    &Indices
  );

  if (VirtioFlush (Dev->VirtIo, 0, &Dev->Ring, &Indices, &Len) != EFI_SUCCESS) {
      Status = EFI_DEVICE_ERROR;
      goto UnmapBuffer;
  }

  DEBUG ((DEBUG_INFO, "PtpCrbTpmCommand Receive - "));
  for (Index = 0; Index < TpmOutSize; Index++) {
    DEBUG ((DEBUG_INFO, "%02x ", BufferOut[Index]));
  }

  DEBUG ((DEBUG_INFO, "\n"));

  Status = EFI_SUCCESS;

UnmapBuffer:
  
  if (EFI_ERROR (Status)) {
    Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, Mapping);
  }

  return Status;
}


STATIC
EFI_STATUS
EFIAPI
VirtioTpmInit (
  IN OUT VIRTIO_TPM_DEV  *Dev
  )
{
  UINT8       NextDevStat;
  EFI_STATUS  Status;
  UINT64      Features;
  UINT16      QueueSize;
  UINT64      RingBaseShift;
  EFI_TCG2_PROTOCOL* mTcg2Protocol;
  EFI_GUID gMyHobGuid = { 0xaabbccdd, 0x1234, 0x5678, { 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34, 0x56, 0x78 } };
  EFI_PHYSICAL_ADDRESS *FunctionPointer;
  EFI_HOB_GUID_TYPE *GuidHob;

  DEBUG((EFI_D_INFO, "VirtioTpmInit\n"));
  //
  // Execute virtio-0.9.5, 2.2.1 Device Initialization Sequence.
  //
  NextDevStat = 0;             // step 1 -- reset device
  Status      = Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, NextDevStat);
  if (EFI_ERROR (Status)) {
    goto Failed;
  }

  NextDevStat |= VSTAT_ACK;    // step 2 -- acknowledge device presence
  Status       = Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, NextDevStat);
  if (EFI_ERROR (Status)) {
    goto Failed;
  }

  NextDevStat |= VSTAT_DRIVER; // step 3 -- we know how to drive it
  Status       = Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, NextDevStat);
  if (EFI_ERROR (Status)) {
    goto Failed;
  }

  //
  // Set Page Size - MMIO VirtIo Specific
  //
  Status = Dev->VirtIo->SetPageSize (Dev->VirtIo, EFI_PAGE_SIZE);
  if (EFI_ERROR (Status)) {
    goto Failed;
  }

  //
  // step 4a -- retrieve and validate features
  //
  Status = Dev->VirtIo->GetDeviceFeatures (Dev->VirtIo, &Features);
  if (EFI_ERROR (Status)) {
    goto Failed;
  }

  Features &= VIRTIO_F_VERSION_1 | VIRTIO_F_IOMMU_PLATFORM;

  //
  // In virtio-1.0, feature negotiation is expected to complete before queue
  // discovery, and the device can also reject the selected set of features.
  //
  if (Dev->VirtIo->Revision >= VIRTIO_SPEC_REVISION (1, 0, 0)) {
    Status = Virtio10WriteFeatures (Dev->VirtIo, Features, &NextDevStat);
    if (EFI_ERROR (Status)) {
      goto Failed;
    }
  }
  
  //
  // step 4b -- allocate request/response virtqueue
  //
  Status = Dev->VirtIo->SetQueueSel (Dev->VirtIo, 0);
  if (EFI_ERROR (Status)) {
    goto Failed;
  }

  Status = Dev->VirtIo->GetQueueNumMax (Dev->VirtIo, &QueueSize);
  if (EFI_ERROR (Status)) {
    goto Failed;
  }
  
  Status = VirtioRingInit(Dev->VirtIo, QueueSize, &Dev->Ring);
  if (EFI_ERROR (Status)) {
    goto Failed;
  }

  //
  // If anything fails from here on, we must release the ring resources.
  //
  Status = VirtioRingMap (
             Dev->VirtIo,
             &Dev->Ring,
             &RingBaseShift,
             &Dev->RingMap
             );
  if (EFI_ERROR (Status)) {
    goto ReleaseQueue;
  }

  Status = Dev->VirtIo->SetQueueNum (Dev->VirtIo, QueueSize);
  if (EFI_ERROR (Status)) {
    goto UnmapQueue;
  }

  Status = Dev->VirtIo->SetQueueAlign (Dev->VirtIo, EFI_PAGE_SIZE);
  if (EFI_ERROR (Status)) {
    goto UnmapQueue;
  }

  //
  // step 4c -- Report GPFN (guest-physical frame number) of queue.
  //
  Status = Dev->VirtIo->SetQueueAddress (
                          Dev->VirtIo,
                          &Dev->Ring,
                          RingBaseShift
                          );
  if (EFI_ERROR (Status)) {
    goto UnmapQueue;
  }

  //
  // step 5 -- Report understood features and guest-tuneables.
  //
  if (Dev->VirtIo->Revision < VIRTIO_SPEC_REVISION(1, 0, 0)) {
    Features &= ~(UINT64)(VIRTIO_F_VERSION_1 | VIRTIO_F_IOMMU_PLATFORM);
    Status = Dev->VirtIo->SetGuestFeatures(Dev->VirtIo, Features);
    if (EFI_ERROR(Status)) {
      goto UnmapQueue;
    }
  }

  //
  // step 6 -- initialization complete
  //
  NextDevStat |= VSTAT_DRIVER_OK;
  Status = Dev->VirtIo->SetDeviceStatus(Dev->VirtIo, NextDevStat);
  if (EFI_ERROR(Status)) {
    goto UnmapQueue;
  }

  Dev->Ready = TRUE;


  Status = gBS->LocateProtocol(&gEfiTcg2ProtocolGuid, NULL, (VOID**)&mTcg2Protocol);
  if (EFI_ERROR(Status)) {
    //
    // Tcg2 protocol is not installed. So, TPM2 is not present.
    //
    DEBUG((DEBUG_ERROR, "Tpm2SubmitCommand - Tcg2 - %r\n", Status));
    return EFI_NOT_FOUND;
  }

  Dev->Tpm = mTcg2Protocol;

  GuidHob = GetFirstGuidHob(&gMyHobGuid);
  if (GuidHob != NULL) {
    DEBUG((EFI_D_INFO, "[VirtioTpmInit] Hob change!\n"));
    FunctionPointer = (EFI_PHYSICAL_ADDRESS*)GET_GUID_HOB_DATA(GuidHob);
    *FunctionPointer = (EFI_PHYSICAL_ADDRESS)(UINTN)Tpm2VirtioTpmCommandDXE; // TODO: 未完成。且 OVMF 启动会陷入奇怪的错误中，需要回溯一下确认一下原因！
  }


UnmapQueue:
  Dev->VirtIo->UnmapSharedBuffer(Dev->VirtIo, Dev->RingMap);

ReleaseQueue:
  VirtioRingUninit(Dev->VirtIo, &Dev->Ring);

Failed:
  //
  // Notify the host about our failure to setup: virtio-0.9.5, 2.2.2.1 Device
  // Status. VirtIo access failure here should not mask the original error.
  //
  NextDevStat |= VSTAT_FAILED;
  Dev->VirtIo->SetDeviceStatus(Dev->VirtIo, NextDevStat);

  return Status; // reached only via Failed above
}
  
STATIC
VOID
EFIAPI
VirtioTpmUninit (
  IN OUT VIRTIO_TPM_DEV  *Dev
  )
{
  //
  // Reset the virtual device -- see virtio-0.9.5, 2.2.2.1 Device Status. When
  // VIRTIO_CFG_WRITE() returns, the host will have learned to stay away from
  // the old comms area.
  //
  Dev->Ready = FALSE;
  Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, 0);
  Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, Dev->RingMap);

  VirtioRingUninit (Dev->VirtIo, &Dev->Ring);
}

//
// Event notification function enqueued by ExitBootServices().
//

STATIC
VOID
EFIAPI
VirtioTpmExitBoot (
  IN  EFI_EVENT  Event,
  IN  VOID       *Context
  )
{
  VIRTIO_TPM_DEV  *Dev;

  DEBUG ((DEBUG_INFO, "%a: Context=0x%p\n", __func__, Context));
  //
  // Reset the device. This causes the hypervisor to forget about the virtio
  // ring.
  //
  // We allocated said ring in EfiBootServicesData type memory, and code
  // executing after ExitBootServices() is permitted to overwrite it.
  //
  Dev        = Context;
  Dev->Ready = FALSE;
  Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, 0);
}

//
// Probe, start and stop functions of this driver, called by the DXE core for
// specific devices.
//
// The following specifications document these interfaces:
// - Driver Writer's Guide for UEFI 2.3.1 v1.01, 9 Driver Binding Protocol
// - UEFI Spec 2.3.1 + Errata C, 10.1 EFI Driver Binding Protocol
//
// The implementation follows:
// - Driver Writer's Guide for UEFI 2.3.1 v1.01
//   - 5.1.3.4 OpenProtocol() and CloseProtocol()
// - UEFI Spec 2.3.1 + Errata C
//   -  6.3 Protocol Handler Services
//

STATIC
EFI_STATUS
EFIAPI
VirtioTpmDriverBindingSupported (
  IN EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN EFI_HANDLE                   DeviceHandle,
  IN EFI_DEVICE_PATH_PROTOCOL     *RemainingDevicePath
  )
{
  EFI_STATUS              Status;
  VIRTIO_DEVICE_PROTOCOL  *VirtIo;

  //
  // Attempt to open the device with the VirtIo set of interfaces. On success,
  // the protocol is "instantiated" for the VirtIo device. Covers duplicate
  // open attempts (EFI_ALREADY_STARTED).
  //
  Status = gBS->OpenProtocol (
                  DeviceHandle,               // candidate device
                  &gVirtioDeviceProtocolGuid, // for generic VirtIo access
                  (VOID **)&VirtIo,           // handle to instantiate
                  This->DriverBindingHandle,  // requestor driver identity
                  DeviceHandle,               // ControllerHandle, according to
                                              // the UEFI Driver Model
                  EFI_OPEN_PROTOCOL_BY_DRIVER // get exclusive VirtIo access to
                                              // the device; to be released
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (VirtIo->SubSystemDeviceId != VIRTIO_SUBSYSTEM_TRUSTED_PLATFORM_MODULE) {
    Status = EFI_UNSUPPORTED;
  }

  //
  // We needed VirtIo access only transitorily, to see whether we support the
  // device or not.
  //
  gBS->CloseProtocol (
         DeviceHandle,
         &gVirtioDeviceProtocolGuid,
         This->DriverBindingHandle,
         DeviceHandle
         );
  return Status;
}

STATIC
EFI_STATUS
EFIAPI
VirtioTpmDriverBindingStart (
  IN EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN EFI_HANDLE                   DeviceHandle,
  IN EFI_DEVICE_PATH_PROTOCOL     *RemainingDevicePath
  )
{
  VIRTIO_TPM_DEV  *Dev;
  EFI_STATUS      Status;
  DEBUG ((EFI_D_INFO, "VirtioTpmDriverBindingStart\n" ));
  Dev = (VIRTIO_TPM_DEV *)AllocateZeroPool (sizeof *Dev);
  if (Dev == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  Status = gBS->OpenProtocol (
                  DeviceHandle,
                  &gVirtioDeviceProtocolGuid,
                  (VOID **)&Dev->VirtIo,
                  This->DriverBindingHandle,
                  DeviceHandle,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
  if (EFI_ERROR (Status)) {
    goto FreeVirtioTpm;
  }

  //
  // VirtIo access granted, configure virtio-rng device.
  //
  Status = VirtioTpmInit (Dev);
  if (EFI_ERROR (Status)) {
    goto CloseVirtIo;
  }

  Status = gBS->CreateEvent (
                  EVT_SIGNAL_EXIT_BOOT_SERVICES,
                  TPL_CALLBACK,
                  &VirtioTpmExitBoot,
                  Dev,
                  &Dev->ExitBoot
                  );
  if (EFI_ERROR (Status)) {
    goto UninitDev;
  }

  //
  // Setup complete, attempt to export the driver instance's EFI_RNG_PROTOCOL
  // interface.
  //
  Dev->Signature = VIRTIO_TPM_SIG;
  Status         = gBS->InstallProtocolInterface (
                          &DeviceHandle,
                          &gEfiTcg2ProtocolGuid,
                          EFI_NATIVE_INTERFACE,
                          &Dev->Tpm
                          );
  if (EFI_ERROR (Status)) {
    goto CloseExitBoot;
  }

  // 设置一个 TPM 的属性，然后。
  PTP_CRB_REGISTERS_PTR CrbReg = (PTP_CRB_REGISTERS_PTR)(UINTN)PcdGet64 (PcdTpmBaseAddress);
  MmioWrite8 ((UINTN)&CrbReg->Reserved1[0], 1);

  DEBUG ((EFI_D_INFO, "[VirtioTpmDriverBindingStart]: %d\n", MmioRead8 ((UINTN)&CrbReg->Reserved1[0])));

  return EFI_SUCCESS;

CloseExitBoot:
  gBS->CloseEvent (Dev->ExitBoot);

UninitDev:
  VirtioTpmUninit (Dev);

CloseVirtIo:
  gBS->CloseProtocol (
         DeviceHandle,
         &gVirtioDeviceProtocolGuid,
         This->DriverBindingHandle,
         DeviceHandle
         );

FreeVirtioTpm:
  FreePool (Dev);

  return Status;
}

STATIC
EFI_STATUS
EFIAPI
VirtioTpmDriverBindingStop (
  IN EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN EFI_HANDLE                   DeviceHandle,
  IN UINTN                        NumberOfChildren,
  IN EFI_HANDLE                   *ChildHandleBuffer
  )
{
  EFI_STATUS        Status;
  EFI_TCG2_PROTOCOL  *Tpm;
  VIRTIO_TPM_DEV    *Dev;

  Status = gBS->OpenProtocol (
                  DeviceHandle,                     // candidate device
                  &gEfiTcg2ProtocolGuid,             // retrieve the RNG iface
                  (VOID **)&Tpm,                    // target pointer
                  This->DriverBindingHandle,        // requestor driver ident.
                  DeviceHandle,                     // lookup req. for dev.
                  EFI_OPEN_PROTOCOL_GET_PROTOCOL    // lookup only, no new ref.
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Dev = VIRTIO_TRUSTED_PLATFORM_MODULE_FROM_TPM (Tpm);

  //
  // Handle Stop() requests for in-use driver instances gracefully.
  //
  Status = gBS->UninstallProtocolInterface (
                  DeviceHandle,
                  &gEfiTcg2ProtocolGuid,
                  &Dev->Tpm
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  gBS->CloseEvent (Dev->ExitBoot);

  VirtioTpmUninit (Dev);

  gBS->CloseProtocol (
         DeviceHandle,
         &gVirtioDeviceProtocolGuid,
         This->DriverBindingHandle,
         DeviceHandle
         );

  FreePool (Dev);

  return EFI_SUCCESS;
}

//
// The static object that groups the Supported() (ie. probe), Start() and
// Stop() functions of the driver together. Refer to UEFI Spec 2.3.1 + Errata
// C, 10.1 EFI Driver Binding Protocol.
//
STATIC EFI_DRIVER_BINDING_PROTOCOL  gDriverBinding = {
  &VirtioTpmDriverBindingSupported,
  &VirtioTpmDriverBindingStart,
  &VirtioTpmDriverBindingStop,
  0x10, // Version, must be in [0x10 .. 0xFFFFFFEF] for IHV-developed drivers
  NULL, // ImageHandle, to be overwritten by
        // EfiLibInstallDriverBindingComponentName2() in VirtioRngEntryPoint()
  NULL  // DriverBindingHandle, ditto
};

//
// The purpose of the following scaffolding (EFI_COMPONENT_NAME_PROTOCOL and
// EFI_COMPONENT_NAME2_PROTOCOL implementation) is to format the driver's name
// in English, for display on standard console devices. This is recommended for
// UEFI drivers that follow the UEFI Driver Model. Refer to the Driver Writer's
// Guide for UEFI 2.3.1 v1.01, 11 UEFI Driver and Controller Names.
//

STATIC
EFI_UNICODE_STRING_TABLE  mDriverNameTable[] = {
  { "eng;en", L"Virtio Trusted Platform Module Driver" },
  { NULL,     NULL                                     }
};

STATIC
EFI_COMPONENT_NAME_PROTOCOL  gComponentName;

STATIC
EFI_STATUS
EFIAPI
VirtioTpmGetDriverName (
  IN  EFI_COMPONENT_NAME_PROTOCOL  *This,
  IN  CHAR8                        *Language,
  OUT CHAR16                       **DriverName
  )
{
  return LookupUnicodeString2 (
           Language,
           This->SupportedLanguages,
           mDriverNameTable,
           DriverName,
           (BOOLEAN)(This == &gComponentName) // Iso639Language
           );
}

STATIC
EFI_STATUS
EFIAPI
VirtioTpmGetDeviceName (
  IN  EFI_COMPONENT_NAME_PROTOCOL  *This,
  IN  EFI_HANDLE                   DeviceHandle,
  IN  EFI_HANDLE                   ChildHandle,
  IN  CHAR8                        *Language,
  OUT CHAR16                       **ControllerName
  )
{
  return EFI_UNSUPPORTED;
}

STATIC
EFI_COMPONENT_NAME_PROTOCOL  gComponentName = {
  &VirtioTpmGetDriverName,
  &VirtioTpmGetDeviceName,
  "eng" // SupportedLanguages, ISO 639-2 language codes
};

STATIC
EFI_COMPONENT_NAME2_PROTOCOL  gComponentName2 = {
  (EFI_COMPONENT_NAME2_GET_DRIVER_NAME)&VirtioTpmGetDriverName,
  (EFI_COMPONENT_NAME2_GET_CONTROLLER_NAME)&VirtioTpmGetDeviceName,
  "en" // SupportedLanguages, RFC 4646 language codes
};

//
// Entry point of this driver.
//
EFI_STATUS
EFIAPI
VirtioTpmEntryPoint (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  DEBUG ((EFI_D_INFO, "VirtioTpmEntryPoint: 1\n" ));
  return EfiLibInstallDriverBindingComponentName2(
           ImageHandle,
           SystemTable,
           &gDriverBinding,
           ImageHandle,
           &gComponentName,
           &gComponentName2
           );
}
