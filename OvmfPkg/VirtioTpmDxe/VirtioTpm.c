/** @file

  This driver produces EFI_TCG2_PROTOCOL instances for virtio-tpm devices.

  The implementation is based on OvmfPkg/VirtioRngDxe/VirtioRng.c

**/

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/VirtioLib.h>

#include "VirtioTpm.h"

/**
  This service enables the sending of commands to the TPM.

  @param[in]  This                     Indicates the calling context
  @param[in]  InputParameterBlockSize  Size of the TPM input parameter block.
  @param[in]  InputParameterBlock      Pointer to the TPM input parameter block.
  @param[in]  OutputParameterBlockSize Size of the TPM output parameter block.
  @param[in]  OutputParameterBlock     Pointer to the TPM output parameter block.

  @retval EFI_SUCCESS            The command byte stream was successfully sent to the device and a response was successfully received.
  @retval EFI_DEVICE_ERROR       The command was not successfully sent to the device or a response was not successfully received from the device.
  @retval EFI_INVALID_PARAMETER  One or more of the parameters are incorrect.
  @retval EFI_BUFFER_TOO_SMALL   The output parameter block is too small.
**/
STATIC
EFI_STATUS
EFIAPI
VirtioTpmSubmitCommand (
  IN EFI_TCG2_PROTOCOL  *This,
  IN UINT32             InputParameterBlockSize,
  IN UINT8              *InputParameterBlock,
  IN UINT32             OutputParameterBlockSize,
  IN UINT8              *OutputParameterBlock
  )
{
  VIRTIO_TPM_DEV        *Dev;
  DESC_INDICES          Indices;
  volatile UINT8        *Buffer;
  UINTN                 Index;
  UINT32                Len;
  UINT32                BufferSize;
  EFI_STATUS            Status;
  EFI_PHYSICAL_ADDRESS  DeviceAddress;
  VOID                  *Mapping;

  if ((This == NULL) ||
      (InputParameterBlockSize == 0) || (InputParameterBlock == NULL) ||
      (OutputParameterBlockSize == 0) || (OutputParameterBlock == NULL))
  {
    return EFI_INVALID_PARAMETER;
  }

  // 获取 mTcgDxeData 变量。

  // if (!mTcgDxeData.BsCap.TPMPresentFlag) {
  //   return EFI_DEVICE_ERROR;
  // }

  // if (InputParameterBlockSize > mTcgDxeData.BsCap.MaxCommandSize) {
  //   return EFI_INVALID_PARAMETER;
  // }

  // if (OutputParameterBlockSize > mTcgDxeData.BsCap.MaxResponseSize) {
  //   return EFI_INVALID_PARAMETER;
  // }

  Buffer = (volatile UINT8 *)AllocatePool (RNGValueLength);
  if (Buffer == NULL) {
    return EFI_DEVICE_ERROR;
  }

  Dev = VIRTIO_TRUSTED_PLATFORM_MODULE_FROM_TPM (This); // 利用结构体成员获取整个结构体
  if (!Dev->Ready) {
    DEBUG ((DEBUG_INFO, "%a: not ready\n", __func__));
    return EFI_DEVICE_ERROR;
  }

  //
  // Map Buffer's system physical address to device address
  //
  Status = VirtioMapAllBytesInSharedBuffer (
             Dev->VirtIo,
             VirtioOperationBusMasterWrite,
             (VOID *)Buffer,
             RNGValueLength, // TODO?
             &DeviceAddress,
             &Mapping
             );
  if (EFI_ERROR (Status)) {
    Status = EFI_DEVICE_ERROR;
    goto FreeBuffer;
  }

  // 几步走？
  // 参考：PtpCrbTpmCommand
  // Step.1 Wait IDLE 不需要考虑，已经删除 READY/IDLE 属性。
  // Step.2 将要发送的 TPM 命令的长度描述等信息写入到寄存器空间中。
  // Step.3 Ring 1 到 CrbReg->CrbControlStart 处，触发后端 TPM 命令发射。发射后进行 Poll，待 CrbControlStart 处寄存器设置为 0 时，命令处理完成，出现 Response。
  
  // 直接发射？
  // poll or 等 Notify？

  // （后两步差不多。   ）
  // 读取 Res 头，检查。
  // 读取 Res 内容。

  

  //
  // The Virtio RNG device may return less data than we asked it to, and can
  // only return MAX_UINT32 bytes per invocation. So loop as long as needed to
  // get all the entropy we were asked for.
  //
  for (Index = 0; Index < RNGValueLength; Index += Len) {
    BufferSize = (UINT32)MIN (RNGValueLength - Index, (UINTN)MAX_UINT32);

    VirtioPrepare (&Dev->Ring, &Indices);
    VirtioAppendDesc (
      &Dev->Ring,
      DeviceAddress + Index,
      BufferSize,
      VRING_DESC_F_WRITE,
      &Indices
      );

    if (VirtioFlush (Dev->VirtIo, 0, &Dev->Ring, &Indices, &Len) !=
        EFI_SUCCESS)
    {
      Status = EFI_DEVICE_ERROR;
      goto UnmapBuffer;
    }

    ASSERT (Len > 0);
    ASSERT (Len <= BufferSize);
  }

  //
  // Unmap the device buffer before accessing it.
  //
  Status = Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, Mapping);
  if (EFI_ERROR (Status)) {
    Status = EFI_DEVICE_ERROR;
    goto FreeBuffer;
  }

  for (Index = 0; Index < RNGValueLength; Index++) {
    RNGValue[Index] = Buffer[Index];
  }

  Status = EFI_SUCCESS;

UnmapBuffer:
  //
  // If we are reached here due to the error then unmap the buffer otherwise
  // the buffer is already unmapped after VirtioFlush().
  //
  if (EFI_ERROR (Status)) {
    Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, Mapping);
  }

FreeBuffer:
  FreePool ((VOID *)Buffer);
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
  UINT16      QueueSize;
  UINT64      Features;
  UINT64      RingBaseShift;

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
  // step 4b -- allocate request virtqueue, just use #0
  //
  Status = Dev->VirtIo->SetQueueSel (Dev->VirtIo, 0);
  if (EFI_ERROR (Status)) {
    goto Failed;
  }

  Status = Dev->VirtIo->GetQueueNumMax (Dev->VirtIo, &QueueSize);
  if (EFI_ERROR (Status)) {
    goto Failed;
  }

  //
  // VirtioRngGetRNG() uses one descriptor
  //
  if (QueueSize < 1) {
    Status = EFI_UNSUPPORTED;
    goto Failed;
  }

  Status = VirtioRingInit (Dev->VirtIo, QueueSize, &Dev->Ring);
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

  //
  // Additional steps for MMIO: align the queue appropriately, and set the
  // size. If anything fails from here on, we must unmap the ring resources.
  //
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
  if (Dev->VirtIo->Revision < VIRTIO_SPEC_REVISION (1, 0, 0)) {
    Features &= ~(UINT64)(VIRTIO_F_VERSION_1 | VIRTIO_F_IOMMU_PLATFORM);
    Status    = Dev->VirtIo->SetGuestFeatures (Dev->VirtIo, Features);
    if (EFI_ERROR (Status)) {
      goto UnmapQueue;
    }
  }

  //
  // step 6 -- initialization complete
  //
  NextDevStat |= VSTAT_DRIVER_OK;
  Status       = Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, NextDevStat);
  if (EFI_ERROR (Status)) {
    goto UnmapQueue;
  }

  //
  // populate the exported interface's attributes
  //
  Dev->Tpm.SubmitCommand = VirtioTpmSubmitCommand;
  // Tcg2HashLogExtendEvent 没有头文件，如何导入到这里？X
  // 或者将 Dev->Tpm 获取 mTcg2Protocol 变量？
  // 如此一来，不需要再进行 switch 对应的 interface 类型了。
  

  Dev->Ready       = TRUE;

  return EFI_SUCCESS;

UnmapQueue:
  Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, Dev->RingMap);

ReleaseQueue:
  VirtioRingUninit (Dev->VirtIo, &Dev->Ring);

Failed:
  //
  // Notify the host about our failure to setup: virtio-0.9.5, 2.2.2.1 Device
  // Status. VirtIo access failure here should not mask the original error.
  //
  NextDevStat |= VSTAT_FAILED;
  Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, NextDevStat);

  return Status; // reached only via Failed above
}

STATIC
VOID
EFIAPI
VirtioRngUninit (
  IN OUT VIRTIO_RNG_DEV  *Dev
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
VirtioRngExitBoot (
  IN  EFI_EVENT  Event,
  IN  VOID       *Context
  )
{
  VIRTIO_RNG_DEV  *Dev;

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
                          &gEfiTpmProtocolGuid,
                          EFI_NATIVE_INTERFACE,
                          NULL // &Dev->Tpm
                          );
  if (EFI_ERROR (Status)) {
    goto CloseExitBoot;
  }

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
  EFI_RNG_PROTOCOL  *Rng;
  VIRTIO_RNG_DEV    *Dev;

  Status = gBS->OpenProtocol (
                  DeviceHandle,                     // candidate device
                  &gEfiRngProtocolGuid,             // retrieve the RNG iface
                  (VOID **)&Rng,                    // target pointer
                  This->DriverBindingHandle,        // requestor driver ident.
                  DeviceHandle,                     // lookup req. for dev.
                  EFI_OPEN_PROTOCOL_GET_PROTOCOL    // lookup only, no new ref.
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Dev = VIRTIO_ENTROPY_SOURCE_FROM_RNG (Rng);

  //
  // Handle Stop() requests for in-use driver instances gracefully.
  //
  Status = gBS->UninstallProtocolInterface (
                  DeviceHandle,
                  &gEfiRngProtocolGuid,
                  &Dev->Rng
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  gBS->CloseEvent (Dev->ExitBoot);

  VirtioRngUninit (Dev);

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
  return EfiLibInstallDriverBindingComponentName2 (
           ImageHandle,
           SystemTable,
           &gDriverBinding,
           ImageHandle,
           &gComponentName,
           &gComponentName2
           );
}
