#include <IndustryStandard/Tpm20.h>
#include <IndustryStandard/TpmPtp.h>

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Library/DebugLib.h>
#include <Library/Tpm2DeviceLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/VirtioLib.h> // No such file or directory

#include <Protocol/ComponentName.h>
#include <Protocol/DriverBinding.h>
#include <Protocol/Tcg2Protocol.h>

#include <IndustryStandard/TpmVirt.h>
#include <IndustryStandard/Virtio.h>

#define VIRTIO_TPM_SIG  SIGNATURE_32 ('V', 'T', 'P', 'M')

typedef struct {
  //
  // Parts of this structure are initialized / torn down in various functions
  // at various call depths. The table to the right should make it easier to
  // track them.
  //
  //                        field              init function       init depth
  //                        ----------------   ------------------  ----------
  UINT32                    Signature;      // DriverBindingStart   0
  VIRTIO_DEVICE_PROTOCOL    *VirtIo;        // DriverBindingStart   0
  EFI_EVENT                 ExitBoot;       // DriverBindingStart   0
  VRING                     Ring;           // VirtioRingInit       2
  EFI_TCG2_PROTOCOL         *Tpm;           // VirtioTpmInit        1
  VOID                      *RingMap;       // VirtioRingMap        2
  BOOLEAN                   Ready;
} VIRTIO_TPM_DEV;

#define VIRTIO_TRUSTED_PLATFORM_MODULE_FROM_TPM(TpmPointer) \
          CR (TpmPointer, VIRTIO_TPM_DEV, Tpm, VIRTIO_TPM_SIG)





EFI_STATUS
Tpm2VirtioTpmCommand( // SecurityPkg 需要，但是它位于 OvmfPkg
  IN     PTP_CRB_REGISTERS_PTR  CrbReg,
  IN     UINT8                 *BufferIn,
  IN     UINT32                SizeIn,
  IN OUT UINT8                 *BufferOut,
  IN OUT UINT32                *SizeOut
) {
  DEBUG ((EFI_D_INFO, "[Tpm2VirtioTpmCommand]\n" ));
  
  VIRTIO_TPM_DEV        *Dev;
  DESC_INDICES          Indices;
  volatile UINT8        *Buffer;
  UINTN                 Index;
  UINT32                Len;
  UINT32                BufferSize;
  EFI_STATUS            Status;
  EFI_PHYSICAL_ADDRESS  DeviceAddress;
  VOID                  *Mapping;
  EFI_TCG2_PROTOCOL  *mTcg2Protocol;

  Status = gBS->LocateProtocol (&gEfiTcg2ProtocolGuid, NULL, (VOID **)&mTcg2Protocol);
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
  // 改 （Step.3）

  //
  // Map Buffer's system physical address to device address
  //
  // Status = VirtioMapAllBytesInSharedBuffer (
  //            Dev->VirtIo,
  //            VirtioOperationBusMasterWrite,
  //            (VOID *)Buffer,
  //            RNGValueLength, // TODO?
  //            &DeviceAddress,
  //            &Mapping
  //            );
  // if (EFI_ERROR (Status)) {
  //   Status = EFI_DEVICE_ERROR;
  //   goto FreeBuffer;
  // }

  // 发送后，再接收。
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
  
  // Unmap the device buffer before accessing it.
  
  Status = Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, Mapping);
  if (EFI_ERROR (Status)) {
    Status = EFI_DEVICE_ERROR;
    goto FreeBuffer;
  }


  Status = EFI_SUCCESS;
  
UnmapBuffer:
  
  // If we are reached here due to the error then unmap the buffer otherwise
  // the buffer is already unmapped after VirtioFlush().
  
  if (EFI_ERROR (Status)) {
    Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, Mapping);
  }

FreeBuffer:
  FreePool ((VOID *)Buffer);
  return Status;
}
