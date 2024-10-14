#include <IndustryStandard/Tpm20.h>
#include <IndustryStandard/TpmPtp.h>

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Library/DebugLib.h>
#include <Library/Tpm2DeviceLib.h>
#include <Library/PcdLib.h>

#include <IndustryStandard/TpmVirt.h>


EFI_STATUS
Tpm2VirtioTpmCommand(
  IN     PTP_CRB_REGISTERS_PTR  TisReg,
  IN     UINT8                 *BufferIn,
  IN     UINT32                SizeIn,
  IN OUT UINT8                 *BufferOut,
  IN OUT UINT32                *SizeOut
) {
  DEBUG ((EFI_D_INFO, "[Tpm2VirtioTpmCommand]\n" ));
  
  // 获取 Virtio queue，利用 ProtocolID 再找。

  // 然后 Kickoff
  return EFI_SUCCESS; // ？ How to ？？？

}




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
// STATIC
// EFI_STATUS
// EFIAPI
// VirtioTpmSubmitCommand (
//   IN EFI_TCG2_PROTOCOL  *This,
//   IN UINT32             InputParameterBlockSize,
//   IN UINT8              *InputParameterBlock,
//   IN UINT32             OutputParameterBlockSize,
//   IN UINT8              *OutputParameterBlock
//   )
// {

  // VIRTIO_TPM_DEV        *Dev;
  // DESC_INDICES          Indices;
  // volatile UINT8        *Buffer;
  // UINTN                 Index;
  // UINT32                Len;
  // UINT32                BufferSize;
  // EFI_STATUS            Status;
  // EFI_PHYSICAL_ADDRESS  DeviceAddress;
  // VOID                  *Mapping;

  // if ((This == NULL) ||
  //     (InputParameterBlockSize == 0) || (InputParameterBlock == NULL) ||
  //     (OutputParameterBlockSize == 0) || (OutputParameterBlock == NULL))
  // {
  //   return EFI_INVALID_PARAMETER;
  // }

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

  // Buffer = (volatile UINT8 *)AllocatePool (RNGValueLength);
  // if (Buffer == NULL) {
  //   return EFI_DEVICE_ERROR;
  // }

  // Dev = VIRTIO_TRUSTED_PLATFORM_MODULE_FROM_TPM (This); // 利用结构体成员获取整个结构体
  // if (!Dev->Ready) {
  //   DEBUG ((DEBUG_INFO, "%a: not ready\n", __func__));
  //   return EFI_DEVICE_ERROR;
  // }

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
  // for (Index = 0; Index < RNGValueLength; Index += Len) {
  //   BufferSize = (UINT32)MIN (RNGValueLength - Index, (UINTN)MAX_UINT32);

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

  
  Unmap the device buffer before accessing it.
  
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
  
  If we are reached here due to the error then unmap the buffer otherwise
  the buffer is already unmapped after VirtioFlush().
  
  if (EFI_ERROR (Status)) {
    Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, Mapping);
  }

FreeBuffer:
  FreePool ((VOID *)Buffer);
  return Status;
}

