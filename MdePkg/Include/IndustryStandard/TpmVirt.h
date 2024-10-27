
#ifndef _TPM_VIRTIO_H_
#define _TPM_VIRTIO_H_

#include <IndustryStandard/TpmPtp.h>

EFI_STATUS
Tpm2VirtioTpmCommand(
  IN     PTP_CRB_REGISTERS_PTR  CrbReg,
  IN     UINT8                 *BufferIn,
  IN     UINT32                SizeIn,
  IN OUT UINT8                 *BufferOut,
  IN OUT UINT32                *SizeOut
);

typedef EFI_STATUS (*Tpm2VirtioTpmCommandFunc)(
    IN     PTP_CRB_REGISTERS_PTR CrbReg,
    IN     UINT8* BufferIn,
    IN     UINT32 SizeIn,
    IN OUT UINT8* BufferOut,
    IN OUT UINT32* SizeOut
);

// 声明全局函数指针
extern Tpm2VirtioTpmCommandFunc Tpm2VirtioTpmCommandPtr;

#endif