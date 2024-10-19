/** @file

  Private definitions of the VirtioTpm TPM driver


**/

#ifndef _VIRTIO_TPM_DXE_H_
#define _VIRTIO_TPM_DXE_H_

#include <Protocol/ComponentName.h>
#include <Protocol/DriverBinding.h>
#include <Protocol/Tcg2Protocol.h>

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

#endif
