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
  VRING                     Ring;        // VirtioRingInit       2
  EFI_TCG2_PROTOCOL         *Tpm;           // VirtioTpmInit        1
  VOID                      *RingMap;       // VirtioRingMap        2
  BOOLEAN                   Ready;
} VIRTIO_TPM_DEV;

#define VIRTIO_TRUSTED_PLATFORM_MODULE_FROM_TPM(TpmPointer) \
          CR (TpmPointer, VIRTIO_TPM_DEV, Tpm, VIRTIO_TPM_SIG)
