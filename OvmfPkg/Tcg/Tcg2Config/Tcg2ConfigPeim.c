/** @file
  Set TPM device type

  In SecurityPkg, this module initializes the TPM device type based on a UEFI
  variable and/or hardware detection. In OvmfPkg, the module only performs TPM
  hardware detection.

  Copyright (c) 2015, Intel Corporation. All rights reserved.<BR>
  Copyright (C) 2018, Red Hat, Inc.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <PiPei.h>

#include <Guid/TpmInstance.h>
#include <Library/DebugLib.h>
#include <Library/PeiServicesLib.h>
#include <Library/Tpm2DeviceLib.h>
#include <Ppi/TpmInitialized.h>

#include <Library/HobLib.h>

#include <IndustryStandard/TpmVirt.h>

#include "Tpm12Support.h"

STATIC CONST EFI_PEI_PPI_DESCRIPTOR  mTpmSelectedPpi = {
  (EFI_PEI_PPI_DESCRIPTOR_PPI | EFI_PEI_PPI_DESCRIPTOR_TERMINATE_LIST),
  &gEfiTpmDeviceSelectedGuid,
  NULL
};

STATIC CONST EFI_PEI_PPI_DESCRIPTOR  mTpmInitializationDonePpiList = {
  EFI_PEI_PPI_DESCRIPTOR_PPI | EFI_PEI_PPI_DESCRIPTOR_TERMINATE_LIST,
  &gPeiTpmInitializationDonePpiGuid,
  NULL
};

/**
  The entry point for Tcg2 configuration driver.

  @param  FileHandle  Handle of the file being invoked.
  @param  PeiServices Describes the list of possible PEI Services.
**/
EFI_STATUS
EFIAPI
Tcg2ConfigPeimEntryPoint (
  IN       EFI_PEI_FILE_HANDLE  FileHandle,
  IN CONST EFI_PEI_SERVICES     **PeiServices
  )
{
  UINTN       Size;
  EFI_STATUS  Status;

  DEBUG ((DEBUG_INFO, "%a\n", __func__));
  DEBUG ((EFI_D_INFO, "[Tcg2ConfigPeimEntryPoint]\n" ));
  Status = InternalTpm12Detect ();
  if (!EFI_ERROR (Status)) {
    DEBUG ((DEBUG_INFO, "%a: TPM1.2 detected\n", __func__));
    Size   = sizeof (gEfiTpmDeviceInstanceTpm12Guid);
    Status = PcdSetPtrS (
               PcdTpmInstanceGuid,
               &Size,
               &gEfiTpmDeviceInstanceTpm12Guid
               );
    ASSERT_EFI_ERROR (Status);
  } else {
    Status = Tpm2RequestUseTpm ();
    if (!EFI_ERROR (Status)) {
      DEBUG ((DEBUG_INFO, "%a: TPM2 detected\n", __func__));
      Size   = sizeof (gEfiTpmDeviceInstanceTpm20DtpmGuid);
      Status = PcdSetPtrS (
                 PcdTpmInstanceGuid,
                 &Size,
                 &gEfiTpmDeviceInstanceTpm20DtpmGuid
                 );
      ASSERT_EFI_ERROR (Status);
    } else {
      DEBUG ((DEBUG_INFO, "%a: no TPM detected\n", __func__));
      //
      // If no TPM2 was detected, we still need to install
      // TpmInitializationDonePpi. Namely, Tcg2Pei will exit early upon seeing
      // the default (all-bits-zero) contents of PcdTpmInstanceGuid, thus we have
      // to install the PPI in its place, in order to unblock any dependent
      // PEIMs.
      //
      Status = PeiServicesInstallPpi (&mTpmInitializationDonePpiList);
      ASSERT_EFI_ERROR (Status);
    }
  }

  //
  // Selection done
  //
  Status = PeiServicesInstallPpi (&mTpmSelectedPpi);
  ASSERT_EFI_ERROR (Status);

  // Hob
  // 创建一个 HOB 并存储函数指针的地址

  EFI_GUID gMyHobGuid = { 0xaabbccdd, 0x1234, 0x5678, { 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34, 0x56, 0x78 } };

  EFI_PHYSICAL_ADDRESS FunctionPointer = (EFI_PHYSICAL_ADDRESS)(UINTN)Tpm2VirtioTpmCommandPtr;
  EFI_HOB_GUID_TYPE* Hob = BuildGuidDataHob(
      &gMyHobGuid,    // 自定义的 HOB GUID
      &FunctionPointer,            // 函数指针地址
      sizeof(EFI_PHYSICAL_ADDRESS)  // 存储的大小
  );

  // 检查 HOB 是否成功创建

  DEBUG ((DEBUG_INFO, "[Tcg2ConfigPeimEntryPoint] Hob Created Succ?\n", Hob != NULL));
  
  return Status;
}
