//// MixedRealityManager.cpp
//#include "MixedRealityManager.h"
//#include "Kismet/BlueprintFunctionLibrary.h"
//#include "Engine/Engine.h"
//
//AMixedRealityManager::AMixedRealityManager()
//{
//    PrimaryActorTick.bCanEverTick = false; // We don't need ticking unless you add runtime monitoring
//}
//
//void AMixedRealityManager::BeginPlay()
//{
//    Super::BeginPlay();
//
//    //if (!UVarjoOpenXRFunctionLibrary::IsMixedRealitySupported())
//    //{
//    //    UE_LOG(LogTemp, Warning, TEXT("Mixed Reality not supported on this system."));
//    //}
//    //else
//    //{
//    //    UE_LOG(LogTemp, Log, TEXT("Mixed Reality supported and ready."));
//    //}
//}
//
//void AMixedRealityManager::SetMixedRealityEnabled(bool bEnable)
//{/*
//    bool Success = UVarjoOpenXRFunctionLibrary::SetMixedRealityEnabled(bEnable);
//    if (Success)
//    {
//        UE_LOG(LogTemp, Log, TEXT("Mixed Reality %s"), bEnable ? TEXT("Enabled") : TEXT("Disabled"));
//    }
//    else
//    {
//        UE_LOG(LogTemp, Error, TEXT("Failed to toggle Mixed Reality."));
//    }*/
//}
//
////bool AMixedRealityManager::IsMixedRealitySupported() const
////{
////    //return UVarjoOpenXRFunctionLibrary::IsMixedRealitySupported();
////}
//
////bool AMixedRealityManager::IsMixedRealityEnabled() const
////{
////    //return UVarjoOpenXRFunctionLibrary::IsMixedRealityEnabled();
////}
//
////void AMixedRealityManager::ToggleMixedReality()
////{
////  /*  bool bCurrentlyEnabled = UVarjoOpenXRFunctionLibrary::IsMixedRealityEnabled();
////    SetMixedRealityEnabled(!bCurrentlyEnabled);*/
////}
