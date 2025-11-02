//// MixedRealityManager.h
//#pragma once
//
//#include "CoreMinimal.h"
//#include "GameFramework/Actor.h"
//#include "MixedRealityManager.generated.h"
//
//UCLASS(Blueprintable, BlueprintType)
//class VARJOXRGAME_API AMixedRealityManager : public AActor
//{
//    GENERATED_BODY()
//
//public:
//    AMixedRealityManager();
//
//protected:
//    virtual void BeginPlay() override;
//
//public:
//    // ? Enable or disable Mixed Reality mode
//    UFUNCTION(BlueprintCallable, Category = "Varjo|MixedReality")
//    void SetMixedRealityEnabled(bool bEnable);
//
//    // ? Check if MR is supported
//    UFUNCTION(BlueprintCallable, Category = "Varjo|MixedReality")
//    bool IsMixedRealitySupported() const;
//
//    // ? Check if MR is currently enabled
//    UFUNCTION(BlueprintCallable, Category = "Varjo|MixedReality")
//    bool IsMixedRealityEnabled() const;
//
//    // ? (Optional) Toggle MR state
//    UFUNCTION(BlueprintCallable, Category = "Varjo|MixedReality")
//    void ToggleMixedReality();
//};
