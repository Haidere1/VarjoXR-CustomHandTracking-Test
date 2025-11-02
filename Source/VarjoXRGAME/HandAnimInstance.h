// HandAnimInstance.h
#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimInstance.h"
#include "HandAnimInstance.generated.h"

UCLASS(Blueprintable)
class UHandAnimInstance : public UAnimInstance
{
    GENERATED_BODY()
public:
    // Read these in your ABP's EventGraph/AnimGraph
    UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "XR|Gestures")
    bool bIsPinching = false;

    UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "XR|Gestures")
    bool bIsGrabbing = false;

    UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "XR|Gestures")
    bool bIsPointing = false;

    // Optional: expose events you can call from code to start montages
    UFUNCTION(BlueprintCallable, Category = "XR|Gestures")
    void OnPinchStarted() { bIsPinching = true; }

    UFUNCTION(BlueprintCallable, Category = "XR|Gestures")
    void OnPinchEnded() { bIsPinching = false; }

    UFUNCTION(BlueprintCallable, Category = "XR|Gestures")
    void OnGrabStarted() { bIsGrabbing = true; }

    UFUNCTION(BlueprintCallable, Category = "XR|Gestures")
    void OnGrabEnded() { bIsGrabbing = false; }
};
