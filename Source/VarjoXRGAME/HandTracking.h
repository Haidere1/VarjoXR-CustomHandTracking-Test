// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "IXRTrackingSystem.h"
#include "Components/PoseableMeshComponent.h"
#include "MotionControllerComponent.h"
#include "Components/WidgetComponent.h"
#include "CesiumGlobeAnchorComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "HandTracking.generated.h"
class UCameraComponent;
UENUM() enum class EPathPhase : uint8 { Free, ToTarget, ToZZero };
enum class EState : uint8 { Normal, AtOrigin, Tweener }; // Tweener: active smoothing (used transiently)
struct FTween
{
    bool    bActive = false;
    double  T = 0.0;
    double  Duration = 0.5;

    // endpoints
    FVector StartPos, EndPos;
    FQuat   StartRot, EndRot;

    // spherical-safe extras
    bool    bUseSafeSpherical = false;
    FVector Center;            // globe center (use your RefPosUU)
    double  ClearanceUU = 300000.0; // push path this far outside the smaller endpoint radius
    FVector Dir0, Dir1;        // normalized directions from Center to endpoints
    double  R0 = 0.0, R1 = 0.0;

    static double EaseInOut(double t)
    {
        return (t < 0.5) ? (2.0 * t * t) : (1.0 - FMath::Pow(-2.0 * t + 2.0, 2.0) / 2.0);
    }

    // legacy linear begin (kept for compatibility)
    void Begin(const FVector& SP, const FQuat& SR, const FVector& EP, const FQuat& ER, double Dur)
    {
        StartPos = SP; StartRot = SR; EndPos = EP; EndRot = ER;
        Duration = FMath::Max(1e-3, Dur);
        T = 0.0;
        bActive = true;
        bUseSafeSpherical = false;
    }

    // NEW: spherical/clearance-safe begin
    void BeginSafe(const FVector& SP, const FQuat& SR,
        const FVector& EP, const FQuat& ER,
        double Dur,
        const FVector& InCenter,
        double InClearanceUU /*~300–800 km in UU, tune to taste*/)
    {
        Begin(SP, SR, EP, ER, Dur);
        bUseSafeSpherical = true;
        Center = InCenter;
        ClearanceUU = FMath::Max(0.0, InClearanceUU);

        Dir0 = (SP - Center).GetSafeNormal();
        Dir1 = (EP - Center).GetSafeNormal();
        R0 = FVector::Dist(SP, Center);
        R1 = FVector::Dist(EP, Center);

        if (Dir0.IsNearlyZero() || Dir1.IsNearlyZero())
            bUseSafeSpherical = false; // fall back to linear if degenerate
    }

    bool Tick(double dt, USceneComponent* Root)
    {
        if (!bActive || !Root) return true;

        T = FMath::Min(1.0, T + dt / Duration);
        const double k = EaseInOut(T);

        // interpolate rotation first (unchanged)
        const FQuat R = FQuat::Slerp(StartRot, EndRot, k);

        FVector P;
        if (bUseSafeSpherical)
        {
            // direction lerp around globe (normalize to keep on sphere-like arc)
            FVector Dir = (Dir0 * (1.0 - k) + Dir1 * k).GetSafeNormal();
            if (Dir.IsNearlyZero())
            {
                // emergency fallback: project linear point outward
                const FVector Plinear = FMath::Lerp(StartPos, EndPos, k);
                Dir = (Plinear - Center).GetSafeNormal();
            }

            // radius lerp but never go inside min endpoint radius + clearance
            double Rlin = FMath::Lerp(R0, R1, k);
            const double RminAllowed = FMath::Min(R0, R1) + ClearanceUU;
            if (Rlin < RminAllowed) Rlin = RminAllowed;

            P = Center + Dir * Rlin;
        }
        else
        {
            // original straight-line tween
            P = FMath::Lerp(StartPos, EndPos, k);
        }

        Root->SetWorldLocation(P);
        Root->SetWorldRotation(R);

        if (T >= 1.0) { bActive = false; return true; }
        return false;
    }
};
enum class EPinchAxis : uint8 { None, Horizontal, Vertical };

static FTween Tween;

UCLASS()

class VARJOXRGAME_API AHandTracking : public ACharacter
{
    GENERATED_BODY()

public:
    AHandTracking();
    UPROPERTY(EditAnywhere, Category = "Orbit|Input")
    float BaselineArmUU = 2.0f;
    double PrevHeightM = 0.0;
    UPROPERTY(EditAnywhere, Category = "Orbit|Entry")
    double EntryHeightM = 8000000.0; 

    // Hysteresis so we don't bounce around the threshold
    UPROPERTY(EditAnywhere, Category = "Orbit|Entry")
    double EntryUpHysteresisM = 2000.0;   // how far above to confirm entry crossing
    UPROPERTY(EditAnywhere, Category = "Orbit|Entry")
  
    double ExitDownHysteresisM = 100.0; // how far below to re-arm entry

    // Optional: keep camera aimed at globe while above the threshold
    UPROPERTY(EditAnywhere, Category = "Orbit|Entry")
    bool bAimAtGlobeWhenHigh = true;

    bool bEntryArmedByAlt=false;

    UPROPERTY(VisibleAnywhere, Category = "Orbit|Entry")
    FVector StraightDirOnEntry = FVector::ForwardVector;

    double MaxOutAltitudeM = 8.0e7;
    FVector EnterPosition = FVector::ZeroVector;

    float ExitLerpSec = 0.75f;   // duration of the exit tween
    bool  bTweeningExit = false;

    UPROPERTY(EditAnywhere, Category = "Orbit")
    float CurrentTrueAnomalyDeg = 2.0f; // running anomaly

    UPROPERTY(EditAnywhere, Category = "Orbit")
    float RAANDeg = 0.0f;

    UPROPERTY(EditAnywhere, Category = "Orbit")
    float OrbitInclinationDeg = 0.0f;
    bool bOrbitControlsPawnRotation = false;
    // --- Pinch?Angular speed mapping ---
    // Degrees per second per 1 Unreal unit of pinch delta (tune to taste).


    void   RefreshEarthCenterIfNeeded();
    FVector UnrealFromLLH(double LonDeg, double LatDeg, double HeightMeters) const;
    double GetHeightM() const;
    double GetHeightMAtPos(const FVector& Pos) const;
    FRotator MakeUprightRotAtPos(const FVector& Pos, const FVector& PrefFwd) const;
    FRotator MakeLookAtEarthRotAtPos(const FVector& Pos) const;




    const double HMin = 4000.0;     // height where you want MinDeg
    const double HMax = 100000.0;



    UPROPERTY(Transient)
    TObjectPtr<class ACesiumGeoreference> CachedGeo = nullptr;

    // If you want Earth center cheaply in various helpers.
    UPROPERTY(Transient)
    FVector CachedEarthCenterUU = FVector::ZeroVector;

    double LastEarthCenterRefreshTime = -1.0;

    // How often to refresh Earth center (defensive; tweak for your app)
    UPROPERTY(EditAnywhere, Category = "Move|Perf")
    float EarthCenterRefreshInterval = 0.25f;

    UPROPERTY(Transient) bool    bPinchActiveLatched = false;
    UPROPERTY(Transient) FVector PrevSmMid = FVector::ZeroVector;
    UPROPERTY(Transient) FVector BaseMid = FVector::ZeroVector;
    UPROPERTY(Transient) bool    bArmed = false;




    UPROPERTY(EditAnywhere, Category = "Pinch Orbit")
    float PinchToDegPerSec = 30.0f;

    // Max angular speed clamp (comfort!)
    UPROPERTY(EditAnywhere, Category = "Pinch Orbit")
    float MinDegPerSec = 0.001f;
    UPROPERTY(EditAnywhere, Category = "Pinch Orbit")
    float MaxDegPerSec = 15.0f;
 
    // Optional smoothing time-constant (seconds) for speed changes.
    UPROPERTY(EditAnywhere, Category = "Pinch Orbit")
    float SpeedSmoothingTime = 0.08f;

    // Deadzone around baseline (Unreal units) to ignore tiny jitters.
    UPROPERTY(EditAnywhere, Category = "Pinch Orbit")
    float PinchDeadzone = 0.15f;

    UPROPERTY(EditAnywhere, Category = "Pinch Orbit")
    float DeltaDeadzone = 2.0f;

    // Baseline state while a pinch is held
    bool  bPinchBaselineSet = false;
    float BaselinePinchLen = 0.0f;
    FVector BaselinePinchMidLS = FVector::ZeroVector;
    float SmoothedDY = 0.f, SmoothedDZ = 0.f;
    int32 LockedSign = 0;
    FTransform PinchRefInv;
    UPROPERTY(EditAnywhere, Category = "Pinch Orbit")
    float DeadzoneEnter = 2.0f;
    UPROPERTY(EditAnywhere, Category = "Pinch Orbit")
    float DeadzoneExit = 1.4f;
    // Current angular speed derived from pinch delta (deg/sec)
    float AngularSpeedDegPerSec = 0.0f;

    UPROPERTY(EditAnywhere, Category = "Pinch Orbit|Radius Scale")
    double SpeedScaleMaxRadius = 8.0e5;       // uu
    UPROPERTY(EditAnywhere, Category = "Pinch Orbit|Radius Scale")
    float  SpeedAtMinRadius = 0.35f;       // multiplier at/under MinRadius
    UPROPERTY(EditAnywhere, Category = "Pinch Orbit|Radius Scale")
    float  SpeedAtMaxRadius = 1.00f;
    UPROPERTY(EditAnywhere, Category = "Pinch Orbit|Radius Scale")
    double SpeedScaleMinRadius = 2.0e5;


    EPinchAxis LockedAxis = EPinchAxis::None;



    /** Keep the camera's 'up' roughly aligned with world up to avoid roll. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
    bool bStabilizeCameraUp = true;

    // --- Pinch-driven orbit (tunables) ---
    UPROPERTY(EditAnywhere, Category = "Orbit|Input")
    float PinchDeadzoneUU = 1.0f;          // ignore tiny pinch-distance changes

    UPROPERTY(EditAnywhere, Category = "Orbit|Input")
    float MidDeltaDeadzoneUU = 0.5f;       // ignore tiny midpoint jitter (PrevMidWS delta)

    // "Degrees per second per UU" at a reference radius (RRefCM). Angular speed scales ? (RRef / R)
    UPROPERTY(EditAnywhere, Category = "Orbit|Input")
    float DegPerUnitPinchAtRef = 120.0f;

    UPROPERTY(EditAnywhere, Category = "Orbit|Input")
    double RRefCM = 2.0e6;                 // reference radius (cm) for speed scaling

    UPROPERTY(EditAnywhere, Category = "Orbit|Input")
    float MaxDegPerTick = 20.0f;           // safety cap
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit", meta = (ClampMin = "-180.0", ClampMax = "180.0"))
    float InitialTrueAnomalyDeg = 0.0f;
    float PrevPinchDistUU = -1.0f;
    UPROPERTY(EditAnywhere, Category = "Orbit|Input")
    float DegPerUnitSlideAtRef = 0.02f;

    // tiny sustain so motion never stalls while pinch stays active
    UPROPERTY(EditAnywhere, Category = "Orbit|Input")
    float MinSustainDegPerSec = 1.0f;

    UPROPERTY(EditAnywhere, Category = "Orbit|Input", meta = (ClampMin = "0.0")) float SpeedFactor = 1.0f;
    // --- Internal state ---
    float  PinchBaselineUU = -1.0f;        // set on pinch start
    bool   bPrevPinchActive = false;       // tracks pinch transitions
    FVector PrevMidWS = FVector::ZeroVector; // previous world-space pinch midpoint
    UPROPERTY(EditAnywhere, Category = "Orbit|Speed")
    double SpeedHeightMinM = 1.0e3; // start boosting around 100 km (tweak)

    UPROPERTY(EditAnywhere, Category = "Orbit|Speed")
    double SpeedHeightMaxM = 1.0e9;

    UPROPERTY(EditAnywhere, Category = "Orbit|Speed")
    double SpeedScaleMin = 0.1;     // multiplier near the surface

    UPROPERTY(EditAnywhere, Category = "Orbit|Speed")
    double SpeedScaleMax = 42.0;

    UPROPERTY(EditAnywhere, Category = "Orbit|Speed")
    double RadiusBoostExponent = 0.8;


    FVector EntryPointLocation;
    // Flag to check if the entry point has been fixed
    bool bEntryPointFixed;

    const double EnterAltitudeM = 4200000.0;    // enter when H <= this (e.g., 4,200 km)
    const double ExitAltitudeM = 4900000.0;
 	float AltitudeHysteresisM = 100000.0;
 	float MinAltitudeOvershootM = 20.0f;
 	

    bool    bRightPinchOrbitActive = false;
    FVector PrevSmoothedPinchMid = FVector::ZeroVector;

    // Tunables
    float   PinchSmoothHz = 8.0f;     // low-pass smoothing (higher = snappier)
//    float   PinchDeadzoneUU = 6.0f;     // ignore < this much hand motion per tick
    float   PinchSoftK = 0.6f;     // soft deadzone shaping
    float   HandDeltaMaxUU = 60.0f;    // clamp hand delta
    float   AngleGainNear = 10.6f;     // angle gain when close to earth
    float   AngleGainFar = 30.0f;     // angle gain when far
    float   MaxAnglePerTickDeg = 30.0f;     // safety clamp per tick
    double  R_Planar = 2.0e5;    // <= mostly planar feel
    double  R_Orbital = 2.0e6;
    // Tunables (move to UPROPERTY if you want)
    const double AltRefFastM = 1.0e6; // 10,000,000 km in meters
    const double AltBoostBase = 6.0;    // base multiplier near the reference
    const double AltExponent = 0.9;    // how aggressively speed grows with altitude (1.0 = linear)
    const double AltScaleMax = 400.0;  // absolute cap to avoid crazy spins
    const double HyperBandMult = 3.0;
    bool    bRightPinchWasActive = false;
    FVector PrevRightPinchMid = FVector::ZeroVector;
    FVector OrbitCenterUU = FVector::ZeroVector;


    FVector CrossDirUnit = FVector::ForwardVector;

    // Height baseline while at origin (for return)
    double OriginBaselineHeightM = 0.0;

    // NEW: fixed path direction used BEFORE entering (cached once)
    FVector PreEntryDirUnit = FVector::ForwardVector;
    bool  bGateArmed = false;     // armed only after inside-dwell
    double InsideDwellTime = 0.5; // accumulated time spent well inside
    double ReenterCooldown = 0.5;
    bool bInit = false;
    FVector RefPosUU = FVector::ZeroVector;
    EState State = EState::Normal;
    // Prior-frame position (for crossing interpolation)
     FVector PrevPosUU = FVector::ZeroVector;
    const double DistanceThresholdUU = 500000000.0; // ring radius (UU)
    const double EnterHysteresisUU = DistanceThresholdUU * 0.05; // 0.5% outside band
    const double RearmHysteresisUU = DistanceThresholdUU * 0.020; // 1.0% inside band
    const double MinOvershootUU = DistanceThresholdUU * 0.010; // must exceed ring by this to enter
    const double InsideDwellSec = 0.50;  // must dwell inside before arming
    const double ReenterCooldownSec = 1.00;  // cooldown after return before re-entry possible
    const double ReturnHeightDeltaM = 700000.0;// zoom-out height to trigger return

    // Smooth transition durations
    const double EntryLerpSec = 0.60;  // Normal -> AtOrigin
   // AtOrigin -> Normal

    // Wider "entry area" on return:
    const double EntryConeHalfAngleDeg = 35.0;
    const double FacingInfluence = 0.65;

    FRotator enterRotaion;

    // Enter target (origin LLH)
    const double OriginLonDeg = 72.4;
    const double OriginLatDeg = 33.8692;
    const double OriginHeightM = 100000.0;
    FVector DynamicEntryPoint;
  

    UPROPERTY(EditAnywhere, Category = "Path") FVector TriggerLoc = FVector(-1145985040.f, 0.f, -303592670.f);
    UPROPERTY(EditAnywhere, Category = "Path") FVector TargetLoc = FVector(-493232790.0f, 0.f, 287083830.0f);
    UPROPERTY(EditAnywhere, Category = "Path") float TriggerRadius = 35000000;
    UPROPERTY(EditAnywhere, Category = "Path") float ArriveRadius = 10000.f;
    UPROPERTY(EditAnywhere, Category = "Path") float ZTolerance = 10000.f;

    UPROPERTY(VisibleAnywhere, Category = "Path") EPathPhase Phase = EPathPhase::Free;


    UMotionControllerComponent* LeftHandController;
    UMotionControllerComponent* RightHandController;
    // Cache rest lengths once (per hand)
    TMap<FName, float> RestLenL, RestLenR;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Globe|Motion")
    double TangentLockAltitudeMeters = 5000000.0;   
    // HandTracking.h (add)
    UPROPERTY(EditAnywhere, Category = "Globe|Motion")
    double ApproachSwitchAltitudeMeters = 5000.0;

    UPROPERTY(EditAnywhere, Category = "Globe|Motion")
    double ApproachBlendMeters = 1000.0; // smooth transition band
    // threshold to enter lock

    
    
     // true when above threshold (+hysteresis)
   

   UPROPERTY(EditAnywhere, Category = "Globe|Motion")
    double TangentLockHysteresisMeters = 500.0;

    UPROPERTY(EditAnywhere, Category = "Globe|Motion")
    double UnlockReturnDistanceMeters = 250.0; // how close to lock point to auto-unlock

    UPROPERTY(Transient)
    bool bTangentLocked = false;

    UPROPERTY(Transient)
    FVector LockedUp = FVector::UpVector;

    // Store where we locked (lon/lat in DEGREES)
    UPROPERTY(Transient)
    FVector2D LockLonLatDeg = FVector2D::ZeroVector;

    // After we unlock-by-return, suppress re-lock until we dip below threshold once
    UPROPERTY(Transient)
    bool bSuppressRelockAboveThreshold = false;


    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit")
    float OrbitRadius;

private:
    bool bPrevPinching = false;

    // Forward-declare
  
 
    UInstancedStaticMeshComponent* TipSpheresL = nullptr;

   
    UInstancedStaticMeshComponent* TipSpheresR = nullptr;

    TArray<int32> TipInstanceIdxL;
    TArray<int32> TipInstanceIdxR;

    // Radius you already use for scaling in Init
    UPROPERTY(EditAnywhere, Category = "Hands|Tips", meta = (ClampMin = "0.1"))
    float TipSphereRadiusCm = 1.8f;


     // create component + instances once
   // per-frame update
    void HideTipSpheres(EControllerHand Hand);

    void PanOrbitAroundCesiumCenter_Stable(const TArray<FVector>& RightHandPositions, bool bPinchActive, float DeltaSeconds);

    //void PanOrbitAroundCesiumCenter_Stable(TArray<FVector> RightHandPositions, bool bPinchActive, float DeltaSeconds);

    //void PanOrbitAroundCesiumCenter_Stable(const TArray<FVector>& RightHandPositions, bool bPinchActive, float DeltaSeconds);

   //void PanOrbitAroundCesiumCenter_Stable(TArray<FVector> RightHandPositions, bool bPinchActive, float DeltaSeconds);

    UCesiumGlobeAnchorComponent* globeAnchor;
   
protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
   
    FVector GetEarthCenterWorld();
    FVector ComputeOrbitPosition(float TrueAnomalyDeg, const FVector& EarthWorld);

    void AimCameraAt(const FVector& EarthWorld);
    float RadiusSpeedScale(double Radius) const;
    void UpdateOrbitFromPinchDistance(const TArray<FVector>& RightHandPositions, bool bPinchActive, float DeltaSeconds);


    UPROPERTY()
    FQuat OrbitalPlaneQuat;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit")
    float OrbitAngularSpeedDeg = 10.0f;
   // FVector ComputeOrbitPosition(float TrueAnomalyDeg, const FVector& EarthWorld) const;
    //FVector ComputeOrbitPosition(float TrueAnomalyDeg, const FVector& EarthWorld);
    //void UpdateOrbitFromPinchDistance(const TArray<FVector>& RightHandPositions, bool bPinchActive, float DeltaSeconds);
    UCapsuleComponent* body;
    UStaticMeshComponent* cubeMesh;
   
    bool GetHandKeypoints(
        EControllerHand Hand,
        TArray<FVector>& OutPositions,
        TArray<FQuat>& OutRotations,
        TArray<float>& OutRadii) const;
    

    bool IsGrabbing(const TArray<FVector>& Positions, float PinchThresholdCm = 4.0f) const;

    bool XRInfo();

    bool IsPinching(
        const TArray<FVector>& Positions,
        float PinchThresholdCm = 2.0f,
        float PinchReleaseThresholdCm=7.0f) ;
   bool IsPalmFacingCamera(const TArray<FQuat>& Rotations) const;
   //void OnTriggerEnter(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, UPrimitiveComponent* OtherComponent, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);
  // void OnTriggerExit(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, UPrimitiveComponent* OtherComponent, int32 OtherBodyIndex);
   void MoveActorWithSpeed(float speed, float DeltaTime);
  // void DebugDrawContainer(ACesiumGeoreference* Geo, double ContainerRadiusM, double EnterHysteresisM, double RearmHysteresisM, const FVector& EntryPointUU, const FVector& PawnPosUU);
   bool IsPointing(
       const TArray<FVector>& Positions,
       const TArray<FQuat>& Rotations,
       float TipThreshold=4.0f,    // how far index tip must be from palm
       float CurlThreshold=4.0f    // how close other fingers must be to palm
   ) const;
   
    void DebugDrawTips(const TArray<FVector>& Positions, const FColor& Color) const;
   // void LogXRInfo() const;
    bool bPrevLeftPinching = false;
    bool bPrevRightPinching = false;
  



    UPROPERTY()
    UPoseableMeshComponent* LeftHandMesh;

    UPROPERTY()
    UPoseableMeshComponent* RightHandMesh;

    UPROPERTY(EditAnywhere)
    UWidgetComponent* VRHUDWidget;
    float CubeSizeCm = 8.f; // cube side in cm

    void UpdateHandMeshPoseable(const TArray<FVector>& Positions, const TArray<FQuat>& Rotations,
        UPoseableMeshComponent* Poseable,
        bool bIsRightHand);

    void InitTipSpheresFor(EControllerHand Hand);

    void InitTipSpheresBoth();

    void UpdateTipSpheresFor(const TArray<FVector>& Positions, EControllerHand Hand);

    void UpdateTipSpheresBoth(const TArray<FVector>& LeftPositions, const TArray<FVector>& RightPositions);

  

    
   

    // HandTracking.h





   
    

    // Tuning (editable in BP)
    UPROPERTY(EditAnywhere, Category = "XR|Ray")
    float PosSmoothHz = 12.f;

    UPROPERTY(EditAnywhere, Category = "XR|Ray")
    float RotSmoothHz = 12.f;

    //UPROPERTY(EditAnywhere, Category = "XR|Ray")
    //float RayLength = 250.f;

    UPROPERTY(EditAnywhere, Category = "XR|Ray")
    float PalmUnderOffset = 2.0f;

    // New function (rotation-aware, smoothed)
    void UpdateRaycastFromAim(EControllerHand Hand, const FXRMotionControllerData& ControllerData, bool bIsPinching);

    //HANDMESH BONE DATA


    static float   ExpAlpha(float Hz, float Dt) { return (Hz <= 0.f) ? 1.f : 1.f - FMath::Exp(-Hz * Dt); }
    static FVector LerpExp(const FVector& A, const FVector& B, float Hz, float Dt) { return FMath::Lerp(A, B, ExpAlpha(Hz, Dt)); }
    static FQuat   SlerpExp(const FQuat& A, const FQuat& B, float Hz, float Dt) { return FQuat::Slerp(A, B, ExpAlpha(Hz, Dt)).GetNormalized(); }

    //Test FOR PINXH

  
    // --- Per-hand pinch ray state ---
    //struct FPinchRayState
    //{
    //    bool   bActive = false;          // true while that hand is pinching
    //    FVector OriginWS = FVector::ZeroVector;
    //    FVector DirWS = FVector::ForwardVector;
    //    TWeakObjectPtr<AActor> HitActor;
    //    FHitResult Hit;
    //};
    struct FPinchRayState
    {
        bool bActive = false;
        FVector OriginWS = FVector::ZeroVector;
        FVector DirWS = FVector::ForwardVector; // locked while active
        FVector LastPinchPos = FVector::ZeroVector; // NEW: to compute deltas
        FHitResult Hit;
        TWeakObjectPtr<AActor> HitActor;
    };


    FPinchRayState LeftPinch, RightPinch;

    // Two-hand scaling latch
    TWeakObjectPtr<USceneComponent> TwoHandTargetComp;
    FVector TwoHandBaselineScale = FVector::OneVector;
    float   TwoHandBaselineDist = 10.0f;

    // Tuning
    float RayLength = 10000000.f;
    float RaySmoothHz = 18.f;   // origin smoothing
    bool  bLockDirOnPinchStart = true; // lock direction until pinch ends

    // API
    void UpdatePinchRay(EControllerHand Hand,
        const TArray<FVector>& Positions,
        const TArray<FQuat>& Rotations,
        bool bIsPinching,
        float DeltaSeconds);







    // Small helper
    FVector GetPinchMid(const TArray<FVector>& Positions) const;



    //TRYING MOVEMENT APPROACH TO MIMIC ZOOM IN AND OUT 

    // Zoom feel
    float TwoHandZoomGainCmPerCm = 10000.0f;   // cm moved per cm of hand distance change
    float TwoHandZoomDeadzoneCm = 0.2f;   // ignore tiny jitter
    float TwoHandZoomMaxSpeedCmS = 10000.f;  // clamp speed
    float TwoHandZoomBaselineFollowHz = 3.f; // slowly re-center baseline
    bool  bInvertTwoHandZoom = false;  // set true to flip direction
    float TwoHandZoomSpeedExponent = 1.5f;
    float TwoHandZoomStartThresholdCm = 2.0f;
    float initialPinchDistance = 0.0f;
    bool bIsMoving = false;
    float currentSpeed = 0.f;
    float CalculateTwoHandSpeed(const TArray<FVector>& LeftP,
        const TArray<FVector>& RightP,
        bool bLeftPinch, bool bRightPinch,
        float DeltaSeconds);

    float height=0.f;

    // Add these to your header (recommended) or keep as statics inside the function.
    float ArcMinHeightFt = 2000.f;      // floor height
    float ArcMaxHeightFt = 50000000.f;     // apex height (your “certain height”)
    bool  bArcAscending = true;       // phase: going up+forward or down+back
    float ArcHorzSpeedScale = 1.0f;     // scale horizontal vs vertical
    float ArcVertSpeedScale = 1.0f;   // scale vertical vs horizontal
    // Zoom gesture state (lock on pinch start, clear on pinch end)
    bool     bZoomHasLockedRot = false;
    FRotator ZoomLockedRot;

    bool     bZoomHasBasis = false;
    FVector  ZoomBasisFwd;  // flattened forward captured once per gesture
    FVector  ZoomBasisUp;   // usually world up
    

protected:
    UPROPERTY(VisibleAnywhere)
    UCameraComponent* Camera;

};
