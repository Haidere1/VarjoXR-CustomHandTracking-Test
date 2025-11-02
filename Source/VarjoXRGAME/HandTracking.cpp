
// Fill out your copyright notice in the Description page of Project Se
#include "HandTracking.h"
#include "IXRTrackingSystem.h"
#include "IHandTracker.h"
#include "HeadMountedDisplayTypes.h"
#include "Components/CapsuleComponent.h"
#include "IHeadMountedDisplay.h"
#include "Engine/StaticMeshActor.h"
#include "Kismet/GameplayStatics.h"
#include "Camera/CameraComponent.h"
#include "HeadMountedDisplayFunctionLibrary.h"
#include <Kismet/KismetMathLibrary.h>

#include "CesiumGeoreference.h"
#include <CesiumGeospatial/Ellipsoid.h>
#include <glm/vec3.hpp>
#define ORBIT_DBG 1

static const TMap<EHandKeypoint, FString> HandBoneMap = {

	{EHandKeypoint::Wrist, "hand"},
	{EHandKeypoint::Palm, "palm"},
	{EHandKeypoint::ThumbMetacarpal, "thumb_01"},
	{EHandKeypoint::ThumbProximal,   "thumb_02"},
	{EHandKeypoint::ThumbDistal,     "thumb_03"},
	{EHandKeypoint::ThumbTip,     "joint5"},
	{EHandKeypoint::IndexMetacarpal, "index_metacarpal"},
	{EHandKeypoint::IndexProximal, "index_01"},
	{EHandKeypoint::IndexIntermediate,   "index_02"},
	{EHandKeypoint::IndexDistal, "index_03"},
	{EHandKeypoint::IndexTip,     "joint1"}, // optional
	{EHandKeypoint::MiddleMetacarpal,  "middle_metacarpal"},
	{EHandKeypoint::MiddleProximal,  "middle_01"},
	{EHandKeypoint::MiddleIntermediate,         "middle_02"},
	{EHandKeypoint::MiddleDistal,         "middle_03"},
	{EHandKeypoint::MiddleTip,         "joint2"},
	{EHandKeypoint::RingMetacarpal,  "ring_metacarpal"},
	{EHandKeypoint::RingProximal,  "ring_01"},
	{EHandKeypoint::RingIntermediate,         "ring_02"},
	{EHandKeypoint::RingDistal,         "ring_03"},
	{EHandKeypoint::RingTip,         "joint3"},
	{EHandKeypoint::LittleMetacarpal,  "pinky_metacarpal"},
	{EHandKeypoint::LittleProximal,  "pinky_01"},
	{EHandKeypoint::LittleIntermediate,         "pinky_02"},
	{EHandKeypoint::LittleDistal,         "pinky_03"},
	{EHandKeypoint::LittleTip,         "joint4"},
};
AHandTracking::AHandTracking()
{
	PrimaryActorTick.bCanEverTick = true;
	body = CreateDefaultSubobject<UCapsuleComponent>(TEXT("Body"));
	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	globeAnchor = CreateDefaultSubobject<UCesiumGlobeAnchorComponent>(TEXT("globeAnchor"));
	SetRootComponent(body);// <-- fix
	Camera->SetupAttachment(body);
	Camera->bLockToHmd = true;

	LeftHandController = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("LeftController"));
	LeftHandController->SetupAttachment(body);
	LeftHandController->SetTrackingSource(EControllerHand::Left);


	RightHandController = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("RightController"));
	RightHandController->SetupAttachment(body);
	RightHandController->SetTrackingSource(EControllerHand::Right);


	VRHUDWidget = CreateDefaultSubobject<UWidgetComponent>(TEXT("VRHUDWidget"));
	VRHUDWidget->SetupAttachment(body);
	LeftHandMesh = CreateDefaultSubobject<UPoseableMeshComponent>(TEXT("LeftHand"));    RightHandMesh = CreateDefaultSubobject<UPoseableMeshComponent>(TEXT("RightHand"));
	VRHUDWidget->SetWidgetSpace(EWidgetSpace::World);
	VRHUDWidget->SetDrawSize(FVector2D(50.0f, 50.0f));
	VRHUDWidget->SetRelativeLocation(FVector(200.0f, 0.0f, 0.0f));
	VRHUDWidget->SetRelativeRotation(FRotator(180.f, 90.f, 180.f));
	LeftHandMesh->SetupAttachment(LeftHandController);
	//RightHandMesh->SetupAttachment(RightHandController);
	//RightHandMesh->SetRelativeRotation(FRotator(0.0f, 0, 0.0f));


	TipSpheresL = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("TipSpheres_Left"));
	TipSpheresR = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("TipSpheres_Right"));

	TipSpheresL->SetupAttachment(GetRootComponent());
	TipSpheresR->SetupAttachment(GetRootComponent());

	// Load engine sphere once (optional; or assign in BP)
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMesh(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	if (SphereMesh.Succeeded())
	{
		TipSpheresL->SetStaticMesh(SphereMesh.Object);
		TipSpheresR->SetStaticMesh(SphereMesh.Object);
	}
	TipSpheresL->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	TipSpheresR->SetCollisionEnabled(ECollisionEnabled::NoCollision);


	static ConstructorHelpers::FClassFinder<UUserWidget> WidgetBP(TEXT("/Game/VRMenu/VRMenu.VRMenu_C")); // Replace path
	if (WidgetBP.Succeeded())
	{
		VRHUDWidget->SetWidgetClass(WidgetBP.Class);
	}
	///Script/Engine.SkeletalMesh'/Game/Characters/MannequinsXR/Meshes/SKM_QuinnXR_right.SKM_QuinnXR_right'
	//static ConstructorHelpers::FObjectFinder<USkeletalMesh> RightHand(TEXT("/Game/HandMesh/Mustansar_Zeeshan/MetaverseRightHandRig.MetaverseRightHandRig"));
	/*static ConstructorHelpers::FObjectFinder<USkeletalMesh> RightHand(TEXT("/Game/Characters/MannequinsXR/Meshes/SKM_QuinnXR_right.SKM_QuinnXR_right"));
	if (RightHand.Succeeded())
	{
		RightHandMesh->SetSkeletalMesh(RightHand.Object);
	}*/

	static ConstructorHelpers::FObjectFinder<USkeletalMesh> RightHand(TEXT("/Game/HandMesh/Hand.Hand"));
	if (RightHand.Succeeded())
	{
		RightHandMesh->SetSkeletalMesh(RightHand.Object);
	}
	



}


void AHandTracking::BeginPlay()
{
	Super::BeginPlay();
	// UHeadMountedDisplayFunctionLibrary::SetTrackingOrigin(EHMDTrackingOrigin::Stage);
	// SetActorLocationAndRotation(FVector((0.000000, 1434269970.000000, -319826060.000000)), FRotator(0.0f, 0.0f, 0.0f));
	CurrentTrueAnomalyDeg = InitialTrueAnomalyDeg;


	UHeadMountedDisplayFunctionLibrary::SetTrackingOrigin(EHMDTrackingOrigin::LocalFloor);
	UHeadMountedDisplayFunctionLibrary::ResetOrientationAndPosition(0.f, EOrientPositionSelector::OrientationAndPosition);
	Camera->SetWorldRotation(FRotator::ZeroRotator);
	InitTipSpheresBoth();
	//RightHandMesh->SetRelativeRotation(FRotator(0.f, 0.f, 0.f));

	//LogXRInfo();
}







void AHandTracking::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);


	TArray<FVector> LP, RP;  TArray<FQuat> LR, RR;  TArray<float> LRd, RRd;
	const bool bL = GetHandKeypoints(EControllerHand::Left, LP, LR, LRd);
	const bool bR = GetHandKeypoints(EControllerHand::Right, RP, RR, RRd);

	bool bLPinch = false, bRPinch = false;

	if (bL) {
		bLPinch = IsPinching(LP);
		//DebugDrawTips(LP,  FColor::Green);
		UpdatePinchRay(EControllerHand::Left, LP, LR, bLPinch, DeltaSeconds);
		TipSpheresL->SetVisibility(true);
		UpdateTipSpheresFor(LP, EControllerHand::Left);

	}
	else {
		LeftPinch.bActive = false;
		LeftPinch.HitActor = nullptr;
		TipSpheresL->SetVisibility(false);

	}


	if (bR) {
		bRPinch = IsPinching(RP);
		// DebugDrawTips(RP, FColor::Green);

		UpdatePinchRay(EControllerHand::Right, RP, RR, bRPinch, DeltaSeconds);
		//TipSpheresR->SetVisibility(true);
		//UpdateTipSpheresFor(RP, EControllerHand::Right);

	}
	else {
		bRPinch = false;
		RightPinch.bActive = false;
		RightPinch.HitActor = nullptr;
		TipSpheresR->SetVisibility(false);
	}
	if (bR && bRPinch&&!bLPinch)
	{
		PanOrbitAroundCesiumCenter_Stable(RP, true, DeltaSeconds);
	}
	else
	{
		PanOrbitAroundCesiumCenter_Stable(RP, /*bPinchActive*/ false, DeltaSeconds); // resets state
	}

	// Two-hand scaling if both rays hit same actor while both pinching

	float speed = CalculateTwoHandSpeed(LP, RP, bLPinch, bRPinch, DeltaSeconds);

	if (speed != 0.f)
	{
		// Use the speed to move the actor
		MoveActorWithSpeed(speed, DeltaSeconds);
	}


	height = globeAnchor->GetHeight();

	TArray<FVector> P; TArray<FQuat> R; TArray<float> Rad;
	if (GetHandKeypoints(EControllerHand::Right, P, R, Rad))
	{
		UpdateHandMeshPoseable(P, R, RightHandMesh, true);
	}


}






//TESTING FOR ROTATION::

void AHandTracking::RefreshEarthCenterIfNeeded()
{
	const double Now = FApp::GetCurrentTime();
	if (LastEarthCenterRefreshTime < 0.0 || (Now - LastEarthCenterRefreshTime) >= EarthCenterRefreshInterval)
	{
		if (!CachedGeo)
			CachedGeo = ACesiumGeoreference::GetDefaultGeoreference(this);

		if (CachedGeo)
		{
			CachedEarthCenterUU = CachedGeo->TransformEarthCenteredEarthFixedPositionToUnreal(FVector::ZeroVector);
			LastEarthCenterRefreshTime = Now;
		}
	}
}

// ---------- Geometry/geo helpers (no lambdas, no allocs) ----------
FVector AHandTracking::UnrealFromLLH(double LonDeg, double LatDeg, double HeightMeters) const
{
	if (!CachedGeo) return GetActorLocation();

	const CesiumGeospatial::Cartographic carto(
		FMath::DegreesToRadians(LonDeg),
		FMath::DegreesToRadians(LatDeg),
		HeightMeters);

	const glm::dvec3 ecef = CesiumGeospatial::Ellipsoid::WGS84.cartographicToCartesian(carto);
	return CachedGeo->TransformEarthCenteredEarthFixedPositionToUnreal({ (double)ecef.x, (double)ecef.y, (double)ecef.z });
}

double AHandTracking::GetHeightM() const
{
	if (!CachedGeo) return 0.0;
	const FVector ecefF = CachedGeo->TransformUnrealPositionToEarthCenteredEarthFixed(GetActorLocation());
	const glm::dvec3 ecef(ecefF.X, ecefF.Y, ecefF.Z);
	const auto carto = CesiumGeospatial::Ellipsoid::WGS84.cartesianToCartographic(ecef);
	return carto ? carto->height : 0.0;
}

double AHandTracking::GetHeightMAtPos(const FVector& Pos) const
{
	if (!CachedGeo) return 0.0;
	const FVector ecefF = CachedGeo->TransformUnrealPositionToEarthCenteredEarthFixed(Pos);
	const glm::dvec3 ecef(ecefF.X, ecefF.Y, ecefF.Z);
	const auto carto = CesiumGeospatial::Ellipsoid::WGS84.cartesianToCartographic(ecef);
	return carto ? carto->height : 0.0;
}

FRotator AHandTracking::MakeUprightRotAtPos(const FVector& Pos, const FVector& PrefFwd) const
{
	// Use cached Earth center to avoid an extra transform each call
	const FVector Earth = CachedEarthCenterUU;
	const FVector up = (Pos - Earth).GetSafeNormal(); // geodetic up (radial)

		FVector fwd = FVector::VectorPlaneProject(PrefFwd, up).GetSafeNormal(); // tangent forward
		if (fwd.IsNearlyZero())
		{
			FVector north = FVector::VectorPlaneProject(FVector::UpVector, up).GetSafeNormal();
			if (north.IsNearlyZero()) north = FVector::ForwardVector;
			fwd = FVector::CrossProduct(up, north).GetSafeNormal();
		}
		return FRotationMatrix::MakeFromXZ(fwd, up).Rotator();
}

FRotator AHandTracking::MakeLookAtEarthRotAtPos(const FVector& Pos) const
{
	const FVector Earth = CachedEarthCenterUU;
	const FVector up = (Pos - Earth).GetSafeNormal();          // geodetic up
	FVector fwd = (Earth - Pos).GetSafeNormal();                // look at globe
	fwd = FVector::VectorPlaneProject(fwd, up).GetSafeNormal(); // stabilize roll
	return FRotationMatrix::MakeFromXZ(fwd, up).Rotator();
}

// ---------- Main behavior ----------
void AHandTracking::MoveActorWithSpeed(float Speed, float DeltaTime)
{
	// Keep Cesium cached and Earth center throttled
	if (!CachedGeo)
		CachedGeo = ACesiumGeoreference::GetDefaultGeoreference(this);
	if (!CachedGeo) return;

	RefreshEarthCenterIfNeeded();

	// ---------------- Init ----------------
	if (!bInit)
	{
		bInit = true;
		PreEntryDirUnit = GetActorForwardVector().GetSafeNormal();
		if (PreEntryDirUnit.IsNearlyZero()) PreEntryDirUnit = FVector::ForwardVector;

		PrevHeightM = GetHeightM();
		bEntryArmedByAlt = (PrevHeightM <= EntryHeightM - ExitDownHysteresisM);
		State = EState::Normal;
	}

	const bool bZoomIn = (Speed >= 0.f);
	const bool bZoomOut = !bZoomIn;

	// ---------------- Tween tick ----------------
	if (Tween.bActive)
	{
		if (USceneComponent* Root = GetRootComponent())
		{
			const bool bDone = Tween.Tick(DeltaTime, Root);
			if (!bDone) { PrevHeightM = GetHeightM(); return; }
		}
	}

	const double HeightM = GetHeightM();

	// ---------------- ENTRY ----------------
	if (State == EState::Normal)
	{
		if (!bEntryArmedByAlt && HeightM <= EntryHeightM - ExitDownHysteresisM)
			bEntryArmedByAlt = true;

		const bool bCrossedUp =
			bEntryArmedByAlt &&
			bZoomIn &&
			PrevHeightM < (EntryHeightM + EntryUpHysteresisM) &&
			HeightM <= (EntryHeightM + EntryUpHysteresisM);

		if (bCrossedUp)
		{
			// remember where & how we entered (for returning later)
			EnterPosition = GetActorLocation();
			enterRotaion = GetActorRotation();

			const FVector TargetPos = UnrealFromLLH(OriginLonDeg, OriginLatDeg, OriginHeightM);

			// End rotation: tangent/upright at the target
			const FVector prefFwd = (Camera ? Camera->GetForwardVector() : GetActorForwardVector());
			const FRotator UprightEndRot = MakeUprightRotAtPos(TargetPos, prefFwd);
			const FQuat StartRot = GetActorQuat();
			const FQuat EndRot = UprightEndRot.Quaternion();

			if (USceneComponent* Root = GetRootComponent())
			{
				Tween.Begin(GetActorLocation(), StartRot, TargetPos, EndRot, EntryLerpSec);
				bTweeningExit = false;   // this is the entry tween
				State = EState::Tweener;
			}
			else
			{
				SetActorLocation(TargetPos);
				SetActorRotation(UprightEndRot); // snap upright if not tweening
				State = EState::AtOrigin;
			}

			bEntryArmedByAlt = false;
			PrevHeightM = HeightM;
			return;
		}
	}

	// ---------------- tween end ----------------
	if (State == EState::Tweener && !Tween.bActive)
	{
		if (bTweeningExit)
		{
			// exit tween finished -> back to Normal flight
			bTweeningExit = false;

			// ensure movement resumes the same way we entered
			FVector StraightDir = enterRotaion.Vector().GetSafeNormal();
			if (StraightDir.IsNearlyZero()) StraightDir = GetActorForwardVector().GetSafeNormal();
			PreEntryDirUnit = StraightDir;

			State = EState::Normal;
			bEntryArmedByAlt = true; // allow future re-entry
			PrevHeightM = GetHeightM();
		}
		else
		{
			// entry tween finished -> now at origin
			State = EState::AtOrigin;
		}
	}

	// ---------------- EXIT (tween back to entry pose) ----------------
	if (State == EState::AtOrigin && bZoomOut)
	{
		const bool bCrossedDown =
			PrevHeightM > (EntryHeightM - ExitDownHysteresisM) &&
			HeightM >= (EntryHeightM - ExitDownHysteresisM);

		if (bCrossedDown)
		{
			const FQuat StartRot = GetActorQuat();
			const FQuat EndRot = enterRotaion.Quaternion();

			if (USceneComponent* Root = GetRootComponent())
			{
				Tween.Begin(GetActorLocation(), StartRot, EnterPosition, EndRot, EntryLerpSec);
				bTweeningExit = true;
				State = EState::Tweener;
			}
			else
			{
				// Fallback: snap if no tween
				SetActorLocation(EnterPosition);
				SetActorRotation(enterRotaion);

				FVector StraightDir = enterRotaion.Vector().GetSafeNormal();
				if (StraightDir.IsNearlyZero()) StraightDir = GetActorForwardVector().GetSafeNormal();
				PreEntryDirUnit = StraightDir;

				State = EState::Normal;
				bEntryArmedByAlt = true;
				PrevHeightM = GetHeightM();
			}
		}
	}

	// ---------------- Movement (+ zoom-out height cap) ----------------
	FVector MoveDir =
		(State == EState::AtOrigin)
		? (Camera ? Camera->GetForwardVector() : GetActorForwardVector())
		: PreEntryDirUnit;

	MoveDir = MoveDir.GetSafeNormal();
	FVector MoveDelta = MoveDir * Speed * DeltaTime;

	// Cap outward travel to a maximum geodetic height
	const double MaxOutHeightM = 8.0e7; // 80,000 km
	if (bZoomOut && !MoveDelta.IsNearlyZero())
	{
		const FVector CurrPos = GetActorLocation();
		const double  currH = HeightM; // already computed
		const double  nextH = GetHeightMAtPos(CurrPos + MoveDelta);

		if (nextH > MaxOutHeightM && currH < MaxOutHeightM)
		{
			// Binary search scale 't' in [0,1] so height(CurrPos + t*MoveDelta) ~= MaxOutHeightM
			double lo = 0.0, hi = 1.0;
			for (int i = 0; i < 12; ++i)
			{
				const double mid = 0.5 * (lo + hi);
				const double hMid = GetHeightMAtPos(CurrPos + MoveDelta * mid);
				(hMid > MaxOutHeightM) ? hi = mid : lo = mid;
			}
			MoveDelta *= lo; // stop right at the cap
		}
		else if (currH >= MaxOutHeightM)
		{
			// Already at/above cap: block additional outward movement
			const FVector Earth = CachedEarthCenterUU;
			if (FVector::DotProduct(MoveDir, (CurrPos - Earth).GetSafeNormal()) > 0.0)
			{
				MoveDelta = FVector::ZeroVector;
			}
		}
	}

	if (USceneComponent* Root = GetRootComponent())
	{
		Root->MoveComponent(MoveDelta, Root->GetComponentQuat(), /*bSweep*/ true);
	}
	else
	{
		AddActorLocalOffset(MoveDelta, /*bSweep*/ true);
	}

	PrevHeightM = GetHeightM();
}









//void AHandTracking::MoveActorWithSpeed(float speed, float DeltaTime)
//{
//	ACesiumGeoreference* Geo = ACesiumGeoreference::GetDefaultGeoreference(this);
//
//	auto UnrealFromLLH = [&](double lonDeg, double latDeg, double hM) -> FVector {
//		if (!Geo) return GetActorLocation();
//		const CesiumGeospatial::Cartographic carto(
//			FMath::DegreesToRadians(lonDeg),
//			FMath::DegreesToRadians(latDeg),
//			hM);
//		const glm::dvec3 ecef = CesiumGeospatial::Ellipsoid::WGS84.cartographicToCartesian(carto);
//		return Geo->TransformEarthCenteredEarthFixedPositionToUnreal({ (double)ecef.x,(double)ecef.y,(double)ecef.z });
//		};
//
//	auto GetHeightM = [&]() -> double {
//		if (!Geo) return 0.0;
//		const FVector ecefF = Geo->TransformUnrealPositionToEarthCenteredEarthFixed(GetActorLocation());
//		const glm::dvec3 ecef(ecefF.X, ecefF.Y, ecefF.Z);
//		const auto carto = CesiumGeospatial::Ellipsoid::WGS84.cartesianToCartographic(ecef);
//		return carto ? carto->height : 0.0;
//		};
//
//	// --- height at arbitrary Unreal position ---
//	auto GetHeightMAtPos = [&](const FVector& Pos) -> double {
//		if (!Geo) return 0.0;
//		const FVector ecefF = Geo->TransformUnrealPositionToEarthCenteredEarthFixed(Pos);
//		const glm::dvec3 ecef(ecefF.X, ecefF.Y, ecefF.Z);
//		const auto carto = CesiumGeospatial::Ellipsoid::WGS84.cartesianToCartographic(ecef);
//		return carto ? carto->height : 0.0;
//		};
//
//	auto MakeUprightRotAtPos = [&](const FVector& Pos, const FVector& PrefFwd) -> FRotator
//		{
//			const FVector Earth = Geo->TransformEarthCenteredEarthFixedPositionToUnreal(FVector::ZeroVector);
//			const FVector up = (Pos - Earth).GetSafeNormal(); // geodetic up (radial)
//			FVector fwd = FVector::VectorPlaneProject(PrefFwd, up).GetSafeNormal(); // tangent forward
//			if (fwd.IsNearlyZero())
//			{
//				// fallback "east": up x north, where north is world-up projected
//				FVector north = FVector::VectorPlaneProject(FVector::UpVector, up).GetSafeNormal();
//				if (north.IsNearlyZero()) north = FVector::ForwardVector;
//				fwd = FVector::CrossProduct(up, north).GetSafeNormal();
//			}
//			return FRotationMatrix::MakeFromXZ(fwd, up).Rotator();
//		};
//
//	auto MakeLookAtEarthRotAtPos = [&](const FVector& Pos) -> FRotator
//		{
//			const FVector Earth = Geo->TransformEarthCenteredEarthFixedPositionToUnreal(FVector::ZeroVector);
//			const FVector up = (Pos - Earth).GetSafeNormal();           // geodetic up
//			FVector fwd = (Earth - Pos).GetSafeNormal();                 // look at globe
//			fwd = FVector::VectorPlaneProject(fwd, up).GetSafeNormal();  // stabilize roll
//			return FRotationMatrix::MakeFromXZ(fwd, up).Rotator();
//		};
//
//	// ---------------- Init ----------------
//	if (!bInit)
//	{
//		bInit = true;
//		PreEntryDirUnit = GetActorForwardVector().GetSafeNormal();
//		if (PreEntryDirUnit.IsNearlyZero()) PreEntryDirUnit = FVector::ForwardVector;
//
//		PrevHeightM = GetHeightM();
//		bEntryArmedByAlt = (PrevHeightM <= EntryHeightM - ExitDownHysteresisM);
//		State = EState::Normal;
//	}
//
//	const bool bZoomIn = (speed >= 0.f);
//	const bool bZoomOut = !bZoomIn;
//
//	// ---------------- Tween tick ----------------
//	if (Tween.bActive)
//	{
//		if (USceneComponent* Root = GetRootComponent())
//		{
//			const bool bDone = Tween.Tick(DeltaTime, Root);
//			if (!bDone) { PrevHeightM = GetHeightM(); return; }
//		}
//	}
//
//	const double HeightM = GetHeightM();
//
//	// ---------------- ENTRY ----------------
//	if (State == EState::Normal && Geo)
//	{
//		if (!bEntryArmedByAlt && HeightM <= EntryHeightM - ExitDownHysteresisM)
//			bEntryArmedByAlt = true;
//
//		const bool bCrossedUp =
//			bEntryArmedByAlt &&
//			bZoomIn &&
//			PrevHeightM < (EntryHeightM + EntryUpHysteresisM) &&
//			HeightM <= (EntryHeightM + EntryUpHysteresisM);
//
//		if (bCrossedUp)
//		{
//			// remember where & how we entered (for returning later)
//			EnterPosition = GetActorLocation();
//			enterRotaion = GetActorRotation();
//
//			const FVector TargetPos = UnrealFromLLH(OriginLonDeg, OriginLatDeg, OriginHeightM);
//
//			// End rotation: tangent/upright at the target
//			const FVector prefFwd = (Camera ? Camera->GetForwardVector() : GetActorForwardVector());
//			const FRotator UprightEndRot = MakeUprightRotAtPos(TargetPos, prefFwd);
//			const FQuat StartRot = GetActorQuat();
//			const FQuat EndRot = UprightEndRot.Quaternion();
//
//			if (USceneComponent* Root = GetRootComponent())
//			{
//				Tween.Begin(GetActorLocation(), StartRot, TargetPos, EndRot, EntryLerpSec);
//				bTweeningExit = false;   // this is the entry tween
//				State = EState::Tweener;
//			}
//			else
//			{
//				SetActorLocation(TargetPos);
//				SetActorRotation(UprightEndRot); // snap upright if not tweening
//				State = EState::AtOrigin;
//			}
//
//			bEntryArmedByAlt = false;
//			PrevHeightM = HeightM;
//			return;
//		}
//	}
//
//	// ---------------- tween end ----------------
//	if (State == EState::Tweener && !Tween.bActive)
//	{
//		if (bTweeningExit)
//		{
//			// exit tween finished -> back to Normal flight
//			bTweeningExit = false;
//
//			// ensure movement resumes the same way we entered
//			FVector StraightDir = enterRotaion.Vector().GetSafeNormal();
//			if (StraightDir.IsNearlyZero()) StraightDir = GetActorForwardVector().GetSafeNormal();
//			PreEntryDirUnit = StraightDir;
//
//			State = EState::Normal;
//			bEntryArmedByAlt = true; // allow future re-entry
//			PrevHeightM = GetHeightM();
//		}
//		else
//		{
//			// entry tween finished -> now at origin
//			State = EState::AtOrigin;
//		}
//	}
//
//	// ---------------- EXIT (tween back to entry pose) ----------------
//	if (State == EState::AtOrigin && bZoomOut)
//	{
//		const bool bCrossedDown =
//			PrevHeightM > (EntryHeightM - ExitDownHysteresisM) &&
//			HeightM >= (EntryHeightM - ExitDownHysteresisM);
//
//		if (bCrossedDown)
//		{
//			const FQuat StartRot = GetActorQuat();
//			const FQuat EndRot = enterRotaion.Quaternion();
//
//			if (USceneComponent* Root = GetRootComponent())
//			{
//				Tween.Begin(GetActorLocation(), StartRot, EnterPosition, EndRot, EntryLerpSec);
//				bTweeningExit = true;
//				State = EState::Tweener;
//			}
//			else
//			{
//				// Fallback: snap if no tween
//				SetActorLocation(EnterPosition);
//				SetActorRotation(enterRotaion);
//
//				FVector StraightDir = enterRotaion.Vector().GetSafeNormal();
//				if (StraightDir.IsNearlyZero()) StraightDir = GetActorForwardVector().GetSafeNormal();
//				PreEntryDirUnit = StraightDir;
//
//				State = EState::Normal;
//				bEntryArmedByAlt = true;
//				PrevHeightM = GetHeightM();
//			}
//		}
//	}
//
//	// ---------------- Movement (+ zoom-out height cap) ----------------
//	FVector MoveDir =
//		(State == EState::AtOrigin)
//		? (Camera ? Camera->GetForwardVector() : GetActorForwardVector())
//		: PreEntryDirUnit;
//
//	MoveDir = MoveDir.GetSafeNormal();
//
//	FVector MoveDelta = MoveDir * speed * DeltaTime;
//
//	// Cap outward travel to a maximum geodetic height
//	const double MaxOutHeightM = 8.0e7; // 80,000 km
//	if (bZoomOut && Geo && !MoveDelta.IsNearlyZero())
//	{
//		const FVector CurrPos = GetActorLocation();
//		const double  currH = HeightM; // already computed
//		const double  nextH = GetHeightMAtPos(CurrPos + MoveDelta);
//
//		if (nextH > MaxOutHeightM && currH < MaxOutHeightM)
//		{
//			// Find a scale 't' in [0,1] so height(CurrPos + t*MoveDelta) ~= MaxOutHeightM
//			double lo = 0.0, hi = 1.0;
//			for (int i = 0; i < 12; ++i)
//			{
//				const double mid = 0.5 * (lo + hi);
//				const double hMid = GetHeightMAtPos(CurrPos + MoveDelta * mid);
//				if (hMid > MaxOutHeightM) hi = mid; else lo = mid;
//			}
//			MoveDelta *= lo; // stop right at the cap
//		}
//		else if (currH >= MaxOutHeightM)
//		{
//			// Already at/above cap: block additional outward movement
//			const FVector Earth = Geo->TransformEarthCenteredEarthFixedPositionToUnreal(FVector::ZeroVector);
//			if (FVector::DotProduct(MoveDir, (CurrPos - Earth).GetSafeNormal()) > 0.0)
//			{
//				MoveDelta = FVector::ZeroVector;
//			}
//		}
//	}
//
//	if (USceneComponent* Root = GetRootComponent())
//		Root->MoveComponent(MoveDelta, Root->GetComponentQuat(), /*bSweep*/ true);
//	else
//		AddActorLocalOffset(MoveDelta, /*bSweep*/ true);
//
//	PrevHeightM = GetHeightM();
//}


bool AHandTracking::IsPointing(
	const TArray<FVector>& Positions,
	const TArray<FQuat>& Rotations,
	float TipThreshold,    // how far index tip must be from palm
	float CurlThreshold   // how close other fingers must be to palm
) const
{
	int32 PalmIndex = (int32)EHandKeypoint::Palm;
	int32 IndexTip = (int32)EHandKeypoint::IndexTip;
	int32 MiddleTip = (int32)EHandKeypoint::MiddleTip;
	int32 RingTip = (int32)EHandKeypoint::RingTip;
	int32 LittleTip = (int32)EHandKeypoint::LittleTip;

	if (!Positions.IsValidIndex(PalmIndex) ||
		!Positions.IsValidIndex(IndexTip) ||
		!Positions.IsValidIndex(MiddleTip) ||
		!Positions.IsValidIndex(RingTip) ||
		!Positions.IsValidIndex(LittleTip))
	{
		return false;
	}

	// --- Step 1: Finger shape (index extended, others curled) ---
	float IndexDist = FVector::Dist(Positions[IndexTip], Positions[PalmIndex]);
	float MiddleDist = FVector::Dist(Positions[MiddleTip], Positions[PalmIndex]);
	float RingDist = FVector::Dist(Positions[RingTip], Positions[PalmIndex]);
	float LittleDist = FVector::Dist(Positions[LittleTip], Positions[PalmIndex]);

	bool bIndexExtended = (IndexDist > TipThreshold);
	bool bOthersCurled = (MiddleDist < CurlThreshold &&
		RingDist < CurlThreshold &&
		LittleDist < CurlThreshold);

	if (!(bIndexExtended && bOthersCurled))
	{

		return false;
	}

	// --- Step 2: Direction check (index pointing away from camera) ---
	FVector PointingDir = (Positions[IndexTip] - Positions[PalmIndex]).GetSafeNormal();

	APlayerCameraManager* CamManager = UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0);
	if (!CamManager) return false;

	FVector CameraForward = CamManager->GetActorForwardVector();


	// Dot product < -0.7 → pointing opposite camera direction
	float Dot = FVector::DotProduct(PointingDir, CameraForward);

	return (Dot > 0.7f); // ~ >135° angle away from camera
}





bool AHandTracking::IsPalmFacingCamera(const TArray<FQuat>& Rotations) const
{
	int32 PalmIndex = (int32)EHandKeypoint::Palm;
	if (!Rotations.IsValidIndex(PalmIndex))
		return false;

	// Palm orientation
	FQuat PalmRot = Rotations[PalmIndex];

	// Choose correct axis for palm normal (depends on your skeleton)
	// Try GetForwardVector() or GetUpVector() and test in engine
	FVector PalmNormal = PalmRot.GetUpVector();

	// Get camera forward vector
	APlayerCameraManager* CamManager = UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0);
	if (!CamManager) return false;

	FVector CameraForward = CamManager->GetActorForwardVector();

	// Dot product: >0.7 = facing camera, <-0.7 = facing away
	float Dot = FVector::DotProduct(PalmNormal, CameraForward);

	return (Dot > 0.7f); // return true only if palm is facing the camera
}

static IHandTracker* GetHandTracker()
{
	const FName Feature = IHandTracker::GetModularFeatureName();
	if (!IModularFeatures::Get().IsModularFeatureAvailable(Feature))
		return nullptr;

	const int32 Count = IModularFeatures::Get().GetModularFeatureImplementationCount(Feature);
	return (Count > 0)
		? static_cast<IHandTracker*>(IModularFeatures::Get().GetModularFeatureImplementation(Feature, 0))
		: nullptr;
}

bool AHandTracking::GetHandKeypoints(EControllerHand Hand, TArray<FVector>& OutPositions, TArray<FQuat>& OutRotations, TArray<float>& OutRadii) const
{
	OutPositions.Reset();
	OutRotations.Reset();
	OutRadii.Reset();

	if (!(GEngine && GEngine->XRSystem.IsValid()))
		return false;

	IHandTracker* HT = GetHandTracker();
	if (!HT || !HT->IsHandTrackingStateValid())
		return false;

	// ControllerIndex 0 = first player
	const bool bOK = HT->GetAllKeypointStates(Hand, OutPositions, OutRotations, OutRadii);
	return bOK && OutPositions.Num() > 0 && OutRotations.Num() == OutPositions.Num();
}

bool AHandTracking::IsPinching(
	const TArray<FVector>& Positions,
	float PinchStartThresholdCm, // Threshold for starting pinch
	float PinchReleaseThresholdCm // Threshold for releasing pinch
)
{
	const int32 ThumbTip = (int32)EHandKeypoint::ThumbTip;
	const int32 IndexTip = (int32)EHandKeypoint::IndexTip;

	if (!Positions.IsValidIndex(ThumbTip) || !Positions.IsValidIndex(IndexTip))
		return false;

	// Calculate the distance between ThumbTip and IndexTip
	const float DistCm = FVector::Dist(Positions[ThumbTip], Positions[IndexTip]);

	// Pinch start: if within the threshold, start pinch
	if (DistCm <= PinchStartThresholdCm)
	{
		bPrevPinching = true; // Pinch starts, set state to active
		return true;
	}

	// Pinch release: if distance goes above the release threshold and the pinch was active
	if (DistCm > PinchReleaseThresholdCm && bPrevPinching)
	{
		bPrevPinching = false; // Release pinch
		return false;
	}

	// Return the current pinch state
	return bPrevPinching;
}

void AHandTracking::DebugDrawTips(const TArray<FVector>& Positions, const FColor& Color) const
{
	UWorld* W = GetWorld();
	for (int32 Keypoint = (int32)EHandKeypoint::Palm;
		Keypoint <= (int32)EHandKeypoint::LittleTip;
		++Keypoint)
	{
		if (W && Positions.IsValidIndex(Keypoint))
		{
			DrawDebugSphere(
				W,
				Positions[Keypoint],
				0.3f,        // radius
				5,           // segments
				Color,       // sphere color
				false,       // persistent lines?
				0.f,         // lifetime
				0,           // depth priority
				0.6f         // line thickness
			);
		}
	}
}
bool AHandTracking::IsGrabbing(const TArray<FVector>& Positions, float GrabThreshold) const
{
	bool bIsGrabbing = true; // assume grabbing until proven false

	const int32 Palm = (int32)EHandKeypoint::Palm;
	const int32 IndexTip = (int32)EHandKeypoint::IndexTip;
	const int32 MiddleTip = (int32)EHandKeypoint::MiddleTip;
	const int32 RingTip = (int32)EHandKeypoint::RingTip;
	const int32 LittleTip = (int32)EHandKeypoint::LittleTip;



	TArray<int32> FingerTips = { IndexTip, MiddleTip, RingTip, LittleTip };

	for (int32 Tip : FingerTips)
	{
		if (!Positions.IsValidIndex(Tip) || !Positions.IsValidIndex(Palm))
			return false;

		float Dist = FVector::Dist(Positions[Tip], Positions[Palm]);
		if (Dist > GrabThreshold) // if any finger too far = not grabbing
		{
			bIsGrabbing = false;
			break;
		}
	}

	return bIsGrabbing;
}

void AHandTracking::UpdateRaycastFromAim(EControllerHand Hand, const FXRMotionControllerData& ControllerData, bool bIsPinching)
{
	// Get the Palm Position from the motion controller data
	const FVector PalmPos = ControllerData.PalmPosition;

	// Get the Aim Position (direction the hand is pointing)
	const FVector AimDir = ControllerData.AimPosition - PalmPos; // Vector from palm to aim position

	// If AimDirection is zero (not valid), exit early
	if (AimDir.IsNearlyZero()) return;

	// Normalize direction to ensure consistency
	FVector NormalizedAimDir = AimDir.GetSafeNormal();

	// Define Ray Length (adjustable)
	RayLength = 2000000.0f;

	// Calculate ray end based on palm position and aim direction
	FVector RayEnd = PalmPos + NormalizedAimDir * RayLength;

	// Debug: Draw ray for visualization
	if (GEngine)
	{
		DrawDebugLine(GetWorld(), PalmPos, RayEnd, FColor::Blue, false, 0.1f, 0, 1.0f);
	}

	// Raycast using the Palm Position as the start and the Aim direction for the ray's path
	FHitResult HitResult;
	FCollisionQueryParams CollisionParams;
	CollisionParams.AddIgnoredActor(this);


	bool bHit = GetWorld()->LineTraceSingleByChannel(
		HitResult,
		PalmPos, // Start the ray from the Palm Position
		RayEnd,  // End point calculated from the Aim Position
		ECC_Visibility, // Raycast against visibility (or any other appropriate channel)
		CollisionParams
	);

	// If we hit something and we're pinching, scale the object
	if (bHit && bIsPinching)
	{
		if (AStaticMeshActor* HitActor = Cast<AStaticMeshActor>(HitResult.GetActor()))
		{
			UStaticMeshComponent* MeshComp = HitActor->GetStaticMeshComponent();
			if (MeshComp)
			{
				FVector CurrentScale = MeshComp->GetComponentScale();
				FVector NewScale = CurrentScale * 1.003f; // Example: scaling up when pinching
				MeshComp->SetWorldScale3D(NewScale);
			}
		}
	}
}

void AHandTracking::UpdateHandMeshPoseable(const TArray<FVector>& Positions, const TArray<FQuat>& Rotations,
	UPoseableMeshComponent* Poseable,
	bool bIsRightHand)
{
	if (!Poseable) return;

	const FString Suffix = bIsRightHand ? TEXT("_r") : TEXT("_l");
	const FTransform CompToWorld = Poseable->GetComponentTransform();
	Poseable->SetWorldLocation(Positions[(int32)EHandKeypoint::Wrist]);
	Poseable->SetWorldRotation((Rotations[(int32)EHandKeypoint::Wrist].Rotator() + FRotator(0.0f, -90.0f, 0.0f)));
	return;
	//for (int32 Keypoint = (int32)EHandKeypoint::ThumbMetacarpal;
	//	Keypoint <= (int32)EHandKeypoint::LittleTip;
	//	Keypoint++)
	//{
	//	if (!Positions.IsValidIndex(Keypoint)) continue;

	//	const EHandKeypoint HK = static_cast<EHandKeypoint>(Keypoint);
	//	const FString* Base = HandBoneMap.Find(HK);
	//	if (!Base) continue;

	//	const FString BoneStr = *Base + Suffix;
	//	/*const FString BoneStr = *Base;*/
	//	const FName   BoneName(*BoneStr);
	//	//CompToWorld.InverseTransformPosition(Positions[Keypoint])
	//	const FVector  LocalPos = Positions[Keypoint];
	//	//const FQuat    LocalRot = Rotations.IsValidIndex(Keypoint) ? Rotations[Keypoint] : FQuat::Identity;//FQuat::Identity; // replace if you have joint rot
	//	//const FTransform T(LocalRot, LocalPos, FVector::OneVector);
	//	Poseable->SetBoneLocationByName(BoneName, FVector(Positions[Keypoint].X, Positions[Keypoint].Z, Positions[Keypoint].Y), EBoneSpaces::WorldSpace);
	//	//Poseable->SetBoneRotationByName(BoneName, Rotations[Keypoint].Rotator(), EBoneSpaces::WorldSpace);
	//}

	//Poseable->RefreshBoneTransforms();
	//Poseable->InvalidateCachedBounds();
}


void AHandTracking::InitTipSpheresFor(EControllerHand Hand)
{
	UInstancedStaticMeshComponent* Comp = (Hand == EControllerHand::Left) ? TipSpheresL : TipSpheresR;
	TArray<int32>& Indices = (Hand == EControllerHand::Left) ? TipInstanceIdxL : TipInstanceIdxR;

	if (!Comp) return;

	const int32 First = (int32)EHandKeypoint::Palm;
	const int32 Last = (int32)EHandKeypoint::LittleTip;

	Indices.SetNum(Last - First + 1);
	Comp->ClearInstances();

	const float Scale = FMath::Max(0.001f, TipSphereRadiusCm / 50.f); // engine sphere ~50cm radius at (1,1,1)
	for (int32 i = First; i <= Last; ++i)
	{
		const FTransform T(FQuat::Identity, FVector::ZeroVector, FVector(Scale));
		const int32 Idx = Comp->AddInstance(T);
		Indices[i - First] = Idx;
	}

	Comp->MarkRenderStateDirty();
	Comp->SetVisibility(true, true);
}

// Convenience: init both
void AHandTracking::InitTipSpheresBoth()
{

	InitTipSpheresFor(EControllerHand::Left);
	InitTipSpheresFor(EControllerHand::Right);
}


void AHandTracking::UpdateTipSpheresFor(const TArray<FVector>& Positions, EControllerHand Hand)
{
	UInstancedStaticMeshComponent* Comp = (Hand == EControllerHand::Left) ? TipSpheresL : TipSpheresR;
	const TArray<int32>& Indices = (Hand == EControllerHand::Left) ? TipInstanceIdxL : TipInstanceIdxR;

	if (!Comp) return;

	const int32 First = (int32)EHandKeypoint::Palm;
	const int32 Last = (int32)EHandKeypoint::LittleTip;

	for (int32 kp = First; kp <= Last; ++kp)
	{
		const int32 local = kp - First;
		const int32 instance = Indices.IsValidIndex(local) ? Indices[local] : -1;
		if (instance < 0) continue;

		if (Positions.IsValidIndex(kp))
		{
			const FVector WorldPos = Positions[kp];

			FTransform Xf;
			Comp->GetInstanceTransform(instance, Xf, /*bWorldSpace*/ true);
			Xf.SetLocation(WorldPos);

			// keep your visual size consistent (or remove this to keep Init scale)
			Xf.SetScale3D(FVector(0.018f));
			Comp->UpdateInstanceTransform(instance, Xf, /*bWorldSpace*/ true, /*bMarkRenderStateDirty*/ true, /*bTeleport*/ true);
		}
	}

	Comp->MarkRenderStateDirty();
}

// Convenience: update both
void AHandTracking::UpdateTipSpheresBoth(const TArray<FVector>& LeftPositions,
	const TArray<FVector>& RightPositions)
{
	UpdateTipSpheresFor(LeftPositions, EControllerHand::Left);
	UpdateTipSpheresFor(RightPositions, EControllerHand::Right);
}


void AHandTracking::HideTipSpheres(EControllerHand Hand)
{
	if (Hand == EControllerHand::Left)
	{
		TipSpheresL->SetVisibility(false, true);
	}
	if (Hand == EControllerHand::Right)
	{
		TipSpheresR->SetVisibility(false, true);
	}
}
//PANNING TRY HOTAY HUAY!!!!!!!!!!


// Call this from Tick when RIGHT hand is pinching (you already have RP + DeltaSeconds).
// It uses static per-call state so you don't need extra class members.
// If you prefer, move the tunables to UPROPERTYs.

// AHandTracking.cpp

static FORCEINLINE FVector LerpExpVec(const FVector& from, const FVector& to, float hz, float dt)
{
	const float a = 1.f - FMath::Exp(-FMath::Max(0.f, hz) * dt);
	return FMath::Lerp(from, to, a);
}


//THE FOLLOWING FUNCTION INVOLVES HIGH LEVEL ORBITAL CALCULATIONS AND ANGULAR VELOCITY :: NORMAL VELOCITY FACTORS DONT WORK WILL WORK ON CONTROLLING THE SPEED LATER ON


void AHandTracking::PanOrbitAroundCesiumCenter_Stable(
	const TArray<FVector>& RightHandPositions, // note: const ref
	bool bPinchActive,
	float DeltaSeconds)
{
	// --------- Validate input / georef ---------
	const int32 ThumbTip = (int32)EHandKeypoint::ThumbTip;
	const int32 IndexTip = (int32)EHandKeypoint::IndexTip;

	ACesiumGeoreference* Geo = ACesiumGeoreference::GetDefaultGeoreference(this);
	if (!Geo) return;

	const FVector EarthCenterUU =
		Geo->TransformEarthCenteredEarthFixedPositionToUnreal(FVector::ZeroVector);

	if (!RightHandPositions.IsValidIndex(ThumbTip) ||
		!RightHandPositions.IsValidIndex(IndexTip))
	{
		return;
	}

	// --------- Start / end of pinch (now uses member state) ---------
	if (!bPinchActive)
	{
		bPinchActiveLatched = false;
		PrevSmMid = FVector::ZeroVector;
		BaseMid = FVector::ZeroVector;
		bArmed = false;
		return;
	}

	// Pinch midpoint (raw & smoothed)
	const FVector pinchMidRaw = 0.5f * (RightHandPositions[ThumbTip] + RightHandPositions[IndexTip]);

	if (!bPinchActiveLatched)
	{
		bPinchActiveLatched = true;
		PrevSmMid = pinchMidRaw;
		BaseMid = pinchMidRaw; // baseline (significant displacement measured from here)
		bArmed = false;
		return;
	}

	const FVector smMid = LerpExpVec(PrevSmMid, pinchMidRaw, PinchSmoothHz, DeltaSeconds);

	// --- Baseline check (significant enough) ---
	const FVector dispFromBase = smMid - BaseMid;
	const float   baseMag = dispFromBase.Size();
	if (!bArmed)
	{
		if (baseMag < BaselineArmUU)
		{
			PrevSmMid = smMid;
			return;
		}
		bArmed = true;
	}

	// --- Per-frame delta with deadzone & clamp ---
	FVector d = smMid - PrevSmMid;
	float   mag = d.Size();
	if (mag <= PinchDeadzoneUU) { PrevSmMid = smMid; return; }
	if (mag > HandDeltaMaxUU) { d *= (HandDeltaMaxUU / mag); mag = HandDeltaMaxUU; }

	// --------- Robust tangent basis at pawn ---------
	const FVector P = GetActorLocation();
	const FVector r = P - EarthCenterUU;
	const double  R = r.Size();
	if (R < 1e-3) { PrevSmMid = smMid; return; }

	const FVector up = r / R;

	// Build EAST first (avoid parallel helper), then NORTH = EAST × UP
	FVector helper = FVector::UpVector;
	if (FMath::Abs(FVector::DotProduct(helper, up)) > 0.99f)
		helper = FVector::ForwardVector;

	FVector east = FVector::CrossProduct(up, helper).GetSafeNormal();
	if (east.IsNearlyZero()) east = FVector::RightVector;

	FVector north = FVector::CrossProduct(east, up).GetSafeNormal();

	// Project hand motion to tangent plane
	FVector dTan = d - up * FVector::DotProduct(d, up);
	if (dTan.IsNearlyZero(1e-6f)) { PrevSmMid = smMid; return; }

	// Signed components
	const float dEast = FVector::DotProduct(dTan, east);   // +right/-left
	const float dNorth = FVector::DotProduct(dTan, north);  // +up/-down

	// Signed axis: right swipe => rotate about -north, up swipe => +east
	FVector axis = (-dEast) * north + (dNorth)*east;
	const double axisLn = axis.Size();
	if (axisLn < 1e-8) { PrevSmMid = smMid; return; }
	axis /= axisLn;

	// --------- Height-only theta lerp (smooth from MinDeg to MaxDeg) ---------
	const double h = FMath::Max(0.0, height); // meters


	// Normalize height to [0..1] between HMin and HMax
	double u = 0.0;
	if (HMax > HMin)
	{
		u = FMath::Clamp((h - HMin) / (HMax - HMin), 0.0, 1.0);
	}

	// Lerp degrees/sec, then convert to radians *per frame*
	const double degPerSec = FMath::Lerp(MinDegPerSec, MaxDegPerSec, u);
	const double theta = FMath::DegreesToRadians(degPerSec) * FMath::Max(DeltaSeconds, KINDA_SMALL_NUMBER);

	// Rotate and apply
	const FQuat  qRot(axis, (float)theta);
	const FVector rNew = qRot.RotateVector(r);
	const FVector newPos = EarthCenterUU + rNew;

	if (USceneComponent* root = GetRootComponent())
		root->SetWorldLocation(newPos);
	else
		SetActorLocation(newPos, /*bSweep*/ false);

	// --------- Auto-orient (keep Earth in front, no roll) ---------
	{
		const FVector upDesired = (newPos - EarthCenterUU).GetSafeNormal();
		FVector       fwdDesired = (EarthCenterUU - newPos).GetSafeNormal();
		fwdDesired = FVector::VectorPlaneProject(fwdDesired, upDesired).GetSafeNormal();

		if (!upDesired.IsNearlyZero() && !fwdDesired.IsNearlyZero())
		{
			const FQuat target = FRotationMatrix::MakeFromXZ(fwdDesired, upDesired).ToQuat();
			const FQuat curr = GetActorQuat();
			const float aRot = 1.f - FMath::Exp(-RotSmoothHz * DeltaSeconds);
			const FQuat sm = FQuat::Slerp(curr, target, aRot).GetNormalized();

			if (USceneComponent* root = GetRootComponent())
				root->SetWorldRotation(sm);
			else
				SetActorRotation(sm);
		}
	}

	// --------- Persist for next frame ---------
	PrevSmMid = smMid;
}

//
//void AHandTracking::PanOrbitAroundCesiumCenter_Stable(
//	TArray<FVector> RightHandPositions,
//	bool bPinchActive,
//	float DeltaSeconds)
//{
//	// --------- Validate input / georef ---------
//	const int32 ThumbTip = (int32)EHandKeypoint::ThumbTip;
//	const int32 IndexTip = (int32)EHandKeypoint::IndexTip;
//
//	ACesiumGeoreference* Geo = ACesiumGeoreference::GetDefaultGeoreference(this);
//	if (!Geo) return;
//
//	const FVector EarthCenterUU =
//		Geo->TransformEarthCenteredEarthFixedPositionToUnreal(FVector::ZeroVector);
//
//	if (!RightHandPositions.IsValidIndex(ThumbTip) ||
//		!RightHandPositions.IsValidIndex(IndexTip))
//	{
//		return;
//	}
//
//	// --------- Per-pinch persistent state (single definition) ---------
//	static bool     sPinchActiveLatched = false;   // are we inside a pinch session?
//	static FVector  sPrevSmMid = FVector::ZeroVector;
//	static FVector  sBaseMid = FVector::ZeroVector; // baseline at pinch start
//	static bool     sArmed = false;   // true once we exceeded BaselineArmUU
//
//	// --------- Start / end of pinch ---------
//	if (!bPinchActive)
//	{
//		// Reset ALL pinch state once the pinch ends
//		sPinchActiveLatched = false;
//		sPrevSmMid = FVector::ZeroVector;
//		sBaseMid = FVector::ZeroVector;
//		sArmed = false;
//		return;
//	}
//
//	// Pinch midpoint (raw & smoothed)
//	const FVector pinchMidRaw = 0.5f * (RightHandPositions[ThumbTip] + RightHandPositions[IndexTip]);
//
//	if (!sPinchActiveLatched)
//	{
//		// First active frame: set baseline and previous, but don't move.
//		sPinchActiveLatched = true;
//		sPrevSmMid = pinchMidRaw;
//		sBaseMid = pinchMidRaw; // << baseline (significant displacement measured from here)
//		sArmed = false;
//		return;
//	}
//
//	const FVector smMid = LerpExp(sPrevSmMid, pinchMidRaw, PinchSmoothHz, DeltaSeconds);
//
//	// --- Baseline check (significant enough) ---
//	const FVector dispFromBase = smMid - sBaseMid;
//	const float   baseMag = dispFromBase.Size();
//	if (!sArmed)
//	{
//		// Don’t respond until the hand has moved a meaningful distance from its baseline
//		if (baseMag < BaselineArmUU)
//		{
//			sPrevSmMid = smMid;
//			return;
//		}
//		sArmed = true; // armed once we cross the baseline threshold
//	}
//
//	// --- Per-frame delta with deadzone & clamp ---
//	FVector d = smMid - sPrevSmMid;
//	float   mag = d.Size();
//	if (mag <= PinchDeadzoneUU) { sPrevSmMid = smMid; return; }
//	if (mag > HandDeltaMaxUU) { d *= (HandDeltaMaxUU / mag); mag = HandDeltaMaxUU; }
//
//	// --------- Compute tangent basis at pawn (Up, North, East) ---------
//// --------- Robust tangent basis at pawn ---------
//	const FVector P = GetActorLocation();
//	const FVector r = P - EarthCenterUU;
//	const double  R = r.Size();
//	if (R < 1e-3) { sPrevSmMid = smMid; return; }
//
//	const FVector up = r / R;
//
//	// Build EAST first (avoid parallel helper), then NORTH = EAST × UP
//	FVector helper = FVector::UpVector;
//	if (FMath::Abs(FVector::DotProduct(helper, up)) > 0.99f)  // near-parallel to up?
//		helper = FVector::ForwardVector;
//
//	FVector east = FVector::CrossProduct(up, helper).GetSafeNormal();
//	if (east.IsNearlyZero()) east = FVector::RightVector;
//
//	FVector north = FVector::CrossProduct(east, up).GetSafeNormal();
//
//	// Project hand motion to tangent plane
//	FVector dTan = d - up * FVector::DotProduct(d, up);
//	if (dTan.IsNearlyZero(1e-6f)) { sPrevSmMid = smMid; return; }
//
//	// Signed components
//	const float dEast = FVector::DotProduct(dTan, east);   // +right/-left
//	const float dNorth = FVector::DotProduct(dTan, north);  // +up/-down
//
//	// Signed axis: right swipe => rotate about -north, up swipe => +east
//	FVector axis = (-dEast) * north + (dNorth)*east;
//	const double axisLn = axis.Size();
//	if (axisLn < 1e-8) { sPrevSmMid = smMid; return; }
//	axis /= axisLn;
//
//
//
//
//
//
//	// Height-only theta lerp (smooth from MinDeg to MaxDeg)
//	const double h = FMath::Max(0.0, height); // meters
//
//	// Height range for interpolation
//	const double HMin = 3000.0;     // height where you want MinDeg
//	const double HMax = 100000.0;   // height where you want MaxDeg
//
//	// Normalize height to [0..1] between HMin and HMax
//	double u = 0.0;
//	if (HMax > HMin)
//	{
//		u = FMath::Clamp((h - HMin) / (HMax - HMin), 0.0, 1.0);
//	}
//
//	// Lerp degrees/sec, then convert to radians *per frame*
//	const double degPerSec = FMath::Lerp(MinDegPerSec, MaxDegPerSec, u);
//	const double theta = FMath::DegreesToRadians(degPerSec) * FMath::Max(DeltaSeconds, KINDA_SMALL_NUMBER);
//
//
//
//
//	// Rotate and apply
//	const FQuat  qRot(axis, (float)theta);
//	const FVector rNew = qRot.RotateVector(r);
//	const FVector newPos = EarthCenterUU + rNew;
//	// exact radius preserved
//
//	// Apply location (no sweep)
//	if (USceneComponent* root = GetRootComponent())
//		root->SetWorldLocation(newPos);
//	else
//		SetActorLocation(newPos, /*bSweep*/ false);
//
//	// --------- Auto-orient (keep Earth in front, no roll) ---------
//	{
//		const FVector upDesired = (newPos - EarthCenterUU).GetSafeNormal();
//		FVector       fwdDesired = (EarthCenterUU - newPos).GetSafeNormal();
//		fwdDesired = FVector::VectorPlaneProject(fwdDesired, upDesired).GetSafeNormal();
//
//		if (!upDesired.IsNearlyZero() && !fwdDesired.IsNearlyZero())
//		{
//			const FQuat target = FRotationMatrix::MakeFromXZ(fwdDesired, upDesired).ToQuat();
//			const FQuat curr = GetActorQuat();
//			const float aRot = 1.f - FMath::Exp(-RotSmoothHz * DeltaSeconds);
//			const FQuat sm = FQuat::Slerp(curr, target, aRot).GetNormalized();
//
//			if (USceneComponent* root = GetRootComponent())
//				root->SetWorldRotation(sm);
//			else
//				SetActorRotation(sm);
//		}
//	}
//
//	// --------- Persist for next frame ---------
//	sPrevSmMid = smMid;
//}
//







FVector AHandTracking::GetPinchMid(const TArray<FVector>& Positions) const
{
	const int32 ThumbTip = (int32)EHandKeypoint::ThumbTip;
	const int32 IndexTip = (int32)EHandKeypoint::IndexTip;
	if (!Positions.IsValidIndex(ThumbTip) || !Positions.IsValidIndex(IndexTip))
		return FVector::ZeroVector;
	return 0.5f * (Positions[ThumbTip] + Positions[IndexTip]);
}

void AHandTracking::UpdatePinchRay(EControllerHand Hand,
	const TArray<FVector>& Positions,
	const TArray<FQuat>& Rotations,
	bool bIsPinching,
	float DeltaSeconds)
{
	FPinchRayState& Ray = (Hand == EControllerHand::Left) ? LeftPinch : RightPinch;

	// 1) Detect pinch start/end and lock direction at start
	if (bIsPinching && !Ray.bActive)
	{
		Ray.bActive = true;
		Ray.HitActor = nullptr;
		Ray.Hit = FHitResult();

		// origin starts at pinch midpoint
		Ray.OriginWS = GetPinchMid(Positions);

		// lock a stable direction (camera forward) to avoid flicker
		const APlayerCameraManager* Cam = UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0);
		const FVector LockedDir = Cam ? Cam->GetActorForwardVector() : FVector::ForwardVector;
		Ray.DirWS = LockedDir.GetSafeNormal();
	}
	else if (!bIsPinching && Ray.bActive)
	{
		Ray.bActive = false;
		Ray.HitActor = nullptr;
		Ray.Hit = FHitResult();

		// If either hand released, stop any two-hand scaling latch
		if (TwoHandTargetComp.IsValid())
		{
			TwoHandTargetComp.Reset();
			TwoHandBaselineDist = 0.f;
		}
	}

	if (!Ray.bActive)
		return;


	// 2) Update origin (smoothed), keep direction locked (or optionally smooth to palm normal)
	const FVector PinchPos = GetPinchMid(Positions);
	Ray.OriginWS = LerpExp(Ray.OriginWS, PinchPos, RaySmoothHz, DeltaSeconds);

	// Optional: if you want to *slowly* follow palm normal instead of fully locked:
	if (!bLockDirOnPinchStart && Rotations.IsValidIndex((int32)EHandKeypoint::Palm)) {
		const FVector PalmDown = Rotations[(int32)EHandKeypoint::Palm].GetUpVector() * -1.f;
		Ray.DirWS = LerpExp(Ray.DirWS, PalmDown.GetSafeNormal(), 6.f, DeltaSeconds).GetSafeNormal();
	}

	// 3) Raycast
	const FVector Start = Ray.OriginWS;
	const FVector End = Start + Ray.DirWS * RayLength;

	FCollisionQueryParams QP(SCENE_QUERY_STAT(HandPinchRay), false, this);


	FHitResult Hit;
	const bool bHit = GetWorld()->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility, QP);

	Ray.Hit = Hit;
	Ray.HitActor = bHit ? Hit.GetActor() : nullptr;

	// Debug
	DrawDebugLine(GetWorld(), Start, End, bHit ? FColor::Green : FColor::Blue, false, 0.f, 0, 1.5f);
	DrawDebugSphere(GetWorld(), Start, 1.2f, 8, FColor::Cyan, false, 0.f, 0, 0.8f);
	if (bHit) DrawDebugPoint(GetWorld(), Hit.ImpactPoint, 8.f, FColor::Yellow, false, 0.f);
}






//Helper to ADD SMOOTHNES
static FORCEINLINE float ExpEase(float Hz, float DT)
{
	return 1.f - FMath::Exp(-FMath::Max(0.f, Hz) * DT);
}





//DYNAMIC PINCH DISTANCE CALCULATION::


float AHandTracking::CalculateTwoHandSpeed(const TArray<FVector>& LeftP,
	const TArray<FVector>& RightP,
	bool bLeftPinch, bool bRightPinch,
	float DeltaSeconds)   // <-- use DeltaTime here for smooth scaling
{
	// Only calculate movement if both pinches are active
	static float TwoHandBaseDistCm = -1.f;
	if (!(bLeftPinch && bRightPinch))
	{
		bIsMoving = false;
		currentSpeed = 0.f;
		TwoHandBaseDistCm = -1.f;

		return 0.f;
	}

	// On gesture start (when baseline is latched), also lock basis/rotation next tick
	if (TwoHandBaseDistCm == -1.f) {
		const FVector LPinch = GetPinchMid(LeftP);
		const FVector RPinch = GetPinchMid(RightP);
		TwoHandBaseDistCm = FVector::Dist(LPinch, RPinch);


		return 0.f;
	}
	// Get the current pinch positions and calculate the current distance between the two pinches
	const FVector LPinch = GetPinchMid(LeftP);
	const FVector RPinch = GetPinchMid(RightP);
	const float CurrentDist = FVector::Dist(LPinch, RPinch);
	if (CurrentDist <= KINDA_SMALL_NUMBER) return 0.f;

	// Calculate the change in distance from the initial baseline distance
	float deltaCm = CurrentDist - TwoHandBaseDistCm;

	// Deadzone: Ignore small changes to prevent jitter
	if (FMath::Abs(deltaCm) < TwoHandZoomDeadzoneCm)
		deltaCm = 0.f;
	else
		deltaCm -= FMath::Sign(deltaCm) * TwoHandZoomDeadzoneCm;

	// Apply exponential speed factor based on distance change
	float speed = FMath::Pow(FMath::Abs(deltaCm), 1.5f) * height;
	UE_LOG(LogTemp, Warning, TEXT("Height: %f"), height);// Exponential speed increase
	UE_LOG(LogTemp, Warning, TEXT("Speed: %f"), speed);
	// Direction: Move out if the distance is decreasing (pinching) and move towards if the distance is increasing
	if (deltaCm < 0)
	{
		// If pinch distance is decreasing, set the movement direction to outwards
		speed = -speed;  // Move **out** when the pinch distance decreases

		// Ensure that the speed is consistent while moving outwards, not stopping unless the pinch stops
		if (!bIsMoving)
		{
			bIsMoving = true;
			currentSpeed = speed;  // Start moving outwards with the calculated speed
		}
		else
		{
			// Continue applying the current speed while the pinch is still decreasing
			currentSpeed = speed;  // Maintain the same speed for outward movement
		}
	}
	else
	{
		// If pinch distance is increasing, move towards
		speed = speed;   // Move **towards** when the pinch distance increases

		// Ensure that the movement towards continues, adjusting speed as needed
		if (!bIsMoving)
		{
			bIsMoving = true;
			currentSpeed = speed;  // Start moving with calculated speed towards
		}
		else
		{
			// Keep applying the last calculated speed if pinch distance doesn't change
			if (deltaCm != 0)
			{
				currentSpeed = speed;  // Update speed if distance increases
			}
		}
	}
	// Return the calculated speed
	return currentSpeed;
}