// Fill out your copyright notice in the Description page of Project Settings.


#include "SateliteOrbit.h"
#include "Components/StaticMeshComponent.h"
#include "Camera/CameraComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Math/RotationMatrix.h"

#include "CesiumGeoreference.h"
#include <CesiumGeospatial/Ellipsoid.h>
#include <glm/vec3.hpp>

// Sets default values
ASateliteOrbit::ASateliteOrbit()
{
	PrimaryActorTick.bCanEverTick = true;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	SatelliteMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("SatelliteMesh"));
	SatelliteMesh->SetupAttachment(Root);

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(Root);
	Camera->bLockToHmd = true;
	// Point camera forward by default; we'll steer it every Tick.
}

void ASateliteOrbit::BeginPlay()
{
	Super::BeginPlay();
	CurrentTrueAnomalyDeg = InitialTrueAnomalyDeg;
}

void ASateliteOrbit::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// Advance anomaly
	CurrentTrueAnomalyDeg = FMath::Fmod(CurrentTrueAnomalyDeg + OrbitAngularSpeedDeg * DeltaSeconds, 360.0f);

	ACesiumGeoreference* Geo = ACesiumGeoreference::GetDefaultGeoreference(this);
	if (!Geo) return;

	const FVector EarthWorld = Geo->TransformEarthCenteredEarthFixedPositionToUnreal(FVector::ZeroVector);
	// --- Current radius (dynamic) ---
	

	// Compute and apply satellite world position
	const FVector SatelliteWorldPos = ComputeOrbitPosition(CurrentTrueAnomalyDeg, EarthWorld);
	SetActorLocation(SatelliteWorldPos);

	// (Optional) orient the satellite body so +X points along velocity and +Z roughly up:
	{
		// Estimate velocity direction by sampling a small delta ahead on the orbit
		const float AheadDeg = CurrentTrueAnomalyDeg + 0.1f;
		const FVector AheadPos = ComputeOrbitPosition(AheadDeg, EarthWorld);
		const FVector VelocityDir = (AheadPos - SatelliteWorldPos).GetSafeNormal();

		const FVector UpHint = (SatelliteWorldPos - EarthWorld).GetSafeNormal(); // radial out from Earth
		const FRotator BodyRot = FRotationMatrix::MakeFromXZ(VelocityDir, UpHint).Rotator();
		SetActorRotation(BodyRot);
	}

	// Steer camera to Earth
	AimCameraAt(EarthWorld);
}

FVector ASateliteOrbit::GetEarthCenterWorld() const
{
	return EarthRef ? EarthRef->GetActorLocation() : EarthCenter;
}

FVector ASateliteOrbit::ComputeOrbitPosition(float TrueAnomalyDeg, const FVector& EarthWorld) 
{
	// Base vector at current anomaly in the orbital plane before tilts
	const float AnomRad = FMath::DegreesToRadians(TrueAnomalyDeg);
	const FVector P = GetActorLocation();
	const FVector r = P - EarthWorld;
	const double  R = r.Size();

	OrbitRadius = R;
	FVector posLocal = FVector(OrbitRadius, 0.f, 0.f);
	// Rotate around world up (Z) by anomaly to move around the circle
	FQuat qAnom(FVector::UpVector, AnomRad);
	posLocal = qAnom.RotateVector(posLocal);

	// Apply orbital plane orientation:
	const float RAANRad = FMath::DegreesToRadians(RAANDeg);
	const float IncRad = FMath::DegreesToRadians(OrbitInclinationDeg);

	// Rotate plane by RAAN about Z, then tilt by inclination about X (forward)
	const FQuat qRAAN(FVector::UpVector, RAANRad);
	const FQuat qInc(FVector::ForwardVector, IncRad);

	const FVector worldOffset = qRAAN * qInc.RotateVector(posLocal);
	return EarthWorld + worldOffset;
}

void ASateliteOrbit::AimCameraAt(const FVector& EarthWorld)
{
	if (!Camera) return;

	const FVector CamLoc = Camera->GetComponentLocation();
	const FVector ToEarth = (EarthWorld - CamLoc).GetSafeNormal();

	FRotator CamRot;
	if (bStabilizeCameraUp)
	{
		// Keep a stable horizon: X -> look, Z -> world up (best-effort)
		CamRot = FRotationMatrix::MakeFromXZ(ToEarth, FVector::UpVector).Rotator();
	}
	else
	{
		// Simple look-at, unconstrained roll
		CamRot = UKismetMathLibrary::FindLookAtRotation(CamLoc, EarthWorld);
	}

	Camera->SetWorldRotation(CamRot);
}

