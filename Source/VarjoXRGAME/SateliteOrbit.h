// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SateliteOrbit.generated.h"
class UStaticMeshComponent;
class UCameraComponent;

UCLASS()
class VARJOXRGAME_API ASateliteOrbit : public AActor
{
	GENERATED_BODY()

public:
	ASateliteOrbit();

	virtual void Tick(float DeltaSeconds) override;

protected:
	virtual void BeginPlay() override;

public:
	// Components
	UPROPERTY(VisibleAnywhere, Category = "Components")
	USceneComponent* Root;

	UPROPERTY(VisibleAnywhere, Category = "Components")
	UStaticMeshComponent* SatelliteMesh;

	UPROPERTY(VisibleAnywhere, Category = "Components")
	UCameraComponent* Camera;

public:
	/** If set, the satellite orbits around this actor's world location. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit")
	AActor* EarthRef = nullptr;

	/** Used if EarthRef is not set. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit")
	FVector EarthCenter = FVector::ZeroVector;

	/** Circular orbit radius in cm. (e.g., 100000.0 = 1 km) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit")
	float OrbitRadius; // 10 km demo radius

	/** Angular speed in degrees per second (constant circular orbit). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit")
	float OrbitAngularSpeedDeg = 10.0f;

	/** Inclination of the orbital plane in degrees (tilt). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit", meta = (ClampMin = "-180.0", ClampMax = "180.0"))
	float OrbitInclinationDeg = 0.0f;

	/** Right Ascension of Ascending Node in degrees (rotates the orbital plane about world up). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit", meta = (ClampMin = "-180.0", ClampMax = "180.0"))
	float RAANDeg = 0.0f;

	/** Starting true anomaly (initial position in the orbit) in degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit", meta = (ClampMin = "-180.0", ClampMax = "180.0"))
	float InitialTrueAnomalyDeg = 0.0f;

	/** Keep the camera's 'up' roughly aligned with world up to avoid roll. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
	bool bStabilizeCameraUp = true;

private:
	float CurrentTrueAnomalyDeg = 0.0f;

	/** Returns the world position of the Earth center to look at/orbit around. */
	FVector GetEarthCenterWorld() const;

	/** Computes the current satellite position for the given anomaly. */
	FVector ComputeOrbitPosition(float TrueAnomalyDeg, const FVector& EarthWorld);

	/** Updates camera to look at EarthWorld. */
	void AimCameraAt(const FVector& EarthWorld);

};
