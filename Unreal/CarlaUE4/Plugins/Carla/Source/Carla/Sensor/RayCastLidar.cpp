// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// #include <PxScene.h>
// #include <cmath>
// #include "Carla.h"
// #include "Carla/Sensor/RayCastLidar.h"
// #include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
// #include "carla/geom/Math.h"

// #include <compiler/disable-ue4-macros.h>
// #include "carla/geom/Math.h"
// #include "carla/ros2/ROS2.h"
// #include "carla/geom/Location.h"
// #include <compiler/enable-ue4-macros.h>

// #include "DrawDebugHelpers.h"
// #include "Engine/CollisionProfile.h"
// #include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"

// FActorDefinition ARayCastLidar::GetSensorDefinition()
// {
//   return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast"));
// }


// ARayCastLidar::ARayCastLidar(const FObjectInitializer& ObjectInitializer)
//   : Super(ObjectInitializer) {

//   RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
//   SetSeed(Description.RandomSeed);
// }

// void ARayCastLidar::Set(const FActorDescription &ActorDescription)
// {
//   ASensor::Set(ActorDescription);
//   FLidarDescription LidarDescription;
//   UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
//   Set(LidarDescription);
// }

// void ARayCastLidar::Set(const FLidarDescription &LidarDescription)
// {
//   Description = LidarDescription;
//   LidarData = FLidarData(Description.Channels);
//   CreateLasers();
//   PointsPerChannel.resize(Description.Channels);

//   // Compute drop off model parameters
//   DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
//   DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
//   DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();
// }

// void ARayCastLidar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
// {
//   TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastLidar::PostPhysTick);
//   SimulateLidar(DeltaTime);

//   auto DataStream = GetDataStream(*this);
//   auto SensorTransform = DataStream.GetSensorTransform();

//   {
//     TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
//     DataStream.SerializeAndSend(*this, LidarData, DataStream.PopBufferFromPool());
//   }
//   // ROS2
//   #if defined(WITH_ROS2)
//   auto ROS2 = carla::ros2::ROS2::GetInstance();
//   if (ROS2->IsEnabled())
//   {
//     TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send");
//     auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
//     AActor* ParentActor = GetAttachParentActor();
//     if (ParentActor)
//     {
//       FTransform LocalTransformRelativeToParent = GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
//       ROS2->ProcessDataFromLidar(DataStream.GetSensorType(), StreamId, LocalTransformRelativeToParent, LidarData, this);
//     }
//     else
//     {
//       ROS2->ProcessDataFromLidar(DataStream.GetSensorType(), StreamId, SensorTransform, LidarData, this);
//     }
//   }
//   #endif


// }

// float ARayCastLidar::ComputeIntensity(const FSemanticDetection& RawDetection) const
// {
//   const carla::geom::Location HitPoint = RawDetection.point;
//   const float Distance = HitPoint.Length();

//   const float AttenAtm = Description.AtmospAttenRate;
//   const float AbsAtm = exp(-AttenAtm * Distance);

//   const float IntRec = AbsAtm;

//   return IntRec;
// }

// ARayCastLidar::FDetection ARayCastLidar::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const
// {
//   FDetection Detection;
//   const FVector HitPoint = HitInfo.ImpactPoint;
//   Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

//   const float Distance = Detection.point.Length();

//   const float AttenAtm = Description.AtmospAttenRate;
//   const float AbsAtm = exp(-AttenAtm * Distance);

//   const float IntRec = AbsAtm;

//   Detection.intensity = IntRec;

//   return Detection;
// }

//   void ARayCastLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
//     Super::PreprocessRays(Channels, MaxPointsPerChannel);

//     for (auto ch = 0u; ch < Channels; ch++) {
//       for (auto p = 0u; p < MaxPointsPerChannel; p++) {
//         RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
//       }
//     }
//   }

//   bool ARayCastLidar::PostprocessDetection(FDetection& Detection) const
//   {
//     if (Description.NoiseStdDev > std::numeric_limits<float>::epsilon()) {
//       const auto ForwardVector = Detection.point.MakeUnitVector();
//       const auto Noise = ForwardVector * RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
//       Detection.point += Noise;
//     }

//     const float Intensity = Detection.intensity;
//     if(Intensity > Description.DropOffIntensityLimit)
//       return true;
//     else
//       return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
//   }

//   void ARayCastLidar::ComputeAndSaveDetections(const FTransform& SensorTransform) {
//     for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
//       PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();

//     LidarData.ResetMemory(PointsPerChannel);

//     for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
//       for (auto& hit : RecordedHits[idxChannel]) {
//         FDetection Detection = ComputeDetection(hit, SensorTransform);
//         if (PostprocessDetection(Detection))
//           LidarData.WritePointSync(Detection);
//         else
//           PointsPerChannel[idxChannel]--;
//       }
//     }

//     LidarData.WriteChannelCount(PointsPerChannel);
//   }


  // === File: RayCastLidar.cpp ===
// (Includes unchanged from original)
#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Sensor/RayCastLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "carla/geom/Math.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/ros2/ROS2.h"
#include "carla/geom/Location.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"

FActorDefinition ARayCastLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast"));
}


ARayCastLidar::ARayCastLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer) {

  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  SetSeed(Description.RandomSeed);

  // Initialize progressive scan
  //bProgressiveScan = true; //false pour retrouver celui de base
  //CurrentChannel   = 0;
  CurrentAzimuth = 0;
  AccumulatedDetections.Empty();
}

void ARayCastLidar::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void ARayCastLidar::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  LidarData = FLidarData(Description.Channels);
  CreateLasers();
  PointsPerChannel.resize(Description.Channels);

  // Compute drop off model parameters
  DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
  DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
  DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();
}

void ARayCastLidar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  // stocke Δt pour ComputeAndSaveDetections
  CurrentDeltaTime = DeltaTime;
  SimulateLidar(DeltaTime); //lance le(s) laser
  auto Stream = GetDataStream(*this);
  Stream.SerializeAndSend(*this, LidarData, Stream.PopBufferFromPool());
  // ROS2
  #if defined(WITH_ROS2)
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (ROS2->IsEnabled())
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send");
    auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
    AActor* ParentActor = GetAttachParentActor();
    if (ParentActor)
    {
      FTransform LocalTransformRelativeToParent = GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
      ROS2->ProcessDataFromLidar(DataStream.GetSensorType(), StreamId, LocalTransformRelativeToParent, LidarData, this);
    }
    else
    {
      ROS2->ProcessDataFromLidar(DataStream.GetSensorType(), StreamId, SensorTransform, LidarData, this);
    }
  }
  #endif


}


//inutile ici car intensity calcule apres, mais la de base (utilse dans le semantic), je le lasise pour pas tout niquer
float ARayCastLidar::ComputeIntensity(const FSemanticDetection& RawDetection) const
{
  const carla::geom::Location HitPoint = RawDetection.point;

  const float Distance = HitPoint.Length();

  const float AttenAtm = Description.AtmospAttenRate;

  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  return IntRec;
}

ARayCastLidar::FDetection ARayCastLidar::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const{
  // 1) On crée la struct qui contiendra point et intensité
  FDetection Detection;

  // 2) On récupère le point d’impact en coordonnées monde UE4
  const FVector HitPoint = HitInfo.ImpactPoint;

  // 3) On le transforme en coordonnées locales du capteur
  //    SensorTransf est la transform (position+rotation) du capteur,
  //    Inverse() ramène le point de l’espace monde à l’espace capteur.
  Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

  // 4) On calcule la distance du capteur au point (en mètre)
  const float Distance = Detection.point.Length();

  // 5) On applique la même loi d’atténuation atmosphérique que dans ComputeIntensity
  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  // 6) On stocke l’intensité reçue
  Detection.intensity = AbsAtm;

  // 7) On renvoie la struct complète
  return Detection;
}


void ARayCastLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
    Super::PreprocessRays(Channels, MaxPointsPerChannel);

    for (auto ch = 0u; ch < Channels; ch++) {
      for (auto p = 0u; p < MaxPointsPerChannel; p++) {
        RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
      }
    }
  }

bool ARayCastLidar::PostprocessDetection(FDetection& Detection) const
{
  if (Description.NoiseStdDev > std::numeric_limits<float>::epsilon()) {
    const auto ForwardVector = Detection.point.MakeUnitVector();
    const auto Noise = ForwardVector * RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
    Detection.point += Noise;
  }

  const float Intensity = Detection.intensity;
  if(Intensity > Description.DropOffIntensityLimit)
    return true;
  else
    return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
}

void ARayCastLidar::ComputeAndSaveDetections(const FTransform& SensorTransform)
{
    // ===== Batch (original) =====
    if (!Description.EnableEgoMotion) {
      //std::cout << "Disable EgoMotion" << std::endl;
      for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel){
        PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();
      }

      LidarData.ResetMemory(PointsPerChannel);

      for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
        for (auto& hit : RecordedHits[idxChannel]) {
          FDetection Detection = ComputeDetection(hit, SensorTransform);
          if (PostprocessDetection(Detection))
            LidarData.WritePointSync(Detection);
          else
            PointsPerChannel[idxChannel]--;
        }
      }

      LidarData.WriteChannelCount(PointsPerChannel);
      for (auto &hits : RecordedHits) {
        hits.clear();
      }
      return;
    }


    //std::cout << "EgoMotion" << std::endl;


    // 1) Prépare le buffer pour tous les channels
    std::vector<uint32_t> counts(Description.Channels, 0u);
    LidarData.ResetMemory(counts);

    // 2) Pour chaque channel vertical, on tire un rayon au CurrentAzimuth
    const uint32_t batchSize = 10; //  on tire 10 colonne par tick
    const float vstep = (Description.UpperFovLimit - Description.LowerFovLimit) / float(Description.Channels - 1);

      // on calcule la variation horizontale totale pour ce tick
    const float deltaAzTotal = 360.0f * Description.RotationFrequency * CurrentDeltaTime;
    int i;

    FRotator lidar_rotation = SensorTransform.GetRotation().Rotator();
    FVector location = SensorTransform.GetLocation();
    FRotator rotation = SensorTransform.Rotator();  
    float x = location.X;
    float y = location.Y;
    float z = location.Z;
    float rx = rotation.Roll;  
    float ry = rotation.Pitch; 
    float rz = rotation.Yaw; 
    float base_yaw = lidar_rotation.Yaw;

    FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
    TraceParams.bTraceComplex = true;
    TraceParams.bReturnPhysicalMaterial = false;
    TraceParams.AddIgnoredActor(this);

    for (i=0; i<batchSize;i++){
      float hAngle = FMath::Fmod(base_yaw + CurrentAzimuth+ deltaAzTotal * (float(i) / float(batchSize)), 360.0f);
      for (uint32_t ch = 0; ch < Description.Channels; ++ch)
        {
          // Calcul de l’angle vertical pour ce channel
          float vangle = Description.LowerFovLimit + vstep * float(ch);

          // Construction de la direction du rayon
          FRotator rot(vangle, hAngle, 0.f);
          FVector  dir   = rot.RotateVector(FVector::ForwardVector);
          FVector  start = SensorTransform.GetLocation();
          FVector  end   = start + dir * Description.Range;

          // Lancement du ray‐cast UE4
          TArray<FHitResult> hits;
          bool bHit = GetWorld()->LineTraceMultiByChannel(
              hits, start, end, ECC_GameTraceChannel2, TraceParams);

          FHitResult hit;
          if (!bHit) {
            continue;
          }
          for (const FHitResult& testHit : hits)
          {
            const AActor* actor = testHit.Actor.Get();
            int32 id = 0;
            if (actor)
            {
              const FCarlaActor* view = GetEpisode().GetActorRegistry().FindCarlaActor(actor);
              if (view)
                id = view->GetActorId();
            }
            if (!Description.IgnoredActorIds.Contains(id))
            {
              hit = testHit;
              break;
            }
          }
          if (!hit.bBlockingHit)
          {
            continue;
          }

          // Point d’impact en coords monde
          const FVector &wp_hit = hit.ImpactPoint;
          //carla::geom::Location P_world{ wp_hit.X / 100, wp_hit.Y / 100, wp_hit.Z / 100 };

          // --- UTILISATION DE ComputeDetection ---
          // ComputeDetection transforme le hit en détect., 
          // calcule l’intensité d’après la distance locale.
          FDetection det = ComputeDetection(hit, SensorTransform);

          // Si on veut garder le point EN COORDONNÉES MONDE :
          //det.point = P_world;

          // Enfin on écrit dans le buffer
          if (PostprocessDetection(det)) {
            LidarData.WritePointSync(det);
            counts[ch]++;
          }
        }
    }
    
    
    // 3) Finalise le message
    LidarData.WriteChannelCount(counts);

    CurrentAzimuth = FMath::Fmod(CurrentAzimuth + deltaAzTotal, 360.0f);

}
