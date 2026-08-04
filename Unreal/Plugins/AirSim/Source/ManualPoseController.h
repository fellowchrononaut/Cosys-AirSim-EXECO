#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerInput.h"

#include "ManualPoseController.generated.h"

UCLASS()
class AIRSIM_API UManualPoseController : public UObject
{
    GENERATED_BODY()

public:
    void initializeForPlay();
    void setActor(AActor* actor);
    AActor* getActor() const;
    void updateActorPose(float dt);
    void getDeltaPose(FVector& delta_position, FRotator& delta_rotation) const;
    void resetDelta();
    void updateDeltaPosition(float dt);

    // True while manual mode holds the mouse for free-look. The weather/options menu (F10) checks
    // this so it can hand the cursor back for the duration the menu is open. See WeatherLib.cpp.
    static bool isMouseCaptureActive();
    static void applyMouseCapture(UWorld* world, bool capture);

private:
    void inputManualLeft(float val);
    void inputManualRight(float val);
    void inputManualForward(float val);
    void inputManualBackward(float val);
    void inputManualMoveUp(float val);
    void inputManualDown(float val);
    void inputManualMouseYaw(float val);
    void inputManualMousePitch(float val);
    void inputManualLeftRoll(float val);
    void inputManualRightRoll(float val);
    void inputManualSpeedIncrease(float val);
    void inputManualSpeedDecrease(float val);

    void setupInputBindings();
    void removeInputBindings();
    void clearBindings();

private:
    FInputAxisBinding *left_binding_, *right_binding_, *up_binding_, *down_binding_;
    FInputAxisBinding *forward_binding_, *backward_binding_;
    FInputAxisBinding *mouse_yaw_binding_, *mouse_pitch_binding_;
    FInputAxisBinding *left_roll_binding_, *right_roll_binding_;
    FInputAxisBinding *inc_speed_binding_, *dec_speed_binding_;

    FInputAxisKeyMapping left_mapping_, right_mapping_, up_mapping_, down_mapping_;
    FInputAxisKeyMapping forward_mapping_, backward_mapping_;
    FInputAxisKeyMapping mouse_yaw_mapping_, mouse_pitch_mapping_;
    FInputAxisKeyMapping left_roll_mapping_, right_roll_mapping_;
    FInputAxisKeyMapping inc_speed_mapping_, dec_speed_mapping_;

    FVector delta_position_;
    FRotator delta_rotation_;

    AActor* actor_;

    float acceleration_ = 0, speed_scaler_ = 1000;
    FVector input_positive_, inpute_negative_;
    FVector last_velocity_;
};