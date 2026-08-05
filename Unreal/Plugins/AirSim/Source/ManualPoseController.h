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

    // Set while a menu owns the cursor, so cursor-recentring does not fight it for control.
    static void setMouseInputSuspended(bool suspended);

private:
    void inputManualLeft(float val);
    void inputManualRight(float val);
    void inputManualForward(float val);
    void inputManualBackward(float val);
    void inputManualMoveUp(float val);
    void inputManualDown(float val);
    void inputManualRightMouseButtonPressed();
    void inputManualRightMouseButtonReleased();
    void inputManualMouseYaw(float val);
    void inputManualMousePitch(float val);
    void inputManualKeyLeftYaw(float val);
    void inputManualKeyRightYaw(float val);
    void inputManualKeyUpPitch(float val);
    void inputManualKeyDownPitch(float val);
    void inputManualLeftRoll(float val);
    void inputManualRightRoll(float val);
    void inputManualSpeedIncrease(float val);
    void inputManualSpeedDecrease(float val);

    void setupInputBindings();
    void removeInputBindings();
    void clearBindings();

    static bool useRecenterMode();
    void pollRecenteredMouse();
    void updateMouseCaptureState();

private:
    FInputAxisBinding *left_binding_, *right_binding_, *up_binding_, *down_binding_;
    FInputAxisBinding *forward_binding_, *backward_binding_;
    FInputAxisBinding *mouse_yaw_binding_, *mouse_pitch_binding_;
    FInputAxisBinding *key_left_yaw_binding_, *key_right_yaw_binding_;
    FInputAxisBinding *key_up_pitch_binding_, *key_down_pitch_binding_;
    FInputAxisBinding *left_roll_binding_, *right_roll_binding_;
    FInputAxisBinding *inc_speed_binding_, *dec_speed_binding_;

    FInputAxisKeyMapping left_mapping_, right_mapping_, up_mapping_, down_mapping_;
    FInputAxisKeyMapping forward_mapping_, backward_mapping_;
    FInputAxisKeyMapping mouse_yaw_mapping_, mouse_pitch_mapping_;
    FInputActionKeyMapping right_mouse_button_mapping_;
    FInputAxisKeyMapping key_left_yaw_mapping_, key_right_yaw_mapping_;
    FInputAxisKeyMapping key_up_pitch_mapping_, key_down_pitch_mapping_;
    FInputAxisKeyMapping left_roll_mapping_, right_roll_mapping_;
    FInputAxisKeyMapping inc_speed_mapping_, dec_speed_mapping_;

    FVector delta_position_;
    FRotator delta_rotation_;

    AActor* actor_;
    bool right_mouse_button_down_ = false;

    float key_yaw_positive_ = 0, key_yaw_negative_ = 0;
    float key_pitch_positive_ = 0, key_pitch_negative_ = 0;

    float acceleration_ = 0, speed_scaler_ = 1000;
    FVector input_positive_, inpute_negative_;
    FVector last_velocity_;
};
