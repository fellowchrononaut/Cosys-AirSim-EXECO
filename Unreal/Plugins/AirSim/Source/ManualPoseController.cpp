#include "ManualPoseController.h"
#include "AirBlueprintLib.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"
#include "HAL/IConsoleManager.h"

// Mouse look, U-7/U-8. The camera is driven WASD-to-move + mouse-to-look; the arrow keys are left
// free for the vehicle, which is what removes the Manual-mode input conflict.
//
// Note DefaultInput.ini already applies Sensitivity=0.07 to the MouseX/MouseY axes, so this scaler
// multiplies on top of that. Exposed as a CVar because tuning it otherwise costs a full rebuild.
static TAutoConsoleVariable<float> CVarManualCamMouseSensitivity(
    TEXT("airsim.ManualCamMouseSensitivity"),
    1.0f,
    TEXT("Mouse look sensitivity for the Manual-mode free camera. Multiplies the engine's own MouseX/MouseY axis sensitivity."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarManualCamInvertY(
    TEXT("airsim.ManualCamInvertY"),
    0,
    TEXT("1 to invert mouse pitch for the Manual-mode free camera."),
    ECVF_Default);

// The project ships DefaultViewportMouseCaptureMode=NoCapture, so without this the cursor is never
// locked to the viewport and a look sweep dies at the window edge. Kept switchable at runtime: if
// capture misbehaves on this platform, set 0 and the bindings still work with the cursor in-window.
static TAutoConsoleVariable<int32> CVarManualCamCaptureMouse(
    TEXT("airsim.ManualCamCaptureMouse"),
    1,
    TEXT("1 to lock/hide the cursor while the Manual-mode free camera is active."),
    ECVF_Default);

namespace
{
    bool g_manual_mouse_capture_active = false;
}

bool UManualPoseController::isMouseCaptureActive()
{
    return g_manual_mouse_capture_active;
}

void UManualPoseController::applyMouseCapture(UWorld* world, bool capture)
{
    if (world == nullptr)
        return;

    APlayerController* pc = UGameplayStatics::GetPlayerController(world, 0);
    if (pc == nullptr)
        return;

    if (capture) {
        pc->bShowMouseCursor = false;
        pc->SetInputMode(FInputModeGameOnly());
    }
    else {
        pc->SetInputMode(FInputModeGameAndUI().SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock));
    }
}

void UManualPoseController::initializeForPlay()
{
    actor_ = nullptr;
    clearBindings();

    left_mapping_ = FInputAxisKeyMapping("inputManualLeft", EKeys::A);
    right_mapping_ = FInputAxisKeyMapping("inputManualRight", EKeys::D);
    forward_mapping_ = FInputAxisKeyMapping("inputManualForward", EKeys::W);
    backward_mapping_ = FInputAxisKeyMapping("inputManualBackward", EKeys::S);
    up_mapping_ = FInputAxisKeyMapping("inputManualUp", EKeys::PageUp);
    down_mapping_ = FInputAxisKeyMapping("inputManualDown", EKeys::PageDown);
    mouse_yaw_mapping_ = FInputAxisKeyMapping("inputManualMouseYaw", EKeys::MouseX);
    mouse_pitch_mapping_ = FInputAxisKeyMapping("inputManualMousePitch", EKeys::MouseY);
    left_roll_mapping_ = FInputAxisKeyMapping("inputManualLefRoll", EKeys::Q);
    right_roll_mapping_ = FInputAxisKeyMapping("inputManualRightRoll", EKeys::E);
    inc_speed_mapping_ = FInputAxisKeyMapping("inputManualSpeedIncrease", EKeys::LeftShift);
    dec_speed_mapping_ = FInputAxisKeyMapping("inputManualSpeedDecrease", EKeys::LeftControl);
    input_positive_ = inpute_negative_ = last_velocity_ = FVector::ZeroVector;
}

void UManualPoseController::clearBindings()
{
    left_binding_ = right_binding_ = up_binding_ = down_binding_ = nullptr;
    forward_binding_ = backward_binding_ = nullptr;
    mouse_yaw_binding_ = mouse_pitch_binding_ = nullptr;
    left_roll_binding_ = right_roll_binding_ = nullptr;
    inc_speed_binding_ = dec_speed_binding_ = nullptr;
}

void UManualPoseController::setActor(AActor* actor)
{
    //if we already have attached actor
    if (actor_) {
        removeInputBindings();
    }

    UWorld* world = (actor_ != nullptr) ? actor_->GetWorld() : (actor != nullptr ? actor->GetWorld() : nullptr);

    actor_ = actor;

    if (actor_ != nullptr) {
        resetDelta();
        setupInputBindings();
        if (CVarManualCamCaptureMouse.GetValueOnGameThread() != 0) {
            applyMouseCapture(world, true);
            g_manual_mouse_capture_active = true;
        }
    }
    else if (g_manual_mouse_capture_active) {
        applyMouseCapture(world, false);
        g_manual_mouse_capture_active = false;
    }
}

AActor* UManualPoseController::getActor() const
{
    return actor_;
}

void UManualPoseController::updateActorPose(float dt)
{
    if (actor_ != nullptr) {
        updateDeltaPosition(dt);

        FVector location = actor_->GetActorLocation();
        FRotator rotation = actor_->GetActorRotation();

        FRotator new_rotation = rotation + delta_rotation_;
        // Mouse look reaches the poles in a single sweep, where unclamped pitch flips the camera
        // over. The keyboard rotations were slow enough to hide this; they no longer are.
        new_rotation.Pitch = FMath::ClampAngle(new_rotation.Pitch, -89.9f, 89.9f);

        actor_->SetActorLocationAndRotation(location + delta_position_, new_rotation);
        resetDelta();
    }
    else {
        UAirBlueprintLib::LogMessageString("UManualPoseController::updateActorPose should not be called when actor is not set", "", LogDebugLevel::Failure);
    }
}

void UManualPoseController::getDeltaPose(FVector& delta_position, FRotator& delta_rotation) const
{
    delta_position = delta_position_;
    delta_rotation = delta_rotation_;
}

void UManualPoseController::resetDelta()
{
    delta_position_ = FVector::ZeroVector;
    delta_rotation_ = FRotator::ZeroRotator;
}

void UManualPoseController::removeInputBindings()
{
    if (left_binding_)
        UAirBlueprintLib::RemoveAxisBinding(left_mapping_, left_binding_, actor_);
    if (right_binding_)
        UAirBlueprintLib::RemoveAxisBinding(right_mapping_, right_binding_, actor_);
    if (forward_binding_)
        UAirBlueprintLib::RemoveAxisBinding(forward_mapping_, forward_binding_, actor_);
    if (backward_binding_)
        UAirBlueprintLib::RemoveAxisBinding(backward_mapping_, backward_binding_, actor_);
    if (up_binding_)
        UAirBlueprintLib::RemoveAxisBinding(up_mapping_, up_binding_, actor_);
    if (down_binding_)
        UAirBlueprintLib::RemoveAxisBinding(down_mapping_, down_binding_, actor_);
    if (mouse_yaw_binding_)
        UAirBlueprintLib::RemoveAxisBinding(mouse_yaw_mapping_, mouse_yaw_binding_, actor_);
    if (mouse_pitch_binding_)
        UAirBlueprintLib::RemoveAxisBinding(mouse_pitch_mapping_, mouse_pitch_binding_, actor_);
    if (left_roll_binding_)
        UAirBlueprintLib::RemoveAxisBinding(left_roll_mapping_, left_roll_binding_, actor_);
    if (right_roll_binding_)
        UAirBlueprintLib::RemoveAxisBinding(right_roll_mapping_, right_roll_binding_, actor_);
    if (inc_speed_binding_)
        UAirBlueprintLib::RemoveAxisBinding(inc_speed_mapping_, inc_speed_binding_, actor_);
    if (dec_speed_binding_)
        UAirBlueprintLib::RemoveAxisBinding(dec_speed_mapping_, dec_speed_binding_, actor_);

    clearBindings();
}

void UManualPoseController::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(actor_);

    left_binding_ = &UAirBlueprintLib::BindAxisToKey(left_mapping_, actor_, this, &UManualPoseController::inputManualLeft);
    right_binding_ = &UAirBlueprintLib::BindAxisToKey(right_mapping_, actor_, this, &UManualPoseController::inputManualRight);
    forward_binding_ = &UAirBlueprintLib::BindAxisToKey(forward_mapping_, actor_, this, &UManualPoseController::inputManualForward);
    backward_binding_ = &UAirBlueprintLib::BindAxisToKey(backward_mapping_, actor_, this, &UManualPoseController::inputManualBackward);
    up_binding_ = &UAirBlueprintLib::BindAxisToKey(up_mapping_, actor_, this, &UManualPoseController::inputManualMoveUp);
    down_binding_ = &UAirBlueprintLib::BindAxisToKey(down_mapping_, actor_, this, &UManualPoseController::inputManualDown);
    mouse_yaw_binding_ = &UAirBlueprintLib::BindAxisToKey(mouse_yaw_mapping_, actor_, this, &UManualPoseController::inputManualMouseYaw);
    mouse_pitch_binding_ = &UAirBlueprintLib::BindAxisToKey(mouse_pitch_mapping_, actor_, this, &UManualPoseController::inputManualMousePitch);
    left_roll_binding_ = &UAirBlueprintLib::BindAxisToKey(left_roll_mapping_, actor_, this, &UManualPoseController::inputManualLeftRoll);
    right_roll_binding_ = &UAirBlueprintLib::BindAxisToKey(right_roll_mapping_, actor_, this, &UManualPoseController::inputManualRightRoll);
    inc_speed_binding_ = &UAirBlueprintLib::BindAxisToKey(inc_speed_mapping_, actor_, this, &UManualPoseController::inputManualSpeedIncrease);
    dec_speed_binding_ = &UAirBlueprintLib::BindAxisToKey(dec_speed_mapping_, actor_, this, &UManualPoseController::inputManualSpeedDecrease);
}

void UManualPoseController::updateDeltaPosition(float dt)
{
    FVector input = input_positive_ - inpute_negative_;
    if (!FMath::IsNearlyZero(input.SizeSquared())) {
        if (FMath::IsNearlyZero(acceleration_))
            last_velocity_ = input * speed_scaler_;
        else
            last_velocity_ += input * (acceleration_ * dt);
        delta_position_ += actor_->GetActorRotation().RotateVector(last_velocity_ * dt);
    }
    else {
        delta_position_ = last_velocity_ = FVector::ZeroVector;
    }
}

void UManualPoseController::inputManualSpeedIncrease(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        speed_scaler_ += val * 20;
}
void UManualPoseController::inputManualSpeedDecrease(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        speed_scaler_ -= val * 20;

    if (speed_scaler_ <= 0.0)
        speed_scaler_ = 20.0;
}

void UManualPoseController::inputManualLeft(float val)
{
    inpute_negative_.Y = val;
}
void UManualPoseController::inputManualRight(float val)
{
    input_positive_.Y = val;
}
void UManualPoseController::inputManualForward(float val)
{
    input_positive_.X = val;
}
void UManualPoseController::inputManualBackward(float val)
{
    inpute_negative_.X = val;
}
void UManualPoseController::inputManualMoveUp(float val)
{
    input_positive_.Z = val;
}
void UManualPoseController::inputManualDown(float val)
{
    inpute_negative_.Z = val;
}
// Mouse axes deliver a per-frame displacement, not a held state, so unlike the keyboard rotations
// these must NOT be scaled by dt - the delta already carries the magnitude of the movement.
void UManualPoseController::inputManualMouseYaw(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_rotation_.Add(0, val * CVarManualCamMouseSensitivity.GetValueOnGameThread(), 0);
}
void UManualPoseController::inputManualMousePitch(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f)) {
        const float sign = (CVarManualCamInvertY.GetValueOnGameThread() != 0) ? -1.f : 1.f;
        delta_rotation_.Add(val * CVarManualCamMouseSensitivity.GetValueOnGameThread() * sign, 0, 0);
    }
}
void UManualPoseController::inputManualLeftRoll(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_rotation_.Add(0, 0, -val);
}
void UManualPoseController::inputManualRightRoll(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_rotation_.Add(0, 0, val);
}
