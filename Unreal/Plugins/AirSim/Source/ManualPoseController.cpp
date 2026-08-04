#include "ManualPoseController.h"
#include "AirBlueprintLib.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"
#include "HAL/IConsoleManager.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"

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

static TAutoConsoleVariable<float> CVarManualCamKeyLookRate(
    TEXT("airsim.ManualCamKeyLookRate"),
    90.0f,
    TEXT("Turn rate in degrees/second for the IJKL Manual-mode camera look keys."),
    ECVF_Default);

// Fallback for platforms where mouse capture will not hold and the cursor still clamps at the
// screen edge. Poll the cursor's offset from the viewport centre and warp it back each frame,
// instead of reading the MouseX/MouseY axes. Intended to be used with ManualCamCaptureMouse 0.
static TAutoConsoleVariable<int32> CVarManualCamRecenterCursor(
    TEXT("airsim.ManualCamRecenterCursor"),
    0,
    TEXT("1 to drive Manual-mode mouse look by recentring the cursor each frame instead of reading the mouse axes. Use with airsim.ManualCamCaptureMouse 0 if look still stops at the screen edge."),
    ECVF_Default);

namespace
{
    bool g_manual_mouse_capture_active = false;
    bool g_manual_mouse_input_suspended = false;

    // Matches the MouseX/MouseY Sensitivity in Blocks/Config/DefaultInput.ini, so that the polled
    // path and the axis path feel identical at the same ManualCamMouseSensitivity value.
    constexpr float kRecenterSensitivityScale = 0.07f;
}

bool UManualPoseController::isMouseCaptureActive()
{
    return g_manual_mouse_capture_active;
}

void UManualPoseController::setMouseInputSuspended(bool suspended)
{
    g_manual_mouse_input_suspended = suspended;
}

void UManualPoseController::applyMouseCapture(UWorld* world, bool capture)
{
    if (world == nullptr)
        return;

    APlayerController* pc = UGameplayStatics::GetPlayerController(world, 0);
    if (pc == nullptr)
        return;

    UGameViewportClient* viewport = world->GetGameViewport();

    if (capture) {
        // SetInputMode(FInputModeGameOnly) alone is not enough: its lock mode is LockOnCapture, and
        // capture itself never happens without a click, so the cursor stays a free desktop cursor
        // and a look sweep dies at the screen edge. The viewport client also starts from
        // DefaultInput.ini's NoCapture / DoNotLock. Set it directly so the pointer is locked from
        // the moment Manual mode starts rather than from the first click.
        if (viewport) {
            viewport->SetMouseLockMode(EMouseLockMode::LockAlways);
            viewport->SetMouseCaptureMode(EMouseCaptureMode::CapturePermanently_IncludingInitialMouseDown);
            viewport->SetHideCursorDuringCapture(true);
        }
        pc->bShowMouseCursor = false;
        pc->SetInputMode(FInputModeGameOnly());
    }
    else {
        if (viewport) {
            viewport->SetMouseLockMode(EMouseLockMode::DoNotLock);
            viewport->SetMouseCaptureMode(EMouseCaptureMode::NoCapture);
            viewport->SetHideCursorDuringCapture(false);
        }
        pc->SetInputMode(FInputModeGameAndUI().SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock));
    }
}

bool UManualPoseController::useRecenterMode()
{
    return CVarManualCamRecenterCursor.GetValueOnGameThread() != 0;
}

// Fallback for when platform mouse capture will not hold. Instead of reading the MouseX/MouseY
// axes - which are OS cursor deltas and therefore stop at the desktop edge - measure how far the
// cursor has moved from the viewport centre, then warp it back. Movement is unbounded because the
// cursor never travels more than half a viewport from the middle.
//
// The absolute-position read is what makes the warp safe: the warp's own motion is not counted,
// since each frame measures against the centre rather than accumulating deltas.
void UManualPoseController::pollRecenteredMouse()
{
    if (actor_ == nullptr || g_manual_mouse_input_suspended)
        return;

    APlayerController* pc = UGameplayStatics::GetPlayerController(actor_->GetWorld(), 0);
    if (pc == nullptr)
        return;

    int32 size_x = 0, size_y = 0;
    pc->GetViewportSize(size_x, size_y);
    if (size_x <= 0 || size_y <= 0)
        return;

    const int32 center_x = size_x / 2;
    const int32 center_y = size_y / 2;

    float mouse_x = 0.f, mouse_y = 0.f;
    if (!pc->GetMousePosition(mouse_x, mouse_y)) {
        // Cursor is outside the viewport - recentre and skip this frame rather than apply a jump.
        pc->SetMouseLocation(center_x, center_y);
        return;
    }

    const float dx = mouse_x - static_cast<float>(center_x);
    const float dy = mouse_y - static_cast<float>(center_y);

    if (!FMath::IsNearlyZero(dx) || !FMath::IsNearlyZero(dy)) {
        // kRecenterSensitivityScale mirrors the engine's own MouseX/MouseY sensitivity from
        // DefaultInput.ini, so this path feels the same as the axis path at the same CVar value.
        const float sens = CVarManualCamMouseSensitivity.GetValueOnGameThread() * kRecenterSensitivityScale;
        const float invert = (CVarManualCamInvertY.GetValueOnGameThread() != 0) ? -1.f : 1.f;

        // Screen Y grows downward, so moving the mouse up must pitch up: negate.
        delta_rotation_.Add(-dy * sens * invert, dx * sens, 0);
    }

    pc->SetMouseLocation(center_x, center_y);
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
    // IJKL mirrors the mouse for anyone without one to spare. Front and Backup view moved off I/K
    // to make room - see AirSimCameraDirector::setupInputBindings.
    key_left_yaw_mapping_ = FInputAxisKeyMapping("inputManualKeyLeftYaw", EKeys::J);
    key_right_yaw_mapping_ = FInputAxisKeyMapping("inputManualKeyRightYaw", EKeys::L);
    key_up_pitch_mapping_ = FInputAxisKeyMapping("inputManualKeyUpPitch", EKeys::I);
    key_down_pitch_mapping_ = FInputAxisKeyMapping("inputManualKeyDownPitch", EKeys::K);
    left_roll_mapping_ = FInputAxisKeyMapping("inputManualLefRoll", EKeys::Q);
    right_roll_mapping_ = FInputAxisKeyMapping("inputManualRightRoll", EKeys::E);
    inc_speed_mapping_ = FInputAxisKeyMapping("inputManualSpeedIncrease", EKeys::LeftShift);
    dec_speed_mapping_ = FInputAxisKeyMapping("inputManualSpeedDecrease", EKeys::LeftControl);
    input_positive_ = inpute_negative_ = last_velocity_ = FVector::ZeroVector;
    key_yaw_positive_ = key_yaw_negative_ = key_pitch_positive_ = key_pitch_negative_ = 0.f;
}

void UManualPoseController::clearBindings()
{
    left_binding_ = right_binding_ = up_binding_ = down_binding_ = nullptr;
    forward_binding_ = backward_binding_ = nullptr;
    mouse_yaw_binding_ = mouse_pitch_binding_ = nullptr;
    key_left_yaw_binding_ = key_right_yaw_binding_ = nullptr;
    key_up_pitch_binding_ = key_down_pitch_binding_ = nullptr;
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
        if (useRecenterMode())
            pollRecenteredMouse();

        const float key_yaw = key_yaw_positive_ - key_yaw_negative_;
        const float key_pitch = key_pitch_positive_ - key_pitch_negative_;
        if (!FMath::IsNearlyZero(key_yaw) || !FMath::IsNearlyZero(key_pitch)) {
            const float step = CVarManualCamKeyLookRate.GetValueOnGameThread() * dt;
            delta_rotation_.Add(key_pitch * step, key_yaw * step, 0);
        }

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
    if (key_left_yaw_binding_)
        UAirBlueprintLib::RemoveAxisBinding(key_left_yaw_mapping_, key_left_yaw_binding_, actor_);
    if (key_right_yaw_binding_)
        UAirBlueprintLib::RemoveAxisBinding(key_right_yaw_mapping_, key_right_yaw_binding_, actor_);
    if (key_up_pitch_binding_)
        UAirBlueprintLib::RemoveAxisBinding(key_up_pitch_mapping_, key_up_pitch_binding_, actor_);
    if (key_down_pitch_binding_)
        UAirBlueprintLib::RemoveAxisBinding(key_down_pitch_mapping_, key_down_pitch_binding_, actor_);
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
    key_left_yaw_binding_ = &UAirBlueprintLib::BindAxisToKey(key_left_yaw_mapping_, actor_, this, &UManualPoseController::inputManualKeyLeftYaw);
    key_right_yaw_binding_ = &UAirBlueprintLib::BindAxisToKey(key_right_yaw_mapping_, actor_, this, &UManualPoseController::inputManualKeyRightYaw);
    key_up_pitch_binding_ = &UAirBlueprintLib::BindAxisToKey(key_up_pitch_mapping_, actor_, this, &UManualPoseController::inputManualKeyUpPitch);
    key_down_pitch_binding_ = &UAirBlueprintLib::BindAxisToKey(key_down_pitch_mapping_, actor_, this, &UManualPoseController::inputManualKeyDownPitch);
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
    if (useRecenterMode())
        return; //polled in updateActorPose instead; applying both would double-count

    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_rotation_.Add(0, val * CVarManualCamMouseSensitivity.GetValueOnGameThread(), 0);
}
void UManualPoseController::inputManualMousePitch(float val)
{
    if (useRecenterMode())
        return; //polled in updateActorPose instead; applying both would double-count

    if (!FMath::IsNearlyEqual(val, 0.f)) {
        const float sign = (CVarManualCamInvertY.GetValueOnGameThread() != 0) ? -1.f : 1.f;
        delta_rotation_.Add(val * CVarManualCamMouseSensitivity.GetValueOnGameThread() * sign, 0, 0);
    }
}
// IJKL look. Unlike the mouse these are a held state, so they are accumulated here and applied
// dt-scaled in updateActorPose - otherwise the turn rate would track the frame rate, which is the
// flaw the old A/D/W/S rotation had.
void UManualPoseController::inputManualKeyLeftYaw(float val)
{
    key_yaw_negative_ = val;
}
void UManualPoseController::inputManualKeyRightYaw(float val)
{
    key_yaw_positive_ = val;
}
void UManualPoseController::inputManualKeyUpPitch(float val)
{
    key_pitch_positive_ = val;
}
void UManualPoseController::inputManualKeyDownPitch(float val)
{
    key_pitch_negative_ = val;
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
