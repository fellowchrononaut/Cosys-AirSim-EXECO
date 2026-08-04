#include "AirSimCameraDirector.h"
#include "GameFramework/PlayerController.h"
#include "AirBlueprintLib.h"

AAirSimCameraDirector::AAirSimCameraDirector()
{
    PrimaryActorTick.bCanEverTick = true;

    // Create a spring arm component for our chase camera
    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    this->SetRootComponent(SpringArm);
    SpringArm->SetRelativeLocation(FVector(0.0f, 0.0f, 34.0f));
    SpringArm->SetWorldRotation(FRotator(-20.0f, 0.0f, 0.0f));
    SpringArm->TargetArmLength = 125.0f;
    SpringArm->bEnableCameraLag = false;
    SpringArm->bEnableCameraRotationLag = false;
    SpringArm->CameraRotationLagSpeed = 10.0f;
    SpringArm->bInheritPitch = true;
    SpringArm->bInheritYaw = true;
    SpringArm->bInheritRoll = true;
}

void AAirSimCameraDirector::BeginPlay()
{
    Super::BeginPlay();
}

void AAirSimCameraDirector::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL) {
        manual_pose_controller_->updateActorPose(DeltaTime);
    }
    else if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE) {
        //do nothing, spring arm is pulling the camera with it
    }
    else if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY) {
        //do nothing, we have camera turned off
    }
    else { //make camera move in desired way
        UAirBlueprintLib::FollowActor(ExternalCamera, follow_actor_, initial_ground_obs_offset_, ext_obs_fixed_z_);
    }
}

ECameraDirectorMode AAirSimCameraDirector::getMode()
{
    return mode_;
}

// ---------------------------------------------------------------------------------------------
// U-9: vehicle and camera cycling.
//
// One global "current vehicle" shared by every mode, so switching mode never moves you to a
// different vehicle. A mode key enters the mode; pressing it again while already in that mode
// advances to the next vehicle. Shift+key advances the camera within the current vehicle, which
// only means anything in the modes that look through a vehicle camera.
// ---------------------------------------------------------------------------------------------

void AAirSimCameraDirector::registerVehicle(AActor* pawn, const FString& vehicle_name,
                                            const TArray<APIPCamera*>& cameras, const TArray<FString>& camera_names)
{
    if (pawn == nullptr)
        return;

    FDirectorVehicle entry;
    entry.pawn = pawn;
    entry.name = vehicle_name;
    entry.cameras = cameras;
    entry.camera_names = camera_names;
    vehicles_.Add(MoveTemp(entry));
}

bool AAirSimCameraDirector::isShiftHeld() const
{
    // Read the modifier rather than binding a second Shift+key action. Two action mappings that
    // differ only by modifier rely on UE's chord masking to not both fire; reading the key state
    // is deterministic and needs one binding per mode instead of two.
    const APlayerController* pc = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
    if (pc == nullptr)
        return false;

    return pc->IsInputKeyDown(EKeys::LeftShift) || pc->IsInputKeyDown(EKeys::RightShift);
}

bool AAirSimCameraDirector::modeUsesVehicleCamera(ECameraDirectorMode mode) const
{
    return mode == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV ||
           mode == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_BACKUP ||
           mode == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FRONT;
}

APIPCamera* AAirSimCameraDirector::currentCamera() const
{
    if (!vehicles_.IsValidIndex(current_vehicle_))
        return nullptr;

    const FDirectorVehicle& vehicle = vehicles_[current_vehicle_];
    return vehicle.cameras.IsValidIndex(current_camera_) ? vehicle.cameras[current_camera_] : nullptr;
}

void AAirSimCameraDirector::announceSelection(ECameraDirectorMode mode) const
{
    if (!vehicles_.IsValidIndex(current_vehicle_))
        return;

    const FDirectorVehicle& vehicle = vehicles_[current_vehicle_];

    FString message = FString::Printf(TEXT("%s  [vehicle %d/%d]"),
                                      *vehicle.name, current_vehicle_ + 1, vehicles_.Num());

    if (modeUsesVehicleCamera(mode)) {
        const FString camera_name = vehicle.camera_names.IsValidIndex(current_camera_)
                                        ? vehicle.camera_names[current_camera_]
                                        : TEXT("<none>");
        message += FString::Printf(TEXT("  camera %s  [%d/%d]"),
                                   *camera_name, current_camera_ + 1, vehicle.cameras.Num());
    }

    //5s rather than the 60s default: this is a transient confirmation, not a status line
    UAirBlueprintLib::LogMessageString("View: ", TCHAR_TO_UTF8(*message), LogDebugLevel::Informational, 5);
    UE_LOG(LogTemp, Log, TEXT("CameraDirector view: %s"), *message);
}

void AAirSimCameraDirector::selectCameraByName(const FString& preferred)
{
    if (!vehicles_.IsValidIndex(current_vehicle_))
        return;

    const FDirectorVehicle& vehicle = vehicles_[current_vehicle_];
    const int32 found = vehicle.camera_names.IndexOfByKey(preferred);
    if (found != INDEX_NONE)
        current_camera_ = found;
    else if (!vehicle.cameras.IsValidIndex(current_camera_))
        current_camera_ = 0; //this vehicle has no camera by that name; fall back to its first
}

// Each camera mode has a default viewpoint. Entering the mode, or arriving at a new vehicle, snaps
// to it; Shift+key then walks away from it through the rest of the vehicle's cameras.
void AAirSimCameraDirector::selectPreferredCamera(ECameraDirectorMode mode)
{
    switch (mode) {
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV:
        selectCameraByName(TEXT("fpv"));
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_BACKUP:
        selectCameraByName(TEXT("back_center"));
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FRONT:
        selectCameraByName(TEXT("front_center"));
        break;
    default:
        //external-camera mode: no vehicle camera in use, just keep the index valid
        if (vehicles_.IsValidIndex(current_vehicle_) &&
            !vehicles_[current_vehicle_].cameras.IsValidIndex(current_camera_))
            current_camera_ = 0;
        break;
    }
}

void AAirSimCameraDirector::advanceVehicle()
{
    if (vehicles_.Num() < 2)
        return;

    if (APIPCamera* outgoing = currentCamera())
        outgoing->disableMain();

    current_vehicle_ = (current_vehicle_ + 1) % vehicles_.Num();
}

void AAirSimCameraDirector::advanceCamera()
{
    if (!vehicles_.IsValidIndex(current_vehicle_))
        return;

    const FDirectorVehicle& vehicle = vehicles_[current_vehicle_];
    if (vehicle.cameras.Num() < 2)
        return;

    // Once cycling has moved off a mode's default camera, the outgoing one is no longer in any of
    // the three tracked slots, so disableCameras() will not turn it off. Do it explicitly.
    if (APIPCamera* outgoing = currentCamera())
        outgoing->disableMain();

    current_camera_ = (current_camera_ + 1) % vehicle.cameras.Num();
}

// Point whichever camera slot the target mode reads at the current selection, so cycling reuses the
// existing show/disable machinery in the input handlers rather than duplicating it.
void AAirSimCameraDirector::applySelection(ECameraDirectorMode target_mode)
{
    if (!vehicles_.IsValidIndex(current_vehicle_))
        return;

    APIPCamera* selected = currentCamera();
    follow_actor_ = vehicles_[current_vehicle_].pawn;

    switch (target_mode) {
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV:
        fpv_camera_ = selected;
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_BACKUP:
        backup_camera_ = selected;
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FRONT:
        front_camera_ = selected;
        break;
    default:
        break; //external-camera modes follow follow_actor_, no slot to set
    }

    // Staying in SpringArmChase while changing vehicle: setMode() will not re-run the attach,
    // because neither its detach condition (mode changed) nor the attach guard (camera not already
    // on the arm) is met. Re-parent explicitly or the arm keeps hanging off the old vehicle.
    if (target_mode == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE &&
        mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE) {
        attachSpringArm(false);
        attachSpringArm(true);
    }
}

void AAirSimCameraDirector::handleModeKey(ECameraDirectorMode mode)
{
    if (vehicles_.Num() == 0)
        return;

    const bool cycling = !suppress_cycle_;

    if (cycling && isShiftHeld()) {
        if (modeUsesVehicleCamera(mode))
            advanceCamera();
        //else: external-camera mode has no vehicle cameras to cycle
    }
    else if (cycling && mode_ == mode) {
        advanceVehicle();
        selectPreferredCamera(mode); //a new vehicle starts at this mode's default viewpoint
    }
    else {
        selectPreferredCamera(mode); //entering the mode, or initial setup
    }

    applySelection(mode);
    announceSelection(mode);
}

void AAirSimCameraDirector::initializeForBeginPlay(ECameraDirectorMode view_mode,
                                             AActor* follow_actor, APIPCamera* fpv_camera, APIPCamera* front_camera, APIPCamera* back_camera)
{
    manual_pose_controller_ = NewObject<UManualPoseController>(this, "CameraDirector_ManualPoseController");
    manual_pose_controller_->initializeForPlay();

    setupInputBindings();

    mode_ = view_mode;

    follow_actor_ = follow_actor;
    fpv_camera_ = fpv_camera;
    front_camera_ = front_camera;
    backup_camera_ = back_camera;
    camera_start_location_ = ExternalCamera->GetActorLocation();
    camera_start_rotation_ = ExternalCamera->GetActorRotation();
    initial_ground_obs_offset_ = camera_start_location_ -
                                 (follow_actor_ ? follow_actor_->GetActorLocation() : FVector::ZeroVector);

    // U-9: start the cycle on whichever vehicle was chosen as the default, so the first press of a
    // mode key advances from what is actually on screen rather than jumping to vehicle 1.
    current_vehicle_ = 0;
    current_camera_ = 0;
    if (follow_actor_ != nullptr) {
        const int32 found = vehicles_.IndexOfByPredicate(
            [this](const FDirectorVehicle& v) { return v.pawn == follow_actor_; });
        if (found != INDEX_NONE)
            current_vehicle_ = found;
    }

    //set initial view mode. suppress_cycle_ keeps these calls from reading as keypresses.
    TGuardValue<bool> no_cycle(suppress_cycle_, true);

    switch (mode_) {
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME:
        inputEventFlyWithView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV:
        inputEventFpvView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER:
        inputEventGroundView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL:
        inputEventManualView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE:
        inputEventSpringArmChaseView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_BACKUP:
        inputEventBackupView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY:
        inputEventNoDisplayView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FRONT:
        inputEventFrontView();
        break;
    default:
        throw std::out_of_range("Unsupported view mode specified in CameraDirector::initializeForBeginPlay");
    }
}

void AAirSimCameraDirector::attachSpringArm(bool attach)
{
    if (attach) {
        //If we do have actor to follow AND don't have sprint arm attached to that actor, we will attach it
        if (follow_actor_ && ExternalCamera->GetRootComponent()->GetAttachParent() != SpringArm) {
            //For car, we want a bit of camera lag, as that is customary of racing video games
            //If the lag is missing, the camera will also occasionally shake.
            //But, lag is not desired when piloting a drone
            SpringArm->bEnableCameraRotationLag = camera_rotation_lag_enabled_;

            //attach spring arm to actor
            SpringArm->AttachToComponent(follow_actor_->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
            SpringArm->SetRelativeLocation(FVector(0.0f, 0.0f, 34.0f));

            // Reset the arm's orientation relative to the new parent, not just its position.
            // Detaching uses KeepWorldTransform (so the free camera stays put when leaving this
            // mode, U-7b), which bakes the old parent's world rotation into the arm's RELATIVE
            // rotation. Re-attaching with KeepRelativeTransform then carries that stale attitude
            // onto the new vehicle: after chasing a pitched-up drone, switching to the ground robot
            // left the arm still pitched up, putting the camera under the floor looking upward.
            // -20 pitch matches the value the constructor gives the arm while it is unparented.
            SpringArm->SetRelativeRotation(FRotator(-20.0f, 0.0f, 0.0f));

            //remember current parent for external camera. Later when we remove external
            //camera from spring arm, we will attach it back to its last parent
            last_parent_ = ExternalCamera->GetRootComponent()->GetAttachParent();
            ExternalCamera->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
            //now attach camera to spring arm
            ExternalCamera->AttachToComponent(SpringArm, FAttachmentTransformRules::KeepRelativeTransform);
        }

        //For car, we need to move the camera back a little more than for a drone.
        //Otherwise, the camera will be stuck inside the car
        ExternalCamera->SetActorRelativeLocation(FVector(follow_distance_ * 100.0f, 0.0f, 0.0f));
        ExternalCamera->SetActorRelativeRotation(FRotator(10.0f, 0.0f, 0.0f));
        //ExternalCamera->bUsePawnControlRotation = false;
    }
    else { //detach
        // The detach used to be gated on `last_parent_` being non-null, which made it dead code:
        // ExternalCamera is spawned standalone (SimModeBase::initializeCameraDirector) so it has no
        // attach parent, and `last_parent_` above therefore captures nullptr. Once SpringArmChase
        // had been entered - which is the DEFAULT startup mode for every non-multirotor sim, see
        // AirSimSettings::loadViewModeSettings - the camera stayed parented to the spring arm, and
        // so to the followed vehicle, for the rest of the session. In Manual mode that meant the
        // free camera was still dragged around by vehicle 1: you could move it, but it kept the
        // offset you gave it and travelled with the vehicle.
        //
        // Detach whenever we are attached to the spring arm; re-parent only if there was a real
        // parent to go back to. KeepWorldTransform so the camera stays where the user last put it
        // instead of snapping.
        if (ExternalCamera->GetRootComponent()->GetAttachParent() == SpringArm) {
            ExternalCamera->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
            if (last_parent_)
                ExternalCamera->AttachToComponent(last_parent_, FAttachmentTransformRules::KeepRelativeTransform);
        }

        // The spring arm is this actor's root and stays attached to the followed vehicle otherwise,
        // which keeps dragging the director around and would misdirect U-9 vehicle cycling.
        if (GetAttachParentActor() != nullptr)
            DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
    }
}

void AAirSimCameraDirector::setMode(ECameraDirectorMode mode)
{
    { //first remove any settings done by previous mode

        //detach spring arm
        if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE &&
            mode != ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE) {
            attachSpringArm(false);
        }

        // Re-enable rendering
        if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY &&
            mode != ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY) {
            UAirBlueprintLib::enableViewportRendering(this, true);
        }

        //Remove any existing key bindings for manual mode
        if (mode != ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL) {
            if (ExternalCamera != nullptr && manual_pose_controller_->getActor() == ExternalCamera) {

                manual_pose_controller_->setActor(nullptr);
            }
            //else someone else is bound to manual pose controller, leave it alone
        }
    }

    { //perform any settings to enter in to this mode

        switch (mode) {
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL:
            //if new mode is manual mode then add key bindings
            manual_pose_controller_->setActor(ExternalCamera);
            break;
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE:
            //if we switched to spring arm mode then attach to spring arm (detachment was done earlier in method)
            attachSpringArm(true);
            break;
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY:
            UAirBlueprintLib::enableViewportRendering(this, false);
            break;
        default:
            //other modes don't need special setup
            break;
        }
    }

    //make switch official
    mode_ = mode;
}

void AAirSimCameraDirector::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("inputEventFpvView", EKeys::F, this, &AAirSimCameraDirector::inputEventFpvView);
    UAirBlueprintLib::BindActionToKey("inputEventFlyWithView", EKeys::B, this, &AAirSimCameraDirector::inputEventFlyWithView);
    UAirBlueprintLib::BindActionToKey("inputEventGroundView", EKeys::Backslash, this, &AAirSimCameraDirector::inputEventGroundView);
    UAirBlueprintLib::BindActionToKey("inputEventManualView", EKeys::M, this, &AAirSimCameraDirector::inputEventManualView);
    // Was EKeys::Slash. Moved to G because '/' is relocated or a modifier combination on non-QWERTY
    // layouts (U-10). G is free: no other EKeys::G in the C++ source, and F10 is the only input-key
    // event in any Blueprint asset under the plugin or Blocks content.
    UAirBlueprintLib::BindActionToKey("inputEventSpringArmChaseView", EKeys::G, this, &AAirSimCameraDirector::inputEventSpringArmChaseView);
    // Was EKeys::K, moved to free K for Manual-mode camera look (IJKL). The director's bindings are
    // always active, so K would have switched view mode mid-look.
    UAirBlueprintLib::BindActionToKey("inputEventBackupView", EKeys::C, this, &AAirSimCameraDirector::inputEventBackupView);
    UAirBlueprintLib::BindActionToKey("inputEventNoDisplayView", EKeys::Hyphen, this, &AAirSimCameraDirector::inputEventNoDisplayView);
    // Was EKeys::I, moved to free I for Manual-mode camera look (IJKL). Same reason as Backup above.
    UAirBlueprintLib::BindActionToKey("inputEventFrontView", EKeys::V, this, &AAirSimCameraDirector::inputEventFrontView);
}

void AAirSimCameraDirector::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    vehicles_.Empty();
    manual_pose_controller_ = nullptr;
    SpringArm = nullptr;
    ExternalCamera = nullptr;
    fpv_camera_ = nullptr;
    backup_camera_ = nullptr;
    front_camera_ = nullptr;
    follow_actor_ = nullptr;

    Super::EndPlay(EndPlayReason);
}

APIPCamera* AAirSimCameraDirector::getFpvCamera() const
{
    return fpv_camera_;
}

APIPCamera* AAirSimCameraDirector::getExternalCamera() const
{
    return ExternalCamera;
}

APIPCamera* AAirSimCameraDirector::getBackupCamera() const
{
    return backup_camera_;
}

APIPCamera* AAirSimCameraDirector::getFrontCamera() const
{
    return front_camera_;
}

void AAirSimCameraDirector::inputEventSpringArmChaseView()
{
    handleModeKey(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE);

    if (ExternalCamera) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE);
        ExternalCamera->showToScreen();
        disableCameras(true, true, true);
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "ExternalCamera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventGroundView()
{
    handleModeKey(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER);

    if (ExternalCamera) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER);
        ExternalCamera->showToScreen();
        disableCameras(true, true, true);
        ext_obs_fixed_z_ = true;
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "ExternalCamera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventManualView()
{
    handleModeKey(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL);

    if (ExternalCamera) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL);
        ExternalCamera->showToScreen();
        disableCameras(true, true, true);
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "ExternalCamera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventNoDisplayView()
{
    handleModeKey(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY);

    if (ExternalCamera) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY);
        disableCameras(true, true, true);
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "ExternalCamera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventBackupView()
{
    handleModeKey(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_BACKUP);

    if (backup_camera_) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_BACKUP);
        backup_camera_->showToScreen();
        disableCameras(true, false, true);
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "backup_camera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventFrontView()
{
    handleModeKey(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FRONT);

    if (front_camera_) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FRONT);
        front_camera_->showToScreen();
        disableCameras(true, true, false);
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "backup_camera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventFlyWithView()
{
    handleModeKey(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME);

    if (ExternalCamera) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME);
        ExternalCamera->showToScreen();

        if (follow_actor_)
            ExternalCamera->SetActorLocationAndRotation(
                follow_actor_->GetActorLocation() + initial_ground_obs_offset_, camera_start_rotation_);
        disableCameras(true, true, true);
        ext_obs_fixed_z_ = false;
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "ExternalCamera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventFpvView()
{
    handleModeKey(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV);

    if (fpv_camera_) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV);
        fpv_camera_->showToScreen();
        disableCameras(false, true, true);
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "fpv_camera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::disableCameras(bool fpv, bool backup, bool front)
{
    if (fpv && fpv_camera_)
        fpv_camera_->disableMain();
    if (backup && backup_camera_)
        backup_camera_->disableMain();
    if (front && front_camera_)
        front_camera_->disableMain();
}

void AAirSimCameraDirector::notifyViewModeChanged()
{
    bool nodisplay = ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY == mode_;

    UWorld* world = GetWorld();
    UGameViewportClient* gameViewport = world->GetGameViewport();
    gameViewport->bDisableWorldRendering = nodisplay;
}