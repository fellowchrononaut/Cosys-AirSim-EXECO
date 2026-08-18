#include "RecordingThread.h"
#include "Async/TaskGraphInterfaces.h"
#include "HAL/RunnableThread.h"

#include <thread>
#include <mutex>
#include "RenderRequest.h"
#include "PIPCamera.h"
#include "AirBlueprintLib.h"

std::unique_ptr<FRecordingThread> FRecordingThread::running_instance_;
std::unique_ptr<FRecordingThread> FRecordingThread::finishing_instance_;
msr::airlib::WorkerThreadSignal FRecordingThread::finishing_signal_;
bool FRecordingThread::first_ = true;

FRecordingThread::FRecordingThread()
    : stop_task_counter_(0), recording_file_(nullptr), is_ready_(false)
{
    thread_.reset(FRunnableThread::Create(this, TEXT("FRecordingThread"), 0, TPri_BelowNormal)); // Windows default, possible to specify more priority
}

void FRecordingThread::startRecording(const RecordingSetting& settings,
                                      const common_utils::UniqueValueMap<std::string, VehicleSimApiBase*>& vehicle_sim_apis)
{
    stopRecording();

    //TODO: check FPlatformProcess::SupportsMultithreading()?
    assert(!isRecording());

    running_instance_.reset(new FRecordingThread());
    running_instance_->settings_ = settings;
    running_instance_->vehicle_sim_apis_ = vehicle_sim_apis;

    for (const auto& vehicle_sim_api : vehicle_sim_apis) {
        auto vehicle_name = vehicle_sim_api->getVehicleName();

        running_instance_->image_captures_[vehicle_name] = vehicle_sim_api->getImageCapture();
        running_instance_->last_poses_[vehicle_name] = msr::airlib::Pose();
    }

    running_instance_->last_screenshot_on_ = 0;

    running_instance_->recording_file_.reset(new RecordingFile());
    // Just need any 1 instance, to set the header line of the record file
    running_instance_->recording_file_->startRecording(*(vehicle_sim_apis.begin()), settings.folder);

    // Set is_ready at the end, setting this before can cause a race when the file isn't open yet
    running_instance_->is_ready_ = true;
}

FRecordingThread::~FRecordingThread()
{
    if (this == running_instance_.get()) stopRecording();
}

void FRecordingThread::init()
{
    first_ = true;
}

bool FRecordingThread::isRecording()
{
    return running_instance_ != nullptr;
}

void FRecordingThread::stopRecording()
{
    if (running_instance_) {
        assert(finishing_instance_ == nullptr);
        finishing_instance_ = std::move(running_instance_);
        assert(!isRecording());
        finishing_instance_->Stop();
    }
}

void FRecordingThread::killRecording()
{
    if (first_) return;

    stopRecording();
    bool finished = finishing_signal_.waitForRetry(1, 5);
    if (!finished) {
        UE_LOG(LogTemp, Log, TEXT("killing thread"));
        finishing_instance_->thread_->Kill(false);
    }
}

/*********************** methods for instance **************************************/

bool FRecordingThread::Init()
{
    if (first_) {
        first_ = false;
    }
    else {
        finishing_signal_.wait();
    }
    if (recording_file_) {
        UAirBlueprintLib::LogMessage(TEXT("Initiated recording thread"), TEXT(""), LogDebugLevel::Success);
    }
    return true;
}

uint32 FRecordingThread::Run()
{
    while (stop_task_counter_.GetValue() == 0) {
        //make sure all vars are set up
        if (is_ready_) {
            bool interval_elapsed = msr::airlib::ClockFactory::get()->elapsedSince(last_screenshot_on_) > settings_.record_interval;

            if (interval_elapsed) {
                last_screenshot_on_ = msr::airlib::ClockFactory::get()->nowNanos();

                for (const auto& vehicle_sim_api : vehicle_sim_apis_) {
                    const auto& vehicle_name = vehicle_sim_api->getVehicleName();
                    if (failed_vehicles_.count(vehicle_name))
                        continue;

                    const auto* kinematics = vehicle_sim_api->getGroundTruthKinematics();
                    bool is_pose_unequal = kinematics && last_poses_[vehicle_name] != kinematics->pose;

                    if (!settings_.record_on_move || is_pose_unequal) {
                        last_poses_[vehicle_name] = kinematics->pose;

                        std::vector<ImageCaptureBase::ImageResponse> responses;

                        // ⚠ CATCH HERE, and do not let anything escape this thread.
                        //
                        // This is an FRunnable, not a UE task: an exception leaving Run() finds no
                        // handler anywhere on the stack, so it reaches std::terminate and ABORTS
                        // THE WHOLE EDITOR. That is what happened on 2026-08-18 — a recording
                        // request named a camera the vehicle did not have, std::map::at threw
                        // std::out_of_range, and a bad camera name in a settings file took the
                        // simulator down with a SIGABRT and no usable message.
                        //
                        // Recording is a diagnostic, so it must never be able to destroy the run
                        // it is observing. A vehicle that cannot be captured is dropped for the
                        // rest of the session, once, with the reason; the others keep recording.
                        try {
                            image_captures_[vehicle_name]->getImages(settings_.requests[vehicle_name], responses);
                            recording_file_->appendRecord(responses, vehicle_sim_api);
                        }
                        catch (const std::exception& e) {
                            // ⚠ Logged once per vehicle, not per frame. At the default 0.05 s
                            // interval an unconditional log would emit 20 identical lines a second
                            // and bury whatever else the operator was reading.
                            if (failed_vehicles_.insert(vehicle_name).second) {
                                UAirBlueprintLib::LogMessageString(
                                    "recording DISABLED for vehicle '" + vehicle_name + "': ",
                                    e.what(), LogDebugLevel::Failure);
                            }
                        }
                    }
                }
            }
        }
    }

    recording_file_.reset();

    return 0;
}

void FRecordingThread::Stop()
{
    stop_task_counter_.Increment();
}

void FRecordingThread::Exit()
{
    assert(this == finishing_instance_.get());
    if (recording_file_)
        recording_file_.reset();
    finishing_signal_.signal();
}
