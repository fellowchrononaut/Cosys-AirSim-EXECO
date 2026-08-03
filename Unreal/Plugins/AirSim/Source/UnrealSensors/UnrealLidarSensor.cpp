// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealLidarSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "Async/ParallelFor.h"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "HAL/IConsoleManager.h"
#include "common/ClockFactory.hpp"
#include "PhysicsTiming.h"
#include <random>

/** Shared with LidarCamera.cpp (GPU LiDAR, SensorType 8); this is the raycast LiDAR
 *  (SensorType 6). Both accumulate a revolution across several ticks and hand it out with a single
 *  timestamp and pose, so the oldest points carry a vehicle frame that is up to one revolution
 *  stale. Declared extern here because the CVar object itself lives in LidarCamera.cpp. */
extern TAutoConsoleVariable<int32> CVarLogLidarSweep;

/** Resolves LidarDeskew against its debug override (airsim.LidarDeskew). Defined in
 *  LidarCamera.cpp so both LiDAR sensors share one decision. */
extern bool ShouldDeskewLidar();

// ctor
UnrealLidarSensor::UnrealLidarSensor(const AirSimSettings::LidarSetting& setting,
	AActor* actor, const NedTransform* ned_transform)
	: LidarSimple(setting), actor_(actor), ned_transform_(ned_transform),
	sensor_params_(getParams()),
	draw_time_(1.05f / sensor_params_.horizontal_rotation_frequency),
	external_(getParams().external)
{
	// Seed and initiate noise
	std::random_device rd;
	gen_ = std::mt19937(rd());
	dist_ = std::normal_distribution<float>(0, getParams().min_noise_standard_deviation);
	point_cloud_draw_.clear();
	createLasers();
}

// initializes information based on lidar configuration
void UnrealLidarSensor::createLasers()
{


	msr::airlib::LidarSimpleParams params = getParams();

	const auto number_of_lasers = params.number_of_channels;

	// Fencepost: dividing the span by (measurement_per_cycle - 1) makes the table INCLUSIVE of both
	// endpoints, which is right for a bounded sector (-45..45 should sample both -45 and +45) but
	// wrong for a full circle, where the two endpoints are the same bearing. With the common
	// -180..180 setting that produced angles[0] = -180 and angles[511] = +180: 512 columns but only
	// 511 distinct azimuths, one bearing sampled twice per revolution (64 redundant points in a
	// 64-channel cloud), and a true resolution of 360/511 = 0.7045 deg rather than the 0.7031 deg
	// that "512 per circle" implies.
	//
	// Divide by measurement_per_cycle when the span is a full circle so the last column stops one
	// step short of wrapping onto the first.
	const float horizontal_span = params.horizontal_FOV_end - params.horizontal_FOV_start;
	const bool spans_full_circle = FMath::IsNearlyEqual(FMath::Abs(horizontal_span), 360.0f, 0.001f);
	// measurement_per_cycle is unsigned, so compute the -1 branch with a comparison rather than
	// FMath::Max, which would see an underflowed huge value instead of clamping it.
	const uint32 horizontal_divisions = (spans_full_circle || params.measurement_per_cycle <= 1)
		                                    ? FMath::Max(params.measurement_per_cycle, 1u)
		                                    : params.measurement_per_cycle - 1;
	const float horizontal_delta = horizontal_span / float(horizontal_divisions);
	for (uint32 i = 0; i < params.measurement_per_cycle; i++) {
		horizontal_angles_.Add(params.horizontal_FOV_start + i * horizontal_delta);
	}

	// The azimuth table is otherwise invisible, so the I-O fencepost could only be argued about,
	// never measured. Report the endpoints and the distinct-bearing count. On a full circle, first
	// and last must differ by one delta and distinct must equal measurement_per_cycle; before the
	// fix last == first + 360 and distinct came out one short.
	//
	// Deliberately NOT gated on airsim.LogLidarSweep: createLasers() runs from the constructor,
	// before any console variable could have been set, so gating it would guarantee it never
	// prints. One line per LiDAR sensor at startup.
	if (horizontal_angles_.Num() > 1) {
		TSet<int32> distinct_bearings;
		for (float a : horizontal_angles_)
			distinct_bearings.Add(FMath::RoundToInt(FMath::Fmod(FMath::Fmod(a, 360.0f) + 360.0f, 360.0f) * 1000.0f));
		// Include the actor: sensor names are not unique across vehicles (both vehicles here call
		// their LiDAR 'CPULidar'), so a name-only line cannot be attributed to a vehicle.
		UE_LOG(LogTemp, Log,
			   TEXT("[AirSim] raycast lidar '%s' on '%s' azimuth table: %d columns, %d distinct bearings, delta %.6f deg, first %.4f, last %.4f, full_circle %d"),
			   *FString(getName().c_str()),
			   actor_ ? *actor_->GetName() : TEXT("?"),
			   horizontal_angles_.Num(),
			   distinct_bearings.Num(),
			   horizontal_delta,
			   horizontal_angles_[0],
			   horizontal_angles_.Last(),
			   spans_full_circle ? 1 : 0);
	}

	if (number_of_lasers <= 0)
		return;

	// calculate verticle angle distance between each laser
	float delta_angle = 0;
	if (number_of_lasers > 1)
		delta_angle = (params.vertical_FOV_upper - (params.vertical_FOV_lower)) /
		static_cast<float>(number_of_lasers - 1);

	// store vertical angles for each laser
	laser_angles_.clear();
	for (auto i = 0u; i < number_of_lasers; ++i)
	{
		const float vertical_angle = params.vertical_FOV_upper - static_cast<float>(i) * delta_angle;
		laser_angles_.emplace_back(vertical_angle);
	}

	current_horizontal_angle_index_ = horizontal_angles_.Num()-1;
}

// I-R Phase 0. This runs once per World::update() per sensor, BEFORE the FrequencyLimiter decides
// whether any raycasting happens, so the interval between calls is the true physics loop period.
// The busy figure covers the whole update, so it is ~0 on limiter-suppressed iterations and jumps
// to the raycast burst cost on the ones that do work - which is exactly the duty cycle we want.
void UnrealLidarSensor::update(float delta)
{
	const int32 period = AirSimPhysicsTiming::ReportPeriodSeconds();
	if (period <= 0) {
		LidarSimple::update(delta);
		return;
	}

	const auto t0 = AirSimPhysicsTiming::Clock::now();
	loop_window_.noteEntry(t0);
	LidarSimple::update(delta);
	const auto now = AirSimPhysicsTiming::Clock::now();
	loop_window_.noteBusy(AirSimPhysicsTiming::ToMs(now - t0));

	if (loop_window_.shouldReport(now, period)) {
		const double avg_gap  = loop_window_.gap_ms / FMath::Max<uint64>(loop_window_.calls - 1, 1);
		const double avg_busy = loop_window_.busy_ms / loop_window_.calls;
		// Duty = share of wall time this sensor's update consumes on the physics thread. This is
		// the number that should match the real-time deficit (1 - sim/wall).
		const double duty = 100.0 * loop_window_.busy_ms
		                    / FMath::Max(loop_window_.gap_ms, KINDA_SMALL_NUMBER);
		UE_LOG(LogTemp, Log,
			   TEXT("[AirSim][timing] PHYSICS LOOP via '%s' on '%s': period %.2f ms avg (max %.2f) | update() %.3f ms avg (max %.2f) | DUTY %.1f%% of wall | n=%llu"),
			   *FString(getName().c_str()),
			   actor_ ? *actor_->GetName() : TEXT("?"),
			   avg_gap, loop_window_.gap_max, avg_busy, loop_window_.busy_max, duty,
			   loop_window_.calls);
		loop_window_.reset();
	}
}

// Set echo object in correct pose in physical world
void UnrealLidarSensor::updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose)
{
	sensor_reference_frame_ = VectorMath::add(sensor_pose, vehicle_pose);
	// DRAW DEBUG
	if (sensor_params_.draw_sensor) {
		FVector sensor_position;
		if (external_) {
			sensor_position = ned_transform_->toFVector(sensor_reference_frame_.position, 100, true);
		}
		else {
			sensor_position = ned_transform_->fromLocalNed(sensor_reference_frame_.position);
		}
		UAirBlueprintLib::DrawPoint(actor_->GetWorld(), sensor_position, 5, FColor::Black, false, draw_time_);
		FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1));
		UAirBlueprintLib::DrawCoordinateSystem(actor_->GetWorld(), sensor_position, sensor_direction.Rotation(), 25, false, draw_time_, 10);
	}
}

// Get echo pose in Local NED
void UnrealLidarSensor::getLocalPose(msr::airlib::Pose& sensor_pose)
{
	FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1)); ;
	sensor_pose = ned_transform_->toLocalNed(FTransform(sensor_direction.Rotation(), ned_transform_->toFVector(sensor_reference_frame_.position, 100, true), FVector(1, 1, 1)));
}

// Pause Unreal simulation
void UnrealLidarSensor::pause(const bool is_paused) {
	if (is_paused) {
		saved_clockspeed_ = UAirBlueprintLib::getUnrealClockSpeed(actor_);
		UAirBlueprintLib::setUnrealClockSpeed(actor_, 0);
	}
	else {
		UAirBlueprintLib::setUnrealClockSpeed(actor_, saved_clockspeed_);
	}
}

// returns a point-cloud for the tick
bool UnrealLidarSensor::getPointCloud(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
	const msr::airlib::TTimeDelta delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<std::string>& groundtruth, msr::airlib::vector<msr::airlib::real_T>& point_cloud_final, msr::airlib::vector<std::string>& groundtruth_final)
{

	// I-R Phase 0: cost of one raycast BURST.
	//
	// ⚠ The interval reported here is NOT the physics loop period. getPointCloud runs from
	// updateOutput(), which the FrequencyLimiter gates, so this interval is the UpdateFrequency
	// period. The first version of this probe claimed otherwise and its "% of the loop" figure was
	// meaningless as a result. For the real loop period and duty cycle, see the PHYSICS LOOP line
	// emitted by update() above.
	//
	// What this line is good for: how long the physics thread is blocked in one uninterruptible
	// stretch. That stall is what the game thread waits out on physics_world_->lock().
	const int32 timing_period = AirSimPhysicsTiming::ReportPeriodSeconds();
	const bool timing_on = timing_period > 0;
	const auto timing_t0 = AirSimPhysicsTiming::Clock::now();
	if (timing_on) timing_window_.noteEntry(timing_t0);
	struct TimingExit {
		UnrealLidarSensor* self; bool on; AirSimPhysicsTiming::Clock::time_point t0; int32 period;
		~TimingExit() {
			if (!on) return;
			const auto now = AirSimPhysicsTiming::Clock::now();
			self->timing_window_.noteBusy(AirSimPhysicsTiming::ToMs(now - t0));
			auto& w = self->timing_window_;
			if (w.shouldReport(now, period)) {
				const double avg_busy = w.busy_ms / w.calls;
				const double avg_gap  = w.gap_ms / FMath::Max<uint64>(w.calls - 1, 1);
				UE_LOG(LogTemp, Log,
					   TEXT("[AirSim][timing] RAYCAST BURST '%s' on '%s': getPointCloud %.2f ms avg (max %.2f) | every %.2f ms (limiter-gated, NOT the loop period) | n=%llu"),
					   *FString(self->getName().c_str()),
					   self->actor_ ? *self->actor_->GetName() : TEXT("?"),
					   avg_busy, w.busy_max, avg_gap, w.calls);
				w.reset();
			}
		}
	} timing_exit{ this, timing_on, timing_t0, timing_period };

	updatePose(lidar_pose, vehicle_pose);

	// airsim.LogLidarSweep: mark the first tick contributing to a new revolution.
	if (sweep_tick_count_ == 0) {
		sweep_start_time_ = msr::airlib::ClockFactory::get()->nowNanos();
	}
	sweep_tick_count_++;

	bool refresh = false;
	msr::airlib::LidarSimpleParams params = getParams();
	const auto number_of_lasers = params.number_of_channels;
	uint32 total_points = number_of_lasers * params.measurement_per_cycle;

	if (point_cloud.size() == 0)
	{
		point_cloud.assign(total_points * 3, 0);
		groundtruth.assign(total_points, "out_of_range");
	}

	// calculate needed angle/distance between each point
	const float angle_distance_of_tick = params.horizontal_rotation_frequency * 360.0f * delta_time;
	const double angle_distance_of_laser_measure = 360.0f / params.measurement_per_cycle;

	// calculate number of points needed for each laser/channel
	uint32 points_to_scan_with_one_laser_temp = FMath::RoundHalfFromZero(angle_distance_of_tick / angle_distance_of_laser_measure);
	if (points_to_scan_with_one_laser_temp <= 0)
	{
		//UAirBlueprintLib::LogMessageString("Lidar: ", "No points requested this frame", LogDebugLevel::Failure);
		return refresh;
	}
	// Cap a tick at exactly ONE full revolution, derived from the sensor's own configuration
	// rather than a fixed budget.
	//
	// This used to be a hardcoded `MAX_POINTS_IN_SCAN = 5000`, which is unrelated to how the
	// sensor is configured and therefore acted as a RATE LIMITER: with 512 x 64 = 32768 points per
	// revolution it allowed only 5000/64 = 78 azimuth steps per tick, so a sweep needed 7 ticks and
	// RotationsPerSecond was silently not honoured (measured: a nominal 200 ms revolution took
	// ~612 ms). Now it is an OVERRUN GUARD: the configured rate is respected, and the only thing
	// prevented is a single tick producing more than one revolution.
	//
	// That matters because the completion block below overwrites point_cloud_final. Without this
	// cap a fast sensor (e.g. 20 rev/s at ~87 ms ticks = 1.74 revolutions per tick) completes two
	// sweeps in one call and the first is silently discarded. Paired with the `break` after a
	// completed sweep, a call now yields exactly one revolution and leftover azimuth carries over.
	const uint32 max_points_full_sweep = params.measurement_per_cycle * number_of_lasers;
	if (params.limit_points && points_to_scan_with_one_laser_temp * number_of_lasers > max_points_full_sweep)
	{
		points_to_scan_with_one_laser_temp = max_points_full_sweep / number_of_lasers;
	}
	const uint32 points_to_scan_with_one_laser = points_to_scan_with_one_laser_temp;

	// normalize FOV start/end
	float laser_start = std::fmod(360.0f + params.horizontal_FOV_start, 360.0f);
	float laser_end = std::fmod(360.0f + params.horizontal_FOV_end, 360.0f);

	float previous_horizontal_angle = horizontal_angles_[current_horizontal_angle_index_];

	if (sensor_params_.draw_debug_points) {
		point_cloud_draw_.clear();
		point_cloud_draw_.assign(points_to_scan_with_one_laser * number_of_lasers, FVector());
	}

	// shoot lasers
	for (uint32 i = 1; i <= points_to_scan_with_one_laser; ++i)
	{
		if (current_horizontal_angle_index_ == horizontal_angles_.Num() - 1) {
			current_horizontal_angle_index_ = 0;
		}
		else {
			current_horizontal_angle_index_ += 1;
		}

		float horizontal_angle = horizontal_angles_[current_horizontal_angle_index_];
		//UE_LOG(LogTemp, Display, TEXT("horizontal_angle: %f "), horizontal_angle);


		if ((previous_horizontal_angle > horizontal_angle) && (point_cloud.size() != 0)) {
			if ((((int)point_cloud.size() / 3) != params.measurement_per_cycle * number_of_lasers) || (groundtruth.size() != params.measurement_per_cycle * number_of_lasers))
			{
				UE_LOG(LogTemp, Warning, TEXT("Pointcloud or labels incorrect size! points:%i labels:%i"), (int)(point_cloud.size() / 3), groundtruth.size());
			}
			//UE_LOG(LogTemp, Display, TEXT("Pointcloud completed! points:%i labels:%i"), (int)(point_cloud.size() / 3), groundtruth.size());

			// A full revolution just completed. LidarSimple::updateOutput will stamp the whole
			// cloud with one clock()->nowNanos() and one pose - but its points were measured in
			// the vehicle frame as it was on each contributing tick. The span below is how much
			// vehicle motion the cloud straddles, i.e. how stale the oldest points' frame is.
			if (CVarLogLidarSweep.GetValueOnGameThread() != 0) {
				const uint64 now_ns = msr::airlib::ClockFactory::get()->nowNanos();
				// sweep_tick_count_ is reset on each completion, so a SECOND sweep finishing inside
				// the same getPointCloud call sees 0 - which reads as "0 ticks" and is impossible.
				// Report at least 1 and flag the case explicitly: it means this tick produced more
				// than a full revolution of azimuth, which is the good outcome, not an error.
				UE_LOG(LogTemp, Log,
					   TEXT("[AirSim] raycast lidar '%s' on '%s' sweep complete: span %.1f ms over %d tick(s)%s, %d points; oldest points use a vehicle frame ~that old"),
					   *FString(getName().c_str()),
					   actor_ ? *actor_->GetName() : TEXT("?"),
					   (double)((int64)now_ns - (int64)sweep_start_time_) * 1e-6,
					   FMath::Max(sweep_tick_count_, 1),
					   sweep_tick_count_ == 0 ? TEXT(" [extra sweep completed within the same tick]") : TEXT(""),
					   (int32)(point_cloud.size() / 3));
			}
			sweep_tick_count_ = 0;

			// De-skew: the accumulator holds world-frame points measured across several ticks.
			// Re-express them all in the lidar frame as it is NOW - the instant this cloud's
			// timestamp and pose refer to - so the cloud is internally consistent.
			// Misses were pre-filled as exact zeros and labelled "out_of_range"; leave those alone,
			// since transforming (0,0,0) would place a phantom return at the sensor offset.
			if (ShouldDeskewLidar()) {
				const msr::airlib::Pose sweep_end_pose = lidar_pose + vehicle_pose;
				const size_t point_count = point_cloud.size() / 3;
				for (size_t p = 0; p < point_count; ++p) {
					if (p < groundtruth.size() && groundtruth[p] == "out_of_range") {
						continue;
					}
					Vector3r world_point(point_cloud[p * 3], point_cloud[p * 3 + 1], point_cloud[p * 3 + 2]);
					const Vector3r body_point = VectorMath::transformToBodyFrame(world_point, sweep_end_pose, true);
					point_cloud[p * 3] = body_point.x();
					point_cloud[p * 3 + 1] = body_point.y();
					point_cloud[p * 3 + 2] = body_point.z();
				}
			}

			point_cloud_final = point_cloud;
			groundtruth_final = groundtruth;
			point_cloud.clear();
			groundtruth.clear();
			point_cloud.assign(total_points * 3, 0);
			groundtruth.assign(total_points, "out_of_range");
			refresh = true;

			// Do NOT break unconditionally here. R-4's overrun guard above already caps this call
			// at measurement_per_cycle azimuth steps - exactly one revolution - and the loop
			// advances the index by one step per iteration, so the wrap can be crossed at most
			// once. The double-completion this break was added to prevent is therefore already
			// impossible whenever that cap is in force.
			//
			// Breaking anyway threw away the unused remainder of this tick's point budget. The
			// angle index carries over, but the budget does not, so every revolution restarted
			// from a fresh budget and cost ceil(steps_per_cycle / steps_per_tick) ticks instead of
			// the true average. Measured: Car1 pinned at exactly 5 ticks/sweep (zero variance) for
			// 512/123 = 4.16, i.e. 16.67 Hz against a configured 20 Hz.
			//
			// Continuing is safe: point_cloud has just been cleared, so the remaining points of
			// this tick accumulate into the next sweep's buffer, which is where they belong.
			//
			// The one case where the original hazard is real is limit_points == false, when the
			// cap does not apply and a single call can span more than one revolution. Keep the
			// guard for exactly that case.
			if (!params.limit_points)
				break;
		}

		// Both skip branches below must advance previous_horizontal_angle before continuing.
		// Sweep completion is detected purely by the azimuth ramp stepping backwards (above), so a
		// stale previous_ angle silently weakens that test: if a skip run begins at index 0 the
		// frozen value is the array minimum, the wrap compares equal rather than greater, the
		// completion is missed and the next revolution overwrites this one in place - one cloud
		// spanning two revolutions, de-skewed into the wrong frame.
		//
		// The FOV branch is unreachable: horizontal_angles_ is generated from the same FOV bounds it
		// tests against, so every angle is in-FOV by construction.
		//
		// The duplicate branch is now *evaluated* on the wrap iteration, because the completion
		// block above no longer breaks out of the loop. Its first term, (h - prev) <= eps, is a
		// backwards-step test rather than an equality test, so the wrap satisfies it; only the
		// second term, h >= eps, stops it firing. That holds for the FOV in use (-180..180, so
		// angles[0] == -180) and for the 0..360 default (angles[0] == 0), but a config with
		// HorizontalFOVStart above ~0 would satisfy both terms and silently drop the first azimuth
		// column of every revolution. The proper fix is to make this an equality test (fabs) -
		// tracked as I-N.
		//
		// Advancing previous_horizontal_angle here is defensive hygiene that keeps the invariant
		// true if the angle table or the FOV handling is ever changed. Matches upstream HERCULES.

		// check if horizontal angle is a duplicate
		if ((horizontal_angle - previous_horizontal_angle) <= 0.00005f && (horizontal_angle - 0) >= 0.00005f) {
			UE_LOG(LogTemp, Display, TEXT("duplicate horizontal angle! angle! previous:%f current:%f"), previous_horizontal_angle, horizontal_angle);
			previous_horizontal_angle = horizontal_angle;
			continue;
		}

		// check if the laser is outside the requested horizontal FOV
		if (!VectorMath::isAngleBetweenAngles(horizontal_angle, laser_start, laser_end)) {
			UE_LOG(LogTemp, Display, TEXT("outside of FOV: %f "), horizontal_angle);
			previous_horizontal_angle = horizontal_angle;
			continue;
		}

		ParallelFor(number_of_lasers, [&](uint32 laser) {
			float vertical_angle = laser_angles_[laser];
			uint32 current_point_index = number_of_lasers * current_horizontal_angle_index_ + laser;
			// The loop counter i is 1-based, so `number_of_lasers * i` shifted the whole range one
			// block up: slot [0, number_of_lasers) was never written and the final iteration wrote
			// past the end of point_cloud_draw_, which is a std::vector - no bounds check, so this
			// was a silent heap overrun of number_of_lasers * sizeof(FVector) bytes from inside a
			// ParallelFor. Only reachable with DrawDebugPoints enabled. Matches upstream HERCULES.
			uint32 draw_index = number_of_lasers * (i - 1) + laser;
			Vector3r point;
			FVector draw_point;
			std::string label;

			// shoot laser and get the impact point, if any
			if (shootLaser(lidar_pose, vehicle_pose, laser, horizontal_angle, vertical_angle, params, point, label, draw_point))
			{
				point_cloud[current_point_index * 3] = point.x();
				point_cloud[current_point_index * 3 + 1] = point.y();
				point_cloud[current_point_index * 3 + 2] = point.z();
				groundtruth[current_point_index] = label;
				if (sensor_params_.draw_debug_points)
					point_cloud_draw_[draw_index] = draw_point;
			}
			});


		
		previous_horizontal_angle = horizontal_angles_[current_horizontal_angle_index_];
	}

	if (sensor_params_.draw_debug_points) {
		for (uint32 j = 0; j < point_cloud_draw_.size(); j++)
		{
			UAirBlueprintLib::DrawPoint(
				actor_->GetWorld(),
				point_cloud_draw_[j],
				5,                       //size
				FColor::Green,
				false,                    //persistent (never goes away)
				(1 / (sensor_params_.horizontal_rotation_frequency * 2))                //point leaves a trail on moving object
			);
		}
	}
	return refresh;
}

FVector UnrealLidarSensor::Vector3rToFVector(const Vector3r& input_vector) {
	return FVector(input_vector.x(), input_vector.y(), -input_vector.z());
}

// simulate shooting a laser via Unreal ray-tracing.
bool UnrealLidarSensor::shootLaser(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
	const uint32 laser, const float horizontal_angle, const float vertical_angle,
	const msr::airlib::LidarSimpleParams params, Vector3r &point, std::string &label, FVector& raw_point)
{
	// start position
	Vector3r start = VectorMath::add(lidar_pose, vehicle_pose).position;

	// We need to compose rotations here rather than rotate a vector by a quaternion
	// Hence using coordOrientationAdd(..) rather than rotateQuaternion(..)

	// get ray quaternion in lidar frame (angles must be in radians)
	msr::airlib::Quaternionr ray_q_l = msr::airlib::VectorMath::toQuaternion(
		msr::airlib::Utils::degreesToRadians(vertical_angle),   //pitch - rotation around Y axis
		0,                                                      //roll  - rotation around X axis
		msr::airlib::Utils::degreesToRadians(horizontal_angle));//yaw   - rotation around Z axis

	// get ray quaternion in body frame
	msr::airlib::Quaternionr ray_q_b = VectorMath::coordOrientationAdd(ray_q_l, lidar_pose.orientation);

	// get ray quaternion in world frame
	msr::airlib::Quaternionr ray_q_w = VectorMath::coordOrientationAdd(ray_q_b, vehicle_pose.orientation);

	// get ray vector (end position)
	Vector3r end = VectorMath::rotateVector(VectorMath::front(), ray_q_w, true) * params.range + start;

	FHitResult hit_result = FHitResult(ForceInit);
	TArray<AActor*> actorArray;
	//actorArray.Add(actor_);
	bool is_hit;
	if (params.external) {
		is_hit = UAirBlueprintLib::GetObstacleAdv(actor_, ned_transform_->toFVector(start, 100, true), ned_transform_->toFVector(end, 100, true), hit_result, actorArray, ECC_Visibility, true, true);
	}
	else {
		is_hit = UAirBlueprintLib::GetObstacleAdv(actor_, ned_transform_->fromLocalNed(start), ned_transform_->fromLocalNed(end), hit_result, actorArray, ECC_Visibility, true, true);
	}
	bool ignoreMaterial = false;
	if (hit_result.PhysMaterial != nullptr) {
		if (hit_result.PhysMaterial.Get()->GetFName().ToString().Contains("Lidar_Ignore_PhysicalMaterial"))
			ignoreMaterial = true;
	}
	if (is_hit && !ignoreMaterial)
	{

		FVector impact_point = hit_result.ImpactPoint;

		//Store the name the hit object.
		auto hitActor = hit_result.GetActor();
		if (hitActor != nullptr)
		{
			label = TCHAR_TO_UTF8(*hitActor->GetName());
		}

		raw_point = impact_point;

		//if (label.empty())
		//{
		//	UE_LOG(LogTemp, Warning, TEXT("Empty label!"));
		//}
		// If enabled add range noise
		if (params.generate_noise) {
			// Add noise based on normal distribution taking into account scaling of noise with distance
			float distance_noise = dist_(gen_) * (1 + ((hit_result.Distance / 100) / params.range) * (params.noise_distance_scale - 1));

			Vector3r impact_point_local = VectorMath::rotateVector(VectorMath::front(), ray_q_w, true) * ((hit_result.Distance / 100) + distance_noise) + start;
			if (params.external) {
				impact_point = ned_transform_->fromRelativeNed(impact_point_local);
			} else {
				impact_point = ned_transform_->fromLocalNed(impact_point_local);
			}			
		}

		raw_point = impact_point;

		Vector3r point_v_i;
		if (params.external) {
			point_v_i = ned_transform_->toVector3r(impact_point, 0.01, true);
		}
		else {
			point_v_i = ned_transform_->toLocalNed(impact_point);
		}

		// When de-skewing, leave the point in the world/local-NED frame. The whole sweep is
		// converted to the lidar frame once, at hand-off, using the vehicle pose at that instant -
		// which is the instant the cloud's timestamp and pose refer to. Converting here instead
		// would bake in this tick's vehicle pose and make the cloud internally inconsistent.
		// See AirSimSettings::lidar_deskew.
		if (ShouldDeskewLidar()) {
			point = point_v_i;
		}
		else {
			// tranform to lidar frame
			point = VectorMath::transformToBodyFrame(point_v_i, lidar_pose + vehicle_pose, true);
		}

		return true;
	}
	else
	{
		return false;
	}
}
