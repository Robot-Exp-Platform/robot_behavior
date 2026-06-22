#pragma once

// Public C++ facade for the robot_behavior FFI shape.
//
// The Rust cxx bridge still owns the concrete ABI declarations generated from
// `src/ffi/to_cxx.rs`. This file is intentionally a small, stable header for
// downstream drivers and C++ examples to share names, data shapes, and feature
// interfaces.

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace robot_behavior {

using Vec = std::vector<double>;

enum class PoseKind {
  Euler,
  Quat,
  Homo,
  AxisAngle,
  Position,
};

struct PoseData {
  PoseKind kind;
  Vec values;
};

using CxxPoseKind = PoseKind;
using CxxPoseData = PoseData;

struct LoadState {
  double m;
  std::array<double, 3> x;
  std::array<double, 9> i;
};

struct JointSample {
  std::optional<Vec> q;
  std::optional<Vec> dq;
  std::optional<Vec> ddq;
  std::optional<Vec> tau;
  std::optional<Vec> dtau;
};

struct SpatialSample {
  std::optional<PoseData> pose;
  std::optional<Vec> vel;
  std::optional<Vec> acc;
  std::optional<Vec> wrench;
};

struct JointState {
  JointSample meas;
  JointSample cmd;
  JointSample des;
};

struct SpatialState {
  SpatialSample meas;
  SpatialSample cmd;
  SpatialSample des;
};

struct ArmState {
  JointState joint;
  SpatialState flange;
  std::optional<SpatialState> tcp;
  std::optional<SpatialState> stiffness;
  std::optional<LoadState> load;
};

class Robot {
public:
  virtual ~Robot() = default;

  virtual std::string version() const = 0;
  virtual void init() = 0;
  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual void shutdown() = 0;
  virtual void reset() = 0;
  virtual void stop() = 0;
  virtual void emergency_stop() = 0;
  virtual void clear_emergency_stop() = 0;
  virtual bool is_moving() = 0;
};

template <std::size_t N>
class Arm : public Robot {
public:
  virtual std::string state() = 0;
  virtual void set_load(const LoadState &load) = 0;
  virtual std::array<double, N> get_joint() const = 0;
  virtual PoseData get_endpoint() const = 0;
};

template <std::size_t N>
class JointMotion {
public:
  virtual ~JointMotion() = default;

  virtual void move_joint(std::array<double, N> target) = 0;
  virtual void move_joint_sync(std::array<double, N> target) = 0;
};

class FlangeMotion {
public:
  virtual ~FlangeMotion() = default;

  virtual void move_flange(PoseData target) = 0;
  virtual void move_flange_sync(PoseData target) = 0;
};

} // namespace robot_behavior
