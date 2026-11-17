#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace
{
using gz::math::Vector3d;
using namespace gz::sim;
using Publisher = gz::transport::Node::Publisher;

double Limit(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

class TetherPlugin final : public System, public ISystemConfigure,
    public ISystemPreUpdate, public ISystemReset
{
  public: void Configure(const Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      EntityComponentManager &, EventManager &) override
  {
    this->world = entity;
    this->stationName = sdf->Get<std::string>("station_model", "tether_station").first;
    this->vehicleName = sdf->Get<std::string>("vehicle_link", "").first;
    this->anchorOffset = sdf->Get<Vector3d>("anchor_offset", {0, 0, 0.32}).first;
    this->vehicleOffset = sdf->Get<Vector3d>("vehicle_offset", {-0.08, 0, -0.06}).first;
    this->radius = sdf->Get<double>("drum_radius", 0.08).first;
    this->initialLength = sdf->Get<double>("initial_length", 3.0).first;
    this->minLength = sdf->Get<double>("min_length", 1.0).first;
    this->maxLength = sdf->Get<double>("max_length", 12.0).first;
    this->maxSpeed = sdf->Get<double>("max_payout_speed", 0.65).first;
    this->stiffness = sdf->Get<double>("stiffness", 70.0).first;
    this->damping = sdf->Get<double>("damping", 3.5).first;
    this->maxTension = sdf->Get<double>("max_tension", 15.0).first;
    this->breakawayTension = sdf->Get<double>("breakaway_tension", 3.0).first;
    this->brakeDamping = sdf->Get<double>("brake_damping", 0.04).first;
    const std::array<double, 10> parameters{this->radius, this->initialLength,
        this->minLength, this->maxLength, this->maxSpeed, this->stiffness,
        this->damping, this->maxTension, this->breakawayTension, this->brakeDamping};
    if (this->vehicleName.empty() || !this->anchorOffset.IsFinite() ||
        !this->vehicleOffset.IsFinite() ||
        !std::all_of(parameters.begin(), parameters.end(),
                     [](double value) { return std::isfinite(value); }) ||
        this->radius <= 0 || this->stiffness <= 0 || this->damping < 0 ||
        this->breakawayTension < 0 || this->brakeDamping < 0 ||
        this->maxSpeed <= 0 || this->maxTension <= 0 || this->minLength <= 0 ||
        this->minLength > this->initialLength ||
        this->initialLength > this->maxLength || this->minLength >= this->maxLength)
    {
      gzerr << "Invalid tether configuration in the world SDF\n";
      return;
    }

    const std::array<const char *, 8> names{"length", "range", "extension",
        "payout_rate", "tension", "azimuth", "elevation", "drum_load_torque"};
    for (std::size_t i = 0; i < names.size(); ++i)
      this->telemetry[i] = this->node.Advertise<gz::msgs::Double>(
          std::string("/tether/") + names[i]);
    this->anchorForce = this->node.Advertise<gz::msgs::Vector3d>("/tether/anchor_force");
    this->configured = this->node.Subscribe(
        "/tether/cmd_length", &TetherPlugin::OnLength, this);
    if (!this->configured)
      gzerr << "Could not subscribe to /tether/cmd_length\n";
  }

  public: void Reset(const UpdateInfo &, EntityComponentManager &) override
  {
    this->targetLength = std::numeric_limits<double>::quiet_NaN();
    this->reelMode = ReelMode::Unconfigured;
    this->lastPublication = std::chrono::steady_clock::duration::zero();
  }

  public: void PreUpdate(const UpdateInfo &info,
      EntityComponentManager &ecm) override
  {
    if (!this->configured || info.paused || info.dt <= std::chrono::milliseconds(0))
      return;
    if (!this->vehicle.Valid(ecm) || !this->anchor.Valid(ecm) ||
        !this->drum.Valid(ecm) || !this->reel.Valid(ecm))
    {
      if (!this->Resolve(ecm))
        return;
    }

    const auto angle = this->reel.Position(ecm);
    const auto angularRate = this->reel.Velocity(ecm);
    const auto vehicleVelocity = this->vehicle.WorldLinearVelocity(ecm, this->vehicleOffset);
    const auto anchorVelocity = this->anchor.WorldLinearVelocity(ecm, this->anchorOffset);
    if (!angle || angle->empty() || !angularRate || angularRate->empty() ||
        !vehicleVelocity || !anchorVelocity)
      return;

    const auto anchorPose = worldPose(this->anchor.Entity(), ecm);
    const auto vehiclePose = worldPose(this->vehicle.Entity(), ecm);
    const Vector3d anchorPoint = anchorPose.Pos() + anchorPose.Rot().RotateVector(this->anchorOffset);
    const Vector3d vehiclePoint = vehiclePose.Pos() + vehiclePose.Rot().RotateVector(this->vehicleOffset);
    const Vector3d separation = vehiclePoint - anchorPoint;
    const double range = separation.Length();
    const Vector3d direction = range > 1e-9 ? separation / range : Vector3d::Zero;
    const double length = this->initialLength + this->radius * angle->front();
    const double payoutRate = this->radius * angularRate->front();
    const double extension = std::max(0.0, range - length);

    const double extensionRate = direction.Dot(*vehicleVelocity - *anchorVelocity)
        - payoutRate;
    const double tension = extension > 0 ? Limit(this->stiffness * extension +
        this->damping * extensionRate, 0.0, this->maxTension) : 0.0;
    const Vector3d force = tension * direction;
    const double drumLoad = this->radius * tension;
    this->UpdateReel(ecm, length, angularRate->front(), drumLoad,
        std::chrono::duration<double>(info.dt).count());
    if (tension > 0)
    {
      // Cable pulls along its line. Apply the load at the attachment points,
      // and transmit the cable's payout torque through the drum and chassis.
      this->vehicle.AddWorldWrench(ecm, -force, Vector3d::Zero, this->vehicleOffset);
      const Vector3d drumTorque = drumLoad * anchorPose.Rot().RotateVector(Vector3d::UnitY);
      this->drum.AddWorldWrench(ecm, Vector3d::Zero, drumTorque);
      this->anchor.AddWorldWrench(ecm, force, -drumTorque, this->anchorOffset);
    }

    if (info.simTime - this->lastPublication >= std::chrono::milliseconds(50))
    {
      this->lastPublication = info.simTime;
      const std::array<double, 8> values{length, range, extension, payoutRate, tension,
          std::atan2(separation.Y(), separation.X()),
          std::atan2(separation.Z(), std::hypot(separation.X(), separation.Y())), drumLoad};
      for (std::size_t i = 0; i < values.size(); ++i)
      {
        gz::msgs::Double sample;
        sample.set_data(values[i]);
        this->telemetry[i].Publish(sample);
      }
      gz::msgs::Vector3d forceSample;
      gz::msgs::Set(&forceSample, force);
      this->anchorForce.Publish(forceSample);
      this->DrawCable(anchorPoint, vehiclePoint, length, range, tension);
    }
  }

  private: enum class ReelMode { Unconfigured, Brake, Length };

  private: void UpdateReel(EntityComponentManager &ecm, double length,
      double angularRate, double drumLoad, double dt)
  {
    double target = this->targetLength.load();
    constexpr double lengthTolerance = 0.001;
    constexpr double speedTolerance = 0.01;
    if (std::isfinite(target) && std::abs(target - length) <= lengthTolerance &&
        std::abs(this->radius * angularRate) <= speedTolerance &&
        this->targetLength.compare_exchange_strong(target,
          std::numeric_limits<double>::quiet_NaN()))
      target = std::numeric_limits<double>::quiet_NaN();

    const auto mode = std::isfinite(target) ? ReelMode::Length : ReelMode::Brake;
    const double maxRate = this->maxSpeed / this->radius;
    if (this->reelMode != mode)
    {
      if (this->reelMode == ReelMode::Unconfigured)
        this->reel.SetPositionLimits(ecm, {{
            (this->minLength - this->initialLength) / this->radius,
            (this->maxLength - this->initialLength) / this->radius}});
      this->reel.SetVelocityLimits(ecm, {{mode == ReelMode::Length ? -maxRate : 0.0, maxRate}});

      // Gazebo gives force commands precedence if both components exist.
      ecm.RemoveComponent<components::JointForceCmd>(this->reel.Entity());
      ecm.RemoveComponent<components::JointVelocityCmd>(this->reel.Entity());
      this->reelMode = mode;
    }

    if (mode == ReelMode::Length)
    {
      const double demand = Limit(6.0 * (target - length), -this->maxSpeed, this->maxSpeed);
      const double speed = Limit(demand, (this->minLength - length) / dt,
          (this->maxLength - length) / dt);
      this->reel.SetVelocity(ecm, {speed / this->radius});
      return;
    }

    // Cable load turns the drum through the external wrench. At rest the
    // brake balances that load up to its breakaway torque, without motoring.
    const double threshold = this->radius * this->breakawayTension;
    const double rate = std::max(0.0, angularRate);
    double brake = rate > 1e-6
        ? threshold + this->brakeDamping * rate
        : std::min(threshold, drumLoad);
    // Ramp in extra braking over the final 10% of the allowable speed.
    const double governor = Limit((rate / maxRate - 0.9) / 0.1, 0.0, 1.0);
    brake += governor * std::max(0.0, drumLoad - brake);
    this->reel.SetForce(ecm, {-brake});
  }

  private: bool Resolve(EntityComponentManager &ecm)
  {
    const auto vehicleEntities = entitiesFromScopedName(this->vehicleName, ecm, this->world);
    const Model station(World(this->world).ModelByName(ecm, this->stationName));
    if (vehicleEntities.size() == 1 && station.Valid(ecm))
    {
      this->vehicle = Link(*vehicleEntities.begin());
      this->anchor = Link(station.LinkByName(ecm, "base_link"));
      this->drum = Link(station.LinkByName(ecm, "drum"));
      this->reel = Joint(station.JointByName(ecm, "drum_joint"));
      if (this->vehicle.Valid(ecm) && this->anchor.Valid(ecm) &&
          this->drum.Valid(ecm) && this->reel.Valid(ecm))
      {
        this->vehicle.EnableVelocityChecks(ecm);
        this->anchor.EnableVelocityChecks(ecm);
        this->reelMode = ReelMode::Unconfigured;
        this->reel.EnablePositionCheck(ecm);
        this->reel.EnableVelocityCheck(ecm);
        return true;
      }
    }
    if (!this->reportedMissing)
    {
      gzwarn << "Tether is waiting for " << this->vehicleName
             << " and the links and drum joint in " << this->stationName << "\n";
      this->reportedMissing = true;
    }
    return false;
  }

  private: void OnLength(const gz::msgs::Double &msg)
  {
    if (std::isfinite(msg.data()))
      this->targetLength = Limit(msg.data(), this->minLength, this->maxLength);
  }

  private: void DrawCable(const Vector3d &anchorPoint, const Vector3d &vehiclePoint,
      double length, double range, double tension)
  {
    // The GUI advertises this service. Avoid queuing visual requests on
    // headless servers, where no GUI consumes them.
    std::vector<gz::transport::ServicePublisher> providers;
    if (!this->node.ServiceInfo("/marker", providers))
      return;

    gz::msgs::Marker cable;
    cable.set_ns("quad_x_tether");
    cable.set_id(1);
    cable.set_type(gz::msgs::Marker::LINE_STRIP);
    cable.set_visibility(gz::msgs::Marker::GUI);
    const gz::math::Color color = tension > 0.1
        ? gz::math::Color(1, 0.55f, 0.15f) : gz::math::Color(0.15f, 0.8f, 1);
    gz::msgs::Set(cable.mutable_material()->mutable_ambient(), color);
    gz::msgs::Set(cable.mutable_material()->mutable_diffuse(), color);
    const double sag = 0.5 * std::sqrt(std::max(0.0, length * length - range * range));
    constexpr int segments = 12;
    for (int i = 0; i <= segments; ++i)
    {
      const double fraction = static_cast<double>(i) / segments;
      Vector3d point = anchorPoint + fraction * (vehiclePoint - anchorPoint);
      if (i > 0 && i < segments)
        point.Z(std::max(0.015, point.Z() - 4 * fraction * (1 - fraction) * sag));
      gz::msgs::Set(cable.add_point(), point);
    }
    this->node.Request("/marker", cable);
  }

  private: Entity world{kNullEntity};
  private: Link vehicle{kNullEntity};
  private: Link anchor{kNullEntity};
  private: Link drum{kNullEntity};
  private: Joint reel{kNullEntity};
  private: std::string stationName, vehicleName;
  private: Vector3d anchorOffset, vehicleOffset;
  private: double radius{}, initialLength{}, minLength{}, maxLength{}, maxSpeed{};
  private: double stiffness{}, damping{}, maxTension{};
  private: bool configured{false}, reportedMissing{false};
  private: double breakawayTension{}, brakeDamping{};
  private: ReelMode reelMode{ReelMode::Unconfigured};
  // A non-finite target leaves the winch in automatic braked payout.
  private: std::atomic<double> targetLength{std::numeric_limits<double>::quiet_NaN()};
  private: std::chrono::steady_clock::duration lastPublication{};
  private: std::array<Publisher, 8> telemetry;
  private: Publisher anchorForce;
  private: gz::transport::Node node;
};
}

GZ_ADD_PLUGIN(TetherPlugin, gz::sim::System, TetherPlugin::ISystemConfigure,
    TetherPlugin::ISystemPreUpdate, TetherPlugin::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(TetherPlugin, "TetherPlugin")
