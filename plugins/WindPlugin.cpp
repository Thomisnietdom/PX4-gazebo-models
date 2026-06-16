#include <gz/math/Rand.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <sdf/sdf.hh>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace gz::sim::v8::systems {

class WindPlugin : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
  public:
    void Configure(const gz::sim::Entity&,
                   const std::shared_ptr<const sdf::Element>& _sdf,
                   gz::sim::EntityComponentManager&,
                   gz::sim::EventManager&) override {
        if (!_sdf)
            return;

        // Mean wind velocity vector (m/s) in world frame (ENU: X=East, Y=North, Z=Up).
        // This is the constant baseline wind that always blows in one direction.
        // Example: (3.0, 1.5, 0) = ~3.4 m/s from the southwest (Beaufort scale 2-3, gentle breeze).
        wind_mean_ = _sdf->Get<gz::math::Vector3d>("wind_velocity_mean", gz::math::Vector3d(3.0, 1.5, 0)).first;

        // Gust amplitude (m/s). Maximum extra wind speed added on top of the mean
        // when a gust event occurs. A gust temporarily increases the wind speed.
        // Example: 2.0 = gusts can add up to 2 m/s extra, so total wind peaks at ~5.4 m/s.
        gust_amp_ = _sdf->Get<double>("wind_gust_amplitude", 2.0).first;

        // Gust frequency (probability per second that a new gust starts).
        // Example: 0.15 = on average a new gust every ~7 seconds.
        // Lower = less frequent gusts, higher = more frequent.
        gust_probability_per_sec_ = _sdf->Get<double>("wind_gust_frequency", 0.15).first;

        // Gust decay time (seconds). Controls how quickly a gust fades away
        // using exponential decay. After this many seconds, the gust is at ~37% strength.
        // Example: 3.0 = a gust takes about 3 seconds to mostly die out.
        gust_decay_time_ = _sdf->Get<double>("wind_gust_decay_time", 3.0).first;

        // Turbulence amplitude (m/s). Small random noise added every simulation tick
        // to simulate natural air turbulence. Creates a slight "jitter" in the wind.
        // Example: 0.4 = random variation of ±0.4 m/s each tick in X and Y.
        turb_amp_ = _sdf->Get<double>("wind_turbulence_amplitude", 0.4).first;

        // Vertical wind shear amplitude (m/s). A slow sinusoidal up/down wind component
        // that simulates thermal updrafts and downdrafts over time.
        // Example: 0.2 = gentle ±0.2 m/s vertical oscillation with a 20-second period.
        vert_amp_ = _sdf->Get<double>("wind_vertical_gust_amplitude", 0.2).first;

        // Aerodynamic drag parameter: Cd × A (m²), where Cd = drag coefficient and
        // A = frontal area of the drone. Used in the drag force formula:
        //   F = 0.5 * air_density * CdA * |wind_speed|² * wind_direction
        // Example: 0.04 = small drone with low drag. Increase for larger drones.
        drag_CdA_ = _sdf->Get<double>("drag_cda", 0.04).first;

        // Maximum force clamp (N). Safety limit to prevent simulation instability.
        // The computed aerodynamic force is clamped to this value.
        // For a ~2 kg drone, 10 N ≈ 0.5g lateral acceleration (noticeable push),
        // 20 N ≈ 1g (very strong gust), 5 N ≈ gentle nudge.
        max_force_ = _sdf->Get<double>("max_force", 10.0).first;

        // Name (or substring) of the target model in the simulation to apply wind to.
        // The plugin searches for a model whose name contains this string.
        target_model_ = _sdf->Get<std::string>("target_model", std::string("evo_tactical")).first;

        configured_ = true;
    }

    void PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) override {
        if (!configured_)
            return;

        if (_info.paused || _info.simTime.count() == 0)
            return;

        // Wait for the world and models to fully load (~2 seconds at 250 Hz physics rate).
        // PX4 spawns the drone model after the world is ready, so we need to wait.
        ++iterations_;
        if (iterations_ < 500)
            return;

        // Find the target drone model entity (done once, then cached)
        if (!modelFound_) {
            _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
                [&](const gz::sim::Entity& entity,
                    const gz::sim::components::Model*,
                    const gz::sim::components::Name* nameComp) -> bool {
                    if (nameComp && nameComp->Data().find(target_model_) != std::string::npos) {
                        targetEntity_ = entity;
                        modelFound_ = true;
                        return false;
                    }
                    return true;
                });

            if (!modelFound_)
                return;

            // Find the first (body/base) link of the drone model.
            // Wind force is only applied to this single link to avoid multiplying
            // the force across all links (rotors, arms, etc.).
            _ecm.Each<gz::sim::components::Link, gz::sim::components::ParentEntity, gz::sim::components::Name>(
                [&](const gz::sim::Entity& linkEntity,
                    const gz::sim::components::Link*,
                    const gz::sim::components::ParentEntity* parentComp,
                    const gz::sim::components::Name* nameComp) -> bool {
                    if (!parentComp || parentComp->Data() != targetEntity_)
                        return true;
                    // Take the first link found (typically the body/base_link)
                    bodyLinkEntity_ = linkEntity;
                    bodyLinkFound_ = true;
                    return false; // stop after first link
                });
        }

        if (!bodyLinkFound_)
            return;

        // Compute elapsed simulation time and timestep
        double t = std::chrono::duration<double>(_info.simTime).count();
        double dt = std::chrono::duration<double>(_info.dt).count();

        // Update the wind vector (mean + gusts + turbulence + vertical shear)
        UpdateWind(t, dt);

        // Apply aerodynamic drag force to the drone's body link
        gz::sim::Link link(bodyLinkEntity_);
        if (!link.Valid(_ecm))
            return;

        // Aerodynamic drag force formula: F = 0.5 * rho * CdA * |V| * V
        // where rho = air density, CdA = drag coefficient × frontal area,
        // V = wind velocity vector, |V| = wind speed magnitude.
        // This gives a force proportional to wind speed squared, in the wind direction.
        const double rho = 1.225; // Air density at sea level (kg/m³)
        double speed = wind_.Length();
        if (speed < 0.01)
            return;

        gz::math::Vector3d force = 0.5 * rho * drag_CdA_ * speed * wind_;

        // Clamp force to max_force for safety (prevents simulation instability)
        if (force.Length() > max_force_)
            force = force.Normalized() * max_force_;

        link.AddWorldForce(_ecm, force);
    }

  private:
    bool configured_{false};
    bool modelFound_{false};
    bool bodyLinkFound_{false};
    uint64_t iterations_{0};
    gz::sim::Entity targetEntity_{gz::sim::kNullEntity};
    gz::sim::Entity bodyLinkEntity_{gz::sim::kNullEntity};

    // Wind parameters (configurable via SDF)
    gz::math::Vector3d wind_mean_{3.0, 1.5, 0}; // Constant baseline wind (m/s)
    gz::math::Vector3d wind_{0, 0, 0};          // Current wind vector (updated each tick)
    gz::math::Vector3d gust_{0, 0, 0};          // Current active gust (decays over time)

    double gust_amp_{2.0};                  // Max gust strength (m/s)
    double gust_probability_per_sec_{0.15}; // Chance of new gust per second
    double gust_decay_time_{3.0};           // Gust exponential decay time constant (s)
    double turb_amp_{0.4};                  // Turbulence noise amplitude (m/s)
    double vert_amp_{0.2};                  // Vertical wind shear amplitude (m/s)
    double drag_CdA_{0.04};                 // Drag coefficient × frontal area (m²)
    double max_force_{10.0};                // Maximum force clamp (N)

    std::string target_model_{"evo_tactical"}; // Target drone model name

    /// Update the wind vector for the current simulation tick.
    /// Wind = mean + active_gust + turbulence_noise + vertical_shear
    void UpdateWind(double t, double dt) {
        // Start from the constant baseline wind direction and speed
        wind_ = wind_mean_;

        // === GUSTS ===
        // Random chance each tick to trigger a new gust event.
        // Probability is scaled by dt so it's framerate-independent.
        double gustChance = gust_probability_per_sec_ * dt;
        if (gz::math::Rand::DblUniform(0.0, 1.0) < gustChance) {
            // New gust: mostly aligned with the mean wind direction,
            // with a small random sideways and vertical component.
            gz::math::Vector3d windDir = wind_mean_.Normalized();
            double mainGust = gz::math::Rand::DblUniform(0.5 * gust_amp_, gust_amp_);
            double sideGust = gz::math::Rand::DblUniform(-0.3 * gust_amp_, 0.3 * gust_amp_);

            // Sideways direction is perpendicular to wind in the horizontal plane
            gz::math::Vector3d sideDir(-windDir.Y(), windDir.X(), 0);
            gust_ = windDir * mainGust + sideDir * sideGust;
            gust_.Z() = gz::math::Rand::DblUniform(-0.2 * gust_amp_, 0.2 * gust_amp_);
        }

        // Exponential decay: gust fades smoothly over time.
        // decay_factor = exp(-dt / decay_time), so after decay_time seconds
        // the gust is at ~37% of its original strength.
        double decayFactor = std::exp(-dt / gust_decay_time_);
        gust_ *= decayFactor;
        wind_ += gust_;

        // === TURBULENCE ===
        // Small random noise added every tick to simulate natural air turbulence.
        // Independent random value each tick in X, Y, and Z (Z is smaller).
        wind_ += gz::math::Vector3d(gz::math::Rand::DblUniform(-turb_amp_, turb_amp_),
                                    gz::math::Rand::DblUniform(-turb_amp_, turb_amp_),
                                    gz::math::Rand::DblUniform(-0.3 * turb_amp_, 0.3 * turb_amp_));

        // === VERTICAL WIND SHEAR ===
        // Slow sinusoidal vertical component simulating thermal updrafts/downdrafts.
        // Period = 1/0.05 = 20 seconds. Very subtle effect.
        wind_.Z() += vert_amp_ * std::sin(2.0 * M_PI * 0.05 * t);
    }
};

} // namespace gz::sim::v8::systems

GZ_ADD_PLUGIN(gz::sim::v8::systems::WindPlugin,
              gz::sim::System,
              gz::sim::v8::systems::WindPlugin::ISystemConfigure,
              gz::sim::v8::systems::WindPlugin::ISystemPreUpdate)