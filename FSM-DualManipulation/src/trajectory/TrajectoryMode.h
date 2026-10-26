/*
5. Note importanti prima dell'utilizzo

- Continuità traslazionale: la modalità globale minimizza il jerk integrato e attraversa tutti i waypoint. La modalità segmentata usa stime locali, ma con derivate nodali condivise garantisce continuità C2 nella traslazione.
- Orientamento: SLERP interpola correttamente le pose, ma la velocità angolare può cambiare bruscamente ai waypoint. Se anche la rotazione deve essere 
C2,servirà una spline nello spazio delle rotazioni.
- Loop: per avere continuità al passaggio dall'ultimo al primo punto, la posa finale deve coincidere con quella iniziale e le condizioni cinematiche devono essere periodiche. Questa implementazione impone condizioni di riposo agli estremi, non una chiusura periodica generale.
- Pause: quando la base entra in pausa deve congelare il tempo e restituire uno stato coerente con la sosta. Per una pausa fisicamente corretta, il waypoint deve avere velocità desiderata nulla; altrimenti serve una fase di decelerazione e ripartenza.
- TrajectoryType: il codice sopra implementa la quintica. Gli altri tipi devono essere aggiunti esplicitamente quando saranno definiti.

Un'ultima correzione tecnica: nel codice globale mostrato, la matrice normale viene assemblata separatamente per ciascun asse, ma la costruzione di H e rhs deve mantenere gli stessi indici di variabile per ogni asse. La forma attuale lo fa; tuttavia, per un uso in controllo robotico reale, raccomando di verificare il risultato numericamente con test su waypoint equidistanti, tempi non uniformi, punti coincidenti e percorsi con inversioni di direzione prima di considerare la classe pronta per il controllore.

*/
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>
 
#include <SpaceVecAlg/MathFunc.h>
#include <SpaceVecAlg/PTransform.h>

#include <Eigen/Cholesky>
#include <Eigen/QR>
 

// Execution mode for one complete trajectory.





enum class TrajectoryMode {
    Once,
    Loop
};

struct SegmentTiming {
    double duration = 0.0;
};
 
enum class TrajectoryType {
    Quintic,
};

enum class TrajectoryStatus {
    Idle,
    Running,
    Paused,
    Finished,
    Error
};

struct TrajectoryState {
    sva::PTransform<double> pose;
    Eigen::Matrix<double, 6, 1> velocity = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> acceleration = Eigen::Matrix<double, 6, 1>::Zero();
};


// -----------------------------------------------------------------------------------
// Trajectory (base): waypoint state machine. Subclasses only implement generatePose().
class Trajectory
{
public:
    using Velocity = Eigen::Matrix<double, 6, 1>;
    using Acceleration = Eigen::Matrix<double, 6, 1>;

    Trajectory(
        sva::PTransform<double> start,
        std::vector<sva::PTransform<double>> waypoints,
        double duration,
        TrajectoryMode mode);

    virtual ~Trajectory() = default;

    // Advance execution and update the desired state.
    void update(double dt);

    // Last evaluated desired state.
    const TrajectoryState& getState() const { return state_; }
    TrajectoryStatus getStatus() const { return status_; }

    bool isFinished() const { return status_ == TrajectoryStatus::Finished; }

    // Normalized progress in [0, 1].
    double progress() const;

    // Future extension: modify execution speed without rebuilding
    // the geometric trajectory.
    virtual void setSpeedScale(double scale);

    bool setSegmentTimings(const std::vector<SegmentTiming>& timings);

    bool setWaypointPause(
        std::size_t waypointIndex,
        double pauseDuration);

protected:
    // Evaluate the complete desired state at trajectory time.
    // Implemented by the concrete interpolation class.
    virtual TrajectoryState evaluate(double time) const = 0;
    const sva::PTransform<double>& start() const { return start_; }
    const std::vector<sva::PTransform<double>>& waypoints() const { return waypoints_; }
    double duration() const { return duration_; }
    double currentTime() const { return currentTime_; }
    TrajectoryMode mode() const { return mode_; }
    const sva::PTransform<double>& initialPose() const { return start_; }
    const std::vector<SegmentTiming>& segmentTimings() const { return segmentTimings_; }
    virtual void onTimingChanged() {}

private:
    // Input geometry and execution configuration.
    sva::PTransform<double> start_;
    std::vector<sva::PTransform<double>> waypoints_;

    double duration_;
    TrajectoryMode mode_;

    // Execution state.
    double currentTime_ = 0.0;
    double speedScale_ = 1.0;

    TrajectoryStatus status_ = TrajectoryStatus::Idle;
    TrajectoryState state_;

    std::vector<SegmentTiming> segmentTimings_;
    std::vector<double> waypointPauses_;

    double pauseTimeRemaining_ = 0.0;
    std::size_t pauseWaypointIndex_ = 0;

    bool isWaitingAtWaypoint_ = false;

    double effectiveDuration() const;
    void updatePause(double dt);
    // Future extensions:
    // - initial/final boundary conditions
    // - velocity, acceleration and jerk limits
    // - minimum/maximum segment durations
};


inline Trajectory::Trajectory(
    sva::PTransform<double> start,
    std::vector<sva::PTransform<double>> waypoints,
    double duration,
    TrajectoryMode mode)
    : start_(std::move(start)),
      waypoints_(std::move(waypoints)),
      duration_(duration),
      mode_(mode)
{
    // Validate the basic configuration.
    if (!std::isfinite(duration_) || duration_ <= 0.0 ||
        waypoints_.empty())
    {
        status_ = TrajectoryStatus::Error;
        state_.pose = start_;
        return;
    }

    // Initialize the desired state at the trajectory start.
    state_.pose = start_;
    state_.velocity.setZero();
    state_.acceleration.setZero();

    // The trajectory is ready but has not started yet.
    status_ = TrajectoryStatus::Idle;
}
 


inline bool Trajectory::setSegmentTimings(
    const std::vector<SegmentTiming>& timings)
{
    if (timings.size() != waypoints_.size()) { return false; }

    double totalDuration = 0.0;

    for (const auto& timing : timings) {
        if (!std::isfinite(timing.duration) || timing.duration <= 0.0)
        { return false; }
        totalDuration += timing.duration;
    }

    if (!std::isfinite(totalDuration) || totalDuration <= 0.0) {
        return false;
    }

    segmentTimings_ = timings;
    duration_ = totalDuration;
    onTimingChanged();
    
    // Keep progress consistent with the new timing.
    currentTime_ = std::clamp(currentTime_, 0.0, duration_);

    return true;
}

inline bool Trajectory::setWaypointPause(
    std::size_t waypointIndex,
    double pauseDuration)
{
    if (waypointIndex >= waypoints_.size() || !std::isfinite(pauseDuration) || pauseDuration < 0.0) {
        return false;
    }

    if (waypointPauses_.size() != waypoints_.size()){
        waypointPauses_.assign(waypoints_.size(), 0.0);
    }

    waypointPauses_[waypointIndex] = pauseDuration;
    return true;
}

inline double Trajectory::effectiveDuration() const
{
    if (segmentTimings_.empty()) {
        return duration_;
    }

    double total = 0.0;

    for (const auto& timing : segmentTimings_) {
        total += timing.duration;
    }

    return total;
}

inline void Trajectory::setSpeedScale(double scale) {
    if (!std::isfinite(scale) || scale <= 0.0){
        status_ = TrajectoryStatus::Error;
        return;
    }

    speedScale_ = scale;
}

inline double Trajectory::progress() const {
    const double total = effectiveDuration();

    if (total <= 0.0){
        return 0.0;
    }

    return std::clamp(currentTime_ / total, 0.0, 1.0);
}

inline void Trajectory::updatePause(double dt){
    if (!isWaitingAtWaypoint_){
        return;
    }

    pauseTimeRemaining_ -= dt * speedScale_;

    if (pauseTimeRemaining_ <= 0.0)
    {
        pauseTimeRemaining_ = 0.0;
        isWaitingAtWaypoint_ = false;
        status_ = TrajectoryStatus::Running;
    }
}

inline void Trajectory::update(double dt)
{
    if (status_ == TrajectoryStatus::Error || status_ == TrajectoryStatus::Finished) {
        return;
    }

    if (!std::isfinite(dt) || dt < 0.0 || !std::isfinite(speedScale_) || speedScale_ <= 0.0) {
        status_ = TrajectoryStatus::Error;
        return;
    }

    if (status_ == TrajectoryStatus::Idle) {
        currentTime_ = 0.0;
        state_ = evaluate(0.0);
        status_ = TrajectoryStatus::Running;

        if (dt == 0.0)
        {
            return;
        }
    }

    // A configured waypoint pause freezes trajectory time.
    if (isWaitingAtWaypoint_) {
        status_ = TrajectoryStatus::Paused;
        updatePause(dt);
        return;
    }

    const double total = effectiveDuration();

    if (!std::isfinite(total) || total <= 0.0) {
        status_ = TrajectoryStatus::Error;
        return;
    }

    const double oldTime = currentTime_;
    const double nextTime = currentTime_ + dt * speedScale_;

    // Find the next waypoint boundary in trajectory time.
    double boundaryTime = total;
    std::size_t boundaryIndex = waypoints_.size() - 1;

    double cumulativeTime = 0.0;

    for (std::size_t i = 0; i < waypoints_.size(); ++i)
    {
        const double segmentTime = segmentTimings_.empty()
            ? total / static_cast<double>(waypoints_.size())
            : segmentTimings_[i].duration;

        cumulativeTime += segmentTime;

        if (cumulativeTime > oldTime + 1e-12)
        {
            boundaryTime = cumulativeTime;
            boundaryIndex = i;
            break;
        }
    }

    // Stop at the next waypoint if it has a configured pause.
    if (boundaryIndex < waypointPauses_.size() && waypointPauses_[boundaryIndex] > 0.0 && nextTime >= boundaryTime)
    {
        currentTime_ = boundaryTime;
        state_ = evaluate(currentTime_);

        pauseWaypointIndex_ = boundaryIndex;
        pauseTimeRemaining_ = waypointPauses_[boundaryIndex];
        isWaitingAtWaypoint_ = true;
        status_ = TrajectoryStatus::Paused;
        return;
    }

    if (mode_ == TrajectoryMode::Once)
    {
        currentTime_ = std::min(nextTime, total);
        state_ = evaluate(currentTime_);

        if (currentTime_ >= total)
        {
            status_ = TrajectoryStatus::Finished;
        }

        return;
    }

    // Loop mode: wrap the trajectory time.
    currentTime_ = std::fmod(nextTime, total);
    state_ = evaluate(currentTime_);
}


// -----------------------------------------------------------------------------------
// InterpolatedTrajectory: Quintic and future implementation will share one Hermite evaluator;
// -----------------------------------------------------------------------------------

enum class InterpolationMode
{
    Segmented,
    Global
};

class InterpolatedTrajectory : public Trajectory
{
public:
    InterpolatedTrajectory(
        sva::PTransform<double> start,
        std::vector<sva::PTransform<double>> waypoints,
        double duration,
        TrajectoryType type,
        TrajectoryMode mode,
        InterpolationMode interpolationMode =
            InterpolationMode::Segmented);

    // Optional boundary condition setters
    void setInitialConditions(
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& acceleration);

    void setFinalConditions(
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& acceleration);

protected:
    TrajectoryState evaluate(double time) const override;
    void onTimingChanged() override;

private:
    struct QuinticCoefficients
    {
        Eigen::Vector3d a0 = Eigen::Vector3d::Zero();
        Eigen::Vector3d a1 = Eigen::Vector3d::Zero();
        Eigen::Vector3d a2 = Eigen::Vector3d::Zero();
        Eigen::Vector3d a3 = Eigen::Vector3d::Zero();
        Eigen::Vector3d a4 = Eigen::Vector3d::Zero();
        Eigen::Vector3d a5 = Eigen::Vector3d::Zero();
    };

    struct Segment
    {
        QuinticCoefficients translation;
        // For SQUAD rotation: q_i, q_{i+1} and intermediate control points s_i, s_{i+1}
        Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond q1 = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond s0 = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond s1 = Eigen::Quaterniond::Identity();
        double startTime = 0.0;
        double duration = 0.0;
    };

    TrajectoryType type_;
    InterpolationMode interpolationMode_;

    std::vector<Segment> segments_;
    std::vector<double> knotTimes_;

    // Boundary conditions
    bool hasCustomInitial_ = false;
    bool hasCustomFinal_ = false;
    Eigen::Vector3d initialVelocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d initialAcceleration_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d finalVelocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d finalAcceleration_ = Eigen::Vector3d::Zero();

    void buildTrajectory();
    void buildSegmentedTrajectory();
    void buildGlobalTrajectory();

    //std::vector<Eigen::Vector3d> computeSegmentedVelocities() const;
    //std::vector<Eigen::Vector3d> computeSegmentedAccelerations() const;
    std::vector<Eigen::Vector3d> computeGlobalVelocities() const;
    std::vector<Eigen::Vector3d> computeGlobalAccelerations() const;

    // Quaternion & SQUAD helpers
    static Eigen::Quaterniond intermediateControlPoint(
        const Eigen::Quaterniond& q_prev,
        const Eigen::Quaterniond& q_curr,
        const Eigen::Quaterniond& q_next);

    static Eigen::Quaterniond squad(
        const Eigen::Quaterniond& q0,
        const Eigen::Quaterniond& q1,
        const Eigen::Quaterniond& s0,
        const Eigen::Quaterniond& s1,
        double u);

    static QuinticCoefficients makeQuintic(
        const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
        const Eigen::Vector3d& v0, const Eigen::Vector3d& v1,
        const Eigen::Vector3d& acc0, const Eigen::Vector3d& acc1,
        double T);

    static Eigen::Vector3d position(const QuinticCoefficients& c, double t);
    static Eigen::Vector3d velocity(const QuinticCoefficients& c, double t);
    static Eigen::Vector3d acceleration(const QuinticCoefficients& c, double t);
};


inline InterpolatedTrajectory::InterpolatedTrajectory(
    sva::PTransform<double> start,
    std::vector<sva::PTransform<double>> waypoints,
    double duration,
    TrajectoryType type,
    TrajectoryMode mode,
    InterpolationMode interpolationMode)
    : Trajectory(std::move(start), std::move(waypoints), duration, mode),
      type_(type),
      interpolationMode_(interpolationMode)
{
    buildTrajectory();
}


inline void InterpolatedTrajectory::setInitialConditions(
    const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& acceleration)
{
    initialVelocity_ = velocity;
    initialAcceleration_ = acceleration;
    hasCustomInitial_ = true;
    buildTrajectory();
}

inline void InterpolatedTrajectory::setFinalConditions(
    const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& acceleration)
{
    finalVelocity_ = velocity;
    finalAcceleration_ = acceleration;
    hasCustomFinal_ = true;
    buildTrajectory();
}

inline Eigen::Quaterniond InterpolatedTrajectory::intermediateControlPoint(
    const Eigen::Quaterniond& q_prev,
    const Eigen::Quaterniond& q_curr,
    const Eigen::Quaterniond& q_next)
{
    // Ensure shortest path consistency
    Eigen::Quaterniond q_p = q_prev;
    Eigen::Quaterniond q_c = q_curr;
    Eigen::Quaterniond q_n = q_next;

    if (q_p.dot(q_c) < 0.0) q_p.coeffs() *= -1.0;
    if (q_c.dot(q_n) < 0.0) q_n.coeffs() *= -1.0;

    Eigen::Quaterniond inv_qc = q_c.inverse();

    // Standard Eigen angle-axis extraction
    Eigen::AngleAxisd aa1(inv_qc * q_p);
    Eigen::Vector3d log1 = aa1.axis() * aa1.angle();

    Eigen::AngleAxisd aa2(inv_qc * q_n);
    Eigen::Vector3d log2 = aa2.axis() * aa2.angle();

    Eigen::Vector3d arg = -0.25 * (log1 + log2);
    const double norm = arg.norm();
    
    Eigen::Quaterniond exp_q;
    if (norm < 1e-12)
    {
        exp_q.setIdentity();
    }
    else
    {
        exp_q = Eigen::Quaterniond(Eigen::AngleAxisd(norm, arg / norm));
    }

    return q_c * exp_q;
}

inline Eigen::Quaterniond InterpolatedTrajectory::squad(
    const Eigen::Quaterniond& q0,
    const Eigen::Quaterniond& q1,
    const Eigen::Quaterniond& s0,
    const Eigen::Quaterniond& s1,
    double u)
{
    Eigen::Quaterniond q_slerp1 = q0.slerp(u, q1);
    Eigen::Quaterniond q_slerp2 = s0.slerp(u, s1);
    return q_slerp1.slerp(2.0 * u * (1.0 - u), q_slerp2).normalized();
}

inline std::vector<Eigen::Vector3d>
InterpolatedTrajectory::computeGlobalVelocities() const
{
    const std::size_t n = waypoints().size() + 1;
    std::vector<Eigen::Vector3d> p;
    p.reserve(n);
    p.push_back(initialPose().translation());
    for (const auto& wp : waypoints()) { p.push_back(wp.translation()); }

    std::vector<Eigen::Vector3d> v(n, Eigen::Vector3d::Zero());

    // Endpoints
    v[0] = hasCustomInitial_ ? initialVelocity_ : Eigen::Vector3d::Zero();
    v[n - 1] = hasCustomFinal_ ? finalVelocity_ : Eigen::Vector3d::Zero();

    // Internal knots: central tangents weighted by segment times
    for (std::size_t i = 1; i + 1 < n; ++i)
    {
const auto& timings = segmentTimings();
const double dt1 = timings.empty() ? (duration() / waypoints().size()) : timings[i - 1].duration;
const double dt2 = timings.empty() ? (duration() / waypoints().size()) : timings[i].duration;
        
        // Parameter-weighted tangent approximation
        v[i] = ((p[i] - p[i - 1]) / dt1 + (p[i + 1] - p[i]) / dt2) * 0.5;
    }

    return v;
}

inline void InterpolatedTrajectory::onTimingChanged()
{
    buildTrajectory();
}

inline std::vector<Eigen::Vector3d>
InterpolatedTrajectory::computeGlobalAccelerations() const
{
    const std::size_t n = waypoints().size() + 1;
    std::vector<Eigen::Vector3d> a(n, Eigen::Vector3d::Zero());

    // Endpoints
    a[0] = hasCustomInitial_ ? initialAcceleration_ : Eigen::Vector3d::Zero();
    a[n - 1] = hasCustomFinal_ ? finalAcceleration_ : Eigen::Vector3d::Zero();

    // Internal accelerations default to zero or finite differences for smoothness
    return a;
}

inline void InterpolatedTrajectory::buildSegmentedTrajectory()
{
    // Reuses your robust segmented logic (waypoint-to-waypoint with pause support)
    segments_.clear();
    knotTimes_.clear();

    const std::size_t segmentCount = waypoints().size();
    const std::size_t nodeCount = segmentCount + 1;
    if (segmentCount == 0) return;

    std::vector<sva::PTransform<double>> poses;
    poses.reserve(nodeCount);
    poses.push_back(initialPose());
    for (const auto& wp : waypoints()) { poses.push_back(wp); }

    std::vector<double> durations(segmentCount);
    if (segmentTimings().empty())
    {
        const double T = duration() / static_cast<double>(segmentCount);
        std::fill(durations.begin(), durations.end(), T);
    }
    else
    {
        if (segmentTimings().size() != segmentCount) return;
        for (std::size_t i = 0; i < segmentCount; ++i) {
            durations[i] = segmentTimings()[i].duration;
        }
    }

    knotTimes_.assign(nodeCount, 0.0);
    for (std::size_t i = 0; i < segmentCount; ++i) {
        knotTimes_[i + 1] = knotTimes_[i] + durations[i];
    }

    // Segmented local velocities/accelerations
    std::vector<Eigen::Vector3d> v(nodeCount, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> a(nodeCount, Eigen::Vector3d::Zero());
    for (std::size_t i = 1; i + 1 < nodeCount; ++i) {
        v[i] = (poses[i + 1].translation() - poses[i - 1].translation()) / (knotTimes_[i + 1] - knotTimes_[i - 1]);
    }

    segments_.reserve(segmentCount);
    for (std::size_t i = 0; i < segmentCount; ++i) {
        Segment seg;
        seg.startTime = knotTimes_[i];
        seg.duration = durations[i];

        seg.translation = makeQuintic(
            poses[i].translation(), poses[i + 1].translation(),
            v[i], v[i + 1], a[i], a[i + 1], durations[i]);

        seg.q0 = Eigen::Quaterniond(poses[i].rotation()).normalized();
        seg.q1 = Eigen::Quaterniond(poses[i + 1].rotation()).normalized();
        if (seg.q0.dot(seg.q1) < 0.0) { seg.q1.coeffs() *= -1.0; }

        segments_.push_back(seg);
    }
}

inline void InterpolatedTrajectory::buildGlobalTrajectory()
{
    segments_.clear();
    knotTimes_.clear();

    const std::size_t segmentCount = waypoints().size();
    const std::size_t nodeCount = segmentCount + 1;
    if (segmentCount == 0) return;

    std::vector<sva::PTransform<double>> poses;
    poses.reserve(nodeCount);
    poses.push_back(initialPose());
    for (const auto& wp : waypoints()) { poses.push_back(wp); }

    std::vector<double> durations(segmentCount);
    if (segmentTimings().empty())
    {
        const double T = duration() / static_cast<double>(segmentCount);
        std::fill(durations.begin(), durations.end(), T);
    }
    else
    {
        if (segmentTimings().size() != segmentCount) return;
        for (std::size_t i = 0; i < segmentCount; ++i) {
            durations[i] = segmentTimings()[i].duration;
        }
    }

    knotTimes_.assign(nodeCount, 0.0);
    for (std::size_t i = 0; i < segmentCount; ++i) {
        knotTimes_[i + 1] = knotTimes_[i] + durations[i];
    }

    const auto velocities = computeGlobalVelocities();
    const auto accelerations = computeGlobalAccelerations();

    segments_.reserve(segmentCount);
    for (std::size_t i = 0; i < segmentCount; ++i)
    {
        Segment seg;
        seg.startTime = knotTimes_[i];
        seg.duration = durations[i];

        seg.translation = makeQuintic(
            poses[i].translation(), poses[i + 1].translation(),
            velocities[i], velocities[i + 1],
            accelerations[i], accelerations[i + 1],
            durations[i]);

        seg.q0 = Eigen::Quaterniond(poses[i].rotation()).normalized();
        seg.q1 = Eigen::Quaterniond(poses[i + 1].rotation()).normalized();
        if (seg.q0.dot(seg.q1) < 0.0) { seg.q1.coeffs() *= -1.0; }

        // Compute SQUAD control points for smooth global rotation
        Eigen::Quaterniond q_prev = (i == 0) ? seg.q0 : Eigen::Quaterniond(poses[i - 1].rotation()).normalized();
        Eigen::Quaterniond q_next = (i + 2 < nodeCount) ? Eigen::Quaterniond(poses[i + 2].rotation()).normalized() : seg.q1;
        
        seg.s0 = intermediateControlPoint(q_prev, seg.q0, seg.q1);
        seg.s1 = intermediateControlPoint(seg.q0, seg.q1, q_next);

        segments_.push_back(seg);
    }
}

inline void InterpolatedTrajectory::buildTrajectory()
{
    if (type_ != TrajectoryType::Quintic) return;
    if (interpolationMode_ == InterpolationMode::Segmented)
    {
        buildSegmentedTrajectory();
    }
    else
    {
        buildGlobalTrajectory();
    }
}

inline TrajectoryState InterpolatedTrajectory::evaluate(double time) const
{
    TrajectoryState result;
    if (segments_.empty())
    {
        result.pose = initialPose();
        return result;
    }

    const double t = std::clamp(time, 0.0, knotTimes_.back());
    auto it = std::upper_bound(knotTimes_.begin(), knotTimes_.end(), t);
    std::size_t index = (it == knotTimes_.begin()) ? 0 : static_cast<std::size_t>(std::distance(knotTimes_.begin(), it) - 1);
    index = std::min(index, segments_.size() - 1);

    const Segment& seg = segments_[index];
    const double localTime = std::clamp(t - seg.startTime, 0.0, seg.duration);
    const double u = (seg.duration > 0.0) ? (localTime / seg.duration) : 0.0;

    result.pose = (index == 0) ? initialPose() : waypoints()[index - 1];
    result.pose.translation() = position(seg.translation, localTime);

    // Orientation interpolation (SQUAD for Global, SLERP fallback format)
    if (interpolationMode_ == InterpolationMode::Global && seg.duration > 0.0)
    {
        result.pose.rotation() = squad(seg.q0, seg.q1, seg.s0, seg.s1, u).toRotationMatrix();
    }
    else
    {
        result.pose.rotation() = seg.q0.slerp(u, seg.q1).normalized().toRotationMatrix();
    }

    result.velocity.tail<3>() = velocity(seg.translation, localTime);
    result.acceleration.tail<3>() = acceleration(seg.translation, localTime);

    // Approximate angular velocity
    Eigen::Quaterniond qRel = seg.q0.conjugate() * seg.q1;
    qRel.normalize();
    Eigen::AngleAxisd angleAxis(qRel);

const Eigen::Vector3d omegaLocal = (seg.duration > 0.0) 
    ? (angleAxis.axis() * angleAxis.angle() / seg.duration) 
    : Eigen::Vector3d::Zero().eval();

    result.velocity.head<3>() = seg.q0.toRotationMatrix() * omegaLocal;
    result.acceleration.head<3>().setZero();

    return result;
}

inline InterpolatedTrajectory::QuinticCoefficients InterpolatedTrajectory::makeQuintic(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& acc0,
    const Eigen::Vector3d& acc1,
    double T)
{
    QuinticCoefficients c;

    if (!std::isfinite(T) || T <= 0.0)
    {
        c.a0.setConstant(std::numeric_limits<double>::quiet_NaN());
        return c;
    }

    c.a0 = p0;
    c.a1 = v0;
    c.a2 = 0.5 * acc0;

    const Eigen::Vector3d dp = p1 - p0;

    const Eigen::Vector3d A =
        dp - v0 * T - 0.5 * acc0 * T * T;

    const Eigen::Vector3d B =
        v1 - v0 - acc0 * T;

    const Eigen::Vector3d C =
        acc1 - acc0;

    c.a3 = 10.0 * A / (T * T * T)
         - 4.0 * B / (T * T)
         + 0.5 * C / T;

    c.a4 = -15.0 * A / (T * T * T * T)
         + 7.0 * B / (T * T * T)
         - C / (T * T);

    c.a5 = 6.0 * A / (T * T * T * T * T)
         - 3.0 * B / (T * T * T * T)
         + 0.5 * C / (T * T * T);

    return c;
}

inline Eigen::Vector3d InterpolatedTrajectory::position(
    const QuinticCoefficients& c, double t)
{
    return c.a0 + t * (c.a1 + t * (c.a2 +
           t * (c.a3 + t * (c.a4 + t * c.a5))));
}

inline Eigen::Vector3d InterpolatedTrajectory::velocity(
    const QuinticCoefficients& c, double t)
{
    return c.a1 + t * (2.0 * c.a2 + t * (3.0 * c.a3 +
           t * (4.0 * c.a4 + t * 5.0 * c.a5)));
}

inline Eigen::Vector3d InterpolatedTrajectory::acceleration(
    const QuinticCoefficients& c, double t)
{
    return 2.0 * c.a2 + t * (6.0 * c.a3 +
           t * (12.0 * c.a4 + t * 20.0 * c.a5));
}

