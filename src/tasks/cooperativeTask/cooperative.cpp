#include "cooperative.h"
#include <mc_rtc/logging.h>
#include <algorithm>
#include <stdexcept>

// ---------------------------------------------------------------------------
// HELPER FUNCTIONS
// ---------------------------------------------------------------------------
namespace
{
// skew matrix
// skew matrix
Eigen::Matrix3d skew(const Eigen::Vector3d& r)
{
    return Eigen::Matrix3d{
        {  0.0, -r.z(),  r.y()},
        {  r.z(),  0.0, -r.x()},
        { -r.y(),  r.x(),  0.0}
    };
}

// SVA motion vector -> [angular; linear]
Eigen::Vector6d toVec6(const sva::MotionVecd& v)
{
    Eigen::Vector6d result;
    result << v.angular(), v.linear();
    return result;
}


// Grasp matrices
GraspMatrices computeGraspMatricesGraspFrame(const EndEffectorOffsets& offsets, const GraspFrame& graspFrame_)
{


    //offsets from world frame to object
    //graspFrame_.X_world_object.rotation().transpose()*offsets.leftWorld;
    //graspFrame_.X_world_object.rotation().transpose()*offsets.rightWorld;
    // const Eigen::Matrix3d R = graspFrame_.X_world_object.rotation(); // object -> world (your convention)

    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d Z = Eigen::Matrix3d::Zero();
    
    
    const Eigen::Matrix<double, 6, 12> G = (Eigen::Matrix<double, 6, 12>() << 
        I, -skew(graspFrame_.X_world_object.rotation().transpose()*offsets.leftWorld), I, -skew(graspFrame_.X_world_object.rotation().transpose()*offsets.rightWorld), 
        Z, I, Z, I).finished();
        
    const Eigen::Matrix<double, 6, 6> M = G * G.transpose();


    const double L1 = (graspFrame_.X_world_object.rotation().transpose()*offsets.leftWorld).norm();
    const double L2 = (graspFrame_.X_world_object.rotation().transpose()*offsets.rightWorld).norm();

Eigen::Matrix<double, 12, 12> W = Eigen::Matrix<double, 12, 12>::Identity();

W(0, 0) *= L1;
W(2, 2) *= L2;

Eigen::Matrix<double, 12, 12> Winv = Eigen::Matrix<double, 12, 12>::Identity();

Winv(0, 0) /= L1;
Winv(2, 2) /= L2;


    //const Eigen::Matrix<double, 12, 6> Gpinv = G.transpose() * M.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
    const Eigen::Matrix<double, 12, 6> Gpinv = Winv * G.transpose()* (G * Winv * G.transpose()).inverse();
    const Eigen::Matrix<double, 12, 12> N = Eigen::Matrix<double, 12, 12>::Identity() - Gpinv * G;
    
    return {G, Gpinv, N};
}
}

// ---------------------------------------------------------------------------
// CONSTRUCTOR
// ---------------------------------------------------------------------------
CooperativeGrasp::CooperativeGrasp(
const mc_rbdyn::Robots & robots,
unsigned int leftRobotIndex,
unsigned int rightRobotIndex,
const std::string & leftFrameName,
const std::string & rightFrameName,
double dt)
: 
robots_(robots),
leftRobotIndex_(leftRobotIndex),
rightRobotIndex_(rightRobotIndex),
leftFrame_(leftFrameName),
rightFrame_(rightFrameName),
lowPassLeft_(dt, cutoffPeriod_),
lowPassRight_(dt, cutoffPeriod_),
dt_(dt)
{
}
// ---------------------------------------------------------------------------
// State queries
// ---------------------------------------------------------------------------
EndEffectorPoses CooperativeGrasp::getEndEffectorPoses() const
{
    const auto & L = robots_.robot(leftRobotIndex_);
    const auto & R = robots_.robot(rightRobotIndex_);
    return { L.bodyPosW(leftFrame_),  R.bodyPosW(rightFrame_),
             L.bodyVelW(leftFrame_),  R.bodyVelW(rightFrame_) };
}

Eigen::Vector3d CooperativeGrasp::buildObjectCenter(const EndEffectorPoses & poses) const
{
    if(cameraCenter_) return *cameraCenter_;
    return 0.5 * (poses.left.translation() + poses.right.translation());
}


GraspFrame CooperativeGrasp::buildGraspFrame() const {
// SAVE END EFFECTOR POSITIONS IN WORLD FRAME
    const auto eePoses = getEndEffectorPoses();
    const Eigen::Vector3d pL = eePoses.left.translation();
    const Eigen::Vector3d pR = eePoses.right.translation();

// CALCULATE OBJECT CENTER
    const auto objectCenter = buildObjectCenter(eePoses);

// BUILD GRASP FRAME
    Eigen::Vector3d x = (pR - pL).normalized();
    Eigen::Vector3d z(0,0,1);
    if(std::abs(x.dot(z)) > 0.99) z = Eigen::Vector3d(0,1,0);
    Eigen::Vector3d y = z.cross(x).normalized();
    z = x.cross(y).normalized();
    Eigen::Matrix3d R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;
    sva::PTransformd X_world_object(R, objectCenter);

// POSA FRAME END EFFECTORS IN OBJECT FRAME
    sva::PTransformd X_object_left = eePoses.left * X_world_object.inv();
    sva::PTransformd X_object_right = eePoses.right * X_world_object.inv();
    
    return {X_world_object,X_object_left,X_object_right,eePoses.left,eePoses.right};
}

// End-effector-to-center offsets
EndEffectorOffsets CooperativeGrasp::computeInitialOffsets(const EndEffectorPoses& poses, const Eigen::Vector3d& center) const
{
    Eigen::Vector3d leftWorld = center - poses.left.translation();
    Eigen::Vector3d rightWorld = center - poses.right.translation();
    
    return { leftWorld, rightWorld, 
             poses.left.rotation().transpose() * leftWorld,
             poses.right.rotation().transpose() * rightWorld };
}

void CooperativeGrasp::init(){
const auto eePoses = getEndEffectorPoses();
const auto objectCenter = buildObjectCenter(eePoses);

initialOffsets_ = computeInitialOffsets(eePoses, objectCenter);
graspFrame_ = buildGraspFrame();

graspMatrices_ = computeGraspMatricesGraspFrame(initialOffsets_, graspFrame_);

const Eigen::Matrix3d R0 = graspFrame_.X_world_object.rotation();

resetStiffnessEstimate();


const Eigen::Vector3d squeezeAxis = graspFrame_.X_world_object.rotation().col(0);
Eigen::Matrix<double, 12, 1> n_s = Eigen::Matrix<double, 12, 1>::Zero();

n_s.segment<3>(3) =  squeezeAxis;
n_s.segment<3>(9) = -squeezeAxis;

n_squeeze_ = graspMatrices_.nullSpace * n_s;
if(n_squeeze_.norm() > 1e-12) n_squeeze_.normalize();


const Eigen::Matrix3d RL0 = eePoses.left.rotation().transpose();
const Eigen::Matrix3d RR0 = eePoses.right.rotation().transpose();
relPosWorld_ = eePoses.right.translation() - eePoses.left.translation();
relRot_      = RL0.transpose() * RR0;


gap0_ = (eePoses.right.translation() - eePoses.left.translation()).norm();
cRef_ = 0.0;


internalForceIntegral_ = 0.0;
initialized_ = true;
}

GraspMatrices CooperativeGrasp::computeGraspMatricesWorldFrame() const
{
  const Eigen::Matrix3d R = computeCurrentObjectPose().rotation().transpose(); // object -> world (your convention)
  const Eigen::Matrix3d Z = Eigen::Matrix3d::Zero();
  const Eigen::Matrix<double, 6, 6> Z6 = Eigen::Matrix<double, 6, 6>::Zero();

  const Eigen::Matrix<double, 6, 6> R6 = (Eigen::Matrix<double, 6, 6>() << R, Z, Z, R).finished();
  const Eigen::Matrix<double, 12, 12> R12 = (Eigen::Matrix<double, 12, 12>() << R6, Z6, Z6, R6).finished();

  const auto & m = graspMatrices_;
  const Eigen::Matrix<double, 6, 12> G_world = R6 * m.G * R12.transpose();
  const Eigen::Matrix<double, 12, 6> Gpinv_world = R12 * m.Gpinv * R6.transpose();
  const Eigen::Matrix<double, 12, 12> N_world = R12 * m.nullSpace * R12.transpose();

  return {G_world, Gpinv_world, N_world};
}

void CooperativeGrasp::setImpedanceGains( const Eigen::Vector6d& K, const Eigen::Vector6d& M, double zeta)
{
    K_ = K;
    M_ = M;
    D_ = 2.0* zeta * (M_.array() * K_.array()).sqrt();
}

void CooperativeGrasp::setRelativeImpedanceGains( const Eigen::Vector6d& K, const Eigen::Vector6d& M, double zeta)
{
    K_rel_ = K;
    M_rel_ = M;
    D_rel_ = 2.0* zeta * (M_rel_.array() * K_rel_.array()).sqrt();
}

// ---------------------------------------------------------------------------
// Control Logic
// ---------------------------------------------------------------------------


sva::PTransformd CooperativeGrasp::computeCurrentObjectPose() const
{
const auto & grasp = graspFrame_;
const auto X_left = robots_.robot(leftRobotIndex_).bodyPosW(leftFrame_);
const auto X_right = robots_.robot(rightRobotIndex_).bodyPosW(rightFrame_);

// rotation: (EE -> world) * (object -> EE) = E^T * C^T, averaged over both arms
const sva::PTransformd X_object_left = grasp.X_object_left.inv() * X_left;
const sva::PTransformd X_object_right = grasp.X_object_right.inv() * X_right;


const Eigen::Vector3d p_avg = 0.5 * (X_object_left.translation() + X_object_right.translation()); // Average position
// Average orientation
Eigen::Quaterniond q_left(X_object_left.rotation());
Eigen::Quaterniond q_right(X_object_right.rotation());
const Eigen::Quaterniond q_avg = q_left.slerp(0.5, q_right);
return sva::PTransformd(q_avg.toRotationMatrix(),p_avg);
}


Eigen::Matrix<double, 12, 1> CooperativeGrasp::measureWorldWrenches(EndEffectorPoses eePoses)
{//mc_rtc::log::info("leftWrench): {}", leftWrench);

lowPassLeft_.update(robots_.robot(leftRobotIndex_).frame(leftFrame_).wrench());
lowPassRight_.update(robots_.robot(rightRobotIndex_).frame(rightFrame_).wrench());

const auto leftFiltered = lowPassLeft_.eval();
const auto rightFiltered = lowPassRight_.eval();

forceOut_.filteredLeftForce = leftFiltered.force();
forceOut_.filteredRightForce = rightFiltered.force();
mc_rtc::log::info("leftFiltered.force() = {}", leftFiltered.force());
mc_rtc::log::info("worldleftFiltered.force() = {}", eePoses.left.rotation() * leftFiltered.force());
return (Eigen::Matrix<double, 12, 1>() << 
    eePoses.left.rotation() * leftFiltered.couple(),
    eePoses.left.rotation() * leftFiltered.force(),
    eePoses.right.rotation() * rightFiltered.couple(),
    eePoses.right.rotation() * rightFiltered.force()
).finished(); 
} // Stacked Forces in world frame


Eigen::Matrix6d CooperativeGrasp::selectionMatrix(Eigen::Vector3d u) const 
{
    Eigen::Matrix6d S = Eigen::Matrix6d::Identity();
    S.bottomRightCorner<3,3>() -= u * u.transpose();
    return S;
}


// computeRelativeImpedance(): returns the 6D command applied +left / -right, no projection
Eigen::Matrix<double, 12, 1> CooperativeGrasp::computeRelativeImpedance(EndEffectorPoses eePoses, Eigen::Matrix6d Sel) const
{
    const Eigen::Matrix3d RL = eePoses.left.rotation().transpose();   // world orientation
    const Eigen::Matrix3d RR = eePoses.right.rotation().transpose();

    const Eigen::Vector3d d = eePoses.right.translation() - eePoses.left.translation();

    // world-fixed target: does not follow the left EE if it tilts
    const Eigen::Vector3d pos_err = d - relPosWorld_;       // set in init(): d at t=0

    // orientation of right relative to left, vs. initial
    Eigen::AngleAxisd aa(RR * (RL * relRot_).transpose());
    const Eigen::Vector3d rot_err = aa.angle() * aa.axis();

    //mc_rtc::log::info("e_rel pos={} rot={}", pos_err.norm(), rot_err.norm());
    // relative velocity, with the rigid rotation of the pair removed
    const Eigen::Vector3d w_c = 0.5 
                                * (eePoses.leftVelocity.angular() 
                                + eePoses.rightVelocity.angular());
    Eigen::Vector6d v_rel;
    v_rel << eePoses.rightVelocity.angular() - eePoses.leftVelocity.angular(),
             (eePoses.rightVelocity.linear() - eePoses.leftVelocity.linear()) - w_c.cross(d);

    Eigen::Vector6d e;
    e << rot_err, pos_err;

    const Eigen::Vector6d w =  Sel * (K_rel_.cwiseProduct(e) + D_rel_.cwiseProduct(v_rel)); //multiply by S

    Eigen::Matrix<double, 12, 1> out;
    out.segment<6>(0) =  w;   // left
    out.segment<6>(6) = -w;   // right

    return out;
}




// =================================================================================================
//  SQUEEZE (INTERNAL FORCE) CONTROL OF CooperativeGrasp - cpp part
//  Contents, in order:
//    A) stiffness estimator methods (removable block)
//    B) where to call the reset (removable block)
//    C) computeInternalSqueezeSTA  (documentation inside the function)
//  Code is unchanged w.r.t. the version that was tested; only the update-call block was split
//  out of the adaptive-B block (see "REMOVAL ORDER" in the function documentation).
// =================================================================================================
 
 
// >>>>>>>>>>>>>>>>>>>> BEGIN STIFFNESS ESTIMATION: methods (delete up to END marker to remove) >>>>>>>>>>>>>>>>>>>>
 
/*
  STIFFNESS ESTIMATION  (kHat_)
  =============================
 
  1. GOAL
  -------
  Estimate the effective contact stiffness k_env [N/m] seen by the squeeze loop, with the model
          f = k_env * c + offset
  f = fMeas (= -lambdaMeas, the squeeze force, expected > 0 when squeezing),
  c = gap0_ - gap (closing displacement of the two end effectors).
  k_env is the stiffness of the WHOLE chain between the two measured end-effector frames and the
  force: object + fingers + any compliance between the frame and the contact. Since c is computed
  from the actual end-effector poses (not the commanded ones), the arms' own position-servo
  compliance does NOT enter in simulation; with real hardware, compliance of the structure does.
 
  2. WHY IT IS USEFUL
  -------------------
  The force loop is an admittance: vDes = error / B, and d(force)/dt = k_env * vClose, so the
  force error behaves like a first-order lag with time constant
          tau = B / k_env        (force-loop bandwidth ~ k_env / B [rad/s])
  - It turns "tune B by trial and error" into "choose tauDes": B = tauDes * kHat gives the same
    force response time for soft and stiff objects (adaptive B).
  - It provides the model gain for the reference feedforward: vFF = d(fRef)/dt / kHat.
  - It is a diagnostic: it tells you the loop speed and whether the outer loop might be too fast
    for the inner loop (bandwidth k_env / B must stay well below the inner-loop bandwidth).
 
  3. METHOD
  ---------
   1. High-pass f and c with the SAME first-order filter (x_hp = x - lowpass(x), time constant
      tHighPass). This removes offsets (unknown contact point, preload) and keeps only the
      variations, which are the part that carries the slope df/dc.
   2. Exponentially weighted least squares on the filtered signals (phi = c_hp, y = f_hp):
          Scc = sum(phi^2), Scf = sum(phi*y), Sff = sum(y^2)   (forgetting window tWindow)
          kRaw = Scf / Scc
   3. The estimate is only UPDATED when trustworthy, otherwise it stays frozen at the last value:
        - in contact:     |f| >= fMin
        - excitation:     rms of filtered c >= phiRmsMin (constant force reference -> no
                          excitation -> frozen; the sinusoidal lambdaRef is what keeps it alive)
        - good fit:       coherence = Scf^2 / (Scc*Sff) >= cohMin. Low coherence = noise
                          dominated, or f and c out of phase (delays, filtering) -> not trusted.
        - sensible:       kRaw > 0. A negative value usually means a sign-convention problem or
                          no real contact. Do NOT flip controller signs to "fix" it.
   4. The accepted value is clamped to [kMin, kMax] and low-passed (tK) -> kHat_.
      kValid_ becomes true at the first accepted value and stays true until the reset.
 
  4. PARAMETERS (declared as constexpr inside updateStiffnessEstimate)
  --------------------------------------------------------------------
  tHighPass [s]  1.0   Must be SLOWER than the force variations you excite (period of lambdaRef
                       >> tHighPass), otherwise the signal itself is filtered out.
  tWindow   [s]  2.0   Memory of the least squares. Shorter = follows stiffness changes faster
                       but noisier; longer = smoother but slow to react to a new object.
  tK        [s]  1.0   Low-pass on the accepted value. Smooths kHat_ (and therefore B and vFF).
  fMin      [N]  2.0   Below this squeeze force the object is considered not in contact.
  phiRmsMin [m]  2e-5  Minimum excitation. Needed rms of c ~ (rms of force swing) / k_env.
                       If kHat never becomes valid, this is the first gate to check.
  cohMin    [-]  0.5   Minimum coherence. Raise if kHat is erratic, lower if it never validates.
  kMin, kMax [N/m] 500, 5e4  Clamp. kMax was a guess made before the first measurement; a stiffer
                       object than kMax would silently look like kMax. Raise it (e.g. 1e6) when
                       you test stiffer objects.
 
  5. LIMITATIONS (read before trusting kHat)
  ------------------------------------------
  - It is the LOCAL slope df/dc around the current operating point. Contacts are often nonlinear
    (in MuJoCo the solimp impedance depends on penetration), so kHat can depend on the force.
  - A delay or lag between force and displacement lowers the coherence and biases kHat low. In
    that case the gate freezes the estimate instead of letting it become wrong.
  - It needs excitation. With a constant lambdaRef it holds the last accepted value.
  - After changing object it must be reset (resetStiffnessEstimate); until the first valid
    estimate B falls back to B0 and the feedforward is off.
  - If kRaw/kHat is far from what you expect, validate offline: take a clean squeeze segment from
    the logs and compute delta(f) / delta(c) directly.
 
  6. HOW TO READ THE LOGS
  -----------------------
  kValid_ must be 1; kRaw_ should hover around kHat_; kCoh_ should be close to 1 (>0.9 is great);
  kHat_ should be flat over a cycle of the sinusoid. kHat_ drifting with the phase of the
  sinusoid = nonlinear contact or delay.
 
  7. MEASURED VALUE (first run, sim box, see experiment log in the function documentation)
  ----------------------------------------------------------------------------------------
  kHat ~ 3220 N/m, stable, with lambdaRef ~ -16 N sinusoid, controller at 1 kHz.
  tauCurrent = B0 / kHat = 600 / 3220 ~ 0.186 s  (bandwidth k/B ~ 5.4 rad/s).
*/
void CooperativeGrasp::resetStiffnessEstimate()
{
  kInit_ = false;
  kValid_ = false;
  kLpC_ = kLpF_ = 0.0;
  kScc_ = kScf_ = kSff_ = 0.0;
  kHat_ = kRaw_ = kCoh_ = 0.0;
  // >>>>>>>>>> BEGIN FEEDFORWARD: reset (delete together with the feedforward block) >>>>>>>>>>
  fRefPrevValid_ = false;
  fRefPrev_ = 0.0;
  // <<<<<<<<<< END FEEDFORWARD: reset <<<<<<<<<<
}
 
void CooperativeGrasp::updateStiffnessEstimate(double f, double c)
{
  constexpr double tHighPass = 1.0;   // [s]   must be SLOWER than the force variations you excite
  constexpr double tWindow   = 2.0;   // [s]   forgetting window of the least squares
  constexpr double tK        = 1.0;   // [s]   low-pass on the accepted estimate (smooth B changes)
  constexpr double fMin      = 2.0;   // [N]   minimum |force| to consider the object in contact
  constexpr double phiRmsMin = 2e-5;  // [m]   minimum rms of filtered c (excitation)
  constexpr double cohMin    = 0.5;   // [-]   minimum coherence
  constexpr double kMin      = 500.0; // [N/m] clamp of the estimate
  constexpr double kMax      = 5e4;   // [N/m]
 
  if(!kInit_)
  {
    kLpC_ = c;
    kLpF_ = f;
    kInit_ = true;
    return;
  }
 
  // same high-pass on both signals
  const double aHP = dt_ / (tHighPass + dt_);
  const double phi = c - kLpC_;
  const double y   = f - kLpF_;
  kLpC_ += aHP * phi;
  kLpF_ += aHP * y;
 
  // exponentially weighted sums
  const double lam = 1.0 - dt_ / tWindow;
  kScc_ = lam * kScc_ + phi * phi;
  kScf_ = lam * kScf_ + phi * y;
  kSff_ = lam * kSff_ + y * y;
 
  // gates: freeze the estimate if it cannot be trusted
  if(std::abs(f) < fMin) return;
  const double phiRms = std::sqrt(kScc_ * (1.0 - lam));   // ~ rms over the window
  if(phiRms < phiRmsMin || kScc_ <= 0.0) return;
 
  kRaw_ = kScf_ / kScc_;
  kCoh_ = (kScf_ * kScf_) / (kScc_ * kSff_ + 1e-30);
  if(kRaw_ <= 0.0 || kCoh_ < cohMin) return;
 
  const double kc = std::clamp(kRaw_, kMin, kMax);
  if(!kValid_)
  {
    kHat_ = kc;
    kValid_ = true;
  }
  else
  {
    kHat_ += dt_ / (tK + dt_) * (kc - kHat_);
  }
}
 
// B = tau * kHat saturated; fallback value while there is no valid estimate
double CooperativeGrasp::adaptiveB(double tau, double bMin, double bMax, double bFallback) const
{
  return kValid_ ? std::clamp(tau * kHat_, bMin, bMax) : bFallback;
}
 
// <<<<<<<<<<<<<<<<<<<< END STIFFNESS ESTIMATION: methods <<<<<<<<<<<<<<<<<<<<
 
 
// Where you reset cRef_, zSt_ and set gap0_ (new grasp / new object / squeeze re-activated), call
// resetStiffnessEstimate(). Without it a new object inherits the old kHat_ and B.
//
// >>>>>>>>>> BEGIN STIFFNESS ESTIMATION: reset call >>>>>>>>>>
//   resetStiffnessEstimate();
// <<<<<<<<<< END STIFFNESS ESTIMATION: reset call <<<<<<<<<<
 
 
Eigen::Matrix<double, 12, 1> CooperativeGrasp::computeInternalSqueezeSTA(
    double lambdaRef,
    double lambdaMeas,
    const Eigen::Vector3d & u,
    const EndEffectorPoses & ee)
{
  /*
   ==========================================================================================
   computeInternalSqueezeSTA  --  SQUEEZE (INTERNAL FORCE) CONTROL
   Super-twisting inner loop + stiffness estimation + adaptive B + reference feedforward
   ==========================================================================================
 
   0. WHAT THIS FUNCTION IS
   ------------------------
   Drop-in alternative to computeInternalSqueezePI (same signature, same sign convention, same
   output). Compared to the PI version it has four layers, each one removable:
     (a) STA inner loop      the linear PD (Kp, Kv) replaced by a Super-Twisting sliding-mode law
                             (this is the core of the function, not a marked block)
     (b) stiffness estimator kHat_ = online estimate of the contact stiffness k_env
     (c) adaptive B          B = tauDes * kHat_   instead of the fixed B0
     (d) feedforward         vFF = d(fRef)/dt / kHat_  added to the desired closing speed
 
   To go back to the old controller: call computeInternalSqueezePI instead and delete this
   function + the member zSt_ + the marked blocks. Never call both functions in the same cycle
   (both integrate cRef_).
 
   Goal: regulate the squeeze force between the two end effectors along the axis n that joins
   them, so that the measured force lambdaMeas follows lambdaRef. The arms are position or
   acceleration controlled, so the force is not commanded directly: it is regulated through a
   small relative motion along n (squeeze more = close the gap).
 
   Output w (12x1): stacked wrench [left(6); right(6)], only the LINEAR parts are filled, equal
   and opposite along n (internal wrench: no net force on the object).
       w.segment<3>(3) = +f * n   (left,  pushes towards the right arm)
       w.segment<3>(9) = -f * n   (right, pushes towards the left arm)
   f > 0 means "close the gap more" = squeeze harder.
   (The argument u is not used here; it is kept only so the signature matches the PI.)
 
   REMOVAL ORDER (dependencies)
   ----------------------------
   Marked blocks in this function:
     STIFFNESS ESTIMATION: update   calls updateStiffnessEstimate every cycle. Needed by (c), (d).
     ADAPTIVE B                     needs the update block. Can be deleted alone (B stays B0 and
                                    the estimator keeps running for logging).
     FEEDFORWARD                    needs the update block (uses kHat_ and kValid_). Can be deleted
                                    alone. If you delete the update block but keep this one, the
                                    feedforward silently does nothing (kValid_ stays false).
   Safe order to revert completely: FEEDFORWARD, then ADAPTIVE B, then STIFFNESS ESTIMATION
   (update block, methods, header members, reset call). The declarations "double B = B0;" and
   "double vFF = 0.0;" are OUTSIDE the blocks on purpose: deleting the blocks leaves the original
   fixed-B function with valid code.
 
 
   1. ARCHITECTURE (cascade, two loops + estimator)
   ------------------------------------------------
 
                                         +-------------------------------+
        c = gap0_ - gap  ------------->  | STIFFNESS ESTIMATOR  -> kHat_ |
        fMeas  ----------------------->  +---------------+---------------+
                                                         | kHat_
                                                         v
        lambdaRef -> fRef --(+)--> [ OUTER: admittance ] ---> vDesClose, cRef_ --> [ INNER: STA ] --> aClose --> f --> w
                           (-)      vDes = (fRef-fMeas)/B + vFF                    sliding variable s
        lambdaMeas -> fMeas -+      B   = tauDes*kHat (adaptive) or B0             aClose = -k1 sqrt|s| sgn(s) + z
                                    vFF = d(fRef)/dt / kHat                        dz     = -k2 sgn(s)
                                    cRef_ += vDes*dt
 
   - OUTER loop (force -> motion): force error (plus the reference rate, feedforward) is turned into
     a desired closing speed through a virtual damper B. This is an ADMITTANCE: force in, motion
     out. Integrating that speed gives a position reference cRef_ for the closing displacement.
   - INNER loop (motion -> acceleration): makes the real closing displacement c and closing speed
     vClose follow (cRef_, vDesClose). This is the sliding-mode part.
   - The force never enters the inner loop directly, only through vDesClose / cRef_.
 
 
   2. SIGN CONVENTION (do NOT change, it was validated with the PI version)
   ------------------------------------------------------------------------
   - d = pRight - pLeft, n = d/|d| points from the LEFT to the RIGHT end effector.
   - Forces are used with flipped sign: fRef = -lambdaRef, fMeas = -lambdaMeas.
     So (fRef - fMeas) > 0 means "squeeze is too weak, close more".
   - c = gap0_ - gap     closing displacement (>0 = closer than at grasp time, gap0_)
   - vClose = -n . (vRight - vLeft) = -d(gap)/dt   closing speed (>0 = closing)
   - aClose > 0 = closing acceleration -> f > 0 -> left pushed along +n, right along -n.
   - The estimator relies on the same convention: fMeas must INCREASE when c increases, so kHat_
     is expected positive. A negative kRaw means this convention is violated somewhere.
 
 
   3. OUTER LOOP: how it works and how to tune it
   ----------------------------------------------
   vDesClose = clamp( (fRef - fMeas) / B + vFF , -vMax, +vMax )
   cRef_    += vDesClose * dt_                       (integrate the speed)
   cRef_     = c + clamp(cRef_ - c, -eMax, +eMax)    (anti-windup, see below)
 
   Why it works: if the contact has an effective stiffness k_env [N/m] (object + fingers), then
   d(force)/dt = k_env * vClose. With vClose ~ (fRef - fMeas)/B the force error behaves like a
   first-order lag with time constant  tau = B / k_env.
   So B sets how fast the force converges:
       LOW B  = fast, but amplifies force noise (noise on fMeas enters vDes with gain 1/B) and can
                oscillate if k_env/B approaches the inner-loop bandwidth;
       HIGH B = smooth but slow/delayed. That was exactly the "smooth but a bit delayed"
                behaviour observed (tau ~ 0.19 s with B = 600 and kHat ~ 3220 N/m).
   k_env is now measured online (kHat_, see the stiffness section), not just a tip from the logs.
   A damper-only admittance has a built-in delay on a time-varying reference (a first-order lag
   cannot follow a sinusoid without phase lag and amplitude loss): that is what the feedforward
   (section 8) addresses.
 
   Parameters:
   - B0   [N s/m]  fixed virtual damper (and fallback of the adaptive B). Main knob for force
                   response speed vs smoothness when the adaptive B is off.
   - vMax [m/s]    saturation of the closing speed (applies to feedback + feedforward). Limits how
                   fast the force can change (rate limit on the force ramp) and bounds the
                   disturbance seen by the inner loop. Raise only if large steps in lambdaRef are
                   tracked too slowly; lower if the squeeze feels violent.
   - eMax [m]      anti-windup: cRef_ is never allowed to be further than eMax from the real c.
                   Without it, if contact is lost or the object slips, the force error stays large,
                   cRef_ integrates away and the arms would lunge when contact returns. Too small
                   -> the reference cannot lead the real motion enough and the force loop gets
                   sluggish; too big -> windup / lunge risk.
 
   Outer-loop tuning order (do it AFTER the inner loop tracks well, see section 9):
     1. keep vMax conservative, start with a large B (smooth), check stability;
     2. decrease B (or tauDes) until the force response is fast enough or noise/oscillation appears;
     3. adjust vMax/eMax last, only if steps are rate-limited or windup is seen.
 
 
   4. INNER LOOP: sliding variable and why it equals the old PD
   ------------------------------------------------------------
   Old (linear) law:   aClose = Kp (cRef - c) + Kv (vDes - vClose)
   Define the sliding variable
           s = (vClose - vDesClose) + lambdaS * (c - cRef_)      [m/s]
   Since dc/dt = vClose and d(cRef_)/dt = vDesClose, s = e_dot + lambdaS * e with e = c - cRef_.
   When s = 0 the tracking error decays as e(t) ~ exp(-lambdaS t), so lambdaS [1/s] is the speed of
   the position-tracking error decay (the "sliding dynamics"). The linear law is exactly
   aClose = -Kv * s  with  lambdaS = Kp/Kv (50 and 60 -> 1.2), hence lambdaS = 1.2 to compare fairly
   with the old setup. lambdaS only shapes the surface: bigger = tighter position tracking but more
   sensitivity to noise in c / vClose.
 
 
   5. SUPER-TWISTING LAW and why it is useful here
   -----------------------------------------------
       aClose = -k1 * sqrt(|s|) * sgn(s) + z
       dz/dt  = -k2 * sgn(s)
   The system seen by the controller is  ds/dt = aClose + delta(t),  where delta lumps everything
   we do not model: d(vDesClose)/dt (the force loop moves the reference), force-sensor noise passed
   through 1/B, the arms not realising the commanded acceleration exactly (position-controlled,
   delays), contact stiffness changes, friction, motion of the object while the robots move (the
   spikes seen during motion). The STA treats delta as an unknown bounded-slope disturbance:
   - z is an integral-like term that learns and cancels delta (like an I action, but with
     finite-time convergence);
   - the output aClose is CONTINUOUS (no chattering like first-order sliding mode), which matters
     because it ends up as a force command;
   - s -> 0 in finite time even with delta present, vs. the linear law that only attenuates it
     (steady error ~ delta/Kv).
   Rule of thumb (Levant): with L an upper bound of |d(delta)/dt| [m/s^3],
           k1 = 1.5 * sqrt(L),    k2 = 1.1 * L.
   The current k1 = 3, k2 = 5 correspond to L of about 4-5 m/s^3 (a guess, not identified).
 
   Boundary layer: sgn(s) is implemented as tanh(s/phi). Pure sign() would chatter with a discrete
   controller and noisy s. With tanh, when |s| << phi the law behaves like a smooth linear-ish
   controller (z becomes an integrator with gain k2/phi on s). Cost: the finite-time convergence
   guarantee degrades into an ultimate bound of size ~phi.
   - phi [m/s] bigger = smoother, less aggressive; smaller = closer to ideal sliding, more
     noise/chatter. Needs to be on the order of the noise level of s.
 
   Saturations:
   - zMax [m/s^2]: bounds the integral state z (anti-windup of the STA integrator).
   - aMax [m/s^2]: bounds the total commanded relative acceleration (safety).
     Check from logs that you are NOT living on these limits: if aClose or z sit at the limits, the
     law is saturated and the analysis above no longer holds.
 
   Inner-loop gains cheat-sheet:
   - k1 up   -> faster/stiffer reaction to s, more noise sensitivity (sqrt gives very high gain near
                s = 0, noise in s is amplified more than with a linear law);
   - k2 up   -> stronger/faster disturbance rejection, but more overshoot, oscillation or limit
                cycle (z too aggressive); k2 down if you see periodic oscillation;
   - lambdaS -> see section 4;
   - phi     -> see above.
 
 
   6. WRENCH CONVERSION (unchanged from the PI)
   --------------------------------------------
   aClose is the RELATIVE closing acceleration of the two end effectors. Each arm takes half of it,
   with the average linear mass of the task: f = 0.5 * mLin * aClose.
   mLin = mean of the linear part of M_ (the tail<3> of the 6D mass vector).
 
 
   7. ADAPTIVE B (uses the stiffness estimator)
   --------------------------------------------
   With kAdaptiveB = true the damper is B = clamp(tauDes * kHat_, BMin, BMax), so the force loop
   time constant tau = B / k_env is the same for soft and stiff objects. The estimator ALWAYS runs
   when the update block is present (kHat_ can be logged), but B only follows it when kAdaptiveB is
   true. Without a valid estimate (no contact, no excitation, poor fit) B = B0.
   Workflow used:
     1. kAdaptiveB = false, log kHat, kRaw, kCoh, kValid on the object already tuned.
     2. tauCurrent = B0 / kHat; set tauDes = tauCurrent -> same behaviour on that object.
        Here: 600 / 3220 = 0.186 s, so tauDes = 0.186 and B stays ~600 on the test box.
     3. Change object: B scales with its kHat. Tune tauDes: larger = smoother/slower, smaller =
        faster/noisier.
   Limits: BMin = 100 / BMax = 3000 clip B. With tauDes = 0.186, B hits BMax when
   k_env > 3000 / 0.186 ~ 16000 N/m and BMin when k_env < ~540 N/m: a stiffer or softer object is
   then no longer scaled and does not get the same response time. Raise BMax (and kMax in the
   estimator) before testing stiffer objects.
   Real test of the adaptation: change the object's contact softness (e.g. solref) and check that
   (1) kHat moves accordingly and (2) with kAdaptiveB = true and tauDes fixed, the force response
   time stays about the same.
 
 
   8. REFERENCE FEEDFORWARD (uses kHat_)
   -------------------------------------
   Problem: with the damper-only admittance the force follows the reference as a first-order lag
   (tau = B/k_env), so a time-varying lambdaRef is tracked with delay and amplitude loss, and
   lowering B only trades delay for noise.
   Fix: since d(force)/dt = k_env * vClose, the closing speed needed just to follow the reference is
           vFF = d(fRef)/dt / kHat_          [N/s] / [N/m] = [m/s]
   which is added to the feedback term. The feedback then only has to correct errors.
   Combined with the adaptive B (B = tauDes * kHat_) the whole command is
           vDes = (1/kHat_) * [ (fRef - fMeas)/tauDes + d(fRef)/dt ]
   i.e. the force error is driven to follow the reference rate plus an exponential correction with
   time constant tauDes. (This identity breaks if B is clipped by BMin/BMax.)
   Implementation notes:
   - d(fRef)/dt is a finite difference of fRef (member fRefPrev_). On the first cycle after a reset
     (fRefPrevValid_ = false) vFF = 0. While kValid_ is false vFF = 0.
   - It differentiates the REFERENCE, not the measurement, so it adds no sensor noise. The reference
     must be smooth (generated). A step in lambdaRef gives a one-sample spike that just saturates at
     vMax: rate-limit or filter step references.
   - The sum is clamped by vMax together with the feedback term.
   - If kHat_ is wrong by some percent, the feedforward is wrong by the same percent and the
     feedback corrects the rest. It cannot destabilise the loop by itself because the
     feedback loop is unchanged.
 
 
   9. TUNING PROCEDURE (suggested) AND SYMPTOMS
   --------------------------------------------
   Signals worth logging: lambdaRef, lambdaMeas, vDesClose, vFF, B, cRef_ - c, s, zSt_, aClose, f,
   kHat_, kRaw_, kCoh_, kValid_.
   Test first with robots stationary and a step / slow sinusoid in lambdaRef, then with the robots
   moving (where the spikes appear).
 
   Step A - inner loop alone: check that s stays small and that (cRef_ - c) is small.
            If s is large or noisy -> k1/k2/phi; if tracking is slow -> raise lambdaS or k1.
   Step B - outer loop: tune B / tauDes (then vMax, eMax) as described in section 3.
   Step C - feedforward: compare force tracking error and phase on the sinusoid with and without it
            (delete or comment the block, or set vFF = 0).
   Step D - repeat with the arms moving; if force spikes remain, the cause is likely force-sensor
            noise or motion coupling reaching the loop through fMeas: then low-pass lambdaMeas
            (about 10-20 Hz) before this function, on BOTH versions for a fair comparison.
 
   Symptom                                        Likely cause / action
   ---------------------------------------------  ------------------------------------------
   Slow, delayed force response (but smooth)      B too high / tauDes too large, vMax too low;
                                                  feedforward off or kHat inaccurate
   Force noisy / jittery                          B too low, k1 too high, phi too small
   Sine-like oscillation around the reference     k2 too high (first), or B too low
   z or aClose pinned at zMax / aMax              limits too low or gains too high
   Lunge after contact loss / slip                eMax too large
   Never reaches the force (steady offset)        eMax too small, vMax too low, zMax too low
   kValid never true                              excitation (phiRmsMin), coherence (cohMin),
                                                  force below fMin, or kRaw negative (sign)
   kHat pinned at kMax / B pinned at BMin, BMax   clamps too tight for this object
   Feedforward makes it worse                     kHat wrong (check kCoh, kRaw), reference not
                                                  smooth, delay between force and motion
 
 
   10. STATE AND RESPONSIBILITIES OF THE CALLER
   --------------------------------------------
   Members used: gap0_ (gap at grasp time), cRef_ (position reference, integrated here),
   zSt_ (STA integral state), dt_ (control period), M_ (mass/inertia of the task), plus the
   estimator members (kInit_, kValid_, kLpC_, kLpF_, kScc_, kScf_, kSff_, kHat_, kRaw_, kCoh_) and
   the feedforward members (fRefPrev_, fRefPrevValid_).
   When the grasp starts or the squeeze is (re)activated: set gap0_, cRef_ = 0 (as for the PI),
   zSt_ = 0 and call resetStiffnessEstimate() (it also resets the feedforward state). Do not let
   zSt_ or kHat_ carry over from a previous squeeze or a different object.
 
 
   11. EXPERIMENT LOG (facts, fill in the blanks as you test)
   ----------------------------------------------------------
   Setup: mc_rtc + MuJoCo sim, two arms squeezing a box (geom half-sizes 0.10 x 0.145 x 0.1,
   condim 6, friction 1 1 1, solimp 0.9 0.7 0.01, solref 0.001 1). Controller at 1 kHz
   (dt_ = 0.001 s). lambdaReference is a sinusoid around -16 N.
   - PI / STA with fixed B = 600, STA k1 = 3, k2 = 5, lambdaS = 1.2, phi = 0.002: force tracking
     good (error ~0.03 N out of 16 N when near the sinusoid peak), "not perfectly smooth and a bit
     delayed".
   - Estimator (kAdaptiveB off): kHat ~ 3220 N/m, stable (3219.95 -> 3220.03 between two samples).
     tauCurrent = 600 / 3220 = 0.186 s; force-loop bandwidth ~ 5.4 rad/s.
   - Adaptive B on, tauDes = 0.186 (B ~ 600 on this box): result: ______________________________
   - Feedforward on: result (tracking error, phase delay, spikes): _______________________________
   - Other objects (different solref / stiffness): kHat = ______ , B = ______ , result: _________
   Note on MuJoCo: its soft contact stiffness comes from solref/solimp, the timestep (timeconst is
   clamped to at least 2*timestep by default) and the effective mass at the contact, so kHat cannot
   be read from the XML; it has to be measured.
 
 
   12. CONTEXT FROM THE LITERATURE (from a quick search, not a survey)
   -------------------------------------------------------------------
   - Adaptive Admittance Control for Internal-Force Tracking in Dual-Arm Palm Grasping (2026):
     adapts the admittance damping gain online from the internal-force tracking error; reports
     better tracking and responsiveness than constant gain, even with suboptimal parameters.
     Closest to this setup (adapting B from the error instead of from a stiffness estimate).
   - Adaptive hybrid impedance control for dual-arm cooperative manipulation with object
     uncertainties: estimates the object stiffness from the internal-force tracking error.
   - Identification of Compliant Contact Parameters and Admittance Force Modulation on a
     Non-stationary Compliant Surface (arXiv 2004.09729): online stiffness identification improves
     admittance force control (same idea as kHat_).
   - Force and Time-Optimal Trajectory Planning for Dual-Arm Unilateral Cooperative Grasping:
     chooses the internal-force profile (minimal prestress for the static-friction condition) and
     applies it with admittance control: the "how to choose lambdaRef" side.
   - Admittance control on position-controlled robots is known to become unstable with stiff
     environments and time delays: argument for the planned move to torque control.
   Not found in the quick search: super-twisting applied to this specific squeeze-force problem.
 
 
   13. KNOWN LIMITATIONS AND NEXT STEPS
   ------------------------------------
   - kMax = 5e4 and BMax = 3000 were guesses; widen them before testing stiffer objects.
   - Spikes while the robots move: the measured squeeze force probably still contains inertial,
     gravity and motion-coupling components of the object. Compensate them explicitly (object
     dynamics feedforward) or filter lambdaMeas.
   - Torque-level control (planned): admittance on position-controlled arms gets fragile with
     stiff or varying contacts.
   - STA gains k1, k2 rely on a guessed disturbance bound L; adaptive super-twisting gains would
     remove the guess.
   - lambdaRef itself: real systems usually set the squeeze level from friction/weight margins
     (with slip detection) rather than following a prescribed profile.
   ==========================================================================================
  */
 
  // ---- Outer force loop ----
  constexpr double B0   = 600.0;  // [N s/m]  fixed B (and fallback of the adaptive B)
  constexpr double vMax = 0.05;   // [m/s]    saturation of the desired closing speed
  constexpr double eMax = 0.001;  // [m]      anti-windup bound on (cRef_ - c)
 
  // ---- Sliding surface ----
  constexpr double lambdaS = 1.2;   // [1/s]  = Kp / Kv of the linear version
 
  // ---- Super-twisting gains ----
  constexpr double k1   = 3.0;    // [m/s^2 / sqrt(m/s)]
  constexpr double k2   = 5.0;    // [m/s^3 / (m/s)]
  constexpr double phi  = 0.002;  // [m/s]    boundary layer for sign() -> tanh
  constexpr double zMax = 1.0;    // [m/s^2]  anti-windup on the integral state
  constexpr double aMax = 3.0;    // [m/s^2]  saturation of the commanded relative acceleration
 
  const Eigen::Vector3d d = ee.right.translation() - ee.left.translation();
  const double gap = d.norm();
  const Eigen::Vector3d n = d / gap;
 
  const double fRef  = -lambdaRef;
  const double fMeas = -lambdaMeas;
  const double c = gap0_ - gap;                       // closing displacement
 
  double B = B0;     // OUTSIDE the blocks: deleting them leaves the fixed-B function
 
  // >>>>>>>>>> BEGIN STIFFNESS ESTIMATION: update (needed by adaptive B and feedforward) >>>>>>>>>>
  updateStiffnessEstimate(fMeas, c);    // runs every cycle so kHat_ can be logged
  // <<<<<<<<<< END STIFFNESS ESTIMATION: update <<<<<<<<<<
 
  // >>>>>>>>>> BEGIN ADAPTIVE B (delete up to END marker to remove) >>>>>>>>>>
  constexpr bool   kAdaptiveB = true;   // false = identical behaviour to the fixed-B version
  constexpr double tauDes     = 0.186;  // [s]      desired force-loop time constant: B = tauDes * kHat
                                        //          = B0 / kHat measured on the sim box (600 / 3220)
  constexpr double BMin       = 100.0;  // [N s/m]  limits on the adapted B
  constexpr double BMax       = 3000.0; // [N s/m]
 
  if(kAdaptiveB) B = adaptiveB(tauDes, BMin, BMax, B0);
  // <<<<<<<<<< END ADAPTIVE B <<<<<<<<<<
 
  double vFF = 0.0;  // OUTSIDE the block: stays 0 if the block is deleted
 
  // >>>>>>>>>> BEGIN FEEDFORWARD (delete up to END marker to remove; needs the update block) >>>>>>>>>>
  if(kValid_ && fRefPrevValid_) vFF = (fRef - fRefPrev_) / dt_ / kHat_;   // [N/s] / [N/m] = [m/s]
  fRefPrev_ = fRef;
  fRefPrevValid_ = true;
  // <<<<<<<<<< END FEEDFORWARD <<<<<<<<<<
 
  // Outer loop: force error (+ reference rate) -> desired closing speed
  const double vDesClose = std::clamp((fRef - fMeas) / B + vFF, -vMax, vMax);
 
  // Position reference driven by the force loop
  cRef_ += vDesClose * dt_;
  cRef_  = c + std::clamp(cRef_ - c, -eMax, eMax);    // anti-windup
 
  // Sliding variable
  const double vClose = -n.dot(ee.rightVelocity.linear() - ee.leftVelocity.linear());
  const double s = (vClose - vDesClose) + lambdaS * (c - cRef_);
 
  // Super-twisting (smoothed sign)
  const double sgn = std::tanh(s / phi);
  zSt_ = std::clamp(zSt_ - k2 * sgn * dt_, -zMax, zMax);
  const double aClose = std::clamp(-k1 * std::sqrt(std::abs(s)) * sgn + zSt_, -aMax, aMax);
 
  // Same conversion to wrench as before
  const double mLin = M_.tail<3>().mean();
  const double f    = 0.5 * mLin * aClose;
 
  Eigen::Matrix<double, 12, 1> w = Eigen::Matrix<double, 12, 1>::Zero();
  w.segment<3>(3) =  f * n;
  w.segment<3>(9) = -f * n;
  return w;
}


Eigen::Matrix<double, 12, 1> CooperativeGrasp::computeObjectImpedance( // inputs -> world frame 
    const Test & t,
    const sva::PTransformd & desiredPose, 
    const sva::MotionVecd & desiredVelocity, 
    const std::optional<sva::MotionVecd> & desiredAcceleration)
{
    //==========================================================
    // POSE & VELOCITY ERRORS
    //==========================================================
    const sva::MotionVecd error = sva::transformError(computeCurrentObjectPose(), desiredPose);
    
    
    const Eigen::Vector3d omega = 0.5 * (t.eePoses.leftVelocity.angular() + t.eePoses.rightVelocity.angular());
    const Eigen::Vector3d offset_left_W = t.eePoses.left.rotation() * initialOffsets_.leftEE;
    const Eigen::Vector3d offset_right_W = t.eePoses.right.rotation() * initialOffsets_.rightEE;

    const Eigen::Vector3d v = 0.5 * (
        t.eePoses.leftVelocity.linear() + t.eePoses.leftVelocity.angular().cross(offset_left_W) + 
        t.eePoses.rightVelocity.linear() + t.eePoses.rightVelocity.angular().cross(offset_right_W)
    );

    Eigen::Vector6d current_vel;
    current_vel << omega, v;
    
    // Pack errors into 6D vectors [angular; linear]
    const Eigen::Vector6d e = toVec6(error);
    const Eigen::Vector6d vel_error = toVec6(desiredVelocity) - current_vel;
    
    //==========================================================
    // IMPEDANCE LAW & DISTRIBUTION IN WORLD FRAME
    //==========================================================
    Eigen::Vector6d wrench_cmd = K_.cwiseProduct(e) + D_.cwiseProduct(vel_error);
    if(desiredAcceleration) { wrench_cmd += M_.cwiseProduct(toVec6(*desiredAcceleration)); }
    
    const Eigen::Matrix<double, 12, 1> contactWrenches = t.graspMatrices.Gpinv * wrench_cmd;
    return {contactWrenches};
}




Test CooperativeGrasp::update()
{
    Test t;
    t.eePoses = getEndEffectorPoses();
    t.u = computeCurrentObjectPose().rotation().transpose().col(0).normalized();
    t.offsets = computeInitialOffsets(t.eePoses, buildObjectCenter(t.eePoses));


    
    t.graspMatrices = computeGraspMatricesWorldFrame();   // fresh G, Gpinv, N

    return t;
}


CooperativeWrenchCommand CooperativeGrasp::computeWrenchCommand(
        const sva::PTransformd & desiredPose,
        const sva::MotionVecd & desiredVelocity,
        const std::optional<sva::MotionVecd> & desiredAcceleration,
        double lambdaReference,
        double dt)
{
    if (!initialized_) { throw std::runtime_error("CooperativeGrasp is not initialized");}

    // ------------------------------------------------------------
    // Update values
    // ------------------------------------------------------------
        Test current;
        current = update();


        //mc_rtc::log::info("pL = {}", current.eePoses.left.translation());
        //mc_rtc::log::info("pR = {}", current.eePoses.right.translation());
        //mc_rtc::log::info("pR-pL = {}", current.eePoses.right.translation() - current.eePoses.left.translation());
        //mc_rtc::log::info("u = {}", current.u);
        mc_rtc::log::info("kHat_ = {}", kHat_);
        
    // ------------------------------------------------------------
    // Update Grasp Axis
    // ------------------------------------------------------------
        Eigen::Matrix<double, 12, 1> squeezeAxis = Eigen::Matrix<double, 12, 1>::Zero();
        squeezeAxis.segment<3>(3) =  current.u;
        squeezeAxis.segment<3>(9) = -current.u;
    // ------------------------------------------------------------
    // Update Selection Matrix
    // ------------------------------------------------------------
        Eigen::Matrix6d S = selectionMatrix(current.u);
        //mc_rtc::log::info("selectionMatrix: {}", S);
    // ------------------------------------------------------------
    // Measure Internal Squeeze
    // ------------------------------------------------------------
        Eigen::Matrix<double, 12, 1> internalForces = current.graspMatrices.nullSpace * measureWorldWrenches(current.eePoses);
        double lambdaMeasured = squeezeAxis.dot(internalForces) / squeezeAxis.squaredNorm();
        mc_rtc::log::info("lambdaMeasured = {}", lambdaMeasured);
        mc_rtc::log::info("lambdaReference = {}", lambdaReference);
        forceOut_.lambdaReference = lambdaReference;
        forceOut_.lambdaMeasured = lambdaMeasured;
    // ------------------------------------------------------------
    // Squeeze Force Control
    // ------------------------------------------------------------
        Eigen::Matrix<double, 12, 1> wSqueeze =  computeInternalSqueezeSTA( lambdaReference, lambdaMeasured,current.u ,current.eePoses);
        mc_rtc::log::info("wSqueeze = {}", wSqueeze);
    // ------------------------------------------------------------
    // Relative Impedance
    // ------------------------------------------------------------
        Eigen::Matrix<double, 12, 1> wRelative = current.graspMatrices.nullSpace * computeRelativeImpedance(current.eePoses, S);
    // ------------------------------------------------------------
    // Nullspace Control
    // ------------------------------------------------------------
        Eigen::Matrix<double, 12, 1> wInternal = wRelative + wSqueeze;
    // ------------------------------------------------------------
    // External Control
    // ------------------------------------------------------------    
        Eigen::Matrix<double, 12, 1> w_cmd = computeObjectImpedance(current, desiredPose, desiredVelocity, desiredAcceleration) + wInternal;        
    // ------------------------------------------------------------
    // Split into left/right arm commands
    // ------------------------------------------------------------
        CooperativeWrenchCommand result;
        result.stacked = w_cmd;
        result.left = M_.cwiseInverse().cwiseProduct(w_cmd.segment<6>(0));
        result.right = M_.cwiseInverse().cwiseProduct(w_cmd.segment<6>(6));
        result.leftInternal = M_.cwiseInverse().cwiseProduct(wInternal.segment<6>(0));
        result.rightInternal = M_.cwiseInverse().cwiseProduct(wInternal.segment<6>(6));
        result.leftExternal = Eigen::Matrix<double, 6, 1>::Zero();
        result.rightExternal = Eigen::Matrix<double, 6, 1>::Zero();
        return result;
}


// ---------------------------------------------------------------------------
// Single entry point: every quantity is computed exactly once
// ---------------------------------------------------------------------------
CooperativeGraspOutput CooperativeGrasp::getGraspOutput() const
{
const auto eePoses = getEndEffectorPoses(); // END-EFFECTOR POSES
const auto objectCenter = buildObjectCenter(eePoses); // OBJECT CENTER
const auto allOffsets = computeInitialOffsets(eePoses,objectCenter); // INITIAL OFFSETS
const EndEffectorOffsets offsets{allOffsets.leftWorld,allOffsets.rightWorld}; // INITIAL OFFSETS End Effector
const auto graspFrame = buildGraspFrame(); // GRASP FRAME
const auto R = graspFrame.X_world_object.rotation(); // GRASP FRAME ROTATION
const auto currentObjectPose = computeCurrentObjectPose(); // INITIAL/CURRENt OBJECT POSE
return {
eePoses,
objectCenter,
offsets,
graspFrame,
R,
R.col(0),
R.col(1),
R.col(2),
(eePoses.right.translation() -
eePoses.left.translation()).norm(),
currentObjectPose
};
}