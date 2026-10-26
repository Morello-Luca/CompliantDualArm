#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>




 
 
// >>>>>>>>>>>>>>>>>>>> BEGIN STIFFNESS ESTIMATION >>>>>>>>>>>>>>>>>>>>
 
/*
  STIFFNESS ESTIMATION  (kHat_)
  =============================
 
  1. GOAL
  -------
  Estimate the effective contact stiffness k_env [N/m] seen by the squeeze loop, with the model f = k_env * c + offset
        
        f = fMeas (= -lambdaMeas, the squeeze force, < 0 when squeezing),
        c = gap0_ - gap (closing displacement of the two end effectors).
        k_env is the stiffness of the WHOLE chain between the two measured end-effector frames and the
  
  force: object + fingers + any compliance between the frame and the contact. Since c is computed from the actual end-effector poses (not the commanded ones), the arms' own position-servo compliance does NOT enter in simulation;
  with real hardware, compliance of the structure does.
 
  2. WHY IT IS USEFUL
  -------------------
  The force loop is an admittance:  v_des = error / B, 
  and                               d(force)/dt = k_env * vClose, 
  
  so the force error behaves like a first-order lag with time constant  tau = B / k_env        (force-loop bandwidth ~ k_env / B [rad/s])

  - It turns "tune B by trial and error" into "choose tauDes": B = tauDes * kHat gives the same force response time for soft and stiff objects (adaptive B).
  - It provides the model gain for the reference feedforward: vFF = d(fRef)/dt / kHat.
  - It is a diagnostic: it tells you the loop speed and whether the outer loop might be too fast for the inner loop (bandwidth k_env / B must stay well below the inner-loop bandwidth).
 
  3. METHOD
  ---------
   1. High-pass f and c with the SAME first-order filter (x_hp = x - lowpass(x), time constant tHighPass). This removes offsets (unknown contact point, preload) and keeps only the variations, which are the part that carries the slope df/dc.
   
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
  kValid_ must be 1; 
  kRaw_ should hover around kHat_; 
  kCoh_ should be close to 1 (>0.9 is great);
  kHat_ should be flat over a cycle of the sinusoid. 
  kHat_ drifting with the phase of the
  sinusoid = nonlinear contact or delay.
 
*/




// ============================================================
// CLASS DECLARATION
// ============================================================

class ObjectStiffnessEstimator
{
public:
    // Constructor
    explicit ObjectStiffnessEstimator(double dt);

    // Main functions
    void reset();
    void update(double force, double displacement);

    // Getters
    bool isValid() const noexcept;
    double estimate() const noexcept;
    double rawEstimate() const noexcept;
    double coherence() const noexcept;

private:
    // Sampling time
    double dt_;

    // Initialization
    bool   kInit_  = false;   // filters initialised
    bool   kValid_ = false;   // at least one accepted estimate

    // Filter states
    double kLpC_   = 0.0;     // low-pass state of c   (high-pass = c - kLpC_)
    double kLpF_   = 0.0;     // low-pass state of f

    // Least-squares sums
    double kScc_   = 0.0;     // weighted sum of phi^2
    double kScf_   = 0.0;     // weighted sum of phi*y
    double kSff_   = 0.0;     // weighted sum of y^2
    // Stiffness estimates
    double kHat_   = 0.0;     // [N/m] accepted, filtered stiffness estimate
    double kRaw_   = 0.0;     // [N/m] last raw least-squares value (logging)
    double kCoh_   = 0.0;     // [-]   fit quality of the last evaluation (logging)
};


// ============================================================
// FUNCTION IMPLEMENTATIONS
// ============================================================

// ------------------------------------------------------------
// Constructor
// ------------------------------------------------------------

inline ObjectStiffnessEstimator::ObjectStiffnessEstimator(double dt)
    : dt_(dt)
{
    if(dt_ <= 0.0)
    {
        throw std::invalid_argument(
            "ObjectStiffnessEstimator: dt must be positive");
    }
}


// ------------------------------------------------------------
// Reset
// ------------------------------------------------------------

inline void ObjectStiffnessEstimator::reset()
{
    kInit_ = false;
    kValid_ = false;

    kLpC_ = kLpF_ = 0.0;

    kScc_ = kScf_ = kSff_ = 0.0;

    kHat_ = kRaw_ = kCoh_ = 0.0;
}


// ------------------------------------------------------------
// Update stiffness estimate
// ------------------------------------------------------------

inline void ObjectStiffnessEstimator::update(double f, double c)
{
    constexpr double tHighPass = 1.0;
    constexpr double tWindow   = 2.0;
    constexpr double tK        = 1.0;

    constexpr double fMin      = 2.0;
    constexpr double phiRmsMin = 2e-5;
    constexpr double cohMin    = 0.5;

    constexpr double kMin      = 500.0;
    constexpr double kMax      = 5e4;

    // Initialize filter states
    if(!kInit_)
    {
        kLpC_ = c;
        kLpF_ = f;
        kInit_ = true;
        return;
    }

    // High-pass filtering
    const double aHP = dt_ / (tHighPass + dt_);

    const double phi = c - kLpC_;
    const double y   = f - kLpF_;

    kLpC_ += aHP * phi;
    kLpF_ += aHP * y;

    // Exponentially weighted least-squares sums
    const double lam = 1.0 - dt_ / tWindow;

    kScc_ = lam * kScc_ + phi * phi;
    kScf_ = lam * kScf_ + phi * y;
    kSff_ = lam * kSff_ + y * y;

    // Contact check
    if(std::abs(f) < fMin)
    {
        return;
    }

    // Displacement excitation check
    const double phiRms = std::sqrt(kScc_ * (1.0 - lam));

    if(phiRms < phiRmsMin || kScc_ <= 0.0)
    {
        return;
    }

    // Raw stiffness estimate
    kRaw_ = kScf_ / kScc_;

    // Coherence
    kCoh_ = (kScf_ * kScf_)
          / (kScc_ * kSff_ + 1e-30);

    // Reject unreliable estimates
    if(kRaw_ <= 0.0 || kCoh_ < cohMin)
    {
        return;
    }

    // Clamp stiffness
    const double kc = std::clamp(kRaw_, kMin, kMax);

    // Initialize or filter the accepted estimate
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


// ------------------------------------------------------------
// Getters
// ------------------------------------------------------------

inline bool ObjectStiffnessEstimator::isValid() const noexcept
{
    return kValid_;
}

inline double ObjectStiffnessEstimator::estimate() const noexcept
{
    return kHat_;
}

inline double ObjectStiffnessEstimator::rawEstimate() const noexcept
{
    return kRaw_;
}

inline double ObjectStiffnessEstimator::coherence() const noexcept
{
    return kCoh_;
}