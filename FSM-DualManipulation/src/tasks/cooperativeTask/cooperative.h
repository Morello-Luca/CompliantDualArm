#pragma once
#include <optional>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <mc_filter/LowPass.h>
#include <mc_rbdyn/Robots.h>
#include <mc_tasks/TransformTask.h>

#include <cmath>
#include <limits>

// =========================================================================
// STRUCTS
// =========================================================================
    template<typename T>
    struct EndEffectorPair
    {
        T left;
        T right;
    };


    struct EndEffectorPoses
    {
        sva::PTransformd left;
        sva::PTransformd right;
        sva::MotionVecd leftVelocity;
        sva::MotionVecd rightVelocity;
    };



    struct GraspMatrices
    {
        Eigen::Matrix<double, 6, 12> G;
        Eigen::Matrix<double, 12, 6> Gpinv;      // computed once, here
        Eigen::Matrix<double, 12, 12> nullSpace; // I - Gpinv * G
    };



    struct GraspFrame
    {
    sva::PTransformd X_world_object;
    sva::PTransformd X_object_left;
    sva::PTransformd X_object_right;
    sva::PTransformd X_world_left;
    sva::PTransformd X_world_right;
    };





struct EndEffectorOffsets
{
    Eigen::Vector3d leftWorld;
    Eigen::Vector3d rightWorld;

    Eigen::Vector3d leftEE;
    Eigen::Vector3d rightEE;
};


    struct Test 
    {
        EndEffectorPoses eePoses;
        Eigen::Vector3d u;
        EndEffectorOffsets offsets;
        GraspMatrices graspMatrices;
    };

    struct CooperativeGraspOutput
    {
    EndEffectorPoses eePoses;            // END-EFFECTOR POSES IN WORLD
    Eigen::Vector3d objectCenter;        // OBJECT / GRASP CENTER
    EndEffectorOffsets offsets;          // EE OFFSETS FROM OBJECT CENTER, WORLD FRAME
    GraspFrame graspFrame;               // OBJECT / GRASP FRAME
    Eigen::Matrix3d rotationObject;      // OBJECT FRAME ROTATION
    Eigen::Vector3d xAxis, yAxis, zAxis; // Explicit axes, useful for debugging
    //GraspMatrices graspMatrices;         // GRASP MATRICES
    double graspDistance = 0.0;          // BASIC VALIDITY INFORMATION
    sva::PTransformd currentObjectPose; // Current object pose reconstructed from the grasp
  // Convenience getter/value

};


    struct CooperativeGraspInfo
    {
    EndEffectorPoses poses;
    Eigen::Vector3d objectCenter;
    EndEffectorOffsets offsets;
    GraspFrame graspFrame;
    GraspMatrices graspMatrices;

    Eigen::Vector3d squeezeAxis;
    Eigen::Matrix<double, 12, 1> n_s;
    Eigen::Matrix<double, 12, 1> n_squeeze;

    bool initialized = false;
    };


    struct GraspState
    {
    sva::PTransformd leftPose;
    sva::PTransformd rightPose;

    Eigen::Vector3d objectCenter;
    Eigen::Matrix3d R_object;

    Eigen::Vector3d offsetLeft_world;
    Eigen::Vector3d offsetRight_world;

    GraspFrame graspFrame;

    Eigen::Vector3d squeezeAxis_world;
    };

    struct CooperativeWrenchCommand
    {
    Eigen::Matrix<double, 6, 1> left;
    Eigen::Matrix<double, 6, 1> right;
    Eigen::Matrix<double, 12, 1> stacked;
    Eigen::Matrix<double, 6, 1>leftInternal; 
    Eigen::Matrix<double, 6, 1>rightInternal ;
    Eigen::Matrix<double, 6, 1>leftExternal ;
    Eigen::Matrix<double, 6, 1>rightExternal; 
    };

struct InternalSqueezeInfo
{
    Eigen::Matrix<double, 12, 1> n_s;
    Eigen::Matrix<double, 12, 1> n_squeeze;
    double lambdaMeasured = 0.0;
};


struct CooperativeAccelerationOutput
{
    Eigen::Vector6d left;
    Eigen::Vector6d right;
};






struct ForceControlOutput
{
    double lambdaReference = 0.0;
    double lambdaMeasured = 0.0;

Eigen::Vector3d filteredLeftForce = Eigen::Vector3d::Zero();
Eigen::Vector3d filteredRightForce = Eigen::Vector3d::Zero();






};







class CooperativeGrasp
{
public:

    CooperativeGrasp(
        const mc_rbdyn::Robots & robots,
        unsigned int leftRobotIndex,
        unsigned int rightRobotIndex,
        const std::string& leftFrame,
        const std::string& rightFrame,
        double dt);




ForceControlOutput forceControlInfo() const
{
    return  forceOut_;
}


Eigen::Matrix<double, 12, 1> computeObjectImpedance(
    const Test & t,
    const sva::PTransformd & desiredPose,
    const sva::MotionVecd & desiredVelocity,
    const std::optional<sva::MotionVecd> & desiredAcceleration = std::nullopt);





Eigen::Matrix<double, 12, 1> computeRelativeImpedance(EndEffectorPoses eePoses, Eigen::Matrix6d S) const;


    void setImpedanceGains(
        const Eigen::Vector6d& K,
        const Eigen::Vector6d& M,
        double zeta = 1.0);


        void setRelativeImpedanceGains( const Eigen::Vector6d& K, const Eigen::Vector6d& M, double zeta = 1.0);
        

    const Eigen::Vector6d& stiffness() const { return K_; }
    const Eigen::Vector6d& damping() const { return D_; }
    const Eigen::Vector6d& mass() const { return M_; }


    void initializeGrasp();

    const EndEffectorOffsets& getOffsets() const { return initialOffsets_;}


    const CooperativeGraspInfo & getGraspInfo() const { return graspInfo_; }
    // -------------------------------------------------------------------------
    // IMPEDANCE GAINS
    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    // PASS GRASP INFORMATION FOR DEBUG
    // -------------------------------------------------------------------------
    CooperativeGraspOutput getGraspOutput() const;
    // -------------------------------------------------------------------------
    // OBJECT MOTION
    // -------------------------------------------------------------------------
    Eigen::Vector6d computeObjectVelocity(const EndEffectorOffsets & offsets) const;
    // -------------------------------------------------------------------------
    // OBJECT EXTRNAL CENTER INFO
    // -------------------------------------------------------------------------
    void setCameraCenter(const Eigen::Vector3d & center){ cameraCenter_ = center;}
    void clearCameraCenter() {cameraCenter_.reset();}

    // -------------------------------------------------------------------------
    // INTERNAL FORCE
    // -------------------------------------------------------------------------
    Eigen::Matrix<double, 12, 1> measureWorldWrenches(EndEffectorPoses eePoses);
    // -------------------------------------------------------------------------
    // OBJECT + INTERNAL -> ARM COMMANDS
    // -------------------------------------------------------------------------
    GraspState & updateGrasp();
    CooperativeWrenchCommand computeWrenchCommand(
const sva::PTransformd & desiredPose,
    const sva::MotionVecd & desiredVelocity,
    const std::optional<sva::MotionVecd> & desiredAcceleration,
    double lambdaReference,
    double dt);

    void init();


protected:
        double cutoffPeriod_ = 0.10;
        mc_filter::LowPass<sva::ForceVecd> lowPassLeft_;
        mc_filter::LowPass<sva::ForceVecd> lowPassRight_;


        //void reset() override;
        //void update(mc_solver::QPSolver & solver) override;
        //void addToSolver(mc_solver::QPSolver & solver) override;
        //void removeFromSolver(mc_solver::QPSolver & solver) override;
        //void addToLogger(mc_rtc::Logger & logger) override;
       

private:
    Eigen::Vector6d K_ = Eigen::Vector6d::Zero();
    Eigen::Vector6d D_ = Eigen::Vector6d::Zero();
    Eigen::Vector6d M_ = Eigen::Vector6d::Zero();
    Eigen::Vector6d K_rel_ = Eigen::Vector6d::Zero();
    Eigen::Vector6d D_rel_ = Eigen::Vector6d::Zero();
    Eigen::Vector6d M_rel_ = Eigen::Vector6d::Zero();

    ForceControlOutput forceOut_;

  sva::PTransformd computeCurrentObjectPose() const;
    EndEffectorOffsets initialOffsets_;
    // =========================================================================
    // ROBOT INFO
    // =========================================================================
        const mc_rbdyn::Robots & robots_;
        unsigned int leftRobotIndex_;
        unsigned int rightRobotIndex_;
        std::string leftFrame_;
        std::string rightFrame_;
        CooperativeGraspInfo graspInfo_;
        GraspState graspState_;
    // =========================================================================
    // CAMERA INFO
    // =========================================================================
        std::optional<Eigen::Vector3d> cameraCenter_;
    // =========================================================================
    // FUNCTIONS
    // =========================================================================
        // =========================================================================
        // END EFFECTOR 
        // =========================================================================
            EndEffectorOffsets buildEndEffectorOffsets();
            EndEffectorPoses getEndEffectorPoses() const;
        // =========================================================================
        // GRASP FRAME
        // =========================================================================
            Eigen::Vector3d buildObjectCenter(const EndEffectorPoses & poses) const;
            GraspFrame buildGraspFrame() const;
        // =========================================================================
        // GRASP MATRIX
        // =========================================================================
            GraspMatrices buildGraspMatrices();
            GraspMatrices graspMatrices_;
        // =========================================================================
        // CONTROLS
        // =========================================================================
            // IMPEDANCE
            // =========================================================================


        Eigen::Matrix<double, 12, 1> computeInternalSqueezePI(
            double lambdaRef, 
            double lambdaMeas,
            const Eigen::Vector3d & u, 
            const EndEffectorPoses & ee);

        Eigen::Matrix<double, 12, 1> computeInternalSqueezeSTA(
            double lambdaRef, 
            double lambdaMeas,
            const Eigen::Vector3d & u, 
            const EndEffectorPoses & ee);


    GraspFrame graspFrame_;
    double internalForceIntegral_ = 0.0;

double internalForceKp_ = 0.0007;
double internalForceKi_ = 0.0;



Eigen::Vector3d relPosWorld_ = Eigen::Vector3d::Zero();
Eigen::Matrix3d relRot_ = Eigen::Matrix3d::Identity();

Test update();
bool initialized_{false};

Eigen::Matrix<double, 12, 1> n_squeeze_;

double dt_ = 0.0;

Eigen::Matrix6d selectionMatrix(Eigen::Vector3d u) const;

EndEffectorOffsets computeInitialOffsets(
    const EndEffectorPoses& poses,
    const Eigen::Vector3d& center) const;



GraspMatrices computeGraspMatricesWorldFrame() const;

Eigen::Matrix<double, 12, 12> W = Eigen::Matrix<double, 12, 12>::Identity();
Eigen::Matrix<double, 12, 12> Winv = Eigen::Matrix<double, 12, 12>::Identity();


double ffGain_ = 1.0;
double kp_ = 0.02;
double ki_ = 0.02;
double kd_ = 0.3;
double refRate_ = 5.0;
double internalForceMax_ = 30.0;
double internalForceMin_ = -30.0;
double lambdaRefFiltered_ = 0.0;


Eigen::Matrix3d C_left_, C_right_;

Eigen::Vector3d offLeftObj_;
Eigen::Vector3d offRightObj_;




 
// >>>>>>>>>>>>>>>>>>>> BEGIN STIFFNESS ESTIMATION (header) >>>>>>>>>>>>>>>>>>>>
// Needed by: adaptive B and feedforward. Remove those two first, then this block.
private:
  void resetStiffnessEstimate();                      // call at every new grasp / object
  void updateStiffnessEstimate(double f, double c);   // f = fMeas, c = gap0_ - gap, called every cycle
  double adaptiveB(double tau, double bMin, double bMax, double bFallback) const;
 
  bool   kInit_  = false;   // filters initialised
  bool   kValid_ = false;   // at least one accepted estimate
  double kLpC_   = 0.0;     // low-pass state of c   (high-pass = c - kLpC_)
  double kLpF_   = 0.0;     // low-pass state of f
  double kScc_   = 0.0;     // weighted sum of phi^2
  double kScf_   = 0.0;     // weighted sum of phi*y
  double kSff_   = 0.0;     // weighted sum of y^2
  double kHat_   = 0.0;     // [N/m] accepted, filtered stiffness estimate
  double kRaw_   = 0.0;     // [N/m] last raw least-squares value (logging)
  double kCoh_   = 0.0;     // [-]   fit quality of the last evaluation (logging)
  // Logging examples (mc_rtc), e.g. in your init:
  //   logger.addLogEntry("squeeze_kHat",   this, [this]() { return kHat_; });
  //   logger.addLogEntry("squeeze_kRaw",   this, [this]() { return kRaw_; });
  //   logger.addLogEntry("squeeze_kCoh",   this, [this]() { return kCoh_; });
  //   logger.addLogEntry("squeeze_kValid", this, [this]() { return kValid_; });
// <<<<<<<<<<<<<<<<<<<< END STIFFNESS ESTIMATION (header) <<<<<<<<<<<<<<<<<<<<
   double zSt_ = 0.0;
   double cRef_ = 0.0;   // reference closing displacement [m]
double gap0_ = 0.0;   // initial gap between EEs [m]
// >>>>>>>>>>>>>>>>>>>> BEGIN FEEDFORWARD (header) >>>>>>>>>>>>>>>>>>>>
  double fRefPrev_      = 0.0;    // previous fRef, for the reference derivative
  bool   fRefPrevValid_ = false;  // false on the first cycle after a reset (no derivative yet)
// <<<<<<<<<<<<<<<<<<<< END FEEDFORWARD (header) <<<<<<<<<<<<<<<<<<<<


};

