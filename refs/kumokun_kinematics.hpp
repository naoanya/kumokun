/**********************************************************************/
/**
 * @brief  Kumo-kun Kinematics
 * @author naoa
 */
/**********************************************************************/
#pragma once
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Include files
 *----------------------------------------------------------------------
 */
#include <cstdio>
#include <cstdlib>
#include <cstdbool>
#include <cstdint>
#include <cstring>
#include <string>
#include <memory>
#include <list>

#include "Eigen/Eigen"
#include "vectorpr.hpp"
#include "smart_pointer.hpp"

#include "kumokun_defs.hpp"

using namespace Eigen;

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Definitions
 *----------------------------------------------------------------------
 */

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Class Forword Declarations
 *----------------------------------------------------------------------
 */

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Name Space
 *----------------------------------------------------------------------
 */

namespace KumoKun {

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Definitions
 *----------------------------------------------------------------------
 */

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Class definitions
 *----------------------------------------------------------------------
 */

class Kinematics
{
public:
    class Servo
    {
    public:
        double degreeIk_ = 0.0f;
        Vector3d vectorFk_;
        Vector3d vectorFkRel_;
        Vector3d rot_;
    };

    class Leg
    {
    public:
        int id_;
        double degree_;

        SP<Servo> pHipZServo_;
        SP<Servo> pHipYServo_;
        SP<Servo> pKneeServo_;
        SP<Servo> pToeServo_;
    };

public:
    explicit Kinematics();
    virtual ~Kinematics();

    int setup(void);

public:
    int inverseKinetics(const ToeVectors& toeVectors);
    int inverseKinetics(int legid, const Vector3d& vToe, bool doFk = true);

public:
    int forwordKinetics(void);
    int forwordKinetics(const SP<Leg> & leg);

public:
    ServoDegrees getServosDegree(void);
    ToeVectors getToeVectors(void);
    VectorInfo getKinematicsInfo(void);

public:
    SP<Leg> getLegWithID(int id);
    void updateVectorInfo(void);

public: // private (public for test)
    Vector3d rotBody_;
    std::vector<SP<Leg>> pLegs_;
    VectorInfo vectors_;
    std::vector<SP<Servo>> pServos_;

    // constant parameters
    Vector3d vBodyOrigin_;
    Vector3d vOriginToHipZ_;
    Vector3d vHipZToHipY_;
    Vector3d vHipYToKnee_;
    Vector3d vKneeToToe_;

public:
    static int inverseKinetics(
        const Vector3d& param_vBodyOrigin,
        const Vector3d& param_vOriginToHipZ,
        const Vector3d& param_vHipZToHipY,
        const Vector3d& param_vHipYToKnee,
        const Vector3d& param_vKneeToToe,
        const double&  dLeg,
        const Vector3d& rotBody,
        const Vector3d& vTargetToe,
        double& dHipZ,
        double& dHipY,
        double& dKnee
    );

    static int forwordKinetics(
        const Vector3d& param_vBodyOrigin,
        const Vector3d& param_vOriginToHipZ,
        const Vector3d& param_vHipZToHipY,
        const Vector3d& param_vHipYToKnee,
        const Vector3d& param_vKneeToToe,
        const double& dLeg,
        const double& dHipZ,
        const double& dHipY,
        const double& dKnee,
        Vector3d& vHipZRel,
        Vector3d& vHipYRel,
        Vector3d& vKneeRel,
        Vector3d& vToeRel,
        Vector3d& vHipZ,
        Vector3d& vHipY,
        Vector3d& vKnee,
        Vector3d& vToe,
        Vector3d& rHipZ,
        Vector3d& rHipY,
        Vector3d& rKnee
    );

public:
    void dump(void);

};

}; // namespace KumoKun
