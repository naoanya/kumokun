/******************************************************************************/
/**
 * @brief  Kumo-kun Kinematics
 * @author naoa
 */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Include files
 *----------------------------------------------------------------------
 */
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <cfloat>

#ifdef unix
#include <unistd.h>
#endif
#include <cstdio>
#include <cstdlib>
#include <cstdbool>
#include <cstdint>
#include <cstring>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>
#include <string>

#include "log.hpp"
#include "kumokun_config.hpp"
#include "kumokun_kinematics.hpp"

using namespace KumoKun;

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Definitions - Debug
 *----------------------------------------------------------------------
 */

#define DP__(...)   DP_DEBUG(__VA_ARGS__)
//#define DP__(...)

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Definitions
 *----------------------------------------------------------------------
 */

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Predefine local functions
 *----------------------------------------------------------------------
 */

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Local variables
 *----------------------------------------------------------------------
 */

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions - Public
 *----------------------------------------------------------------------
 */

Kinematics::Kinematics()
{
    // Do Nothing
}

Kinematics::~Kinematics()
{
    // Do Nothing
}

int
Kinematics::setup(void)
{
    double lBodyHeight   = CONFIG["KumoKun"]["lBodyHeight"];
    double lOriginToHipZ = CONFIG["KumoKun"]["lOriginToHipZ"];
    double lXHipServos   = CONFIG["KumoKun"]["lXHipServos"];
    double lZHipServos   = CONFIG["KumoKun"]["lZHipServos"];
    double lHipYToKnee   = CONFIG["KumoKun"]["lHipYToKnee"];
    double lKneeToToe    = CONFIG["KumoKun"]["lKneeToToe"];

    //
    // Initialize Members.
    //

    vOriginToHipZ_ = Vector3d(lOriginToHipZ,0,0);
    vHipZToHipY_   = Vector3d(lXHipServos,0.0,lZHipServos);
    vHipYToKnee_   = Vector3d(lHipYToKnee,0,0);
    vKneeToToe_    = Vector3d(lKneeToToe,0,0);

    for (int l = 0; l < 6; l++) {
        SP<Leg> leg = MakeSP<Leg>();
        leg->id_     = l;
        leg->degree_ = (360.0 / 6.0) * l;
        for (int s = 0; s < 4; s++) {
            SP<Servo> servo = MakeSP<Servo>();
            switch(s) {
            case 0:
                pServos_.push_back(servo);
                leg->pKneeServo_ = servo;
                break;
            case 1:
                pServos_.push_back(servo);
                leg->pHipYServo_ = servo;
                break;
            case 2:
                pServos_.push_back(servo);
                leg->pHipZServo_ = servo;
                break;
            case 3:
                leg->pToeServo_ = servo;
                break;
            }
        }
        pLegs_.push_back(leg);
    }

    //
    // Set Initial Condition
    //

    rotBody_     = Vector3d(0,0,0);
    vBodyOrigin_ = Vector3d(0,0,lBodyHeight);

    forwordKinetics();

    return 0;
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions - Public
 *----------------------------------------------------------------------
 */

int
Kinematics::inverseKinetics(const ToeVectors & toeVectors)
{
    int ret = 0;
    for (int i = 0; i < 6; i++) {
        Vector3d v;
        if (toeVectors.isAbs_) {
            v = toeVectors.v_[i];
        } else {
            v = EigenUtils::makeMatrixR(0,0,RADIAN(getLegWithID(i)->degree_)) * toeVectors.v_[i];
        }
        ret |= inverseKinetics(i, v, false);
    }

    ret |= forwordKinetics();
    if (ret) return ret;

    return ret;
}

int
Kinematics::inverseKinetics(int legid, const Vector3d & vToe, bool doFk)
{
    SP<Leg> leg = getLegWithID(legid);

    int ret = inverseKinetics(
        vBodyOrigin_,
        vOriginToHipZ_,
        vHipZToHipY_,
        vHipYToKnee_,
        vKneeToToe_,
        leg->degree_,
        rotBody_,
        vToe,
        leg->pHipZServo_->degreeIk_,
        leg->pHipYServo_->degreeIk_,
        leg->pKneeServo_->degreeIk_);
    if (ret) return ret;

    if (doFk) {
        ret = forwordKinetics();
        if (ret) return ret;
    }

    return 0;
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions - Public
 *----------------------------------------------------------------------
 */

int
Kinematics::forwordKinetics(void)
{
    for (auto l : pLegs_) {
        forwordKinetics(l);
    }
    return 0;
}

int
Kinematics::forwordKinetics(const SP<Leg>& leg)
{
    int ret;

    ret = forwordKinetics(
        vBodyOrigin_,
        vOriginToHipZ_,
        vHipZToHipY_,
        vHipYToKnee_,
        vKneeToToe_,
        leg->degree_,
        leg->pHipZServo_->degreeIk_,
        leg->pHipYServo_->degreeIk_,
        leg->pKneeServo_->degreeIk_,
        leg->pHipZServo_->vectorFkRel_,
        leg->pHipYServo_->vectorFkRel_,
        leg->pKneeServo_->vectorFkRel_,
        leg->pToeServo_->vectorFkRel_,
        leg->pHipZServo_->vectorFk_,
        leg->pHipYServo_->vectorFk_,
        leg->pKneeServo_->vectorFk_,
        leg->pToeServo_->vectorFk_,
        leg->pHipZServo_->rot_,
        leg->pHipYServo_->rot_,
        leg->pKneeServo_->rot_
    );
    if (ret) return ret;

    updateVectorInfo();

    return 0;
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions - Public
 *----------------------------------------------------------------------
 */

ServoDegrees
Kinematics::getServosDegree(void)
{
    ServoDegrees degrees;
    for (int i = 0; i < SERVO_NUM; i++) {
        degrees[i] = pServos_[i]->degreeIk_;
    }
    return degrees;
}

ToeVectors
Kinematics::getToeVectors(void)
{
    ToeVectors vecs;
    for (int i = 0; i < LEGS_NUM; i++) {
        vecs.v_[i] = getLegWithID(i)->pToeServo_->vectorFkRel_;
    }
    return vecs;
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions - Public
 *----------------------------------------------------------------------
 */

SP<Kinematics::Leg>
Kinematics::getLegWithID(int id)
{
    for (auto l : pLegs_) {
        if (l->id_ == id) {
            return l;
        }
    }
    return nullptr;
}

void
Kinematics::updateVectorInfo(void)
{
    //#define DP_UVI_DEBUGLOG(...)     DP__(__VA_ARGS__)
    #define DP_UVI_DEBUGLOG(...)

    VectorInfo& info = vectors_;

    DP_UVI_DEBUGLOG("================================\n");
    info.bodyOrigin_ = vBodyOrigin_;
    info.bodyRotate_ = rotBody_;
    DP_UVI_DEBUGLOG("(%s)\n", EigenUtils::toString(info.bodyOrigin_).c_str());
    for (int i = 0; i < 6; i++) {
        SP<Leg> leg = pLegs_[i];
        info.legVectors_[i].hipZ = leg->pHipZServo_->vectorFk_;
        info.legVectors_[i].hipY = leg->pHipYServo_->vectorFk_;
        info.legVectors_[i].knee = leg->pKneeServo_->vectorFk_;
        info.legVectors_[i].toe = leg->pToeServo_->vectorFk_;
        info.legVectors_[i].rhipZ = leg->pHipZServo_->rot_;
        info.legVectors_[i].rhipY = leg->pHipYServo_->rot_;
        info.legVectors_[i].rknee = leg->pKneeServo_->rot_;
        info.legVectors_[i].dhipZ = leg->pHipZServo_->degreeIk_;
        info.legVectors_[i].dhipY = leg->pHipYServo_->degreeIk_;
        info.legVectors_[i].dknee = leg->pKneeServo_->degreeIk_;
        DP_UVI_DEBUGLOG("------------------\n");
        DP_UVI_DEBUGLOG("(%s)\n", EigenUtils::toString(info.legVectors_[i].hipZ).c_str());
        DP_UVI_DEBUGLOG("(%s)\n", EigenUtils::toString(info.legVectors_[i].hipY).c_str());
        DP_UVI_DEBUGLOG("(%s)\n", EigenUtils::toString(info.legVectors_[i].knee).c_str());
        DP_UVI_DEBUGLOG("(%s)\n", EigenUtils::toString(info.legVectors_[i].toe).c_str());
    }

    double body_hip_offset = -20.0f;
    double min_z = FLT_MAX;

    min_z = MIN(min_z, vBodyOrigin_.z() + body_hip_offset);

    for (int i = 0; i < 6; i++) {
        min_z = MIN(min_z, info.legVectors_[i].hipZ.z() + body_hip_offset);
        min_z = MIN(min_z, info.legVectors_[i].hipY.z() + body_hip_offset);
        min_z = MIN(min_z, info.legVectors_[i].knee.z());
        min_z = MIN(min_z, info.legVectors_[i].toe.z());
    }

    //DP_UVI_DEBUGLOG("================================\n");
    info.bodyOriginZeroOffset_ = info.bodyOrigin_;
    //DP_UVI_DEBUGLOG("(%s)\n", EigenUtils::toString(info.bodyOriginZeroOffset_).c_str());
    info.legVectorsZeroOffset_ = info.legVectors_;

    info.bodyOriginZeroOffset_.z() -= min_z;
    for (int i = 0; i < 6; i++) {
        info.legVectorsZeroOffset_[i].hipZ.z() -= min_z;
        info.legVectorsZeroOffset_[i].hipY.z() -= min_z;
        info.legVectorsZeroOffset_[i].knee.z() -= min_z;
        info.legVectorsZeroOffset_[i].toe.z() -= min_z;
        //DP_UVI_DEBUGLOG("------------------\n");
        //DP_UVI_DEBUGLOG("(%s)\n", EigenUtils::toString(info.legVectorsZeroOffset_[i].hipZ).c_str());
        //DP_UVI_DEBUGLOG("(%s)\n", EigenUtils::toString(info.legVectorsZeroOffset_[i].hipY).c_str());
        //DP_UVI_DEBUGLOG("(%s)\n", EigenUtils::toString(info.legVectorsZeroOffset_[i].knee).c_str());
        //DP_UVI_DEBUGLOG("(%s)\n", EigenUtils::toString(info.legVectorsZeroOffset_[i].toe).c_str());
    }
}

VectorInfo
Kinematics::getKinematicsInfo(void)
{
    return vectors_;
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions - Static Public
 *----------------------------------------------------------------------
 */

int
Kinematics::inverseKinetics(
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
) {
    //#define DP_IK_DEBUGLOG(...)     DP__(__VA_ARGS__)
    #define DP_IK_DEBUGLOG(...)

    auto calcRadBWithTriangleSides = [](double a, double b, double c)
    {
        if (a + c == b) {
            return M_PI;
        } else {
            double cosB = ((c * c) + (a * a) - (b * b)) / (2 * c * a);
            if (cosB < -1.0f || 1.0f < cosB) {
                DP_IK_DEBUGLOG("cosB is out of range %f\n", cosB);
            }
            double rB = acos(cosB);
            return rB;
        }
    };

    // Convert vTargetToe to the Body Rotaion Coordinate.
    Vector3d vTargetToeCoorded =
            (EigenUtils::makeMatrixT(param_vBodyOrigin) *   // 3. Move vTargetToe to Body Rotate.
            (EigenUtils::makeMatrixR(-rotBody) *      // 2. Reverse Rotate vTargetToe by Body Rotate.
            (EigenUtils::makeMatrixT(-param_vBodyOrigin) *  // 1. Reverse Move the vTargetToe to Body Origin.
                vTargetToe)));

    DP_IK_DEBUGLOG("======================================\n");
    DP_IK_DEBUGLOG("Parameters\n");
    DP_IK_DEBUGLOG("  dHipZ %f\n", dHipZ);
    DP_IK_DEBUGLOG("  dHipY %f\n", dHipY);
    DP_IK_DEBUGLOG("  dKnee %f\n", dKnee);
    DP_IK_DEBUGLOG("  target toe (%s) %f\n", EigenUtils::toString(vTargetToe).c_str(), vTargetToe.norm());
    DP_IK_DEBUGLOG("  body corrd target toe (%s) %f\n", EigenUtils::toString(vTargetToeCoorded).c_str(), vTargetToeCoorded.norm());
    DP_IK_DEBUGLOG("  dLeg %f\n", dLeg);

    DP_IK_DEBUGLOG("Calc HipZ degree\n");
    double tmpdHipZ;
    {
        Vector3d vHipZ = EigenUtils::makeMatrixR(0, 0, RADIAN(dLeg)) * param_vOriginToHipZ;
        Vector3d a = vHipZ;
        Vector3d b = vTargetToeCoorded - vHipZ;
        a.z() = 0; b.z() = 0; // Convert xy dimention vector.

        DP_IK_DEBUGLOG("  a(%s) %f\n", EigenUtils::toString(a).c_str(), a.norm());
        DP_IK_DEBUGLOG("  b(%s) %f\n", EigenUtils::toString(b).c_str(), b.norm());

        if (b == Vector3d::Zero()) {
            // Toe is same position as HipZ.
            // No update HipZ degree.
            tmpdHipZ = dHipZ;
        } else {
            double abDot = a.normalized().dot(b.normalized());
            double abDotRounded = abDot;
            if (abDotRounded > 1.0f) abDotRounded = 1.0f;
            if (abDotRounded < -1.0f) abDotRounded = -1.0f;

            double radOriginToHipZToToe = acos(abDotRounded);
            double dOriginToHipZToToe = DEGREE(radOriginToHipZToToe);
            
            Eigen::Vector3d axis = a.cross(b).normalized();
            double toeDir = (axis.z() >= 0)? 1 : -1;
            tmpdHipZ = toeDir * dOriginToHipZToToe;

            DP_IK_DEBUGLOG("  a.normalized(%s)\n", EigenUtils::toString(a.normalized()).c_str());
            DP_IK_DEBUGLOG("  b.normalized(%s)\n", EigenUtils::toString(b.normalized()).c_str());
            DP_IK_DEBUGLOG("  abDot = a.normalized().dot(b.normalized()) %f\n", abDot);
            DP_IK_DEBUGLOG("  abDotRounded = %f\n", abDotRounded);
            DP_IK_DEBUGLOG("  radOriginToHipZToToe %f\n", radOriginToHipZToToe);
            DP_IK_DEBUGLOG("  dOriginToHipZToToe %f\n", dOriginToHipZToToe);
            DP_IK_DEBUGLOG("  axis(%s) %f\n", EigenUtils::toString(axis).c_str(), axis.norm());
            DP_IK_DEBUGLOG("  toeDir %f\n", toeDir);
        }
        DP_IK_DEBUGLOG("  tmpdHipZ %f\n", tmpdHipZ);
    }

    DP_IK_DEBUGLOG("Calc vHipY and vHipYToToe\n");
    Vector3d vHipY;
    Vector3d vHipYToToe;
    {
        vHipY = param_vHipZToHipY;
        DP_IK_DEBUGLOG("  vHipY(%s)\n", EigenUtils::toString(vHipY).c_str());
        vHipY = EigenUtils::makeMatrixR(0, 0, RADIAN(tmpdHipZ)) * vHipY;
        DP_IK_DEBUGLOG("  vHipY(%s)\n", EigenUtils::toString(vHipY).c_str());
        vHipY = param_vBodyOrigin + param_vOriginToHipZ + vHipY;
        DP_IK_DEBUGLOG("  vHipY(%s)\n", EigenUtils::toString(vHipY).c_str());
        vHipY = EigenUtils::makeMatrixR(0, 0, RADIAN(dLeg)) * vHipY;
        DP_IK_DEBUGLOG("  vHipY(%s)\n", EigenUtils::toString(vHipY).c_str());
        vHipYToToe = vTargetToeCoorded - vHipY;
        DP_IK_DEBUGLOG("  vHipYToToe(%s)\n", EigenUtils::toString(vHipYToToe).c_str());
    }

    DP_IK_DEBUGLOG("Calc HipY to Toe (coorded to y = 0)\n");
    Vector3d vHipYToToeWithOriginXCoorded;
    {
        vHipYToToeWithOriginXCoorded = vTargetToeCoorded;
        DP_IK_DEBUGLOG("  vHipYToToeWithOriginXCoorded(%s) %f\n", EigenUtils::toString(vHipYToToeWithOriginXCoorded).c_str(), vHipYToToeWithOriginXCoorded.norm());
        vHipYToToeWithOriginXCoorded = EigenUtils::makeMatrixR(0, 0, RADIAN(-dLeg)) * vHipYToToeWithOriginXCoorded;
        DP_IK_DEBUGLOG("  vHipYToToeWithOriginXCoorded(%s) %f\n", EigenUtils::toString(vHipYToToeWithOriginXCoorded).c_str(), vHipYToToeWithOriginXCoorded.norm());
        vHipYToToeWithOriginXCoorded = vHipYToToeWithOriginXCoorded - (param_vBodyOrigin + param_vOriginToHipZ);
        DP_IK_DEBUGLOG("  vHipYToToeWithOriginXCoorded(%s) %f\n", EigenUtils::toString(vHipYToToeWithOriginXCoorded).c_str(), vHipYToToeWithOriginXCoorded.norm());
        vHipYToToeWithOriginXCoorded = EigenUtils::makeMatrixR(0, 0, RADIAN(-tmpdHipZ)) * vHipYToToeWithOriginXCoorded;
        DP_IK_DEBUGLOG("  vHipYToToeWithOriginXCoorded(%s) %f\n", EigenUtils::toString(vHipYToToeWithOriginXCoorded).c_str(), vHipYToToeWithOriginXCoorded.norm());
        vHipYToToeWithOriginXCoorded = vHipYToToeWithOriginXCoorded - (param_vHipZToHipY);
        DP_IK_DEBUGLOG("  vHipYToToeWithOriginXCoorded(%s) %f\n", EigenUtils::toString(vHipYToToeWithOriginXCoorded).c_str(), vHipYToToeWithOriginXCoorded.norm());
    }

    DP_IK_DEBUGLOG("Calc edge of triangle of HipY, Knee, Toe\n");
    double dKneeToHipYToToe;
    double dHipYToKneeToToe;
    {
        Vector3d a = param_vHipYToKnee;
        Vector3d b = param_vKneeToToe;
        Vector3d c = vHipYToToeWithOriginXCoorded;
        double radKneeToHipYToToe = calcRadBWithTriangleSides(a.norm(), b.norm(), c.norm());
        double radHipYToKneeToToe = calcRadBWithTriangleSides(a.norm(), c.norm(), b.norm());
        dKneeToHipYToToe = DEGREE(radKneeToHipYToToe);
        dHipYToKneeToToe = DEGREE(radHipYToKneeToToe);

        DP_IK_DEBUGLOG("  ---\n");
        DP_IK_DEBUGLOG("  a(%s) %f\n", EigenUtils::toString(a).c_str(), a.norm());
        DP_IK_DEBUGLOG("  b(%s) %f\n", EigenUtils::toString(b).c_str(), b.norm());
        DP_IK_DEBUGLOG("  c(%s) %f\n", EigenUtils::toString(c).c_str(), c.norm());
        DP_IK_DEBUGLOG("  radKneeToHipYToToe %f\n", radKneeToHipYToToe);
        DP_IK_DEBUGLOG("  dKneeToHipYToToe %f\n", dKneeToHipYToToe);
        DP_IK_DEBUGLOG("  radHipYToKneeToToe %f\n", radHipYToKneeToToe);
        DP_IK_DEBUGLOG("  dHipYToKneeToToe %f\n", dHipYToKneeToToe);
    }

    DP_IK_DEBUGLOG("Calc HipY to Knee (coorded to y = 0)\n");
    Vector3d vHipYtoKneeWithOriginXCoorded;
    {
        vHipYtoKneeWithOriginXCoorded = vHipYToToeWithOriginXCoorded;
        vHipYtoKneeWithOriginXCoorded = EigenUtils::makeMatrixR(0, RADIAN(-dKneeToHipYToToe), 0) * vHipYtoKneeWithOriginXCoorded;
        vHipYtoKneeWithOriginXCoorded = vHipYtoKneeWithOriginXCoorded / (param_vHipYToKnee.norm() / vHipYtoKneeWithOriginXCoorded.norm());

        DP_IK_DEBUGLOG("  ---\n");
        DP_IK_DEBUGLOG("  vHipYtoKneeWithOriginXCoorded(%s) %f\n", EigenUtils::toString(vHipYtoKneeWithOriginXCoorded).c_str(), vHipYtoKneeWithOriginXCoorded.norm());
    }

    DP_IK_DEBUGLOG("Calc Top - Hip - Knee degree (coorded to y = 0)\n");
    double dTopToHipYToKnee;
    Vector3d vTop(0.0, 0.0, 1.0);
    {
        Vector3d a = vTop;
        Vector3d b = vHipYtoKneeWithOriginXCoorded - vTop;
        Vector3d c = vHipYtoKneeWithOriginXCoorded;
        double radTopToHipYToKnee = calcRadBWithTriangleSides(a.norm(), b.norm(), c.norm());
        dTopToHipYToKnee = DEGREE(radTopToHipYToKnee);

        dTopToHipYToKnee = (vHipYtoKneeWithOriginXCoorded.x() >= 0) ? dTopToHipYToKnee : -dTopToHipYToKnee;

        DP_IK_DEBUGLOG("  ---\n");
        DP_IK_DEBUGLOG("  a(%s) %f\n", EigenUtils::toString(a).c_str(), a.norm());
        DP_IK_DEBUGLOG("  b(%s) %f\n", EigenUtils::toString(b).c_str(), b.norm());
        DP_IK_DEBUGLOG("  c(%s) %f\n", EigenUtils::toString(c).c_str(), c.norm());
        DP_IK_DEBUGLOG("  radTopToHipYToKnee %f\n", radTopToHipYToKnee);
        DP_IK_DEBUGLOG("  dTopToHipYToKnee %f\n", dTopToHipYToKnee);
    }

    double tmpdHipY;
    double tmpdKnee;
    {
        tmpdHipZ = tmpdHipZ;
        tmpdHipY = -(90 - dTopToHipYToKnee);
        tmpdKnee = 180 - dHipYToKneeToToe;

        DP_IK_DEBUGLOG("-----\n");
        DP_IK_DEBUGLOG("dHipZ = %f\n", tmpdHipZ);
        DP_IK_DEBUGLOG("dHipY = %f\n", tmpdHipY);
        DP_IK_DEBUGLOG("dKnee = %f\n", tmpdKnee);
    }

    if ((isnan(tmpdHipZ))
    ||  (isnan(tmpdHipY))
    ||  (isnan(tmpdKnee))
    ) {
        DP_IK_DEBUGLOG("Failed to IK\n");
        return -1;
    }

    dHipZ = tmpdHipZ;
    dHipY = tmpdHipY;
    dKnee = tmpdKnee;

    return 0;
}

int
Kinematics::forwordKinetics(
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
) {
    //#define DP_FK_DEBUGLOG(...)     DP__(__VA_ARGS__)
    #define DP_FK_DEBUGLOG(...)

    DP_FK_DEBUGLOG("===========================\n");
    DP_FK_DEBUGLOG("forwordKinetics\n");

    Affine3d matToe  = Affine3d(AngleAxisd(RADIAN(dKnee), Vector3d::UnitY()));
    Affine3d matKnee = Affine3d(AngleAxisd(RADIAN(dHipY), Vector3d::UnitY()));
    Affine3d matHipY = Affine3d(AngleAxisd(RADIAN(dHipZ), Vector3d::UnitZ()));

    Vector3d vTmpHipZ = param_vBodyOrigin + param_vOriginToHipZ;
    Vector3d vTmpToe  = vTmpHipZ + (matHipY * (param_vHipZToHipY + (matKnee * (param_vHipYToKnee + (matToe * param_vKneeToToe)))));
    Vector3d vTmpKnee = vTmpHipZ + (matHipY * (param_vHipZToHipY + (matKnee * param_vHipYToKnee)));
    Vector3d vTmpHipY = vTmpHipZ + (matHipY * param_vHipZToHipY);

    DP_FK_DEBUGLOG("  vTmpToe  (%s) %f\n", EigenUtils::toString(vTmpToe ).c_str(), vTmpToe.norm());
    DP_FK_DEBUGLOG("  vTmpKnee (%s) %f\n", EigenUtils::toString(vTmpKnee).c_str(), vTmpKnee.norm());
    DP_FK_DEBUGLOG("  vTmpHipY (%s) %f\n", EigenUtils::toString(vTmpHipY).c_str(), vTmpHipY.norm());
    DP_FK_DEBUGLOG("  vTmpHipZ (%s) %f\n", EigenUtils::toString(vTmpHipZ).c_str(), vTmpHipZ.norm());

    vHipZRel = vTmpHipZ;
    vHipYRel = vTmpHipY;
    vKneeRel = vTmpKnee;
    vToeRel = vTmpToe;

    {
        Affine3d mat = (Affine3d)AngleAxisd(RADIAN(dLeg), Vector3d::UnitZ());
        vTmpHipZ = mat * vTmpHipZ;
        vTmpHipY = mat * vTmpHipY;
        vTmpKnee = mat * vTmpKnee;
        vTmpToe = mat * vTmpToe;
    }

    vHipZ = vTmpHipZ;
    vHipY = vTmpHipY;
    vKnee = vTmpKnee;
    vToe = vTmpToe;

    rHipZ = Vector3d(
        0,
        0,
        dLeg + dHipZ
    );
    rHipY = Vector3d(
        dHipY,
        0,
        dLeg + dHipZ
    );
    rKnee = Vector3d(
        dHipY + dKnee,
        0,
        dLeg + dHipZ
    );

    DP_FK_DEBUGLOG("Result\n");
    DP_FK_DEBUGLOG("  vTmpHipZ(%s) %f\n", EigenUtils::toString(vTmpHipZ).c_str(), vTmpHipZ.norm());
    DP_FK_DEBUGLOG("  vTmpHipY(%s) %f\n", EigenUtils::toString(vTmpHipY).c_str(), vTmpHipY.norm());
    DP_FK_DEBUGLOG("  vTmpKnee(%s) %f\n", EigenUtils::toString(vTmpKnee).c_str(), vTmpKnee.norm());
    DP_FK_DEBUGLOG("  vTmpToe (%s) %f\n", EigenUtils::toString(vTmpToe).c_str(), vTmpToe.norm());

    return 0;
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Functions - Debug
 *----------------------------------------------------------------------
 */

void
Kinematics::dump(void)
{
    DP__("----------------------------\n");
    for (auto leg : pLegs_) {
        DP__("-----\n");
        DP__("leg id %d, deg %f\n", leg->id_, leg->degree_);
        DP__("hipz deg %f, (%s), (%s)\n",
            leg->pHipZServo_->degreeIk_,
            EigenUtils::toString(leg->pHipZServo_->vectorFk_).c_str(),
            EigenUtils::toString(leg->pHipZServo_->vectorFkRel_).c_str());
        DP__("hipy deg %f, (%s), (%s)\n",
            leg->pHipYServo_->degreeIk_,
            EigenUtils::toString(leg->pHipYServo_->vectorFk_).c_str(),
            EigenUtils::toString(leg->pHipYServo_->vectorFkRel_).c_str());
        DP__("knee deg %f, (%s), (%s)\n",
            leg->pKneeServo_->degreeIk_,
            EigenUtils::toString(leg->pKneeServo_->vectorFk_).c_str(),
            EigenUtils::toString(leg->pKneeServo_->vectorFkRel_).c_str());
        DP__("toe abs (%s)\n", EigenUtils::toString(leg->pToeServo_->vectorFk_).c_str());
        DP__("toe rel (%s)\n", EigenUtils::toString(leg->pToeServo_->vectorFkRel_).c_str());
    }
    DP__("----------------------------\n");
}
