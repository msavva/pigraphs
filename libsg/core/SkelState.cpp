#include "common.h"  // NOLINT

#include "core/SkelState.h"

#include "core/Skeleton.h"
#include "geo/geo.h"

namespace sg {
namespace core {

using namespace geo;

const arr<string, SkelState::kNumJoints> SkelState::kJointNames = {
  "SpineBase",      // 0
  "SpineMid",       // 1
  "Neck",           // 2
  "Head",           // 3
  "ShoulderLeft",   // 4
  "ElbowLeft",      // 5
  "WristLeft",      // 6
  "HandLeft",       // 7
  "ShoulderRight",  // 8
  "ElbowRight",     // 9
  "WristRight",     // 10
  "HandRight",      // 11
  "HipLeft",        // 12
  "KneeLeft",       // 13
  "AnkleLeft",      // 14
  "FootLeft",       // 15
  "HipRight",       // 16
  "KneeRight",      // 17
  "AnkleRight",     // 18
  "FootRight",      // 19
  "SpineShoulder",  // 20
  "HandTipLeft",    // 21
  "ThumbLeft",      // 22
  "HandTipRight",   // 23
  "ThumbRight"      // 24
};

const Skeleton::BoneDirs SkelState::kBoneDirs = {{
  {{ml::vec3f(0,-1, 0), ml::vec3f(1, 0, 0)}},  // tgt=SpineBase",      // 0
  {{ml::vec3f(0,-1, 0), ml::vec3f(1, 0, 0)}},  // tgt=SpineMid",       // 1
  {{ml::vec3f(0,-1, 0), ml::vec3f(1, 0, 0)}},  // tgt=Neck",           // 2
  {{ml::vec3f(0,-1, 0), ml::vec3f(1, 0, 0)}},  // tgt=Head",           // 3
  {{ml::vec3f(0, 0, 1), ml::vec3f(1, 0, 0)}},  // tgt=ShoulderLeft",   // 4
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=ElbowLeft",      // 5
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=WristLeft",      // 6
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=HandLeft",       // 7
  {{ml::vec3f(0, 0,-1), ml::vec3f(1, 0, 0)}},  // tgt=ShoulderRight",  // 8
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=ElbowRight",     // 9
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=WristRight",     // 10
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=HandRight",      // 11
  {{ml::vec3f(0, 0, 1), ml::vec3f(1, 0, 0)}},  // tgt=HipLeft",        // 12
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=KneeLeft",       // 13
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=AnkleLeft",      // 14
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=FootLeft",       // 15
  {{ml::vec3f(0, 0,-1), ml::vec3f(1, 0, 0)}},  // tgt=HipRight",       // 16
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=KneeRight",      // 17
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=AnkleRight",     // 18
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=FootRight",      // 19
  {{ml::vec3f(0,-1, 0), ml::vec3f(1, 0, 0)}},  // tgt=SpineShoulder",  // 20
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=HandTipLeft",    // 21
  {{ml::vec3f(0, 0,-1), ml::vec3f(1, 0, 0)}},  // tgt=ThumbLeft",      // 22
  {{ml::vec3f(0, 1, 0), ml::vec3f(1, 0, 0)}},  // tgt=HandTipRight",   // 23
  {{ml::vec3f(0, 0, 1), ml::vec3f(1, 0, 0)}}   // tgt=ThumbRight"      // 24
}};

void JointCoords::flatten(Data* out) const {
  (*out)[0].push_back(p.x());
  (*out)[1].push_back(p.y());
  (*out)[2].push_back(p.z());
  (*out)[3].push_back(q.x());
  (*out)[4].push_back(q.y());
  (*out)[5].push_back(q.z());
  (*out)[6].push_back(l);
}

ostream& operator<<(ostream& os, const JointCoords& j) {
  os << "{ \"p\": " << j.p.format(kEigenJSONFormat)
     << ", \"q\": " << j.q.format(kEigenJSONFormat)
     << ", \"l\": " << j.l << " }";
  return os;
}

void SkelState::flatten(Data* out) const {
  for (int iJoint = 0; iJoint < kNumJoints + 1; ++iJoint) {
    const auto& jc = (iJoint == kNumJoints) ? bodyFrame : joints[iJoint];
    jc.flatten(&(*out)[iJoint]);
  }
  (*out)[kNumJoints + 1][0].push_back(spineAngle);
}

string SkelState::getCSVHeader() {
  return "joint,px,py,pz,qlat,qlon,qang,l";
}

ostream& csv(ostream& os, const string& name, const JointCoords& j) {
  os << name << ","
     << j.p[0] << "," << j.p[1] << "," << j.p[2] << ","
     << j.q[0] << "," << j.q[1] << "," << j.q[2] << ","
     << j.l;
  return os;
}

ostream& SkelState::toCSV(ostream& os, const string& prefix) const {
  os << prefix;  csv(os, "body", bodyFrame) << endl;
  for (int iJoint = 0; iJoint < kNumJoints; ++iJoint) {
    os << prefix;  csv(os, kJointNames[iJoint], joints[iJoint]);
    if (iJoint < kNumJoints - 1) { os << endl; }
  }
  return os;
}

ostream& SkelState::toJSON(ostream& os, bool endlines) const {
  os << "{"; if (endlines) { os << endl; }
  if (isHierarchical) {
    os << "\"isHierarchical\":" << isHierarchical << ","; if (endlines) { os << endl; }
  }
  os << "\"spineAngle\":" << spineAngle << ","; if (endlines) { os << endl; }
  os << "\"bodyFrame\":" << bodyFrame << ","; if (endlines) { os << endl; }
  for (int iJoint = 0; iJoint < kNumJoints; ++iJoint) {
    os << "\"" + kJointNames[iJoint] + "\"" << ":" << joints[iJoint];
    if (iJoint < kNumJoints - 1) { os << ","; }
    if (endlines) { os << endl; }
  }
  os << "}";
  if (endlines) { os << endl; }
  return os;
}

using J = Skeleton::JointType;

// refer to:
// https://www.ulb.ac.be/project/vakhum/public_dataset/Doc/VAKHUM-3-Frame_Convention.pdf
// http://www.sciencedirect.com/science/article/pii/S0021929001002226
// http://www.sciencedirect.com/science/article/pii/S002192900400301X
// for anatomical landmarks and joint coordinate systems
void kinectSkel2state(const Skeleton& skel, SkelState* pState) {
  // helper to convert quats to SkelState's angle encoding
  const auto quatAng = [&] (const Vec3f& x, const Vec3f& y, const Vec3f& z) {
    return toAngleEncoding(Rquat(x, y, z), pState->angleEncoding);
  };

  // global body frame
  const Vec3f
    _ow = vec3f(skel.jointPositions[J::JointType_SpineBase]),       // pelvis center
    _xw = vec3f(skel.bodyPlaneNormal),                              // anterior dir = global "front"
    _zw = crossNorm(_xw, Vec3f::UnitZ()),                           // global "right" dir
    _yw = crossNorm(_zw, _xw),                                      // global "up" dir
     qw = quatAng(_xw, _yw, _zw);                                   // world orientation angles
  const Quatf _qw = Rquat(_xw, _yw, _zw);                           // world orientation quat

  // helper to transform from world to body frame points
  const Transform world2body = toRotTrans(_ow, _qw).inverse();
  const auto p = [&] (const J& iJoint) {
    const Vec3f v = world2body * vec3f(skel.jointPositions[iJoint]);
    return v;
  };

  // helper to transform from world joint orientation to body frame orientation
  const Quatf _qwInv = _qw.inverse();
  const auto q = [&] (const J& iJoint, int iCol) {
    const ml::vec4f& qKinect = skel.jointOrientations[iJoint];
    const Quatf qWorld(qKinect.w, qKinect.x, qKinect.y, qKinect.z);
    const Quatf qBody = _qwInv * qWorld;
    const Matrix3f R = qBody.toRotationMatrix();
    const Vec3f v = R.col(iCol);
    return v;
  };

  const Vec3f
    /// TORSO ///

    // pelvis frame - origin = ow, child = ove
    ove = p(J::JointType_SpineMid),             // mid-spine vertebrae
    yp = ove.normalized(),                      // pelvic up
    xp = crossNorm(yp, Vec3f::UnitZ()),         // pelvic front
    zp = crossNorm(xp, yp),                     // pelvic right
    qp = quatAng(xp,  yp,  zp),                 // pelvis

    // torso frame - origin = ove, child = osc
    osc = p(J::JointType_SpineShoulder),        // sternoclavicular joint
    yto = (osc - ove).normalized(),             // torso up
    xto = crossNorm(yto, Vec3f::UnitZ()),       // torso front
    zto = crossNorm(xto, yto),                  // torso right
    qto = quatAng(xto, yto, zto),               // torso

    // shoulder-mid frame - origin = osc, child = ocv
    ocv = p(J::JointType_Neck),                 // cervical vertebrae (neck)
    ysh = (ocv - osc).normalized(),             // shoulder-mid up
    xsh = crossNorm(ysh, Vec3f::UnitZ()),       // shoulder-mid front
    zsh = crossNorm(xsh, ysh),                  // shoulder-mid right
    qsh = quatAng(xsh, ysh, zsh),               // neck

    // neck frame - origin = ocv, child = ohe
   _ohe = p(J::JointType_Head),                 // head observed (unreliable)
    zne = -q(J::JointType_Neck, 0),             // neck right
   _xne = crossNorm(ysh, zne),                  // neck front temporary
    yne = crossNorm(zne, _xne),                 // neck up
    xne = crossNorm(yne, zne),                  // neck front
    ohe = ocv + (_ohe - ocv).norm() * yne,      // predicted head position
    qne = quatAng(xne, yne, zne),               // head

    // L clavicle frame - origin osc, child = ghl
    ghl = p(J::JointType_ShoulderLeft),         // glenohumeral rotation center L
    zcl = -(ghl - osc).normalized(),            // clavicle right L
    xcl = crossNorm(ysh, zcl),                  // clavicle front L
    ycl = crossNorm(zcl, xcl),                  // clavicle up L
    qcl = quatAng(xcl, ycl, zcl),               // clavicle L

    // R clavicle frame - origin osc, child = ghr
    ghr = p(J::JointType_ShoulderRight),        // glenohumeral rotation center R
    zcr = (ghr - osc).normalized(),             // clavicle right R
    xcr = crossNorm(ysh, zcr),                  // clavicle front R
    ycr = crossNorm(zcr, xcr),                  // clavicle up R
    qcr = quatAng(xcr, ycr, zcr),               // clavicle R


    /// ARMS AND HANDS ///

    // L arm frame common
    ecl = p(J::JointType_ElbowLeft),            // epicondyle L
    yhl = (ghl - ecl).normalized(),             // humerus up L
    scl = p(J::JointType_WristLeft),            // radial+ulnar styloid L
    yfl = (ecl - scl).normalized(),             // forearm up L

    // L humerus frame - origin = ghl, child = ecl
    // yhl computed in common arm frame
    _zhl = crossNorm(yfl, yhl),
    zhl = (_zhl.z() > 0) ? _zhl : -_zhl,        // humerus right L
    xhl = crossNorm(yhl, zhl),                  // humerus front L
    qhl = quatAng(xhl, yhl, zhl),               // humerus L
    // TODO(MS): degenerate case: yhl-yfl parallel

    // L forearm frame - origin = scl, child = ecl
    // yfl computed in common arm frame
    zfl = zhl,                                  // forearm right L
    xfl = crossNorm(yfl, zfl),                  // forearm front L
    qfl = quatAng(xfl, yfl, zfl),               // forearm L

    // L palm frame - origin = scl, child = pal
   _pal = p(J::JointType_HandLeft),             // observed trapezium (mid-palm) L (unreliable)
    xrl = xfl,                                  // trapezium front L
    yrl = yfl,                                  // trapezium up L
    zrl = zfl,                                  // trapezium right L
    pal = scl - (scl - _pal).norm() * yrl,      // predicted trapezium L position
    qrl = quatAng(xrl, yrl, zrl),               // trapezium L

    // L thumb frame - origin = pal, child = thl
   _thl = p(J::JointType_ThumbLeft),            // observed thumb L (unreliable)
    xxl = xfl,                                  // thumb front L
    yxl = yfl,                                  // thumb up L
    zxl = zfl,                                  // thumb right L
    thl = pal + (pal - _thl).norm() * zxl,      // predicted thumb L position
    qxl = quatAng(xxl, yxl, zxl),               // thumb L

    // L fingertip frame - origin = pal, child = phl
   _phl = p(J::JointType_HandTipLeft),         // observed distal phalanges (hand tips) L (unreliable)
    xpl = xfl,                                  // distal phalanges front L
    zpl = zfl,                                  // distal phalanges right L
    ypl = yfl,                                  // distal phalanges up L
    phl = pal - (pal - _phl).norm() * yfl,      // predicted distal phalanges L position
    qpl = quatAng(xpl, ypl, zpl),               // hand tips L

    // R arm frame common
    ecr = p(J::JointType_ElbowRight),           // epicondyle R
    yhr = (ghr - ecr).normalized(),             // humerus up R
    scr = p(J::JointType_WristRight),           // radial+ulnar styloid R
    yfr = (ecr - scr).normalized(),             // forearm up R

    // R humerus frame - origin = ghr, child = ecr
    // yhl computed in common arm frame
    _zhr = crossNorm(yfr, yhr),
    zhr = (_zhr.z() > 0) ? _zhr : -_zhr,        // humerus right R
    xhr = crossNorm(yhr, zhr),                  // humerus front R
    qhr = quatAng(xhr, yhr, zhr),               // humerur R
    // TODO(MS): degenerate case: yhr-yfr parallel

    // R forearm frame - origin = scr, child = ecr
    // yfl computed in common arm frame
    zfr = zhr,                                  // forearm right R
    xfr = crossNorm(yfr, zfr),                  // forearm front R
    qfr = quatAng(xfr, yfr, zfr),               // forearm R

    // R palm frame - origin = scr, child = par
   _par = p(J::JointType_HandRight),            // observed trapezium (mid-palm) R (unreliable)
    xrr = xfr,                                  // trapezium front R
    yrr = yfr,                                  // trapezium up R
    zrr = zfr,                                  // trapezium right R
    par = scr - (scr - _par).norm() * yrr,      // predicted trapezium R position
    qrr = quatAng(xrr, yrr, zrr),               // trapezium R

    // R thumb frame - origin = par, child = thr
   _thr = p(J::JointType_ThumbRight),           // observed thumb R (unreliable)
    xxr = xfr,                                  // thumb front R
    yxr = yfr,                                  // thumb up R
    zxr = zfr,                                  // thumb right R
    thr = par - (par - _thr).norm() * zxr,      // predicted thumb R position
    qxr = quatAng(xxr, yxr, zxr),               // thumb R

    // R fingertip frame - origin = par, child = thr
   _phr = p(J::JointType_HandTipRight),         // observed distal phalanges (hand tips) R (unreliable)
    xpr = xfr,                                  // distal phalanges front R
    zpr = zfr,                                  // distal phalanges right R
    ypr = yfr,                                  // distal phalanges up R
    phr = par - (par - _phr).norm() * yfr,      // predicted distal phalanges R position
    qpr = quatAng(xpr, ypr, zpr),               // hand tips R


    /// LEGS AND FEET ///

    // L ilium/hip - origin = ove, child = fhl
    fhl = p(J::JointType_HipLeft),              // hip femur head L
    zil = -fhl.normalized(),                    // ilium/hip right L
    xil = crossNorm(yp, zil),                   // ilium/hip front L
    yil = crossNorm(zil, xil),                  // ilium/hip up L
    qil = quatAng(xil, yil, zil),               // hip R

    // L thigh - origin = fhl, child = otl
    otl = p(J::JointType_KneeLeft),             // knee L
    ytl = (fhl - otl).normalized(),             // thigh up L
    ztl = -q(J::JointType_KneeLeft, 2),         // thigh right L
    xtl = crossNorm(ytl, ztl),                  // thigh front L
    qtl = quatAng(xtl, ytl, ztl),               // thigh L

    // L shank - origin = otl, child osl
    osl = p(J::JointType_AnkleLeft),            // ankle L
    ysl = (otl - osl).normalized(),             // shank up L
    zsl = ztl,                                  // shank right L
    xsl = crossNorm(ysl, zsl),                  // shank front L
    qsl = quatAng(xsl, ysl, zsl),               // shank L

    // L foot
   _sml = p(J::JointType_FootLeft),             // observed unreliable 2nd metatarsal head (foot toe) L
    //yml = (osl - sml).normalized(),           // foot toe up L  // NOTE: Unreliable direction
    yml = -xsl,                                 // foot toe up L
    sml = osl - (_sml - osl).norm() * yml,      // foot toe predicted position L
    zml = zsl,                                  // foot toe right L
    xml = crossNorm(yml, zml),                  // foot toe front L
    qml = quatAng(xml, yml, zml),               // foot L

    // R ilium/hip - origin = ove, child = fhr
    fhr = p(J::JointType_HipRight),             // hip femur head R
    zir = fhr.normalized(),                     // ilium/hip right R
    xir = crossNorm(yp, zir),                   // ilium/hip front R
    yir = crossNorm(zir, xir),                  // ilium/hip up R
    qir = quatAng(xir, yir, zir),               // hip R

    // R thigh - origin = fhr, child = otr
    otr = p(J::JointType_KneeRight),            // knee R
    ytr = (fhr - otr).normalized(),             // thigh up R
    ztr = q(J::JointType_KneeRight, 2),         // thigh right R
    xtr = crossNorm(ytr, ztr),                  // thigh front R
    qtr = quatAng(xtr, ytr, ztr),               // thigh R

    // R shank - origin = otr, child osr
    osr = p(J::JointType_AnkleRight),           // ankle R
    ysr = (otr - osr).normalized(),             // shank up R
    zsr = ztr,                                  // shank right R
    xsr = crossNorm(ysr, zsr),                  // shank front R
    qsr = quatAng(xsr, ysr, zsr),               // shank R

    // R foot
   _smr = p(J::JointType_FootRight),            // observed unreliable 2nd metatarsal head (foot toe) R
    //ymr = (osr - smr).normalized(),           // foot toe up R  // NOTE: Unreliable direction
    ymr = -xsr,                                 // foot toe up R
    smr = osr - (_smr - osr).norm() * ymr,      // foot toe predicted position R
    zmr = zsr,                                  // foot toe right R
    xmr = crossNorm(ymr, zmr),                  // foot toe front R
    qmr = quatAng(xmr, ymr, zmr);               // foot R

  // anatomical lengths
  const float
    lsb = ove.norm(),          // spine 1: pelvis to mid-spine vertebrae
    lsm = (osc - ove).norm(),  // spine 2: mid-spine vertebrae to shoulder-mid
    lne = (ocv - osc).norm(),  // neck: shoulder-mid to cervical vertebrae
    lhe = (ohe - ocv).norm(),  // head: cervical vertebrae to mid-head
    lcl = (ghl - osc).norm(),  // clavicle/shoulder L
    lcr = (ghr - osc).norm(),  // clavicle/shoulder R
    lhl = (ghl - ecl).norm(),  // humerus/upper arm L
    lhr = (ghr - ecr).norm(),  // humerus/upper arm R
    lfl = (ecl - scl).norm(),  // ulna/forearm L
    lfr = (ecr - scr).norm(),  // ulna/forearm R
    lrl = (pal - scl).norm(),  // palm L
    lrr = (par - scr).norm(),  // palm R
    lxl = (thl - pal).norm(),  // thumb L
    lxr = (thr - par).norm(),  // thumb R
    lpl = (phl - pal).norm(),  // phalanges L
    lpr = (phr - par).norm(),  // phalanges R
    lil = fhl.norm(),          // ilium/hip bone L
    lir = fhr.norm(),          // ilium/hip bone R
    ltl = (fhl - otl).norm(),  // femur/thigh L
    ltr = (fhr - otr).norm(),  // femur/thigh R
    lsl = (otl - osl).norm(),  // tibia/shank L
    lsr = (otr - osr).norm(),  // tibia/shank R
    lal = (sml - osl).norm(),  // foot L
    lar = (smr - osr).norm(),  // foot R
    l   = lsb + lsm + lne + lhe + lcl + lcr + lhl + lhr + lfl + lfr
        + lrl + lrr + lxl + lxr + lpl + lpr + lil + lir + ltl + ltr
        + lsl + lsr + lal + lar,
    ln  = 1.f / l;             // inv bone length sum for normalization

  // save out
  pState->bodyFrame             = {_ow, qw, l};        // global body frame
  auto& j = pState->joints;
  const Vec3f oze(0, 0, 0);
  const Vec3f oq = toAngleEncoding(Quatf::Identity(), pState->angleEncoding);
  j[J::JointType_SpineBase]     = {oze, oq,       0};  // 0  pelvis
  j[J::JointType_SpineMid]      = {ove, qp,  lsb*ln};  // 1  torso
  j[J::JointType_Neck]          = {ocv, qsh, lne*ln};  // 2  neck
  j[J::JointType_Head]          = {ohe, qne, lhe*ln};  // 3  head
  j[J::JointType_ShoulderLeft]  = {ghl, qcl, lcl*ln};  // 4  clavicle L
  j[J::JointType_ElbowLeft]     = {ecl, qhl, lhl*ln};  // 5  humerus L
  j[J::JointType_WristLeft]     = {scl, qfl, lfl*ln};  // 6  forearm L
  j[J::JointType_HandLeft]      = {pal, qrl, lrl*ln};  // 7  hand L
  j[J::JointType_ShoulderRight] = {ghr, qcr, lcr*ln};  // 8  clavicle R
  j[J::JointType_ElbowRight]    = {ecr, qhr, lhr*ln};  // 9  humerus R
  j[J::JointType_WristRight]    = {scr, qfr, lfr*ln};  // 10 forearm R
  j[J::JointType_HandRight]     = {par, qrr, lrr*ln};  // 11 hand R
  j[J::JointType_HipLeft]       = {fhl, qil, lil*ln};  // 12 ilium L
  j[J::JointType_KneeLeft]      = {otl, qtl, ltl*ln};  // 13 femur L
  j[J::JointType_AnkleLeft]     = {osl, qsl, lsl*ln};  // 14 shank L
  j[J::JointType_FootLeft]      = {sml, qml, lal*ln};  // 15 foot L
  j[J::JointType_HipRight]      = {fhr, qir, lir*ln};  // 16 ilium R
  j[J::JointType_KneeRight]     = {otr, qtr, ltr*ln};  // 17 femur R
  j[J::JointType_AnkleRight]    = {osr, qsr, lsr*ln};  // 18 shank R
  j[J::JointType_FootRight]     = {smr, qmr, lar*ln};  // 19 foot R
  j[J::JointType_SpineShoulder] = {osc, qto, lsm*ln};  // 20 shoulder-mid
  j[J::JointType_HandTipLeft]   = {phl, qpl, lpl*ln};  // 21 phalanges L
  j[J::JointType_ThumbLeft]     = {thl, qxl, lxl*ln};  // 22 thumb L
  j[J::JointType_HandTipRight]  = {phr, qpr, lpr*ln};  // 23 phalanges R
  j[J::JointType_ThumbRight]    = {thr, qxr, lxr*ln};  // 24 thumb R

  pState->isHierarchical = false;
}

void skel2state(const Skeleton& skel, SkelState* pState) {
  if (!skel.gazeBodyPlaneOkay) {
    SG_LOG_WARN << "Gaze body plane not computed!!!";
  }
  assert(skel.gazeBodyPlaneOkay);
  pState->angleEncoding = LAT_LONG_ROT;  // TODO(MS): Allow setting this externally
  pState->spineAngle = asin(skel.bodyPlaneNormal.z);
  if (skel.isKinectSkel) {  // do kinect to skelstate conversion
    kinectSkel2state(skel, pState);
  } else {  // use our own conventions for skelstates
    const AngleEncoding& enc = pState->angleEncoding;
    const Vec3f _ow = vec3f(skel.jointPositions[J::JointType_SpineBase]);
    pState->bodyFrame.p = _ow;
    const auto& bq = skel.jointOrientations[J::JointType_SpineBase];
    const Quatf _qw = Quatf(bq.w, bq.x, bq.y, bq.z);
    const Quatf _qwInv = _qw.inverse();
    pState->bodyFrame.q = toAngleEncoding(_qw, enc);
    const Transform world2body = toRotTrans(_ow, _qw).inverse();

    pState->joints[J::JointType_SpineBase].p = Vec3f::Zero();
    pState->joints[J::JointType_SpineBase].q = toAngleEncoding(Quatf::Identity(), enc);
    pState->joints[J::JointType_SpineBase].l = 0.f;
    float sumBoneLength = 0.f;
    for (int iBone = 0; iBone < Skeleton::kNumBones; ++iBone) {
      const auto& b = Skeleton::kBones[iBone];
      auto& jc = pState->joints[b[1]];
      const auto& srcP = vec3f(skel.jointPositions[b[0]]);
      const auto& tgtP = vec3f(skel.jointPositions[b[1]]);
      jc.p = world2body * tgtP;
      const auto& q = skel.jointOrientations[b[1]];
      jc.q = toAngleEncoding(_qwInv * Quatf(q.w, q.x, q.y, q.z), enc);
      jc.l = (tgtP - srcP).norm();
      sumBoneLength += jc.l;
    }

    // normalize bone lengths
    pState->bodyFrame.l = sumBoneLength;
    const float invBoneLength = 1.f / sumBoneLength;
    for (int i = 0; i < Skeleton::kNumJoints; ++i) {
      pState->joints[i].l *= invBoneLength;
    }

    pState->isHierarchical = false;
  }
}

void state2skel(const SkelState& ss, Skeleton* pSkel) {
  if (ss.isHierarchical) {
    SG_LOG_ERROR << "Need absolute SkelState but hierarchical SkelState given";
    return;
  }
  const AngleEncoding& enc = ss.angleEncoding;
  const Transform body2world = toRotTrans(ss.bodyFrame.p, fromAngleEncoding(ss.bodyFrame.q, enc));
  const Quatf bodyQuat = fromAngleEncoding(ss.bodyFrame.q, enc);
  for (int iJoint = 0; iJoint < SkelState::kNumJoints; ++iJoint) {
    const JointCoords& coords = ss.joints[iJoint];
    Quatf boneQuat = bodyQuat * fromAngleEncoding(coords.q, enc);
    pSkel->jointPositions[iJoint] = to<ml::vec3f>(body2world * coords.p);
    pSkel->jointOrientations[iJoint] = to<ml::vec4f>(boneQuat.coeffs());
    pSkel->jointConfidences[iJoint] = 1.f;
  }
  pSkel->isKinectSkel = false;
  pSkel->computeGaze();  // recomputes body plane normal as well
}

SkelState makeHierarchical(const SkelState& in) {
  if (in.isHierarchical) {
    SG_LOG_WARN << "Input skeleton already hierarchical";
    return in;
  }
  SkelState out;
  out.spineAngle = in.spineAngle;
  out.angleEncoding = in.angleEncoding;
  const AngleEncoding& enc = out.angleEncoding;
  out.bodyFrame = in.bodyFrame;
  out.joints[Skeleton::JointType_SpineBase] = in.joints[Skeleton::JointType_SpineBase];
  for (int iBone = 0; iBone < Skeleton::kNumBones; ++iBone) {
    const auto& bone = Skeleton::kBones[iBone];
    const JointCoords& srcJoint = in.joints[bone[0]];
    const JointCoords& tgtJoint = in.joints[bone[1]];
    const Quatf srcQinv = fromAngleEncoding(srcJoint.q, enc).inverse();
    const Vec3f relPos = srcQinv * (tgtJoint.p - srcJoint.p);
    const Quatf relQ = (srcQinv * fromAngleEncoding(tgtJoint.q, enc)).normalized();
    JointCoords& tgtJointOut = out.joints[bone[1]];
    tgtJointOut.p = relPos;
    tgtJointOut.q = toAngleEncoding(relQ, enc);
    tgtJointOut.l = tgtJoint.l;  // length is same
  }
  out.isHierarchical = true;
  return out;
}

SkelState makeAbsolute(const SkelState& in) {
  if (!in.isHierarchical) {
    SG_LOG_WARN << "Input skeleton already absolute";
    return in;
  }
  SkelState out;
  out.spineAngle = in.spineAngle;
  out.angleEncoding = in.angleEncoding;
  const AngleEncoding& enc = out.angleEncoding;
  out.bodyFrame = in.bodyFrame;
  out.joints[Skeleton::JointType_SpineBase] = in.joints[Skeleton::JointType_SpineBase];
  const auto& boneDirs = in.getBoneDirs();
  // Assumes that bones are specified in order (parents bones are specified before child bones)
  for (int iBone = 0; iBone < Skeleton::kNumBones; ++iBone) {
    const auto& bone = Skeleton::kBones[iBone];
    const JointCoords& srcJoint = out.joints[bone[0]];
    const JointCoords& tgtJoint = in.joints[bone[1]];
    const Quatf srcQ = fromAngleEncoding(srcJoint.q, enc);
    const Quatf absQ = (srcQ * fromAngleEncoding(tgtJoint.q, enc)).normalized();
    // Use bone length to compute target joint position
    const Vec3f boneDir = vec3f(boneDirs[bone[1]][0]);
    const float boneLength = tgtJoint.l * in.bodyFrame.l;
    const Vec3f absPos = boneLength * (absQ * -boneDir) + srcJoint.p;
    JointCoords& tgtJointOut = out.joints[bone[1]];
    tgtJointOut.p = absPos;
    tgtJointOut.q = toAngleEncoding(absQ, enc);
    tgtJointOut.l = tgtJoint.l;  // length is same
  }
  out.isHierarchical = false;
  return out;
}

umap<string, Transform> jointCoordFrames(const SkelState& ss) {
  umap<string, Transform> xforms;
  const AngleEncoding& enc = ss.angleEncoding;
  const Transform body2world = toRotTrans(ss.bodyFrame.p, fromAngleEncoding(ss.bodyFrame.q, enc));
  for (int iJoint = 0; iJoint < SkelState::kNumJoints; ++iJoint) {
    const string name = SkelState::kJointNames[iJoint];
    const JointCoords& j = ss.joints[iJoint];
    const Transform bone2body = toRotTrans(j.p, fromAngleEncoding(j.q, enc));
    xforms[name] = body2world * bone2body;
  }
  return xforms;
}

}  // namespace core
}  // namespace sg
