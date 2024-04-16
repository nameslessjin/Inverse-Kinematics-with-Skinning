#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

// Converts degrees to radians.
template<typename real>
inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

template<typename real>
Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
{
  Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
  Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
  Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

  switch(order)
  {
    case RotateOrder::XYZ:
      return RZ * RY * RX;
    case RotateOrder::YZX:
      return RX * RZ * RY;
    case RotateOrder::ZXY:
      return RY * RX * RZ;
    case RotateOrder::XZY:
      return RY * RZ * RX;
    case RotateOrder::YXZ:
      return RZ * RX * RY;
    case RotateOrder::ZYX:
      return RX * RY * RZ;
  }
  assert(0);
}

// Performs forward kinematics, using the provided "fk" class.
// This is the function whose Jacobian matrix will be computed using adolc.
// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
//   IKJointIDs is an array of integers of length "numIKJoints"
// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
template<typename real>
void forwardKinematicsFunction(
    int numIKJoints, const int * IKJointIDs, const FK & fk,
    const std::vector<real> & eulerAngles, 
    /*output*/ std::vector<real> & handlePositions)
{
  // Students should implement this.
  // The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
  // The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
  // Then, implement the same algorithm into this function. To do so,
  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
  // Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
  // It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
  // so that code is only written once. We considered this; but it is actually not easily doable.
  // If you find a good approach, feel free to document it in the README file, for extra credit.

  // global Matrix x X_local = X_handle

  // eulerAngles is 22 x 3 = 66

  std::vector<Mat3<real>> localR;
  std::vector<Vec3<real>> localT;
  std::vector<Mat3<real>> globalR;
  std::vector<Vec3<real>> globalT;

  int n = fk.getNumJoints();

  for (int i = 0; i < n; ++i) {

    int jointId = i;

    real eAngle[3];
    // to get the eulerAngles associated with joinId
    for (int j = 0; j < 3; ++j) eAngle[j] = eulerAngles[jointId * 3 + j];

    Vec3d restTranslation = fk.getJointRestTranslation(jointId);
    RotateOrder rotateOrder = fk.getJointRotateOrder(jointId); 
    Vec3d joinOrient = fk.getJointOrient(jointId);

    real oAngle[3] = {joinOrient[0], joinOrient[1], joinOrient[2]};
    Vec3<real> t1 = {restTranslation[0], restTranslation[1], restTranslation[2]};

    Mat3<real> eR = Euler2Rotation(eAngle, rotateOrder);
    Mat3<real> oR = Euler2Rotation(oAngle, rotateOrder);

    localR.push_back(oR * eR);
    localT.push_back(t1);

    Mat3<real> parentR(1.0);
    Vec3<real> parentT(0.0);
    globalR.push_back(parentR);
    globalT.push_back(parentT);
  }

  for (int i = 0; i < n; ++i) {
    int jointId = fk.getJointUpdateOrder(i);
    int parentId = fk.getJointParent(jointId);   

    Mat3<real> parentR(1.0);
    Vec3<real> parentT(0.0);

    Mat3<real> childR = localR[jointId];
    Vec3<real> childT = localT[jointId];

    if (parentId != -1) {
      parentR = globalR[parentId];
      parentT = globalT[parentId];
    }

    Mat3<real> Rout;
    Vec3<real> tout;

    multiplyAffineTransform4ds(parentR, parentT, childR, childT, Rout, tout);

    globalR[jointId] = Rout;
    globalT[jointId] = tout;

  }

  int h = 0;
  for (int i = 0; i < numIKJoints; ++i) {
    int jointId = IKJointIDs[i];

    Mat3<real> gR = globalR[jointId];
    Vec3<real> gT = globalT[jointId];

    // multiply the 4x4 with [0, 0, 0, 1] is just T
    for (int j = 0; j < 3; ++j) handlePositions[3 * i + j] = globalT[jointId][j];
  }
}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;

  train_adolc();
}

void IK::train_adolc()
{
  // Students should implement this.
  // Here, you should setup adol_c:
  //   Define adol_c inputs and outputs. 
  //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
  //   This will later make it possible for you to compute the gradient of this function in IK::doIK
  //   (in other words, compute the "Jacobian matrix" J).
  // See ADOLCExample.cpp .

  int n = FKInputDim; // input dimension, 66
  int m = FKOutputDim; // output dimension, 12

  // std::cout << "FKInputDim: " << FKInputDim << std::endl;
  // std::cout << "FKOutputDim: " << FKOutputDim << std::endl;

  trace_on(this->adolc_tagID);

  vector<adouble> x(n); // define the input of the function f
  for (int i = 0; i < n; ++i) x[i] <<= 0.0;

  vector<adouble> y(m); // define the output of the function f

  // the computation of f here, use forwardKinematicsFunction
  forwardKinematicsFunction(this->numIKJoints, this->IKJointIDs, *(this->fk), x, y);

  vector<double> output(m);
  for (int i = 0; i < m; ++i) y[i] >>= output[i];

  trace_off();


}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles, bool IKMode)
{
  // You may find the following helpful:
  int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!


  // Students should implement this.
  // Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
  // Specifically, use ::function, and ::jacobian .
  // See ADOLCExample.cpp .
  //
  // Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
  // Note that at entry, "jointEulerAngles" contains the input Euler angles. 
  // Upon exit, jointEulerAngles should contain the new Euler angles.

  int n = FKInputDim; // input dimension, 66
  int m = FKOutputDim; // output dimension, 12

  Vec3d handlePositions[numIKJoints];

  // get updated handle position
  ::function(adolc_tagID, m, n, jointEulerAngles->data(), handlePositions->data());

  // for (int i = 0; i < numIKJoints; ++i) {
  //   std::cout << "targetHandlePositions[" << i << "]: " << targetHandlePositions[i] << std::endl;
  //   std::cout << "handlePositions[" << i << "]: " << handlePositions[i] << std::endl;
  // }

  // jacobian
  double jacobianMatrix[m * n]; // row-major order
  double * jacobianMatrixEachRow[m];
  for (int i = 0; i < m; ++i) {
    jacobianMatrixEachRow[i] = &jacobianMatrix[i * n];
  }
  ::jacobian(adolc_tagID, m, n, jointEulerAngles->data(), jacobianMatrixEachRow);


  Eigen::MatrixXd J(m, n);
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < n; ++j) {
      J(i, j) = jacobianMatrix[i * n + j];
    }
  }


  Eigen::VectorXd dX(m); // 12 x 1
  for (int i = 0; i < numIKJoints; ++i) {

    Vec3d d = targetHandlePositions[i] - handlePositions[i];
    for (int j = 0; j < 3; ++j) {
      dX(i * 3 + j) = d[j];
    }
  }

  Eigen::VectorXd dTheta;


  performIK(J, dX, dTheta, IKMode);

  for (int i = 0; i < numJoints; ++i) {
    for (int j = 0; j < 3; ++j) {
      jointEulerAngles[i][j] += dTheta(i * 3 + j);
    }
  }

}

void IK::Tikhonov(Eigen::MatrixXd &J, Eigen::VectorXd &dX, Eigen::VectorXd &dTheta) {

  // Tikhonov: (J^T * J + lambda * I) * delta_theta = J^T * delta_x
  Eigen::MatrixXd JTJ = J.transpose() * J;
  double lambda = 1e-3;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(JTJ.rows(), JTJ.cols());
  Eigen::VectorXd JTx = J.transpose() * dX;
  dTheta = (JTJ + lambda * I).ldlt().solve(JTx);
}

void IK::pseudoInverse(Eigen::MatrixXd &J, Eigen::VectorXd &dX, Eigen::VectorXd &dTheta) {

  // pseudo inverse
  // delta_theta = JDagger * delta_x
  Eigen::MatrixXd JJT = J * J.transpose();
  Eigen::MatrixXd JJT_Inverse = JJT.inverse();
  Eigen::MatrixXd JDagger = J.transpose() * JJT_Inverse; // 66 x 12, J is 12 x 66
  dTheta = JDagger * dX;
}

void IK::performIK(Eigen::MatrixXd &J, Eigen::VectorXd &dX, Eigen::VectorXd &dTheta, int IKMode) {

  bool subdivide = false;
  double threshold = 1.0;
  for (int i = 0; i < dX.rows(); ++i) {
    if (dX(i) > threshold) subdivide = true;
  }

  if (subdivide) {
    std::cout << subdivide << std::endl;
    dX *= 0.5;
    performIK(J, dX, dTheta, IKMode);
    dTheta *= 2;
  } else {

    int numJoints = fk->getNumJoints();

    if (IKMode == 0) {
      // Tikhonov: (J^T * J + lambda * I) * delta_theta = J^T * delta_x
      Tikhonov(J, dX, dTheta);

    } else if (IKMode == 1) {
      // pseudo inverse
      // delta_theta = JDagger * delta_x
      pseudoInverse(J, dX, dTheta);
    } else {
      dTheta = 100 * J.transpose() * dX;
    }

  }


}
