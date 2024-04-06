#include "skinning.h"
#include "vec3d.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

Skinning::Skinning(int numMeshVertices, const double * restMeshVertexPositions,
    const std::string & meshSkinningWeightsFilename)
{
  this->numMeshVertices = numMeshVertices;
  this->restMeshVertexPositions = restMeshVertexPositions;

  cout << "Loading skinning weights..." << endl;
  ifstream fin(meshSkinningWeightsFilename.c_str());
  assert(fin);
  int numWeightMatrixRows = 0, numWeightMatrixCols = 0;
  fin >> numWeightMatrixRows >> numWeightMatrixCols;
  assert(fin.fail() == false);
  assert(numWeightMatrixRows == numMeshVertices);
  int numJoints = numWeightMatrixCols;

  vector<vector<int>> weightMatrixColumnIndices(numWeightMatrixRows);
  vector<vector<double>> weightMatrixEntries(numWeightMatrixRows);
  fin >> ws;
  while(fin.eof() == false)
  {
    int rowID = 0, colID = 0;
    double w = 0.0;
    fin >> rowID >> colID >> w;
    weightMatrixColumnIndices[rowID].push_back(colID);
    weightMatrixEntries[rowID].push_back(w);
    assert(fin.fail() == false);
    fin >> ws;
  }
  fin.close();

  // Build skinning joints and weights.
  numJointsInfluencingEachVertex = 0;
  for (int i = 0; i < numMeshVertices; i++)
    numJointsInfluencingEachVertex = std::max(numJointsInfluencingEachVertex, (int)weightMatrixEntries[i].size());
  assert(numJointsInfluencingEachVertex >= 2);

  // Copy skinning weights from SparseMatrix into meshSkinningJoints and meshSkinningWeights.
  meshSkinningJoints.assign(numJointsInfluencingEachVertex * numMeshVertices, 0);
  meshSkinningWeights.assign(numJointsInfluencingEachVertex * numMeshVertices, 0.0);
  for (int vtxID = 0; vtxID < numMeshVertices; vtxID++)
  {
    vector<pair<double, int>> sortBuffer(numJointsInfluencingEachVertex);
    for (size_t j = 0; j < weightMatrixEntries[vtxID].size(); j++)
    {
      int frameID = weightMatrixColumnIndices[vtxID][j];
      double weight = weightMatrixEntries[vtxID][j];
      sortBuffer[j] = make_pair(weight, frameID);
    }
    sortBuffer.resize(weightMatrixEntries[vtxID].size());
    assert(sortBuffer.size() > 0);
    sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort in descending order using reverse_iterators
    for(size_t i = 0; i < sortBuffer.size(); i++)
    {
      meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].second;
      meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].first;
    }

    // Note: When the number of joints used on this vertex is smaller than numJointsInfluencingEachVertex,
    // the remaining empty entries are initialized to zero due to vector::assign(XX, 0.0) .
  }
}

void Skinning::applySkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions, bool isDQS) const
{
  // Students should implement this

  // The following below is just a dummy implementation.

  if (isDQS) applyDualQuaternionSkinning(jointSkinTransforms, newMeshVertexPositions);
  else applyLinearBlendSkinning(jointSkinTransforms, newMeshVertexPositions);
}

void Skinning::applyLinearBlendSkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions) const {

  for (int i = 0; i < numMeshVertices; ++i) {

    Vec4d restVertexPos = {restMeshVertexPositions[3 * i + 0], restMeshVertexPositions[3 * i + 1], restMeshVertexPositions[3 * i + 2], 1};
    Vec4d newVertexPos(0.0);
  
    for (int j = 0; j < numJointsInfluencingEachVertex; ++j) {
       newVertexPos += meshSkinningWeights[i * numJointsInfluencingEachVertex + j] * jointSkinTransforms[meshSkinningJoints[i * numJointsInfluencingEachVertex + j]] * restVertexPos;
    }

    for (int j = 0; j < 3; ++j) newMeshVertexPositions[3 * i + j] = newVertexPos[j];
  }

}

void Skinning::applyDualQuaternionSkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions) const {

  for (int i = 0; i < numMeshVertices; ++i) {

    Eigen::Quaterniond q0(0, 0, 0, 0);
    Eigen::Quaterniond q1(0, 0, 0, 0);
  
    for (int j = 0; j < numJointsInfluencingEachVertex; ++j) {
      int jointId = meshSkinningJoints[i * numJointsInfluencingEachVertex + j];
      double weight = meshSkinningWeights[i * numJointsInfluencingEachVertex + j];

      double arr[16];
      jointSkinTransforms[jointId].convertToArray(arr);
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> skinningMat(arr);

      Eigen::Matrix3d R = skinningMat.block<3,3>(0,0);
      Eigen::Vector3d t = skinningMat.block<3,1>(0,3);

      Eigen::Quaterniond qi(R);
      if (qi.w() < 0) qi.w() *= -1; // w is the real part of quaternion
      qi.normalized();


      // (qi, 0.5 * ti * qi) = dual quaternion qi
      Eigen::Quaterniond ti (0, t(0), t(1), t(2));
      Eigen::Quaterniond halftq = ti * qi;
      halftq.coeffs() *= 0.5;

      // blend 
      q0.coeffs() += weight * qi.coeffs(); 
      q1.coeffs() += weight * halftq.coeffs();

    }

    q0.normalized();
    q1.normalized();

    // t = 2 * q1_norm() * q0_norm()_inverse
    Eigen::Quaterniond t = q1 * q0.inverse();
    t.coeffs() *= 2;
    Eigen::Vector3d T = {t.x(), t.y(), t.z()};

    // R
    Eigen::Matrix3d R = q0.toRotationMatrix();

    Eigen::Vector3d restVertexPos = {restMeshVertexPositions[3 * i + 0], restMeshVertexPositions[3 * i + 1], restMeshVertexPositions[3 * i + 2]};
    Eigen::Vector3d newVertexPos = R * restVertexPos + T;
    
    for (int j = 0; j < 3; ++j) newMeshVertexPositions[3 * i + j] = newVertexPos[j];
  }

}

