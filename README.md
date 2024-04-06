CSCI 520, Assignment 2

Jinsen Wu

================

<Description of what you have accomplished>
1. Implemented Skinning.
2. Implemented computeLocalAndGlobalTransforms and computeJointTransforms.
3. Implemented forwardKinematicsFunction, train_adolc and doIK.

<Also, explain any extra credit that you have implemented.>
1. Implemented Dual-Quaternion Skinning.
2. Implemented the pseudoinverse IK method and compare it with other IK algorithm
3. When the user moves the IK handle for a long distance, divide the IK process into several sub-steps to improve the solution, where each sub-step solves the IK problem on a portion of the original distance
4. Modified Driver.cpp, allow switching between Linear Blending and Dual Quaternion Blending with key 'd' during runtime.
5. Modified Driver.cpp, allow switching between Tikhonov IK and Pseudo inverse IK with key 'p' during runtime.
6. Provide a detailed report and comparison on Linear Blending Skinning vs Dual Quaternion Skinning, check file HW3Report
7. Provide a detailed report and comparison on Tikhonov Regularization IK vs Pseudo Inverse IK, check file HW3Report
8. Update the title of the app to reflect the current skinning mode and Inverse Kinematic mode


