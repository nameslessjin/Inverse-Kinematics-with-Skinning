CSCI 520, Assignment 3

Jinsen Wu

================

<Description of what you have accomplished>
1. Implemented Skinning.
2. Implemented computeLocalAndGlobalTransforms and computeJointTransforms.
3. Implemented forwardKinematicsFunction, train_adolc and doIK.

<Also, explain any extra credit that you have implemented.>
1. Implemented Dual-Quaternion Skinning.
2. Implemented Spherical Blending Skinning.
2. Implemented the pseudoinverse IK method.
3. Implemented the Jacobian Transpose IK method.
4. Implemented Screen Save, Save screen to PPM
5. When the user moves the IK handle for a long distance, divide the IK process into several sub-steps to improve the solution, where each sub-step solves the IK problem on a portion of the original distance
6. Modified Driver.cpp, allow switching between Linear Blending, Dual Quaternion Blending and Spherical Blending with key 'd' during runtime.
7. Modified Driver.cpp, allow switching between Tikhonov IK, Pseudo inverse IK and Jacobian Transpose IK with key 'p' during runtime.
8. Update the title of the app to reflect the current skinning mode and Inverse Kinematic mode
9. Provide a detailed report and comparison on Linear Blending Skinning vs Dual Quaternion Skinning vs Spherical Blending, check file HW3Report
10. Provide a detailed report and comparison on Tikhonov Regularization IK vs Pseudo Inverse IK vs Jacobian Transpose IK, check file HW3Report


<Note>
The location of the video is in jpeg folder under root.
In this video, we tried the below combination in order:
Tikhonov + Linear Blending
Tikhonov + Dual-Quaternion
Tikhonov + Spherical Skinning Blending
Pseudo Inverse + Linear Blending
Jacobian Transpose + Linear Blending
