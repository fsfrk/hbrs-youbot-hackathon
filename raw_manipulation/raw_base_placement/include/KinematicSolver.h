/*
 * KinematicSolver.h
 *
 *  Created on: May 21, 2012
 *      Author: Giftsun
 */

#ifndef KINEMATICSOLVER_H_
#define KINEMATICSOLVER_H_


#include "HomogenousTransform.h"
#include "math.h"

#include <vector>

typedef Eigen::Matrix<float, 6, Eigen::Dynamic> Jacobian;

//float l0x = 0.024;
//float l0z = 0.096;	
const float dbase   = 0.102838;
const float abase   = 0.156;
const float L_Prior = 0.147;
const float L1 = 0.033;
const float L2 = 0.155;
const float L3 = 0.135;
const float L4 = 0.081;


class KinematicSolver {
public:
	KinematicSolver();
	~KinematicSolver();

	HomogenousTransform calculateForwardKinematics(JointParameter parameter);

	HomogenousTransform getTransform(int dh_line, float theta);
        
        bool solveIK(Pose GoalPose, JointParameter& currentConfig);
 
       // std::vector<HomogenousTransform> getSingleTransformations(JointParameter parameter);

        void UpdateDHTable(JointParameter parameter);

      //Jacobian getJacobian(JointParameter q);


protected:
	DH_Parameters dh_table;
        float x, y, t, t1, t2, t3, t4, t5;
        HomogenousTransform T0;
        float R1, R2, R3;
};

#endif /* FORWARDKINEMATICS_H_ */
