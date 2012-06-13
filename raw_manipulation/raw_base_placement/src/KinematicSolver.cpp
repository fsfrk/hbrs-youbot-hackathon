/*
 * KinematicSolver.cpp
 *
 *  Created on: May 21, 2012
 *      Author: Giftsun
 */
#include <iostream>
using namespace std;

#include "KinematicSolver.h"


KinematicSolver::KinematicSolver()
{
  
  x=0;y=0;t=0;t1=0;t2=0;t3=0;t4=0;t5=0;
  JointParameter JP(1,8);
  JP << x,y,t,t1,t2,t3,t4,t5;
  UpdateDHTable(JP);
  T0 = ht_from_eul(deg2rad(180),deg2rad(-90), 0);

}


KinematicSolver::~KinematicSolver()
{

}

void KinematicSolver::UpdateDHTable(JointParameter p)
{
  DH_Parameters youBot_unifiedDHparam(10,4);
 // cout << youBot_unifiedDHparam.rows()<<endl;
 
  youBot_unifiedDHparam << 0,              p(0,0),    0,    0,
                           deg2rad(90),  p(0,1),    0,    deg2rad(90),
                           deg2rad(90),  0,	     0,    p(0,2),    
                           0,     dbase,   abase,   0, 
                           0,     L_Prior,   0,     0, 
			   0,     0,         0,    p(0,3),
                           deg2rad(90),  0,         L1,   p(0,4),
			   0,     0,         L2,   p(0,5),
			   0,     0,         L3,   p(0,6),
			   deg2rad(-90),  L4,        0,    p(0,7);
  //youBot_unifiedDHparam << 0,p(1,1),0,0,(deg2rad(pi/2)),p(1,2),0,0,deg2rad(pi/2),0,0,p (1,3),0,dbase,abase,0,0,L_Prior,0,0,0,0,0,p(1,4),deg2rad(pi/2),0,L1,p(1,5),   0,0,L2,p(1,6),0,0,L3,p(1,7),deg2rad(-pi/2),L4,0,p(1,8);

  //youBot_unifiedDHparam << 1,p(0,1),0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,p(0,1);
  //cout << "The new table is "<<endl<< youBot_unifiedDHparam<<endl;

 dh_table = youBot_unifiedDHparam;
}


HomogenousTransform KinematicSolver::calculateForwardKinematics(JointParameter parameter)
{	
    UpdateDHTable(parameter);
    
    HomogenousTransform T1 = ht_from_dh(dh_table(0,0), dh_table(0,1), dh_table(0,2), dh_table(0,3));

    HomogenousTransform T2 = ht_from_dh(dh_table(1,0), dh_table(1,1), dh_table(1,2), dh_table(1,3));
    HomogenousTransform T3 = ht_from_dh(dh_table(2,0), dh_table(2,1), dh_table(2,2), dh_table(2,3));
    HomogenousTransform TJoint = ht_from_dh(dh_table(3,0), dh_table(3,1), dh_table(3,2), dh_table(3,3));
    HomogenousTransform TArmPrior = ht_from_dh(dh_table(4,0), dh_table(4,1), dh_table(4,2), dh_table(4,3));
    HomogenousTransform T4 = ht_from_dh(dh_table(5,0), dh_table(5,1), dh_table(5,2), dh_table(5,3));
    HomogenousTransform T5 = ht_from_dh(dh_table(6,0), dh_table(6,1), dh_table(6,2), dh_table(6,3));
    HomogenousTransform T6 = ht_from_dh(dh_table(7,0), dh_table(7,1), dh_table(7,2), dh_table(7,3));
    HomogenousTransform T7 = ht_from_dh(dh_table(8,0), dh_table(8,1), dh_table(8,2), dh_table(8,3));
    HomogenousTransform T8 = ht_from_dh(dh_table(9,0), dh_table(9,1), dh_table(9,2), dh_table(9,3));
    
    /*cout  <<"T0 = "<<T0.affine()<<endl; 
    cout  <<"T1 = "<<T1.affine()<<endl; 
    cout  <<"T2 = "<<T2.affine()<<endl;  
    cout  <<"T3 = "<<T3.affine()<<endl; */
   	
    HomogenousTransform TRobot = T0*T1*T2*T3*TJoint*TArmPrior*T4*T5*T6*T7*T8;    
    
    HomogenousTransform TBase = (T0*T1*T2*T3);

    //cout <<"TRobot = "<<TRobot.affine()<<endl;   
    //cout<<"-----------------"<<endl;
    //Normalize(TBase);
     //cout <<"TRobot = "<<TRobot.affine()<<endl;   
    return TRobot;
}


bool KinematicSolver::solveIK(Pose GoalPose, JointParameter& prefConfig)
{

float Xdash = 0.100 ;

HomogenousTransform GoalTR = ht_from_xyzrpy(GoalPose(0),GoalPose(1),GoalPose(2),GoalPose(3),GoalPose(4),GoalPose(5));

GoalTR = ht_from_xyz(GoalPose(0),GoalPose(1),GoalPose(2)) * ht_from_eul(GoalPose(3),GoalPose(4),GoalPose(5));

float Beta  = atan2(GoalTR(2,2),sqrt( pow(GoalTR(0,2),2)+pow(GoalTR(1,2),2)));

//Vertical Height
float Zw = GoalPose(0,2);
float Z2w = 0.249838 ;
float Zdash  = Zw - Z2w;
float Zddash = Zdash - (L4*cos(Beta));
 cout<<"Zddash"<<Zddash;
// Sin and Cos exchange
float Xddash = Xdash - (L4*sin(Beta));
cout<<"Xddash"<<Xddash;
 
// Theta 3 evaluation
float ct3 = (-pow(Zddash,2)-pow(Xddash,2) + pow(L2,2)+pow(L3,2))/(2*L2*L3);
//float t3 = acos(ct3);
float st3 = sqrt(1-pow(ct3,2));

float t3 = atan2(st3,ct3);


// Theta 4 evaluation
float k2 = L3*sin(t3);
float k1 = L2-(L3*cos(t3));
float t2 = atan2(Zddash,Xddash)+ atan2(k2,k1);
float t4= t2+t3-Beta;

// Base Orientation 
float t = atan2(GoalTR(1,2),GoalTR(0,2));

// Angular transformations to suit the fk solver

//t2 = deg2rad(-90)+t2;
t3 = (deg2rad(180)+t3);
t4 = (deg2rad(180)-t4);
//t4 = deg2rad(270)-t4;

//Base position
JointParameter tmpconfig(1,8);
tmpconfig << 0,0,t,0,t2,t3,t4,0;
HomogenousTransform  TRobot;

//cout<<tmpconfig<<endl;
TRobot = calculateForwardKinematics(tmpconfig); 
cout <<"TRobot-1 = "<<TRobot.affine()<<endl;
float x = GoalPose(0)-TRobot(0,3); 
float y = GoalPose(1)-TRobot(1,3);

//Theta 5 Solution
tmpconfig << x,y,t,0,t2,t3,t4,0;
TRobot = calculateForwardKinematics(tmpconfig);
//cout <<"TRobot02 = "<<TRobot.affine()<<endl;
Eigen::MatrixXf Product(3,3) ; 
Product = (TRobot.rotation().transpose())*GoalTR.rotation();
//HomogenousTransform  Product = GoalTR;
float t5 = atan2(Product(1,0),Product(0,0));
//Theta 1 Solution
float t1=0;

//Final Solution

prefConfig << x,y,t,t1,t2,t3,t4,t5; 
TRobot = calculateForwardKinematics(prefConfig);
cout << TRobot.affine();

return(true);
}






