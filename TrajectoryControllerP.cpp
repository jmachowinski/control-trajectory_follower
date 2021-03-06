/*
 * =====================================================================================
 *
 *       Filename:  TrajectoryControllerP.cpp
 *
 *    Description:  Implementation of trajectory controller with orientation control
 *    				from 'Springer handbook of robotics' chapter 34 pg 805
 *
 *        Version:  1.0
 *        Created:  10/13/09 10:30:32
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ajish Babu, ajish.babu@dfki.de
 *        Company:  DFKI
 *
 * =====================================================================================
 */

#include "TrajectoryControllerP.hpp"

using namespace trajectory_follower;

chainedProportional::chainedProportional ()
{
}  /* -----  end of method chainedProportional::chainedProportional  (constructor)  ----- */

	void
chainedProportional::setConstants(double K2_val, double K3_val)
{
	K2 = K2_val;
	K3 = K3_val;
} 

        Eigen::Vector2d	
chainedProportional::update (double u1, double d, double theta_e, double c, double c_s )
{
 	double d_dot, s_dot, z2, z3, v1, v2, u2;

 	d_dot = u1 * sin(theta_e);
	s_dot = u1 * cos(theta_e) / (1.0-d*c);

	z2 = d;
	z3 = (1.0-(d*c))*tan(theta_e);

	v1 = u1 * cos(theta_e) /(1.0 - d*c);
	v2 = (-v1 * K2 * z2) - (fabs(v1) * K3 * z3);

	u2 = ((v2 + ((d_dot*c + d*c_s*s_dot)*tan(theta_e))) / ((1.0-d*c)*(1+pow(tan(theta_e),2)))) - (s_dot*c);

	return Eigen::Vector2d(u1, u2);
}		/* -----  end of method chainedProportional::update  ----- */
	
   	bool
chainedProportional::checkInitialStability( double d, double theta_e, double c, double c_max)
{
 	double z2, z3;
	z2 = d;
	z3 = (1.0-(d*c))*tan(theta_e);

	if( z2*z2+(z3*z3/K2) < (1/(c_max*c_max)) )
    	    return true;
	else 
	    return false;	    
}
