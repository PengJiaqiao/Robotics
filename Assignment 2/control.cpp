// controlDLL.cpp : Defines the entry point for the DLL application.
//
#include "servo.h"
#include "param.h"
#include "control.h"
//#include "UiAgent.h"
#include "PrVector.h"
#include "PrMatrix.h"
#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.
#include <math.h>
#include <algorithm>
using std::min;
using std::max;

void PrintDebug(GlobalVariables& gv);

struct CubicSpline {
  double t0, tf;
  PrVector a0, a1, a2, a3;
};

// define a global variable to make the spline parameters accessible to your control function
CubicSpline spline;

double circle_start_time;
double proj3_end_time;

double computeTf(GlobalVariables& gv);

// *******************************************************************
// Initialization functions
// *******************************************************************

void InitControl(GlobalVariables& gv) 
{
   // This code runs before the first servo loop
}

void PreprocessControl(GlobalVariables& gv)
{
   // This code runs on every servo loop, just before the control law
   
   
  // 
    if ((gv.dof == 3) || (gv.dof == 6)) {

        //get the correct joint angles depending on the current mode:
        double q1,q2,q3;
        if (gv.dof == 3) {
            q1 = gv.q[0];
            q2 = gv.q[1];
            q3 = gv.q[2];
        } else if (gv.dof == 6) {
            q1 = gv.q[1];
            q2 = gv.q[2];
            q3 = gv.q[4];
        }

        /* PrVector3 g123 = PrVector3(0,0,0); //Variable that holds the torque exerted by gravity for each joint */

        //Compute g123 here!        

        //maps the torques to the right joint indices depending on the current mode:
        /* if (gv.dof == 3) { */
        /*     gv.G[0] = g123[0]; */
        /*     gv.G[1] = g123[1]; */
        /*     gv.G[2] = g123[2]; */
        /* } else if (gv.dof == 6) { */
        /*     gv.G[1] = g123[0]; */
        /*     gv.G[2] = g123[1]; */
        /*     gv.G[4] = g123[2]; */
        /* } */

//        printVariable(g123, "g123");
    /* } else { */
    /*     gv.G = PrVector(gv.G.size()); */
    }   
}

void PostprocessControl(GlobalVariables& gv) 
{
   // This code runs on every servo loop, just after the control law
}

void initFloatControl(GlobalVariables& gv) 
{
    // Control Initialization Code Here
}

void initOpenControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initJgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjtrackControl(GlobalVariables& gv) {
  spline.t0 = gv.curTime;
  spline.tf = computeTf(gv);

  double dt = spline.tf-spline.t0;

  spline.a0 = gv.q;
  spline.a1 = gv.dq;
  spline.a2 =  3/pow(dt,2) * (gv.qd-gv.q) - 2/dt * gv.dq;
  spline.a3 = -2/pow(dt,3) * (gv.qd-gv.q) + 1/pow(dt,2) * gv.dq;
}

void initJtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNxtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initXtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initHoldControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initGotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initTrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initPfmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initLineControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj1Control(GlobalVariables& gv) {
  gv.qd[0] = -0.45;
  gv.qd[1] =  0.65;
  gv.qd[2] = -0.17;
  initNjtrackControl(gv);
}

void initProj2Control(GlobalVariables& gv) {
  circle_start_time = gv.curTime;
}

void initProj3Control(GlobalVariables& gv) {
  proj3_end_time = gv.curTime + 15.0;
}


// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables& gv)
{
}

void floatControl(GlobalVariables& gv)
{   
	gv.tau = gv.G;
	PrintDebug(gv);
}

void openControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void jholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njmoveControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void jmoveControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njgotoControl(GlobalVariables& gv) 
{	
   floatControl(gv);  // Remove this line when you implement openControl
}

void jgotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njtrackControl(GlobalVariables& gv) {
  if (gv.curTime <= spline.tf) {
    double t = gv.curTime - spline.t0;
    gv.qd = spline.a0 + spline.a1*t + spline.a2*pow(t, 2) + spline.a3*pow(t, 3);

    gv.tau = -gv.kp * (gv.q - gv.qd) - gv.kv * (gv.dq - gv.dqd) + gv.G;
  } else {
    floatControl(gv);
  }
}

void jtrackControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void nxtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void xtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void nholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void holdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void ngotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void gotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void ntrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void trackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void pfmoveControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void lineControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void proj1Control(GlobalVariables& gv) {
  njtrackControl(gv);
}

void proj2Control(GlobalVariables& gv) {
  double center[] = {0.45, 0.60};
  double angular_velocity = 2*M_PI/5;
  double angle = (gv.curTime-circle_start_time) * angular_velocity;
  double radius = 0.2;

  gv.xd[0] = center[0] + cos(angle+M_PI/2) * radius;
  gv.xd[1] = center[1] + sin(angle+M_PI/2) * radius;
  gv.xd[2] = 0;

  PrVector F = -gv.kp * (gv.x - gv.xd) - gv.kv * (gv.dx - gv.dxd);
  gv.tau = gv.Jtranspose * F;
}

void proj3Control(GlobalVariables& gv) {
  if (gv.curTime <= proj3_end_time) {
    proj2Control(gv);
  } else {
    floatControl(gv);
  }
}

// *******************************************************************
// Debug function
// *******************************************************************

void PrintDebug(GlobalVariables& gv)
{
   // Replace this code with any debug information you'd like to get
   // when you type "pdebug" at the prompt.
   printf( "This sample code prints the torque and mass\n" );
   gv.tau.display( "tau" );
   gv.A.display( "A" );
}

#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
}
#endif //#ifdef WIN32

/********************************************************

END OF DEFAULT STUDENT FILE 

ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS 

*******************************************************/

double computeTf(GlobalVariables& gv) {
  double tf = gv.curTime;

  PrVector distance = gv.qd - gv.q;

  /* fprintf(stdout, "dof: %d\n", gv.dof); */

  /* PrVector seconds = distance / gv.dqmax; */
  PrVector seconds(gv.dof);
  for (int i=0; i < gv.dof; i++) {
    seconds[i] = abs(distance[i] / gv.dqmax[i]);
    if (seconds[i] > tf) {
      tf = seconds[i];
    }
  }

  /* fprintf(stdout, "dist: %f / max_vel: %f = %f\n", distance[0], gv.dqmax[0], seconds[0]); */
  /* fprintf(stdout, "dist: %f / max_vel: %f = %f\n", distance[1], gv.dqmax[1], seconds[1]); */
  /* fprintf(stdout, "dist: %f / max_vel: %f = %f\n", distance[2], gv.dqmax[2], seconds[2]); */
  /* fprintf(stdout, "tf: %f\n", tf); */

  double accel_offset = 0.0;
  PrVector seconds2(gv.dof);
  for (int i=0; i < gv.dof; i++) {
    seconds2[i] = abs((gv.dqmax[i] - gv.dq[i]) / gv.ddqmax[i]);
    if (seconds2[i] > accel_offset) {
      accel_offset = seconds2[i];
    }
  }

  /* fprintf(stdout, "dqmax: %f / ddqmax: %f = %f\n", gv.dqmax[0], gv.ddqmax[0], seconds2[0]); */
  /* fprintf(stdout, "dqmax: %f / ddqmax: %f = %f\n", gv.dqmax[1], gv.ddqmax[1], seconds2[1]); */
  /* fprintf(stdout, "dqmax: %f / ddqmax: %f = %f\n", gv.dqmax[2], gv.ddqmax[2], seconds2[2]); */
  /* fprintf(stdout, "accel_offset: %f\n", accel_offset); */

  /* fprintf(stdout, "%f\n", tf+2*accel_offset); */

  return tf + 2*accel_offset;
  /* return gv.curTime + 10.0; */
}
