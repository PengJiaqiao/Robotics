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

        PrVector3 g123 = PrVector3(0,0,0); //Variable that holds the torque exerted by gravity for each joint

        //Compute g123 here! 
float cos_q1=cos(gv.q[0]);
float cos_q12=cos((gv.q[0]+gv.q[1])-(M_PI*0.5));
float cos_q123=cos((gv.q[0]+gv.q[1]+gv.q[2])-(M_PI*0.5));
float gravity=-9.81;
float R2val=0.189738;

//first row members
float r11=(R2*M2*gravity*cos_q1);
float r12=(gravity*(M3+M4+M5)*((L2*cos_q1)+(R2val*cos_q12)));
float r13=M6*(L2*cos_q1+L3*cos_q12+R6*cos_q123)*gravity;

//second row members
float r21=0;
float r22=gravity*(M3+M4+M5)*R2val*cos_q12;
float r23=M6*gravity*(L3*cos_q12+(R6*cos_q123));
//third row members
float r31=0;
float r32=0;
float r33=M6*gravity*R6*cos_q123;

float T1=r11+r12+r13;
float T2=r21+r22+r23;
float T3=r31+r32+r33;

	g123[0]=T1;
	g123[1]=T2;
	g123[2]=T3;
       

        //maps the torques to the right joint indices depending on the current mode:
        if (gv.dof == 3) {
            gv.G[0] = g123[0];
            gv.G[1] = g123[1];
            gv.G[2] = g123[2];
        } else if (gv.dof == 6) {
            gv.G[1] = g123[0];
            gv.G[2] = g123[1];
            gv.G[4] = g123[2];
        }

//        printVariable(g123, "g123");
    } else {
        gv.G = PrVector(gv.G.size());
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

void initNjtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
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

void initProj1Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj2Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj3Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}


// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables& gv)
{
}

void floatControl(GlobalVariables& gv)
{   
	gv.tau[0]=gv.G[0];
	gv.tau[1]=gv.G[1];
	gv.tau[2]=gv.G[2];
	PrintDebug(gv);
}

void openControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void njholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void jholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void njmoveControl(GlobalVariables& gv)
{
 gv.tau[0]=gv.kp[0]*(gv.qd[0]-gv.q[0]); // P-controller for every angle 

 gv.tau[1]=gv.kp[1]*(gv.qd[1]-gv.q[1]);

 gv.tau[2]=gv.kp[2]*(gv.qd[2]-gv.q[2]); 
 }

void jmoveControl(GlobalVariables& gv)
{
   floatControl(gv); //Remove
}

void njgotoControl(GlobalVariables& gv) 
{	
gv.tau[0]=gv.kp[0]*(gv.qd[0]-gv.q[0])+gv.G[0];
gv.tau[1]=gv.kp[1]*(gv.qd[1]-gv.q[1])+gv.G[1];
gv.tau[2]=gv.kp[2]*(gv.qd[2]-gv.q[2])+gv.G[2];  
// Remove this line when you implement this controller
}

void jgotoControl(GlobalVariables& gv) 
{
   gv.tau[0]=gv.kp[0]*(gv.qd[0]-gv.q[0])+gv.G[0]-gv.kv[0]*gv.dx[0];
   gv.tau[1]=gv.kp[1]*(gv.qd[1]-gv.q[1])+gv.G[1]-gv.kv[1]*gv.dx[1];  
   gv.tau[2]=gv.kp[2]*(gv.qd[2]-gv.q[2])+gv.G[2]-gv.kv[2]*gv.dx[2];
   
}

void njtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void jtrackControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void nxtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void xtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void nholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void holdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void ngotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void gotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void ntrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void trackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void pfmoveControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void lineControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void proj1Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj1Control
}

void proj2Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj2Control
}

void proj3Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj3Control
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
