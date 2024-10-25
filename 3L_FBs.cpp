#include "dll.h"
#include <windows.h>
#include <math.h>

#define hpwm 1E-04
#define N 167


extern "C"
{
		 
	/////////////////////////////////////////////////////////////////////
	// FUNCTION: SimulationStep
	//   This function runs at every time step.
	//double t: (read only) time
	//double delt: (read only) time step as in Simulation control
	//double *in: (read only) zero based array of input values. in[0] is the first node, in[1] second input...
	//double *out: (write only) zero based array of output values. out[0] is the first node, out[1] second output...
	//int *pnError: (write only)  assign  *pnError = 1;  if there is an error and set the error message in szErrorMsg
	//    strcpy(szErrorMsg, "Error message here..."); 
	DLLIMPORT void SimulationStep(
			double t, double delt, double *in, double *out,
			 int *pnError, char * szErrorMsg,
			 void ** ptrUserData, int nThreadIndex, void * pAppPtr)
	{
		// Constants
		static float  fg = 60., wg = 0., pi = 3.1415, fs = 1/hpwm, vt;
		
		// Processing Loop Control
		static float  td = 0.;
	
		// Switch Status
		static bool   qg = 0, ql = 0; // Switchs

		
		// Inputs (S denotes save variables, read)
		static double egs = 0., igs = 0., vCs = 0., vtris = 0.;
	
		// Scalar PWM
		static double vsh0_ref = 0., vse0_ref = 0.; // 2L
		

		// Grid Current Control - Resonant PI
		static double vse_ref = 0.,vl_ref = 0.,vsh_ref = 0;					// Reference


		static double thetasw_ref = 0.;	
		static double theta = 0., wf = 0, Pd = 0.0;						//  Period-switched phase

		



		
		///////////////////////////////////
		// Input
		vt = in[0];
		egs = in[1];
		
		//if(deg==1) Il_ref = 26.9;
		
		if(td <= t) // Start Process
		{
			td = td + hpwm;
			
			theta = theta + (2*pi*60)*hpwm; // PLL output
			   
			if(theta>=2.*pi) theta -= 2.*pi; 		// Restart Phase
			if(theta<= 0.) theta += 2.*pi;			// Restart Phase
			
			
			vse_ref = egs - vl_ref;
			vsh_ref = 180*sin(theta-0.1598);//shunt
			vl_ref = 110*sqrt(2)*sin(theta); // Open Loop
					
			vsh0_ref = -vsh_ref;
			vse0_ref = -vse_ref;
			
		}
		
		if(vsh0_ref >= vt) qg = 1; else qg = 0;
		if(vse0_ref >= vt) ql = 1; else ql = 0;


		// Output
		
		out[0] = qg;
		out[1] = ql;
		out[2] = vsh_ref;
		out[3] = vse_ref;
		out[4] = vsh0_ref;
		out[5] = vse0_ref;

		// ............end	
	}
}

