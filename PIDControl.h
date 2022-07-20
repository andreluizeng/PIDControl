/* Copyright (C) 2022 Andre Silva. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Andre Silva
 Web      :  http://github.com/andreluizeng
 e-mail   :  andreluizeng@yahoo.com.br
 */

#ifndef __PIDControl_h__
#define __PIDControl_h__

#include <malloc.h>

class PIDControl 
{
	public:
            
		void begin (float kp, float ki, float kd);
		float computePID (float input, float set_point);
		void setOutputLimits (float min, float max);
		void setGains (float kp, float ki, float kd);
		float getOutput (void);
		float getError (void);
		void splitPID (float *pTerm, float *iTerm, float *dTerm);  // get individual control output value
		

	private:
		
		float _error;
		float _last_error;
		float _i_error;
		float _d_error;
		float _kp;
		float _ki;
		float _kd;
		float _set_point;
		float _limit_min; // default 0
		float _limit_max; // default 100
		
		float _pTerm;  // proporcional control output
		float _iTerm;	// integral control output
		float _dTerm;	// derivative control output
		
		float _output;
	

};

#endif
