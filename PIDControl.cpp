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

#include "PIDControl.h"

void PIDControl::begin (float kp, float ki, float kd)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;
	
	_error = 0;
	_last_error = 0;
	_i_error = 0;
	_d_error = 0;
	_set_point = 0;
	_limit_min = 0;
	_limit_max = 100;
	_pTerm = 0;
	_iTerm = 0;
	_dTerm = 0;
	
	_output = 0;
	
	return;
}

void PIDControl::setOutputLimits (float min, float max)
{
	_limit_min = min;
	_limit_max = max;
	
	return;
}

void PIDControl::setGains (float kp, float ki, float kd)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;
	
	return;
}

float PIDControl::getOutput (void)
{
	return _output;
}

float PIDControl::getError (void)
{
	return _error;
}

void PIDControl::splitPID (float *pTerm, float *iTerm, float *dTerm)
{
	*pTerm = _pTerm;
	*iTerm = _iTerm;
	*dTerm = _dTerm;
	
	return;
}

float PIDControl::computePID (float input, float set_point)
{
	_output = 0;
	
	_set_point = set_point;
	
	// calculates the current error
	_error = _set_point - input;
	
	// calculates the accumulative error
	_i_error += _error;
	
	// calculates the derivative error
	_d_error = _error - _last_error;
	
	// calculates the proportional control --> pTerm;
	_pTerm = _kp * _error;
	
	// calculates the integral control --> iTerm
	_iTerm = _ki * _i_error;
	
	// calculates the derivative control --> dTerm
	_dTerm = _kd * _d_error;

	_output = _pTerm + _iTerm + _dTerm;
		
	// updates last_error
	_last_error = _error;
	
	
	return _output;
}