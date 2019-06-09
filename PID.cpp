#include "PID.h"
#include <cmath>
#include <iostream>

PID::PID(double P, double I, double D)
: m_prev_error(0.0),
m_integral(0.0)
{
    m_P = P;
    m_I = I;
    m_D = D;    
}

PID::~PID()
{
}

double PID::Control(double sp, double pv, double dt)
{
    double output = 0.0;
    
    double error = sp - pv;    
    double derror = (error - m_prev_error)/dt;
    
    if (fabs(error) < 2.0)
    {
        m_integral += error;
    }
    else
    {
        m_integral = 0.0;
    }
       
    m_prev_error = error;    
   
    output = m_P * error + m_D * derror + m_I * m_integral * dt;    
    
    return output;
}
