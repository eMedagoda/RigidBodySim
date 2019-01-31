class PID
{
    public:            

        PID(double P, double I, double D);
        
        ~PID();
        
        double Control(double sp, double pv, double dt);
        
    private:        
        
        double m_P;
        double m_I;
        double m_D;
        
        double m_prev_error;
        double m_integral;
        
};
