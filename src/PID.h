#ifndef PID_H
#define PID_H
#include <vector>
#include <chrono>
#include <ctime>

class PID {
public:
    /*
    * Errors
    */
    double p_error;
    double i_error;
    double d_error;
    double d_old_error;
    bool has_d_old_error;
    
    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;
    
    std::chrono::time_point<std::chrono::system_clock> t_old, t;
    
    /*
    * Constructor
    */
    PID();
    
    /*
    * Destructor
    */
    virtual ~PID();
    
    /*
    * Initialize PID
    */
    void Init(double Kp, double Ki, double Kd);
    
    /*
    * Update the PID error variables given cross track error
    */
    void UpdateError(double cte);
    
    /*
    * Calculate the total PID error
    */
    double TotalError();
};

#endif /* PID_H */