#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	// init the errors to 0
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	previous_cte = 0.0;

	// Init the coefficients
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;

}

void PID::UpdateError(double cte) {
	// The proportional error is equal to the cte
	p_error = cte;

	// The derivative error is equal to the current cte minus
	// the previous one.
	d_error = cte - previous_cte;

	// Sum the cte to the integral error
	i_error += cte;

	// Save the cte input value for next iteration
	previous_cte = cte;
}

double PID::TotalError() {
	// Sum up the 3 errors multiplied by their coefficients
	return (-p_error*Kp - i_error*Ki - d_error*Kd);
}
