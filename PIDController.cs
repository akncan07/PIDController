using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PIDController
{
    public class PIDController
    {
        private double kp;  // Proportional gain
        private double ki;  // Integral gain
        private double kd;  // Derivative gain

        private double setPoint;  // Desired value
        private double integral;
        private double lastError;
        private DateTime lastTime;

        public PIDController(double kp, double ki, double kd, double setPoint)
        {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.setPoint = setPoint;
            this.integral = 0.0;
            this.lastError = 0.0;
            this.lastTime = DateTime.Now;
        }

        public double Compute(double inputValue)
        {
            DateTime now = DateTime.Now;
            double timeChange = (now - lastTime).TotalSeconds;

            // Calculate error
            double error = setPoint - inputValue;

            // Proportional term
            double proportional = kp * error;

            // Integral term
            integral += error * timeChange;
            double integralTerm = ki * integral;

            // Derivative term
            double derivative = (error - lastError) / timeChange;
            double derivativeTerm = kd * derivative;

            // Calculate total output
            double output = proportional + integralTerm + derivativeTerm;

            // Remember some variables for next time
            lastError = error;
            lastTime = now;

            return output;
        }
    }
}
