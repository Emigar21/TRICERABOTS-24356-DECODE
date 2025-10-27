package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class PID {
    double prevError = 0;
    double sumError;
    //0.0046
    final double kP = ConfigVariables.kP;
    final double kI = ConfigVariables.kI;
    final double kD = ConfigVariables.kD;
    final double kF = ConfigVariables.kF;

    ElapsedTime timer = new ElapsedTime();
    public double calculatePID(double setPoint, double actualPosition){
        //set the Proportional calculation
        double error = setPoint - actualPosition;

        //Make a condition so there is not saturation on the Integral calculation
        double saturation = error - prevError;
        if(Math.abs(saturation) > 0.1){
            //Calculate the integral sum
            sumError += error * timer.seconds();
        }

        //Calculate the derivative
        double derivative = (error - prevError)/timer.seconds();

        //Get the last error
        prevError = error;

        //Reset timer
        timer.reset();

        //Get the motor output
        double output = (kP * error) + (kI * sumError) + (kD * derivative) + (setPoint * kF);
        return output;
    }
}
