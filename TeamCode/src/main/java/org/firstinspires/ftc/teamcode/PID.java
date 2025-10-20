package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    static double kP = 0.0;
    static double kI = 0.0;
    static double kD = 0.0;
    static double kF = 0.0;
    static double kCP = 1;
    static double kCI = 0.1;
    static double kCD = 0.01;
    static double kCF = 3.0;
    static double error;
    static double sumError = 0.0;
    static double prevError = 0.0;
    static double derivative;
    static double saturation;
    static ElapsedTime timer = new ElapsedTime();

    public static double calculatePID(double setPoint, double actualPoint){
        //set the Proportional calculation
        error = setPoint - actualPoint;

        //Make a condition so there is not saturation on the Integral calculation
        saturation = error - prevError;
        if(saturation < -.1 || saturation < .1){
            //Calculate the integral sum
            sumError += error * timer.seconds();
        }

        //Calculate the derivative
        derivative = (error - prevError)/timer.seconds();

        //Get the last error
        prevError = error;

        //Reset timer
        timer.reset();

        //Get the motor output
        double output = (kP * error) + (kI * sumError) + (kD * derivative) + (setPoint * kF);
        return output;
    }
    public static double calculateOrientationPID(double setPoint, double actualPoint){
        //set the Proportional calculation
        error = angleWrap(setPoint - actualPoint);

        //Make a condition so there is not saturation on the Integral calculation
        saturation = error - prevError;
        if(saturation < -.1 || saturation > .1){
            //Calculate the integral sum
            sumError += error * timer.seconds();
        }

        //Calculate the derivative
        derivative = (error - prevError)/timer.seconds();

        //Get the last error
        prevError = error;

        //Reset timer
        timer.reset();

        //Get the motor output
        double output = (kCP * error) + (kCI * sumError) + (kCD * derivative) + (setPoint * kCF);
        return output;
    }

    public static double angleWrap(double angle) {
        while (angle < -179){
            angle += 360;
        } while (angle > 179){
            angle -= 360;
        }
        return angle;
    }
}
