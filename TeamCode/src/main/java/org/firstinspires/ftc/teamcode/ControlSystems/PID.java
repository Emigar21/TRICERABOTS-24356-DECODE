package org.firstinspires.ftc.teamcode.ControlSystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Variables.ConfigVariables;

public class PID {
    double prevTurretError = 0;
    public static double sumTurretError;
    public static double error;
    static double prevError;
    double sumError;
    double derivative;
    double saturation;
    ElapsedTime timer = new ElapsedTime();
    public double calculatePIDF(double setPoint, double actualPosition, double kP, double kI,
                                      double kD, double kF){
        //set the Proportional calculation
        error = setPoint - actualPosition;

        //Make a condition so there is not saturation on the Integral calculation
        saturation = error - prevError;
        if(Math.abs(saturation) > 0.00001){
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
        return (kP * error) + (kI * sumError) + (kD * derivative) + (setPoint * kF);
    }



    double prevOrientationError = 0;
    double sumOrientationError;
    double orientationSaturation;
    double orientationDerivative;
    ElapsedTime orientationTimer = new ElapsedTime();
    public double calculateAngleChassisPID(double setPoint, double actualOrientation, double kOrientationP, double kOrientationI, double kOrientationD, double kOrientationF){
        //set the Proportional calculation
        double error = angleWrap(setPoint - actualOrientation);

        //Make a condition so there is not saturation on the Integral calculation
        orientationSaturation = error - prevOrientationError;
        if(Math.abs(orientationSaturation) < -.1){
            //Calculate the integral sum
            sumOrientationError += error * orientationTimer.seconds();
        }

        //Calculate the derivative
        orientationDerivative = (error - prevOrientationError)/orientationTimer.seconds();

        //Get the last error
        prevOrientationError = error;

        //Reset timer
        orientationTimer.reset();

        //Get the motor output

        return  (kOrientationP * error) + (kOrientationI * sumOrientationError) + (kOrientationD * orientationDerivative) + (setPoint * kOrientationF);
    }
    //Function that help us go to the nearest path for orientation
    public static double angleWrap(double angle) {
        while (angle < -179){
            angle += 360;
        } while (angle > 179){
            angle -= 360;
        }
        return angle;
    }
}

