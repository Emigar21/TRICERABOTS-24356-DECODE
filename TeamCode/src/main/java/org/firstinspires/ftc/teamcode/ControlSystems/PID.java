package org.firstinspires.ftc.teamcode.ControlSystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Variables.ConfigVariables;

public class PID {
    double prevTurretError = 0;
    public static double sumTurretError;
    public static double error;
    static double prevError;
    static double sumError;
    static double derivative;
    static double saturation;
    static ElapsedTime timer = new ElapsedTime();
    public static double calculatePIDF(double setPoint, double actualPosition, double kP, double kI,
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
        return (kP * error) + (kI * sumError) + (kD * derivative) + (kF * setPoint);
    }



    double prevOrientationError = 0;
    double sumOrientationError;
    double orientationSaturation;
    double orientationDerivative;
    ElapsedTime orientationTimer = new ElapsedTime();
   public static double errorAngle;
    public double calculateAngleChassisPID(double setPoint, double actualOrientation, double kOrientationP, double kOrientationI, double kOrientationD, double kOrientationF){
        //set the Proportional calculation
        errorAngle = setPoint - actualOrientation;

        //Make a condition so there is not saturation on the Integral calculation
        orientationSaturation = errorAngle - prevOrientationError;
        if(Math.abs(orientationSaturation) < -.1){
            //Calculate the integral sum
            sumOrientationError += errorAngle * orientationTimer.seconds();
        }

        //Calculate the derivative
        orientationDerivative = (errorAngle - prevOrientationError)/orientationTimer.seconds();

        //Get the last error
        prevOrientationError = errorAngle;

        //Reset timer
        orientationTimer.reset();

        //Get the motor output

        return  (kOrientationP * errorAngle) + (kOrientationI * sumOrientationError) + (kOrientationD * orientationDerivative) + (setPoint * kOrientationF);
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

