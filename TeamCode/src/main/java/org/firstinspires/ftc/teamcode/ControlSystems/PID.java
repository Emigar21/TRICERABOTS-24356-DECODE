package org.firstinspires.ftc.teamcode.ControlSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Variables.ConfigVariables;
import org.firstinspires.ftc.teamcode.Variables.Constants;

public class PID {
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
    ElapsedTime orientationTimer = new ElapsedTime();
    public double calculateAngleChassisPID(double setPoint, double actualOrientation){
        //set the Proportional calculation
       return calculatePIDF(setPoint,actualOrientation, Constants.chassisConst.TURN_P, Constants.chassisConst.TURN_I, Constants.chassisConst.TURN_D, Constants.chassisConst.TURN_F);
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

