package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Variables.ConfigVariables;

public class PID {
    double prevTurretError = 0;
    double sumTurretError;
    double turretSaturation;
    double turretDerivative;
    final double kTurretP = ConfigVariables.kTurretP;
    final double kTurretI = ConfigVariables.kTurretI;
    final double kTurretD = ConfigVariables.kTurretD;
    final double kTurretF = ConfigVariables.kTurretF;
    ElapsedTime turretTimer = new ElapsedTime();
    public double calculatePID(double setPoint, double actualPosition){
        //set the Proportional calculation
        double error = setPoint - actualPosition;

        //Make a condition so there is not saturation on the Integral calculation
        turretSaturation = error - prevTurretError;
        if(Math.abs(turretSaturation) > 0.00001){
            //Calculate the integral sum
            sumTurretError += error * turretTimer.seconds();
        }

        //Calculate the derivative
        turretDerivative = (error - prevTurretError)/turretTimer.seconds();

        //Get the last error
        prevTurretError = error;

        //Reset timer
        turretTimer.reset();

        //Get the motor output
        return  (kTurretP * error) + (kTurretI * sumTurretError) + (kTurretD * turretDerivative) + (setPoint * kTurretF);
    }



    double prevOrientationError = 0;
    double sumOrientationError;
    double orientationSaturation;
    double orientationDerivative;
    final double kOrientationP = ConfigVariables.kOrientationP;
    final double kOrientationI = ConfigVariables.kOrientationI;
    final double kOrientationD = ConfigVariables.kOrientationD;
    final double kOrientationF = ConfigVariables.kOrientationF;
    ElapsedTime orientationTimer = new ElapsedTime();
    public double calculateAngleChassisPID(double setPoint, double actualOrientation){
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



    double prevChassisError = 0;
    double sumChassisError;
    double chassisSaturation;
    double chassisDerivative;
    final double kChassisP = ConfigVariables.kChassisP;
    final double kChassisI = ConfigVariables.kChassisI;
    final double kChassisD = ConfigVariables.kChassisD;
    final double kChassisF = ConfigVariables.kChassisF;
    ElapsedTime chassisTimer = new ElapsedTime();
    public double calculateChassisPID(double setPoint, double actualPoint){
        double error = setPoint - actualPoint;

        chassisSaturation = error - prevChassisError;
        if (Math.abs(chassisSaturation) > .0001){
            sumChassisError += error * chassisTimer.seconds();
        }

        chassisDerivative = (error - prevChassisError) * chassisTimer.seconds();

        prevChassisError = error;

        chassisTimer.reset();

        return (kChassisP * error) + (kChassisI * sumChassisError) + (kChassisD * chassisDerivative) + (kChassisF * setPoint);
    }
}
