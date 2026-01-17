package org.firstinspires.ftc.teamcode.RobotFunctions.Chassis;


import static org.firstinspires.ftc.teamcode.ControlSystems.PID.errorAngle;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kAngleChassisD;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kAngleChassisF;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kAngleChassisI;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kAngleChassisP;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kChassisD;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kChassisF;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kChassisI;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kChassisP;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kFollowD;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kFollowF;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kFollowI;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kFollowP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.ControlSystems.PID;
import org.firstinspires.ftc.teamcode.RobotMode.Dashboard;
import org.firstinspires.ftc.teamcode.Variables.Constants;


@Config
public class ChassisController {

    //initialize motors

    public static DcMotorEx topLeft;
    public static DcMotorEx topRight;
    public static DcMotorEx rearLeft;
    public static DcMotorEx rearRight;

    //initialize encoders

    public static DcMotorEx deadWheelX;
    public static DcMotorEx deadWheelY;

    //initialize IMU

    public static IMU imu;



    static PID pid = new PID();

    //initialize variables
    static double startPositionX = 0;
    static double startPositionY = 0;
    static double powerErrorX;
    static double powerErrorY;
    static double errorAxisX;
    static double errorAxisY;
    static double currentPositionX;
    static double currentPositionY;

    static double currentCircleX;
    static double currentCircleY;
    double yDistanceSum;
    double xDistancesSum;



    public ChassisController(HardwareMap hardwareMap) {

        //Initialize motors
        topLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        topRight = hardwareMap.get(DcMotorEx.class, "topRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");

        //set motor directions
        topLeft.setDirection(DcMotorEx.Direction.REVERSE);
        topRight.setDirection(DcMotorEx.Direction.FORWARD);
        rearLeft.setDirection(DcMotorEx.Direction.REVERSE);
        rearRight.setDirection(DcMotorEx.Direction.FORWARD);

        //set zero power behavior

        topLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        deadWheelY = topLeft;
        deadWheelX = topRight;


        //Initialize IMU

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(imuParameters);
        imu.resetYaw();
    }

    //functions that give the current position in ticks
    public static double getTicksX(){return -deadWheelX.getCurrentPosition();}
    public static double getTicksY() {return deadWheelY.getCurrentPosition() + 30 ;}

    //functions that give the current revolutions
    public static double getRevsX() {return  getTicksX() / Constants.chassisConst.TICS_PER_REV;}
    public static double getRevsY(){return getTicksY() / Constants.chassisConst.TICS_PER_REV;}

    //functions that give the current distance in inches
    public static double getDistanceInchesX(){return getRevsX() * Constants.chassisConst.WHEEL_CIRC_INCH + startPositionX;}
    public static double getDistanceInchesY(){return getRevsY() * Constants.chassisConst.WHEEL_CIRC_INCH + startPositionY;}

    //function that reset encoders
    public  void resetEncoders() {
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //function that sets the power of the motors in 0
    public void stopMotors() {
        topLeft.setPower(0);
        topRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }
    public void chassisFollow(double bearing){
        mecanumDrive(0,0, pid.calculatePIDF(0,bearing, kFollowP, kFollowI, kFollowD, kFollowF));
    }


    public void mecanumDrive(double X, double Y, double orientation) {

        double power = Math.hypot(X, Y);
        double theta = Math.atan2(Y, X); //create the angle of the robot that we want to move
        double sin = Math.sin(theta - Math.PI / 4); //create the sin of the angle
        double cos = Math.cos(theta - Math.PI / 4); //create the cos of the angle
        double max = Math.max(Math.abs(sin), Math.abs(cos)); //create the max of the sin and the cos
        //double turn = pid.calculateAngleChassisPID(orientation, imu.getRobotYawPitchRollAngles().getYaw());

        topLeft.setPower(power * (cos / max) + orientation); //set the power of the motors
        topRight.setPower(power * (sin / max) - orientation); //set the power of the motors
        rearLeft.setPower(power * (sin / max) + orientation); //set the power of the motors
        rearRight.setPower(power * (cos / max) - orientation); //set the power of the motors
    }


//    public void mecanumDriveAuto(double X, double Y, double orientationSetpoint) {
//
//        errorAxisX = X - getDistanceInchesX(); // error in the X axis that is always positive
//        errorAxisY = Y - getDistanceInchesY(); // error in the Y axis that is always positive
//
//        double turn;
//
//        while (Math.abs(errorAxisX) > 0.4 || Math.abs(errorAxisY) > 0.4) { // while the error is greater than 0.05 is gonig to keep moving
//
//            //Calculate the distances your robot has moved with x placed odometry
//            double xDistanceVerticalOdometry = getDistanceInchesY()*Math.cos(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            double yDistanceVerticalOdometry = getDistanceInchesY()*Math.sin(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//
//            //Calculate the distances your robot has moved with y placed odometry
//            double xDistanceHorizontalOdometry = getDistanceInchesX()*Math.cos(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            double yDistanceHorizontalOdometry = getDistanceInchesX()*Math.sin(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//
//            //Get the x distance and y distance
//            xDistancesSum = xDistanceHorizontalOdometry + xDistanceVerticalOdometry;
//            yDistanceSum = yDistanceHorizontalOdometry + yDistanceVerticalOdometry;
//
//
//            currentCircleX = currentPositionX + xDistancesSum;
//            currentCircleY = currentPositionY + yDistanceSum;
//
//            //Gets the error of your distances
//            errorAxisY = Y - yDistanceSum;
//            errorAxisX = X - xDistancesSum;
//
//            double power = pid.calculatePIDF(Math.hypot(X, Y),Math.hypot(powerErrorX, powerErrorY), kChassisP, kChassisI,kChassisD,kChassisF);
//
//            turn = pid.calculateAngleChassisPID(orientationSetpoint, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), kAngleChassisP, kAngleChassisI,kAngleChassisD,kAngleChassisF);
//
//            double theta = Math.atan2(powerErrorY, powerErrorX); //create the angle of the robot that we want to move
//            double sin = Math.sin(theta - Math.PI / 4); //create the sin of the angle
//            double cos = Math.cos(theta - Math.PI / 4); //create the cos of the angle
//            double max = Math.max(Math.abs(sin), Math.abs(cos)); //create the max of the sin and the cos
//
//            topLeft.setPower(power * (cos / max) ); //set the power of the motors
//            topRight.setPower(power * (sin / max) ); //set the power of the motors
//            rearLeft.setPower(power * (sin / max) ); //set the power of the motors
//            rearRight.setPower(power * (cos / max)  ); //set the power of the motors
//        }
//        //Accumulates the positions of the robot in field
//        currentPositionX += xDistancesSum;
//        currentPositionY += yDistanceSum;
//
//        stopMotors();
//        resetEncoders();
//    }


    public void mecanumDriveAuto(double X, double Y, double multiplier){

        //errorAxisX = Math.abs(X - getDistanceInchesX()); // error in the X axis that is always positive
        resetEncoders();
        errorAxisY = Math.abs(Y - getDistanceInchesY()); // error in the Y axis that is always positive

        double  turn =  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (errorAxisY > 2) { // while the error is greater than 0.05 is gonig to keep moving
            powerErrorX = X - getDistanceInchesX(); //create the error of the power in the X axis
            powerErrorY = Y - getDistanceInchesY(); //create the error of the power in the Y axis

            double power =  multiplier* pid.calculatePIDF(Y,powerErrorY, kChassisP, kChassisI,kChassisD,kChassisF);

            double theta = Math.atan2(powerErrorY, powerErrorX); //create the angle of the robot that we want to move
            double sin = Math.sin(theta - Math.PI / 4); //create the sin of the angle
            double cos = Math.cos(theta - Math.PI / 4); //create the cos of the angle
            double max = Math.max(Math.abs(sin), Math.abs(cos)); //create the max of the sin and the cos

            topLeft.setPower(power * (cos / max) - 0); //set the power of the motors
            topRight.setPower(power * (sin / max) + 0.06); //set the power of the motors
            rearLeft.setPower(power * (sin / max) - 0); //set the power of the motors
            rearRight.setPower(power * (cos / max) + 0.06); //set the power of the motors

            //errorAxisX = Math.abs(X - getDistanceInchesX()); // set the new error of the X axis
            errorAxisY = Math.abs(Y - getDistanceInchesY()); // set the new error of the Y axis
            //is going to be moving while the error is greater than 0.05, if the error is less than 0.05, the robot is going to stop
            //the robot is constantly adjusting the power of the motors to move to the desired position in case the robot is desviated from the desired position

//            if (-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < 1 || -1 * imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < 1){
//                turn = 0.01 * (0 - ( -1 * imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
//            }
            //here the robot is going to correct tne angle of the robot to the desired angle that is 0 degrees

//        if (errorAxisX < 0.3 || errorAxisY < 0.3){ // here when the robot is close to the desired position, the robot is going to reduce the power
//            power = 0.2;
//        }

        }
        stopMotors();
    }

    public void rotateChassis(double angle){
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double pene = pid.calculateAngleChassisPID(angle,currentAngle,kAngleChassisP,kAngleChassisI,kAngleChassisD,kAngleChassisF);

        while( Math.abs(errorAngle) > 1) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


            topLeft.setPower(-pid.calculateAngleChassisPID(angle,currentAngle,kAngleChassisP,kAngleChassisI,kAngleChassisD,kAngleChassisF)); //set the power of the motors
            topRight.setPower(pid.calculateAngleChassisPID(angle,currentAngle,kAngleChassisP,kAngleChassisI,kAngleChassisD,kAngleChassisF)); //set the power of the motors
            rearLeft.setPower(-pid.calculateAngleChassisPID(angle,currentAngle,kAngleChassisP,kAngleChassisI,kAngleChassisD,kAngleChassisF)); //set the power of the motors
            rearRight.setPower(pid.calculateAngleChassisPID(angle,currentAngle,kAngleChassisP,kAngleChassisI,kAngleChassisD,kAngleChassisF));
        }
        stopMotors();

    }
}
