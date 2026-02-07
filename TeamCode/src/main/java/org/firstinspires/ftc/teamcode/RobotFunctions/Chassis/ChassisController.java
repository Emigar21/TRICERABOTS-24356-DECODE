package org.firstinspires.ftc.teamcode.RobotFunctions.Chassis;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ControlSystems.PID;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.RobotMode.AUTO.Blue_Auto_Close;
import org.firstinspires.ftc.teamcode.RobotMode.AUTO.OdometryTesting;
import org.firstinspires.ftc.teamcode.RobotMode.TelemetryMethods;
import org.firstinspires.ftc.teamcode.Variables.ConfigVariables;
import org.firstinspires.ftc.teamcode.Variables.Constants;


@Config
public class ChassisController {

    ///Initialize motors

    public static DcMotorEx topLeft;
    public static DcMotorEx topRight;
    public static DcMotorEx rearLeft;
    public static DcMotorEx rearRight;
    public static DcMotorEx deadWheelX;
    public static DcMotorEx deadWheelY;
    Indexer indexer;

    ///Initialize IMU

    public static IMU imu;
    PID pid;

    ///Initialize variables
    ElapsedTime timer = new ElapsedTime();

    /// initialize auto variables
    public static double errorX;
    public static double errorY;
    public static double turnError;
    public static double errorHyp;
    public static double initialPositionX;
    public static double initialPositionY;
    public static double currentDistanceX;
    public static double currentDistanceY;
    public static double currentAngle;
    public ChassisController(HardwareMap hardwareMap) {

        //Initialize motors
        topLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        topRight = hardwareMap.get(DcMotorEx.class, "topRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");

        deadWheelX = topRight;
        deadWheelY = topLeft;

        resetEncoders();
        configMotors();


        currentDistanceX = getDistanceInchesX();
        currentDistanceY = getDistanceInchesY();


        //Initialize IMU

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(imuParameters);
        imu.resetYaw();

        pid = new PID();
    }

    public void mecanumDrive(double X, double Y, double turn) {
        double power = Math.hypot(X, Y);
        double theta = Math.atan2(Y, X); //create the angle of the robot that we want to move
        double sin = Math.sin(theta - Math.PI / 4); //create the sin of the angle
        double cos = Math.cos(theta - Math.PI / 4); //create the cos of the angle
        double max = Math.max(Math.abs(sin), Math.abs(cos)); //create the max of the sin and the cos
        //double turn = pid.calculateAngleChassisPID(orientation, imu.getRobotYawPitchRollAngles().getYaw());

        topLeft.setPower(power * cos / max + turn); //set the power of the motors
        topRight.setPower(power * sin / max - turn); //set the power of the motors
        rearLeft.setPower(power * sin / max + turn); //set the power of the motors
        rearRight.setPower(power * cos / max - turn); //set the power of the motors
    }
    public void mecanumFollow(double setpointAngle) {
        currentAngle = imu.getRobotYawPitchRollAngles().getYaw();
        mecanumDrive(0,0,pid.calculateAngleChassisPID(setpointAngle, currentAngle));
    }

    public void slowMode(double X, double Y, double turn) {

        turn *= -1;

        double power = Math.hypot(X, Y);
        double theta = Math.atan2(Y, X); //create the angle of the robot that we want to move
        double sin = Math.sin(theta - Math.PI / 4); //create the sin of the angle
        double cos = Math.cos(theta - Math.PI / 4); //create the cos of the angle
        double max = Math.max(Math.abs(sin), Math.abs(cos)); //create the max of the sin and the cos

        topLeft.setPower((power * cos / max + turn) * .3); //set the power of the motors
        topRight.setPower((power * sin / max - turn) * .3); //set the power of the motors
        rearLeft.setPower((power * sin / max + turn) * .3); //set the power of the motors
        rearRight.setPower((power * cos / max - turn) * .3); //set the power of the motors

    }

    /// Autonomous section --------------------------------------
    ///
    //functions that give the current position in ticks
    public static double getTicksX(){return deadWheelX.getCurrentPosition();}
    public static double getTicksY() {return -deadWheelY.getCurrentPosition();}

    //functions that give the current revolutions
    public static double getRevsX() {return getTicksX() / Constants.chassisConst.TICKS_PER_REV;}
    public static double getRevsY(){return getTicksY() / Constants.chassisConst.TICKS_PER_REV;}

    //functions that give the current distance in inches
    public static double getDistanceInchesX(){return getRevsX() * Constants.chassisConst.WHEEL_CIRC_INCH + ConfigVariables.startPositionX;}
    public static double getDistanceInchesY(){return getRevsY() * Constants.chassisConst.WHEEL_CIRC_INCH + ConfigVariables.startPositionY;}

    public void AutoMovement(double setPointX, double setPointY, double desiredAngle) {
        resetEncoders();

        // Save the robot´s starting position

        initialPositionX = currentDistanceX;
        initialPositionY = currentDistanceY;

        // Calculate initial error

        errorX = setPointX - currentDistanceX;
        errorY = setPointY - currentDistanceY;

        timer.reset();

        while ((Math.abs(errorX) > .5 || Math.abs(errorY) > .5) && timer.seconds() < 1) {
            currentDistanceX = getDistanceInchesX();
            currentDistanceY = getDistanceInchesY();

            errorX = setPointX - currentDistanceX;
            errorY = setPointY - currentDistanceY;

            double power = Math.hypot(errorX, errorY);
            double theta = Math.atan2(errorY, errorX); //create the angle of the robot that we want to move
            double sin = Math.sin(theta - Math.PI  / 4); //create the sin of the angle
            double cos = Math.cos(theta - Math.PI / 4); //create the cos of the angle
            double max = Math.max(Math.abs(sin), Math.abs(cos)); //create the max of the sin and the cos
            double turn = pid.calculateAngleChassisPID(desiredAngle, imu.getRobotYawPitchRollAngles().getYaw());


            topLeft.setPower((power * cos / max + turn) * .065); //set the power of the motors
            topRight.setPower((power * sin / max - turn) * .065); //set the power of the motors
            rearLeft.setPower((power * sin / max + turn) * .065); //set the power of the motors
            rearRight.setPower((power * cos / max - turn) * .065); //set the power of the motors

//            Blue_Auto_Close.telemetryMethods.TelemetryUpdateCamera();
        }

        stopChassis();
    }

    public void AutoMovementSlow(double setPointX, double setPointY, double desiredAngle) {
        resetEncoders();

        // Save the robot´s starting position

        initialPositionX = currentDistanceX;
        initialPositionY = currentDistanceY;

        // Calculate initial error

        errorX = setPointX - currentDistanceX;
        errorY = setPointY - currentDistanceY;

        timer.reset();

        while ((Math.abs(errorX) > .5 || Math.abs(errorY) > .5) && timer.seconds() < 1) {
            currentDistanceX = getDistanceInchesX();
            currentDistanceY = getDistanceInchesY();

            errorX = setPointX - currentDistanceX;
            errorY = setPointY - currentDistanceY;

            double power = Math.hypot(errorX, errorY);
            double theta = Math.atan2(errorY, errorX); //create the angle of the robot that we want to move
            double sin = Math.sin(theta - Math.PI  / 4); //create the sin of the angle
            double cos = Math.cos(theta - Math.PI / 4); //create the cos of the angle
            double max = Math.max(Math.abs(sin), Math.abs(cos)); //create the max of the sin and the cos
            double turn = pid.calculateAngleChassisPID(desiredAngle, imu.getRobotYawPitchRollAngles().getYaw());


            topLeft.setPower((power * cos / max + turn) * .0095  ); //set the power of the motors
            topRight.setPower((power * sin / max - turn) * .0095); //set the power of the motors
            rearLeft.setPower((power * sin / max + turn) * .0095); //set the power of the motors
            rearRight.setPower((power * cos / max - turn) * .0095); //set the power of the motors

//            Blue_Auto_Close.telemetryMethods.TelemetryUpdateCamera();
        }

        stopChassis();
    }

//    public void AutoMovement(double setPointX, double setPointY, double desiredAngle) {
//        resetEncoders();
//
//        // Calculate initial error
//
//        errorX = setPointX - currentDistanceX;
//        errorY = setPointY - currentDistanceY;
//
//        timer.reset();
//
//        while ((Math.abs(errorX) > 1 || Math.abs(errorY) > 1) && timer.seconds() < 3) {
//            // Calculate setpoint Hyp for PID tuning
//            setpointHyp = Math.hypot(setPointY,setPointX);
//
//            currentDistanceX = getDistanceInchesX();
//            currentDistanceY = getDistanceInchesY();
//
//            currentHyp = Math.hypot(currentDistanceY,currentDistanceX);
//
//            errorX = setPointX - currentDistanceX;
//            errorY = setPointY - currentDistanceY;
//
////            double power = Math.hypot(errorX, errorY);
//            double power = pid.calculateChassisMovementPID(setpointHyp, currentHyp);
//            double theta = Math.atan2(errorY, errorX); //create the angle of the robot that we want to move
//            double sin = Math.sin(theta - Math.PI  / 4); //create the sin of the angle
//            double cos = Math.cos(theta - Math.PI / 4); //create the cos of the angle
//            double max = Math.max(Math.abs(sin), Math.abs(cos)); //create the max of the sin and the cos
//            double turn = pid.calculateAngleChassisPID(desiredAngle, imu.getRobotYawPitchRollAngles().getYaw());
//
//
//            topLeft.setPower((power * cos / max + turn)); //set the power of the motors
//            topRight.setPower((power * sin / max - turn)); //set the power of the motors
//            rearLeft.setPower((power * sin / max + turn)); //set the power of the motors
//            rearRight.setPower((power * cos / max - turn)); //set the power of the motors
//
//            OdometryTesting.telemetryMethods.TelemetryUpdateCamera();
//        }
//
//        stopChassis();
//    }

    public void AutoTurn(double desiredAngle) {
        resetEncoders();

        // Calculate initial error
        currentAngle = imu.getRobotYawPitchRollAngles().getYaw();
        turnError = desiredAngle - currentAngle;

        timer.reset();

        while (Math.abs(turnError) > .2 && timer.seconds() < .5) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw();
            turnError = desiredAngle - currentAngle;

            double turn = pid.calculateAngleChassisPID(desiredAngle, currentAngle);

            topLeft.setPower(turn); //set the power of the motors
            topRight.setPower((-turn)); //set the power of the motors
            rearLeft.setPower(turn); //set the power of the motors
            rearRight.setPower((-turn)); //set the power of the motors

//            OdometryTesting.telemetryMethods.TelemetryUpdateCamera();
        }

        stopChassis();
    }

    ///this one we have to check it!!
//    public void chassisFollow(double bearing){
//        mecanumDrive(0,0, pid.calculatePIDF(3.5,bearing, Constants.chassisConst.FOLLOW_P, 0, 0, 0));
//    }


    public void mecanumDriveByTime (double powerX, double powerY, double time){
        timer.reset();

        while(timer.seconds() < time){
            mecanumDrive(powerX, powerY, 0);
        }

        mecanumDrive(0,0,0); // Stop driving
    }

    public void mecanumTurnByTime (double spin, double time){
        timer.reset();

        while(timer.seconds() < time){
            mecanumDrive(0,0,spin);
        }

        mecanumDrive(0,0,0); // Stop driving
    }

    /// Configuration functions
    private void configMotors(){

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

        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void resetEncoders() {
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopChassis() {
        topLeft.setPower(0);
        topRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }
}
