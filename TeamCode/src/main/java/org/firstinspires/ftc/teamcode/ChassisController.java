package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class ChassisController {

    //initialize motors

    static DcMotor topLeft;
    static DcMotor topRight;
    static DcMotor rearLeft;
    static DcMotor rearRight;

    //initialize servo

    static Servo yawServo;
    static Servo pitchServo;

    //initialize encoders

    static DcMotor deadWheelX;
    static DcMotor deadWheelY;

    //initialize IMU

    static IMU imu;

    //initialize variables
    static double startPositionX = 0;
    static double startPositionY = 0;
    static double powerErrorX;
    static double powerErrorY;
    static double errorAxisX;
    static double errorAxisY;


    public ChassisController(HardwareMap hardwareMap) {

        //inicialize motors
        topLeft = hardwareMap.get(DcMotor.class, "topLeft");
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        //set motor directions
        topLeft.setDirection(DcMotor.Direction.REVERSE);
        topRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        //set zero power behavior

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //give deadwheel their value

        deadWheelX = topLeft;
        deadWheelY = topRight;

        deadWheelX.setDirection(DcMotorSimple.Direction.REVERSE);

        //servos

        yawServo = hardwareMap.get(Servo.class, "yawServo");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");

        //inicialize IMU

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(imuParameters);
        imu.resetYaw();
    }

    //functions that give the current position in ticks
    public static double getTicksX() {
        return -deadWheelX.getCurrentPosition();
    }

    public static double getTicksY() {
        return deadWheelY.getCurrentPosition();
    }

    //functions that give the current revolutions
    public static double getRevsX() {
        return getTicksX() / Constants.chassisConst.TICS_PER_REV;
    }

    public static double getRevsY() {
        return getTicksY() / Constants.chassisConst.TICS_PER_REV;
    }

    //functions that give the current distance in inches
    public static double getDistanceInchesX() {
        return getRevsX() * Constants.chassisConst.WHEEL_CIRC_INCH + startPositionX;
    }

    public static double getDistanceInchesY() {
        return getRevsY() * Constants.chassisConst.WHEEL_CIRC_INCH + startPositionY;
    }

    //function that reset encoders
    public static void resetEncoders() {
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //function that sets the power of the motors in 0
    public static void stopMotors() {
        topLeft.setPower(0);
        topRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    public static void Shooter (double v){
        topLeft.setPower(v);
        rearLeft.setPower(v);
    }

    //function that move the robot to a specific position with coordinates of the field
    public static void MecanumDriveAuto(double X, double Y, double power) {

        errorAxisX = Math.abs(X - getDistanceInchesX()); // error in the X axis that is always positive
        errorAxisY = Math.abs(Y - getDistanceInchesY()); // error in the Y axis that is always positive

        double turn = 0.01 * (0 - (-1 * imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));

        while (errorAxisX > 0.4 || errorAxisY > 0.4) { // while the error is greater than 0.05 is gonig to keep moving

            powerErrorX = X - getDistanceInchesX(); //create the error of the power in the X axis
            powerErrorY = Y - getDistanceInchesY(); //create the error of the power in the Y axis

            double theta = Math.atan2(powerErrorY, powerErrorX); //create the angle of the robot that we want to move
            double sin = Math.sin(theta - Math.PI / 4); //create the sin of the angle
            double cos = Math.cos(theta - Math.PI / 4); //create the cos of the angle
            double max = Math.max(Math.abs(sin), Math.abs(cos)); //create the max of the sin and the cos

            topLeft.setPower(power * (cos / max) + turn); //set the power of the motors
            topRight.setPower(power * (sin / max) - turn); //set the power of the motors
            rearLeft.setPower(power * (sin / max) + turn); //set the power of the motors
            rearRight.setPower(power * (cos / max) - turn); //set the power of the motors

            errorAxisX = Math.abs(X - getDistanceInchesX()); // set the new error of the X axis
            errorAxisY = Math.abs(Y - getDistanceInchesY()); // set the new error of the Y axis
            //is going to be moving while the error is greater than 0.05, if the error is less than 0.05, the robot is going to stop
            //the robot is constantly adjusting the power of the motors to move to the desired position in case the robot is desviated from the desired position

            if (-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < 1 || -1 * imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < 1) {
                turn = 0.01 * (0 - (-1 * imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            }
            //here the robot is going to correct tne angle of the robot to the desired angle that is 0 degrees

//        if (errorAxisX < 0.3 || errorAxisY < 0.3){ // here when the robot is close to the desired position, the robot is going to reduce the power
//            power = 0.2;
//        }

        }
        stopMotors();
    }


    public static void MecanumDrive(double X, double Y, double turn) {

        double power = Math.hypot(X, Y);
        double theta = Math.atan2(Y, X); //create the angle of the robot that we want to move
        double sin = Math.sin(theta - Math.PI / 4); //create the sin of the angle
        double cos = Math.cos(theta - Math.PI / 4); //create the cos of the angle
        double max = Math.max(Math.abs(sin), Math.abs(cos)); //create the max of the sin and the cos

        topLeft.setPower(power * (cos / max) + turn); //set the power of the motors
        topRight.setPower(power * (sin / max) - turn); //set the power of the motors
        rearLeft.setPower(power * (sin / max) + turn); //set the power of the motors
        rearRight.setPower(power * (cos / max) - turn); //set the power of the motors
    }


    public void  moveIntake (double v){
        topLeft.setPower(v);
    }
}
