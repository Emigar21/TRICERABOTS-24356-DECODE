package org.firstinspires.ftc.teamcode.RobotFunctions;


import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.bearing;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.detection;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.Dashboard;
import org.firstinspires.ftc.teamcode.Variables.Constants;


@Config
public class ChassisController {

    double gain = 0;


    NormalizedColorSensor colorSensor;

    //initialize motors

    static DcMotorEx topLeft;
    static DcMotorEx topRight;
    static DcMotorEx rearLeft;
    static DcMotorEx rearRight;

    //initialize encoders

    static DcMotorEx deadWheelX;
    static DcMotorEx deadWheelY;

    //initialize IMU

    static IMU imu;

    static PID pid = new PID();

    //initialize variables
    static double startPositionX = 0;
    static double startPositionY = 0;
    static double powerErrorX;
    static double powerErrorY;
    static double errorAxisX;
    static double errorAxisY;
    static double currentPosX = 0;
    static double currentPosY = 0;

    Dashboard dashboard;
    Camera_Detection camera;


    public ChassisController(HardwareMap hardwareMap) {

        //Initialize motors
        topLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        topRight = hardwareMap.get(DcMotorEx.class, "topRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");

        //set motor directions
        topLeft.setDirection(DcMotorEx.Direction.FORWARD);
        topRight.setDirection(DcMotorEx.Direction.REVERSE);
        rearLeft.setDirection(DcMotorEx.Direction.FORWARD);
        rearRight.setDirection(DcMotorEx.Direction.REVERSE);

        //set zero power behavior

        topLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        topRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        deadWheelX = topLeft;
        deadWheelY = topRight;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");



        //Initialize IMU

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(imuParameters);
        imu.resetYaw();
    }

    //functions that give the current position in ticks
    public double getTicksX(){return -deadWheelX.getCurrentPosition();}
    public double getTicksY() {return topRight.getCurrentPosition();}

    //functions that give the current revolutions
    public double getRevsX() {return  getTicksX() / Constants.chassisConst.TICS_PER_REV;}
    public double getRevsY (){return getTicksY() / Constants.chassisConst.TICS_PER_REV;}

    //functions that give the current distance in inches
    public double getDistanceInchesX (){return getRevsX() * Constants.chassisConst.WHEEL_CIRC_INCH + startPositionX;}
    public double getDistanceInchesY (){return getRevsY() * Constants.chassisConst.WHEEL_CIRC_INCH + startPositionY;}

    //function that reset encoders
    public  void resetEncoders() {
        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //function that sets the power of the motors in 0
    public  void stopMotors() {
        topLeft.setPower(0);
        topRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    //function that move the robot to a specific position with coordinates of the field
    public static void mecanumDriveAuto(double X, double Y, double orientationSetpoint) {

        errorAxisX = X - currentPosX; // error in the X axis that is always positive
        errorAxisY = Y - currentPosY; // error in the Y axis that is always positive


        double distance =  Math.sqrt(Math.pow((X - currentPosX),2) - Math.pow((Y - currentPosY),2));

        double turn = pid.calculateAngleChassisPID(orientationSetpoint, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        while (Math.abs(errorAxisX) > 0.4 || Math.abs(errorAxisY) > 0.4) { // while the error is greater than 0.05 is gonig to keep moving
            //    if (Math.abs(imu.getRobotYawPitchRollAngles().getYaw()) > 10){
            double xDistance = distance * Math.cos(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            double yDistance = distance * Math.sin(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            errorAxisY = Y - yDistance;
            errorAxisX = X - xDistance;
            //    } else {
            //        errorAxisX = X - getDistanceInchesX();
            //       errorAxisY = Y - getDistanceInchesY();
            //    }
            double power = pid.calculateChassisPID(Math.hypot(X, Y),Math.hypot(powerErrorX, powerErrorY));

            turn = pid.calculateAngleChassisPID(orientationSetpoint, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            double theta = Math.atan2(powerErrorY, powerErrorX); //create the angle of the robot that we want to move
            double sin = Math.sin(theta - Math.PI / 4); //create the sin of the angle
            double cos = Math.cos(theta - Math.PI / 4); //create the cos of the angle
            double max = Math.max(Math.abs(sin), Math.abs(cos)); //create the max of the sin and the cos

            topLeft.setPower(power * (cos / max) + turn); //set the power of the motors
            topRight.setPower(power * (sin / max) - turn); //set the power of the motors
            rearLeft.setPower(power * (sin / max) + turn); //set the power of the motors
            rearRight.setPower(power * (cos / max) - turn); //set the power of the motors
        }
    }


    public static void mecanumDrive(double X, double Y, double orientation) {

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

    public static void chassisFollow(){
        if (detection) {
            mecanumDrive(0,0, -pid.calculateAngleChassisPID(0,bearing));

        }
    }
    public float getColor(){
        final float[] hsvValues = new float[3];
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
        float hsv1 = hsvValues[0]; //purple 210 - 240 green 120 - 180 - elegido
        float hsv2 = hsvValues[1]; //purple 0.3 - 0.6 green 0.7 - 0.8
        float hsv3 = hsvValues[2]; //descartado , el rango de valores es casi el mismo para ambos colores
        return hsv1 ;

    }

    public String getArtifactorColor (){
        if (getColor() >100 && getColor() < 200){
            return "green";
        } else if (getColor() > 200){
            return "purple";
        } else {
            return "null";
        }

    }
}
