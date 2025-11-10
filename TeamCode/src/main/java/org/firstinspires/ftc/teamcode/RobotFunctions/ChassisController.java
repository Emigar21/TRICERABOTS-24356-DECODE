package org.firstinspires.ftc.teamcode.RobotFunctions;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Data.bearing;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Data.detection;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Dashboard;
import org.firstinspires.ftc.teamcode.Variables.Constants;


public class ChassisController {

    //initialize motors

     DcMotorEx topLeft;
     DcMotorEx topRight;
     DcMotorEx rearLeft;
     DcMotorEx rearRight;

    //initialize encoders

     DcMotorEx deadWheelX;
     DcMotorEx deadWheelY;

    //initialize IMU

     IMU imu;

    PID pid = new PID();

    //initialize variables
    double startPositionX = 0;
    double startPositionY = 0;
    double powerErrorX;
    double powerErrorY;
    double errorAxisX;
    double errorAxisY;

    Dashboard dashboard;


    public ChassisController(HardwareMap hardwareMap) {

        //Initialize motors
        topLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        topRight = hardwareMap.get(DcMotorEx.class, "topRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");

        //set motor directions
        topLeft.setDirection(DcMotorEx.Direction.FORWARD);
        topRight.setDirection(DcMotorEx.Direction.FORWARD);
        rearLeft.setDirection(DcMotorEx.Direction.FORWARD);
        rearRight.setDirection(DcMotorEx.Direction.REVERSE);

        //set zero power behavior

        topLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        topRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        deadWheelX = topLeft;
        deadWheelY = topRight;


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
    public void stopMotors() {
        topLeft.setPower(0);
        topRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    //function that move the robot to a specific position with coordinates of the field
    public void mecanumDriveAuto(double X, double Y, double orientationSetpoint) {

        errorAxisX = X - getDistanceInchesX(); // error in the X axis that is always positive
        errorAxisY = Y - getDistanceInchesY(); // error in the Y axis that is always positive

        double turn = pid.calculateAngleChassisPID(orientationSetpoint, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        while (Math.abs(errorAxisX) > 0.4 || Math.abs(errorAxisY) > 0.4) { // while the error is greater than 0.05 is gonig to keep moving
            if ( Math.abs(imu.getRobotYawPitchRollAngles().getYaw()) > 5){
                resetEncoders();
                double xDistance = getDistanceInchesY()*Math.cos(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                double yDistance = getDistanceInchesY()*Math.sin(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                errorAxisY = Y - yDistance;
                errorAxisX = X - xDistance;
            } else {
                errorAxisX = X - getDistanceInchesX();
                errorAxisY = Y - getDistanceInchesY();
            }
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
        stopMotors();
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

    public void motorMove(double inch){
        while (getDistanceInchesX() < inch) {
            topLeft.setPower(1);
        }
        topLeft.setPower(0);
    }

    public void chassisFollow(){
        if (detection) {
            mecanumDrive(0,0, -pid.calculateAngleChassisPID(0,bearing));

        } else {
            stopMotors();
        }
    }
}
