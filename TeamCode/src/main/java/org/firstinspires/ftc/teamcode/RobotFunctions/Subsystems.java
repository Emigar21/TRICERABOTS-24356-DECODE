package org.firstinspires.ftc.teamcode.RobotFunctions;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.detection;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Subsystems {
    public DcMotorEx intakeMotor;
    public Servo feederServo;
     public DcMotorEx indexerMotor;
     public DcMotorEx shooterMotor;
    //public CRServo servoElevation;

    public Subsystems(HardwareMap hardwareMap){

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //ShooterMotor= hardwareMap.get(DcMotor.class, "shooterMotor");
        indexerMotor= hardwareMap.get(DcMotorEx.class, "indexerMotor");

        feederServo = hardwareMap.get(Servo.class,"feederServo");
        shooterMotor = hardwareMap.get(DcMotorEx.class,"shooterMotor");


    }
    //Function for moving the intake inwards and outwards
    public void  moveIntake (double v){
        intakeMotor.setPower(v);
    }
    public void moveIndexer(double v){
        indexerMotor.setPower(v);

    }
    public void moveFeeder (double x) {
        feederServo.setPosition(x);
    }

    public void moveShooterLong (){
        shooterMotor.setPower(0.87);
    }

    public void moveShooterShort(){
        shooterMotor.setPower(0.68);
    }

}
