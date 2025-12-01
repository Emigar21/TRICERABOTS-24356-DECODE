package org.firstinspires.ftc.teamcode.RobotFunctions;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.detection;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Subsystems {
    DcMotor intakeMotor;
    Servo feederServo;
     DcMotor indexerMotor;
     //DcMotor ShooterMotor;
    //public CRServo servoElevation;

    public Subsystems(HardwareMap hardwareMap){

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        //ShooterMotor= hardwareMap.get(DcMotor.class, "shooterMotor");
        indexerMotor= hardwareMap.get(DcMotor.class, "indexerMotor");

        feederServo = hardwareMap.get(Servo.class,"feederServo");


    }
    //Function for moving the intake inwards and outwards
    public void  moveIntake (double v){
        intakeMotor.setPower(v);
    }
    public void moveIndexer(double v){
        indexerMotor.setPower(v);

    }

    //Function which the servo will follow the april tag
    public void FollowServoAprilTag(double bearing) {
        if (detection) {
            bearing = bearing *.5;
            //servoBearing.setPower(-pid.calculatePID(0, bearing));
        } else {
            //servoBearing.setPower(0);
        }
    }

    public void moveFeeder (double x) {
        feederServo.setPosition(x);
    }

}
