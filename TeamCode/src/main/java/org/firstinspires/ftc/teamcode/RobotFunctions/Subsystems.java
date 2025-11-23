package org.firstinspires.ftc.teamcode.RobotFunctions;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.detection;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Subsystems {
    //public static DcMotor intakeMotor;

    public static Servo servo;
    //public CRServo servoBearing2;
    //public CRServo servoElevation;
    PID pid = new PID();

    public Subsystems(HardwareMap hardwareMap){
        //intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        //intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo = hardwareMap.get(Servo.class,"servo");
//        servoBearing2 = hardwareMap.get(CRServo.class,"servoBearing2");
//        servoElevation = hardwareMap.get(CRServo.class,"servoElevation");

    }
    //Function for moving the intake inwards and outwards
    public void  moveIntake (double v, double rv){
        //intakeMotor.setPower(v);
        //intakeMotor.setPower(rv);
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
        servo.setPosition(x);
    }

}
