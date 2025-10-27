package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Camera_Data.detection;
import static org.firstinspires.ftc.teamcode.Camera_Data.id;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Subsystems {
    public static DcMotor intakeMotor;

    public static CRServo servoBearing;
    public CRServo servoBearing2;
    public CRServo servoElevation;
    PID pid = new PID();

    public Subsystems (HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoBearing = hardwareMap.get(CRServo.class,"servoBearing");
//        servoBearing2 = hardwareMap.get(CRServo.class,"servoBearing2");
//        servoElevation = hardwareMap.get(CRServo.class,"servoElevation");

    }
    //Function for moving the intake inwards and outwards
    public void  moveIntake (double v){
        intakeMotor.setPower(v);
    }
    //Function which the servo will follow the april tag
    public void FollowServoAprilTag(double bearing) {
        if (detection) {
            bearing = bearing *.5;
            intakeMotor.setPower(-pid.calculatePID(0, bearing));
        } else {
            intakeMotor.setPower(0);
        }
    }
}
