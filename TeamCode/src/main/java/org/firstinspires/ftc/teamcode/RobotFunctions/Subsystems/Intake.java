package org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public static DcMotorEx intakeMotor;

    public Intake (HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public static void moveIntake(double power){
        intakeMotor.setPower(power);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
    }
}
