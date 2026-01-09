package org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {

    public static DcMotorEx feederMotor;
    public Feeder(HardwareMap hardwareMap){
        feederMotor = hardwareMap.get(DcMotorEx.class, "feederMotor");

        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void moveFeeder(double power){
        feederMotor.setPower(power);
    }

    public void stopFeeder(){
        feederMotor.setPower(0);
    }
}
