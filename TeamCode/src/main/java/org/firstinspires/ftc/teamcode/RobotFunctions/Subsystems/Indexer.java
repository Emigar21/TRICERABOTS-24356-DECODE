package org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Indexer {
    public static DcMotorEx indexerMotor;
    public Indexer (HardwareMap hardwareMap){
        indexerMotor = hardwareMap.get(DcMotorEx.class,"indexerMotor");

        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void moveIndexer (double power){
        indexerMotor.setPower(power);
    }

    public void stopIndexer(){
        indexerMotor.setPower(0);
    }
}
