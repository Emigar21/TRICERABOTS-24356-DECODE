package org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;

import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.HDHEX_TICKS_PER_REV;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Variables.ConfigVariables;


public class Shooter {
    public static DcMotorEx leftShooter;
    public static DcMotorEx rightShooter;
    public static ElapsedTime timer = new ElapsedTime();
    InterpLUT controlPoints;

    public Shooter(HardwareMap hardwareMap) {
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftShooter = hardwareMap.get(DcMotorEx.class,"leftShooter");

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        controlPoints = new InterpLUT();

        createControlPoints();
    }
    public void shoot(double range){
        rightShooter.setVelocity(controlPoints.get(range) * HDHEX_TICKS_PER_REV / 60);
        leftShooter.setVelocity(controlPoints.get(range) * HDHEX_TICKS_PER_REV / 60);
    }

    public static double getActualVel(){
        return (rightShooter.getVelocity()/HDHEX_TICKS_PER_REV) * 60;
    }

    public static void configShooter() {
        rightShooter.setVelocity(ConfigVariables.configSpeed);
        leftShooter.setVelocity(ConfigVariables.configSpeed);
    }
    public void stopShooter(){
        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }

    public void createControlPoints() {
        controlPoints.add(75,3060);
        controlPoints.add(211,4200);
    }
}
