package org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;

import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.HDHEX_TICKS_PER_REV;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Variables.ConfigVariables;
import org.firstinspires.ftc.teamcode.Variables.Constants;


public class Shooter {
    public static DcMotorEx leftShooter;
    public static DcMotorEx rightShooter;
    public static ElapsedTime timer = new ElapsedTime();
    public static InterpLUT controlPoints;

    public Shooter(HardwareMap hardwareMap) {
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftShooter = hardwareMap.get(DcMotorEx.class,"leftShooter");

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.shooterConst.SHOOTER_PIDF);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.shooterConst.SHOOTER_PIDF);


        controlPoints = new InterpLUT();

        createControlPoints();
    }
    public void shoot(double range){
        rightShooter.setVelocity(controlPoints.get(range));
        leftShooter.setVelocity(controlPoints.get(range));
    }

    public  void configShooter() {
        rightShooter.setVelocity(ConfigVariables.configSpeed);
        leftShooter.setVelocity(ConfigVariables.configSpeed);
    }
    public static double getActualVel(){
        return (rightShooter.getVelocity()/HDHEX_TICKS_PER_REV) * 60;
    }
    public void stopShooter(){
        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }

    public void createControlPoints() {
        controlPoints.add(66,3400);
        controlPoints.add(116,3700);
        controlPoints.add(161,4000);
        controlPoints.add(197,4480);
        controlPoints.add(215,5150);
        controlPoints.add(267,5200); // Recalibrate
        controlPoints.add(320,5650);

        controlPoints.createLUT();
    }
}
