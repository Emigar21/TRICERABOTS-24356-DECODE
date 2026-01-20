package org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.artifactPos1;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.artifactsObelisk;
import static org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator.compensateVoltage;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kD;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kF;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kI;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kP;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.power;
import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.HDHEX_TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.maxDist;
//import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.maxVel;
import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.maxVel;
import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.minDist;
import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.minVel;
//import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.minVel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.ControlSystems.PID;
import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;

import java.security.KeyFactory;
import java.util.Objects;

public class Shooter {
    public static DcMotorEx shooterMotor;
    public static ElapsedTime timer = new ElapsedTime();
    public static int actualArtifact = 0;


    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void shoot(double range){
        shooterMotor.setPower(compensateVoltage(getActualVel() < getDesiredRevs(range) ? 2 : getShooterPower(range)));
    }


    ///las dos funciones que siguen se necesitan checar, cambie la resolucaión a ver si es eso lo
    /// que hace que tenga un delay
    public static double getShooterPower(double distance) {
        return compensateVoltage(minVel + (distance - minDist) * ((maxVel - minVel) / (maxDist - minDist)));
        // formula: velmin + (actdist - min_distance) * ((maxvel - minvel) / (distmax - distmin))
    }

    public static double getDesiredRevs(double range){
        return (4090 + (getShooterPower(range) - .6)*((5350 - 4090)/(.742-.6)));
    }

    public static double getActualVel(){
        return (shooterMotor.getVelocity()/HDHEX_TICKS_PER_REV) * 60;
    }

    public void categorizeColor(String actualColor, double cameraDistance) {

        if (Objects.equals(actualColor, artifactsObelisk[actualArtifact])) {
            timer.reset();
            while(timer.seconds() < 5){
                shooterMotor.setPower(0.8);
            }

            stopShooter();
            if (actualArtifact == 2){
                actualArtifact = 0;
            } else{
                actualArtifact++;
            }

        } else if (Sensors.getArtifactColor() != "null" ){
            timer.reset();
            while(timer.seconds() < 5) {
                shooterMotor.setPower(.4);
            }
            stopShooter();

        } else {
            stopShooter();
        }
    }





    public void shootAllArtifacts(double cameraDistance){
        double setpointPower = getShooterPower(cameraDistance);
        for (int i=0; i < 2; i++){
            while (setpointPower < shooterMotor.getPower()) {
                shooterMotor.setPower(getShooterPower(cameraDistance));
                setpointPower = getShooterPower(cameraDistance);
            }
            shooterMotor.setPower(getShooterPower(cameraDistance));
        }
    }
    public void stopShooter(){
        shooterMotor.setPower(0);
    }
}
