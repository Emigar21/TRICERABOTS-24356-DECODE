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
import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.minDist;
//import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.minVel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ControlSystems.PID;
import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;

import java.security.KeyFactory;
import java.util.Objects;

public class Shooter {
    public static DcMotorEx shooterMotor;

    ElapsedTime timer = new ElapsedTime();

    public static int actualArtifact = 0;


    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public static double shooterPower(double distance) {
        //return compensateVoltage(minVel + (distance - minDist) * ((maxVel - minVel) / (maxDist - minDist)));
        // formula: velmin + (actdist - min_distance) * ((maxvel - minvel) / (distmax - distmin))
        //175.76 cm a .7889
        //34.9 cm a .5409
        return 0;
    }

    public static double getDesiredRevs(){
        return (3050 + (power - .67)*((5050 - 3200)/(.915-.67)));
    }
    //TODO: QUE ES?


    public void shootArtifact(double distance) {
        //actualVel = ((shooterMotor.getVelocity()/HDHEX_TICKS_PER_REV) * 60)/6000;
        shooterMotor.setPower(compensateVoltage(power));
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
        double setpointPower = shooterPower(cameraDistance);
        for (int i=0; i < 2; i++){
            while (setpointPower < shooterMotor.getPower()) {
                shooterMotor.setPower(shooterPower(cameraDistance));
                setpointPower = shooterPower(cameraDistance);
            }
            shooterMotor.setPower(shooterPower(cameraDistance));
        }
    }
    public void stopShooter(){
        shooterMotor.setPower(0);
    }
}
