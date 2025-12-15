package org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.artifactPos1;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.artifactsObelisk;
import static org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator.compensateVoltage;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kF;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kI;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kP;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.power;
import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.HDHEX_TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.maxDist;
import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.maxVel;
import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.minDist;
import static org.firstinspires.ftc.teamcode.Variables.Constants.shooterConst.minVel;

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
    public DcMotorEx shooterMotor;

    ElapsedTime timer = new ElapsedTime();

    public static int actualArtifact = 0;

    public static double actualVel;


    public Shooter (HardwareMap hardwareMap){
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public double shooterPower(double distance){
        return compensateVoltage(minVel + (distance - minDist) * ((maxVel - minVel) / (maxDist - minDist)));
        // formula: velmin + (actdist - min_distance) * ((maxvel - minvel) / (distmax - distmin))
        //175.76 cm a .7889
        //34.9 cm a .5409
    }

    public void shootArtifact(double distance){
        actualVel = ((shooterMotor.getVelocity()/HDHEX_TICKS_PER_REV) * 60)/6000;
        PID pid = new PID();
        shooterMotor.setPower(pid.calculatePIDF(shooterPower(distance),shooterMotor.getPower(),1,0,0, 0.986));
    }

    public void categorizeColor(String actualColor, double cameraDistance) {

        if (Objects.equals(actualColor, artifactsObelisk[actualArtifact])) {
            timer.reset();
            while(timer.seconds() < 5){
                shooterMotor.setPower(shooterPower(cameraDistance));
            }

            stopMotors();
            if (actualArtifact == 2){
                actualArtifact = 0;
            } else{
                actualArtifact++;
            }

        } else if (Sensors.getArtifactColor() != "null" ){
            timer.reset();
            while(timer.seconds() < 5) {
                shooterMotor.setPower(.3);
            }
            stopMotors();

        } else {
            stopMotors();
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
    public void stopMotors(){
        shooterMotor.setPower(0);
    }
}
