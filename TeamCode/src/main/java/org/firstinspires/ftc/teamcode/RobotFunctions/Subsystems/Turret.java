package org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.detection;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kAngleChassisD;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kTurretD;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kTurretF;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kTurretI;
import static org.firstinspires.ftc.teamcode.Variables.ConfigVariables.kTurretP;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ControlSystems.PID;


public class Turret {
    public static DcMotorEx turretServo;
    PID pid = new PID();

    public Turret (HardwareMap hardwareMap){
        turretServo = hardwareMap.get(DcMotorEx.class,"turretServo");

        turretServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void moveTurret(double power, double angle, int actualId, int desiredId){
        if (!detection){

            if (desiredId == 24){
                turretServo.setPower(power);
            } else if (desiredId == 20) {
                turretServo.setPower(-power);
            }

        } else{

            if(actualId != desiredId){

                if (desiredId == 24){
                    turretServo.setPower(power);
                } else if (desiredId == 20){
                    turretServo.setPower(-power);
                }

            } else {

                turretServo.setPower(pid.calculatePIDF(0,angle, kTurretP,kTurretI,kTurretD,kTurretF));
            }
        }

    }

    public void moveServo(double power){
        turretServo.setPower(power);
    }

    public void stopTurret(){
        turretServo.setPower(0);
    }

}