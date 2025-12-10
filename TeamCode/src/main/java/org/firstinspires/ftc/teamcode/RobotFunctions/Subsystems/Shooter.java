package org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;

import static org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator.compensateVoltage;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;

public class Shooter {
    public DcMotorEx shooterMotor;


    public Shooter (HardwareMap hardwareMap){
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public void shooterShoot(double distance){
        double power = 0.55 + (distance - 63) * ((.807 - .55)/(300 - 63));
        // formula: velmin + (actdist - mindistance) * ((maxvel - minvel) / (distmax - distmin))
        //300 cm a .807
        //63cm a .55
        shooterMotor.setPower(compensateVoltage(power));
    }

}
