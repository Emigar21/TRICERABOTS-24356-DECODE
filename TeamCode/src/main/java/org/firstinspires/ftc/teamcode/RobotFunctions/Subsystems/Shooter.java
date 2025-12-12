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
        double power = 0.61 + (distance - 37) * ((.824 - .61)/(150 - 37));
        // formula: velmin + (actdist - min_distance) * ((maxvel - minvel) / (distmax - distmin))
        //150 cm a .824
        //37cm a .61
        shooterMotor.setPower(compensateVoltage(power));
    }

}
