package org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.ControlSystems.PID;
import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;

public class SubsystemInitializer {
    //public Turret turret;
    public Intake intake;
    public Indexer indexer;
    public Feeder feeder;
    public Shooter shooter;
    public Sensors sensors;

    VoltageCompensator voltageCompensator;
    PID pid = new PID();
    public SubsystemInitializer (HardwareMap hardwareMap){
        //turret = new Turret(hardwareMap);
        intake= new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        feeder = new Feeder(hardwareMap);
        shooter = new Shooter(hardwareMap);

        sensors = new Sensors(hardwareMap);
        voltageCompensator = new VoltageCompensator(hardwareMap);

    }

    public void stopAllSubMotors(){
        //turret.stopTurret();
        shooter.stopShooter();
        intake.stopIntake();
        indexer.stopIndexer();
        feeder.stopFeeder();
    }

    public void stopCycling(){
        intake.stopIntake();
        indexer.stopIndexer();
        feeder.stopFeeder();
    }
}
