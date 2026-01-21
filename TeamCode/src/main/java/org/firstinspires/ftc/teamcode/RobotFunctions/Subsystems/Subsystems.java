package org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ControlSystems.PID;
import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;

public class Subsystems {
    public Intake intake;
    public Indexer indexer;
    public Feeder feeder;
    public Shooter shooter;
    public Sensors sensors;

    VoltageCompensator voltageCompensator;
    PID pid = new PID();
    public Subsystems(HardwareMap hardwareMap){
        intake= new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        feeder = new Feeder(hardwareMap);
        shooter = new Shooter(hardwareMap);

        sensors = new Sensors(hardwareMap);
        voltageCompensator = new VoltageCompensator(hardwareMap);

    }

    public void stopAllSubMotors(){
        shooter.stopShooter();
        intake.stopIntake();
        indexer.stopIndexer();
        feeder.stopFeeder();
    }

    public void startCycling(){
        intake.moveIntake(1);
        indexer.moveIndexer(1);
        feeder.moveFeeder(1);
    }
    public void stopCycling(){
        intake.stopIntake();
        indexer.stopIndexer();
        feeder.stopFeeder();
    }
}
