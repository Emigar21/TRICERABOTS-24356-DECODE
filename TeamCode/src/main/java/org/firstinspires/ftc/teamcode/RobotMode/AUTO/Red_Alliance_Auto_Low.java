package org.firstinspires.ftc.teamcode.RobotMode.AUTO;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.range;
import static org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Shooter.getActualVel;
import static org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Shooter.getDesiredRevs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.RobotMode.TelemetryMethods;

@Autonomous(name="Red Auto Low", group="red")
public class Red_Alliance_Auto_Low extends LinearOpMode {
    ChassisController chassis;
    Subsystems subsystems;
    Sensors sensors;
    Camera_Detection cameraDetection;
    TelemetryMethods telemetryMethods;
    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new ChassisController(hardwareMap);
        subsystems = new Subsystems(hardwareMap);

        sensors = new Sensors(hardwareMap);
        cameraDetection = new Camera_Detection(hardwareMap);

        telemetryMethods = new TelemetryMethods();

        timer = new ElapsedTime();

        waitForStart(); // Auto Start

        chassis.mecanumDriveByTime(0,.4,1.4); // Drive Back to shooting position
        chassis.mecanumDriveByTime(0,.3,.6); // Drive Back to shooting position

        timer.reset();

        while(timer.seconds() < 6.3){ // Shoot all artifacts
            cameraDetection.CameraDetectionRed();
            subsystems.shooter.shoot(range);
            subsystems.intake.moveIntake(getActualVel() < getDesiredRevs(range) ? 0 : 1);
            subsystems.indexer.moveIndexer(getActualVel() < getDesiredRevs(range) ? 0 : 1);
            subsystems.feeder.moveFeeder(getActualVel() < getDesiredRevs(range) ? 0 : 1);
        }

        subsystems.stopAllSubMotors();

        chassis.mecanumTurnByTime(.25,.85); // Spin to look towards next artifact row // .85 baja

        chassis.mecanumDriveByTime(-.3,0,1.76); // Drive right to next artifact row // 1.2 alta, 1.88 baja

        subsystems.startCycling(); // Trigger intake, indexer and feeder
        chassis.mecanumDriveByTime(0,-.22,2.75);

        subsystems.feeder.stopFeeder();
        chassis.mecanumDriveByTime(0,-.22,.6); // .6 baja

        sleep(1000);

        subsystems.stopAllSubMotors();
        chassis.mecanumDriveByTime(0,.4,1.77); // Drive back to shooting position

        sleep(1000);

        chassis.mecanumTurnByTime(-.25,1.3); // Turn to goal // 1.05 alta, 1.10 baja
        chassis.mecanumDriveByTime(0,-.15,1.2); // Drive towards goal

        timer.reset();

        while(timer.seconds() < 7){ // Shoot all artifacts
            cameraDetection.CameraDetectionRed();
            subsystems.shooter.shoot(range);
            subsystems.intake.moveIntake(getActualVel() < getDesiredRevs(range) ? 0 : 1);
            subsystems.indexer.moveIndexer(getActualVel() < getDesiredRevs(range) ? 0 : 1);
            subsystems.feeder.moveFeeder(getActualVel() < getDesiredRevs(range) ? 0 : 1);
        }
    }
}
