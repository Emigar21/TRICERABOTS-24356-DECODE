package org.firstinspires.ftc.teamcode.RobotMode.AUTO;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.range;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.RobotMode.Dashboard;
import org.firstinspires.ftc.teamcode.RobotMode.TelemetryMethods;

@Autonomous(name="Testeo odometria", group="Testing")
public class OdometryTesting extends LinearOpMode {
    ChassisController chassis;
    Subsystems subsystems;
    Sensors sensors;
    Camera_Detection cameraDetection;
    public static TelemetryMethods telemetryMethods;
    Dashboard dashboard;
    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new ChassisController(hardwareMap);
        subsystems = new Subsystems(hardwareMap);

        sensors = new Sensors(hardwareMap);
        cameraDetection = new Camera_Detection(hardwareMap);
        dashboard = new Dashboard();

        telemetryMethods = new TelemetryMethods();

        timer = new ElapsedTime();

        telemetryMethods.InitTelemetry(telemetry);
        cameraDetection.CameraDetectionBlue();

        waitForStart(); // Auto Start

        telemetryMethods.TelemetryUpdateCamera();
        Dashboard.initDashboard(ChassisController.getDistanceInchesX(),ChassisController.getDistanceInchesY(),0,15);

        chassis.AutoMovement(0,-30,0);

        cameraDetection.CameraDetectionBlue();

        sleep(3000);

        timer.reset();
        while (timer.seconds() < 5){
            cameraDetection.CameraDetectionBlue();

            subsystems.shooter.shoot(range);
            subsystems.intake.moveIntake(Shooter.getActualVel() >= (Shooter.controlPoints.get(range) - 275) ? 1 : 0);
            subsystems.indexer.moveIndexer(Shooter.getActualVel() >= (Shooter.controlPoints.get(range) - 275) ? 1 : 0);
            subsystems.feeder.moveFeeder(Shooter.getActualVel() >= (Shooter.controlPoints.get(range) - 275) ? 1 : 0);
        }

        subsystems.stopAllSubMotors();
        chassis.AutoTurn(35);
        chassis.AutoMovement(-40, 0, 35);
        timer.reset();
        while (timer.seconds() < 3){
            subsystems.startCycling();
            chassis.AutoMovement(0, 37, 35);
        }

        subsystems.stopAllSubMotors();
        chassis.AutoMovement(0, -37, 35);
        chassis.AutoMovement(35,-6,-10);

        timer.reset();
        while (timer.seconds() < 5){
            cameraDetection.CameraDetectionBlue();

            subsystems.shooter.shoot(range);
            subsystems.intake.moveIntake(Shooter.getActualVel() >= (Shooter.controlPoints.get(range) - 275) ? 1 : 0);
            subsystems.indexer.moveIndexer(Shooter.getActualVel() >= (Shooter.controlPoints.get(range) - 275) ? 1 : 0);
            subsystems.feeder.moveFeeder(Shooter.getActualVel() >= (Shooter.controlPoints.get(range) - 275) ? 1 : 0);
        }

        subsystems.stopAllSubMotors();

    }
}

