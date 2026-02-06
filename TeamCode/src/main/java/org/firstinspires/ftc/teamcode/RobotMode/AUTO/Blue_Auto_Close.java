package org.firstinspires.ftc.teamcode.RobotMode.AUTO;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.range;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.Camera_Detection;
import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;
import org.firstinspires.ftc.teamcode.RobotFunctions.Sensors;
import org.firstinspires.ftc.teamcode.RobotFunctions.Subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.RobotMode.Dashboard;
import org.firstinspires.ftc.teamcode.RobotMode.TelemetryMethods;

@Autonomous(name="Blue Auto Close", group="Blue")
public class Blue_Auto_Close extends LinearOpMode {
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
        cameraDetection.CameraDetectionBlue();

        chassis.AutoMovement(0, -30, 0); // Move back to shoot
        chassis.AutoTurn(0);

        cameraDetection.CameraDetectionBlue();

        timer.reset();
        while (timer.seconds() < 3.9) { // Shoot all 3 artifacts
            cameraDetection.CameraDetectionBlue();
            subsystems.shooter.shoot(99.7);

            if (timer.seconds() > 1.115 && timer.seconds() <= 3.9) {
                subsystems.startCycling();
            } else {
                subsystems.stopCycling();
            }
        }
        chassis.AutoTurn(35); // Turn towards artifacts
        chassis.AutoMovement(-17.6, 0, 35);
        chassis.AutoTurn(35);
        timer.reset();

        while (timer.seconds() < 2.1) { // Pick up first artifact row
            subsystems.intake.moveIntake(1);
            subsystems.indexer.moveIndexer(1);
            subsystems.feeder.moveFeeder(.32);
            chassis.AutoMovementSlow(0, 37, 35);
        }

        subsystems.stopCycling();
        chassis.AutoMovement(-10.5,-8,35); // Move towards gate
        chassis.AutoMovement(0,10.9,35); // Open gate
        sleep(500);

        chassis.AutoMovement(20, -20, 35); // Move back to launch zone
        chassis.AutoMovement(10, -10, 35); // Move back to launch zone
        chassis.AutoTurn(-2);

        timer.reset();
        while (timer.seconds() < 4.2) { // Shoot all 3 artifacts
            cameraDetection.CameraDetectionBlue();
            subsystems.shooter.shoot(range);

            if (timer.seconds() > 1.12 && timer.seconds() <= 4.2) {
                subsystems.startCycling();
            } else {
                subsystems.stopCycling();
            }
        }

        chassis.AutoTurn(35);
        chassis.AutoMovement(-25,-5,35);
        chassis.AutoMovement(-22,0,35);
        chassis.AutoTurn(35);

        timer.reset();
        while (timer.seconds() < 2.2) { // Pick up second artifact row
            subsystems.intake.moveIntake(1);
            subsystems.indexer.moveIndexer(1);
            subsystems.feeder.moveFeeder(.32);
            chassis.AutoMovementSlow(0, 36, 35);
        }

        chassis.AutoMovement(25,-25,35);
        chassis.AutoMovement(20,-20,35);
        chassis.AutoTurn(0);

        timer.reset();
        while (timer.seconds() < 4.2) { // Shoot all 3 artifacts
            cameraDetection.CameraDetectionBlue();
            subsystems.shooter.shoot(range);

            if (timer.seconds() > 1.12 && timer.seconds() <= 4.2) {
                subsystems.startCycling();
            } else {
                subsystems.stopCycling();
            }
        }

    }
}