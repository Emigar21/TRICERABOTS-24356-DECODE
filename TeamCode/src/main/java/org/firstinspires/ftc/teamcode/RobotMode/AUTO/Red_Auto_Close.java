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

@Autonomous(name="Red Auto Close", group="Blue")
public class Red_Auto_Close extends LinearOpMode {
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
        Dashboard.initDashboard(ChassisController.getDistanceInchesX(), ChassisController.getDistanceInchesY(), 0, 15);

        chassis.AutoTurn(0);
        chassis.AutoMovement(0, -30, 0); // Move back to shoot

        cameraDetection.CameraDetectionBlue();

//        sleep();

        timer.reset();
        while (timer.seconds() < 5) { // Shoot all 3 artifacts
            cameraDetection.CameraDetectionBlue();

            subsystems.shooter.shoot(range);

            if (timer.seconds() > 2) {
                subsystems.startCycling();
            }
        }

        subsystems.stopAllSubMotors();
        chassis.AutoTurn(-35); // Turn towards artifacts
        chassis.AutoMovement(42, 0, -35);
        timer.reset();

        timer.reset();
        while (timer.seconds() < 3) {
            subsystems.intake.moveIntake(1);
            subsystems.indexer.moveIndexer(1);
            subsystems.feeder.moveFeeder(.1);
            chassis.AutoMovement(0, 36, -35);
        }

        subsystems.stopAllSubMotors();
        chassis.AutoMovement(0, -37, -35);
        chassis.AutoMovement(-35, -6, -35);

        chassis.AutoTurn(0);

        timer.reset();
        while (timer.seconds() < 5) { // Shoot all 3 artifacts
            cameraDetection.CameraDetectionBlue();

            subsystems.shooter.shoot(range);

            if (timer.seconds() > 2) {
                subsystems.startCycling();
            }
        }
    }
}

