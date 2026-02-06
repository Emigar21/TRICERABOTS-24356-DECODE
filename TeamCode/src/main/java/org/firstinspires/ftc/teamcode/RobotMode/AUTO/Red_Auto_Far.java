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

@Autonomous(name="Red Auto Far Away", group="Red")
public class Red_Auto_Far extends LinearOpMode {
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

        waitForStart();

        telemetryMethods.TelemetryUpdateCamera();
        cameraDetection.CameraDetectionBlue();

        chassis.AutoMovement(-7,7,0);
        chassis.AutoTurn(-25);

        timer.reset();
        while (timer.seconds() < 4) { // Shoot all 3 artifacts
            cameraDetection.CameraDetectionBlue();
            subsystems.shooter.shoot(255);

            if (timer.seconds() > 1.115 && timer.seconds() <= 4) {
                subsystems.startCycling();
            } else {
                subsystems.stopCycling();
            }
        }

        chassis.AutoTurn(-90);
        chassis.AutoMovement(21,14,-90);
        chassis.AutoTurn(-90);

        timer.reset();

        while (timer.seconds() < 2.2) { // Pick up first artifact row
            subsystems.intake.moveIntake(1);
            subsystems.indexer.moveIndexer(1);
            subsystems.feeder.moveFeeder(.32);
            chassis.AutoMovementSlow(0, 32, -90);
        }

        chassis.AutoMovement(-15,-30,-90);
        chassis.AutoMovement(0,-16,-90);
        chassis.AutoTurn(20);

        timer.reset();
        while (timer.seconds() < 4) { // Shoot all 3 artifacts
            cameraDetection.CameraDetectionBlue();
            subsystems.shooter.shoot(250);

            if (timer.seconds() > 1.115 && timer.seconds() <= 4) {
                subsystems.startCycling();
            } else {
                subsystems.stopCycling();
            }
        }

        chassis.AutoTurn(-90);
        chassis.AutoMovement(17,15,-90);
        chassis.AutoTurn(-90);
        chassis.AutoMovement(0,2,-90);
    }
}
