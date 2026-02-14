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

@Autonomous(name="Blue Auto Far & Leave", group="Blue")
public class Blue_Auto_Far_Leave extends LinearOpMode {
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

        chassis.AutoMovement(8,8,0);
        chassis.AutoTurn(24);

        timer.reset();
        while (timer.seconds() < 4) { // Shoot all 3 artifacts
            cameraDetection.CameraDetectionBlue();
            subsystems.shooter.shoot(319);

            if (timer.seconds() > 1.2 && timer.seconds() <= 4) {
                subsystems.startCycling();
            } else {
                subsystems.stopCycling();
            }
        }

        chassis.AutoTurn(0);
        chassis.AutoMovement(-15,0,0);

    }
}