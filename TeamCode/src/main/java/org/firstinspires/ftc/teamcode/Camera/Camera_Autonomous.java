package org.firstinspires.ftc.teamcode.Camera;


import static org.firstinspires.ftc.teamcode.RobotMode.Dashboard.ftcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.RobotMode.Dashboard;

import org.firstinspires.ftc.teamcode.RobotFunctions.Chassis.ChassisController;
import org.opencv.core.Scalar;

@Autonomous(name = "Camera_Testeito",group = "Robot Autos")

public class Camera_Autonomous extends LinearOpMode {
    Subsystems2 subsystems;

    Camera_Detection cameraDetection;
    ChassisController chassis;
    Dashboard dashboard;


    public static boolean MASK_TOGGLE = false;
    public static Scalar RANGE_LOW = new Scalar(50, 50, 50, 0);
    public static Scalar RANGE_HIGH = new Scalar(140, 140, 140, 255);

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = new Dashboard();
        cameraDetection = new Camera_Detection(hardwareMap);
        subsystems = new Subsystems2(hardwareMap);
//        chassis = new ChassisController(hardwareMap);
//        chassis.resetEncoders();

        //Dashboard.RunDashboard(chassis.getDistanceInchesX());
        //telemetry.addLine("Field Established");

        //Initializing the telemetry(Dashboard from Camera_Stream) and the Camera Stream
        //telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        telemetry.addLine("Processors updated");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start Autonomous");
        telemetry.addLine();
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            Dashboard.initDashboard(0,0, 15,15);
            cameraDetection.CameraDetection();

            ftcDashboard.sendImage(cameraDetection.streamProcessor.getLastFrame());

            TelemetryUpdateCamera();
        }
    }
    public void TelemetryUpdateCamera() {
        //Giving the telemetry for the FTC Dashboard (the PC "Driver Station")
        // from the array of CameraDetection
        telemetry.addData("Bearing ", Camera_Detection.bearing);
        telemetry.addLine();
        telemetry.addData("April Tag ", cameraDetection.id);
        telemetry.addData("Detection", Camera_Detection.detection);
        telemetry.addData("Auto running", "Look for more");
        telemetry.update();

    }
}