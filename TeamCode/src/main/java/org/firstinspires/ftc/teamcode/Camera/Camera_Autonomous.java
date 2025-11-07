package org.firstinspires.ftc.teamcode.Camera;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Data.bearing;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Data.detection;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Data.id;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Dashboard;
import org.opencv.core.Scalar;

@Autonomous(name = "Camera_Testeito",group = "Robot Autos")

public class Camera_Autonomous extends LinearOpMode {
    Dashboard dashboard = new Dashboard();
    // Set the position of the camera in the robot
    public static final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);

    //Set the orientation of the camera in the robot
    public static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0,-90, 0, 0);

    public static boolean MASK_TOGGLE = false;
    public static Scalar RANGE_LOW = new Scalar(50, 50, 50, 0);
    public static Scalar RANGE_HIGH = new Scalar(140, 140, 140, 255);
    //Talk Volatile vs AtomicReference, final to make sure it is only initialized once, etc.

    // Set the position of the camera in the robot
    @Override
    public void runOpMode() throws InterruptedException {
        //Initializing the telemetry(Dashboard from Camera_Stream) and the Camera Stream
        telemetry = new MultipleTelemetry(telemetry);

//        Subsytems subsytems = new Subsytems(hardwareMap);
        //ChassisController chassis = new ChassisController(hardwareMap);
        Camera_Data camera = new Camera_Data(hardwareMap);

        telemetry.addLine("Field Established");

        telemetry.addLine("Processors updated");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start Autonomous");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            dashboard.RunDashboard();
            camera.CameraDetection();
//            chassis.chassisFollow();

            TelemetryUpdateCamera();
            telemetry.addData("Auto running", "Look for more");
            telemetry.update();
        }
    }


    public void TelemetryUpdateCamera() {
        //Giving the telemetry for the FTC Dashboard (the PC "Driver Station")
        // from the array of CameraDetection

        telemetry.addData("Bearing ", bearing);
        telemetry.addLine();
        telemetry.addData("April Tag ", id);
        telemetry.addData("Detection", detection);
//        telemetry.addData("Motor Power", servoBearing.getPower());
        telemetry.update();

    }

}