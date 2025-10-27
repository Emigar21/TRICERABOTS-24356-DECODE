package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.Camera_Data.bearing;
import static org.firstinspires.ftc.teamcode.Camera_Data.detection;
import static org.firstinspires.ftc.teamcode.Camera_Data.detectionValues;
import static org.firstinspires.ftc.teamcode.Camera_Data.elevation;
import static org.firstinspires.ftc.teamcode.Camera_Data.id;
import static org.firstinspires.ftc.teamcode.Camera_Data.pitch;
import static org.firstinspires.ftc.teamcode.Camera_Data.range;
import static org.firstinspires.ftc.teamcode.Camera_Data.roll;
import static org.firstinspires.ftc.teamcode.Camera_Data.x;
import static org.firstinspires.ftc.teamcode.Camera_Data.y;
import static org.firstinspires.ftc.teamcode.Camera_Data.yaw;
import static org.firstinspires.ftc.teamcode.Camera_Data.z;
import static org.firstinspires.ftc.teamcode.Dashboard.dashboard;
import static org.firstinspires.ftc.teamcode.Dashboard.dashboardTelemetry;
import static org.firstinspires.ftc.teamcode.Subsytems.intakeMotor;
import static org.firstinspires.ftc.teamcode.Subsytems.servoBearing;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

@Autonomous(name = "Camera_Testeito",group = "Robot Autos")

public class Camera_Autonomous extends LinearOpMode {

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
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        Subsytems subsytems = new Subsytems(hardwareMap);
        Camera_Data camera = new Camera_Data(hardwareMap);

        telemetry.addLine("Processors updated");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            // We first use the function of Camera Detection to get the desired values
            // and then we update the FTC Dashboard with that data using TelemetryUpdateCamera
            dashboard.sendImage(camera.processor.getLastFrame());
            camera.CameraDetection();
            TelemetryUpdateCamera();
            subsytems.FollowServoAprilTag(bearing);
            telemetry.addData("Auto running", "Look for more");
            telemetry.update();
        }
    }


    public void TelemetryUpdateCamera() {

        //Giving the telemetry for the FTC Dashboard (the PC "Driver Station")
        // from the array of CameraDetection
//        telemetry.addData("xDistance ", x);
//        telemetry.addData("yDistance ", y);
//        telemetry.addData("zDistance ", z);
//        telemetry.addLine();
//        telemetry.addData("Yaw", yaw);
//        telemetry.addData("Pitch ", pitch);
//        telemetry.addData("Roll ", roll);
//        telemetry.addLine();
//        telemetry.addData("Range ", range);
        telemetry.addData("Bearing ", bearing);
        //telemetry.addData("Servo Power = ", servoBearing.getPower());
//        telemetry.addData("Elevation ", elevation);
        telemetry.addLine();
        telemetry.addData("April Tag ", id);
        telemetry.addData("Detection", detection);
        telemetry.addData("Motor Power", intakeMotor.getPower());
        telemetry.update();
    }
}