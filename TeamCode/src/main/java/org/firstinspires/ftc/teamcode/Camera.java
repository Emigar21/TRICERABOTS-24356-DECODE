package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Dashboard.dashboardTelemetry;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicReference;

@Config
@TeleOp(name="FtcDashboard ColorMasking", group="Linear")

public class Camera extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    CameraStreamProcessor processor = new CameraStreamProcessor();

    private AprilTagProcessor aprilTagProcessor;

    // Set the position of the camera in the robot
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);

    //Set the orientation of the camera in the robot
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    double x;
    double y;

    double z;
    double yaw;
    double pitch;
    double roll;
    double bearing;
    double range;
    double elevation;

    double id;

    //Array that will contain pose values obtained in CameraDetections
    double[] detectionValues = {
            id, x, y, z, yaw, pitch, roll, range, bearing, elevation
    };




    public static boolean MASK_TOGGLE = false;
    public static Scalar RANGE_LOW = new Scalar(50,50,50,0);
    public static Scalar RANGE_HIGH = new Scalar(140,140,140,255);
    //Talk Volatile vs AtomicReference, final to make sure it is only initialized once, etc.
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {

        Mat peopleMask = new Mat();
        private final AtomicReference<Scalar> mean = new AtomicReference<>(new Scalar(0,0,0,0));
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565));
        private final AtomicReference<Bitmap> peopleMaskedFrame = new AtomicReference<>(Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565));

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

        public Bitmap getMaskedFrameBitmap(){
            return peopleMaskedFrame.get();
        }
        public Bitmap getLastFrame(){
            return lastFrame.get();
        }

        public Scalar getMeanColor(){
            return mean.get();
        }

        @Override
        public void init(int width, int height, CameraCalibration cameraCalibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            mean.set(Core.mean(frame));
            Core.inRange(frame, RANGE_LOW, RANGE_HIGH,peopleMask);
            Bitmap b_peopleMask = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(peopleMask, b_peopleMask);
            peopleMaskedFrame.set(b_peopleMask);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {

        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //We init the AprilTag using our function that has the AprilTag processor builder
        initAprilTag();

        //We init the VisionPortal using our function that has the AprilTag processor builder
        initVisionPortal();

        waitForStart();

        while(opModeIsActive()){

            dashboard.sendImage(processor.getLastFrame());
            TelemetryUpdateCamera(CameraDetection());
        }
    }


    private void initAprilTag() {
        // We create the builder with our desired building for the AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                //Draw the axes at the Live view whenever it detect a AprilTag
                .setDrawAxes(true)

                //Set Camera´s position and orientation in the robot
                .setCameraPose(cameraPosition, cameraOrientation)

                //Specify the april Tags that we will use this competition
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                //Specify the units we want to use for the output detections
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        telemetry.addLine("Processor created");
    }

    private void initVisionPortal() {

        //Initializing the visionPortal and its building process
        VisionPortal visionPortal = new VisionPortal.Builder()
                //Create our Camera using the hardwareMap
                .setCamera(hardwareMap.get(WebcamName.class, "WebCam"))
                //We assign the aprilTagProcessor that we gonna use
                .addProcessors(aprilTagProcessor, processor)
                .setCameraResolution(new Size(1280, 720))

                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)

                //Let us see the whats recording the camera
                .setLiveViewContainerId(R.id.cameraMonitorViewId)

                .setAutoStartStreamOnBuild(true)
                .build();
        telemetry.addLine("VisionPortal created");

    }

    private double [] CameraDetection() {
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            // We put the detection values into the detectionValues array

            id = detection.id;

            //Getting xDistance, yDistance and zDistance
            x = detection.ftcPose.x;//aqui
            y = detection.ftcPose.y;
            z = detection.ftcPose.z;

            //Getting Yaw, Pitch and Roll, used on angulation/orientation
            yaw = detection.ftcPose.yaw;
            pitch = detection.ftcPose.pitch;
            roll = detection.ftcPose.roll;

            //Getting range, bearing and elevation
            range = detection.ftcPose.range;
            bearing = detection.ftcPose.bearing;
            elevation = detection.ftcPose.elevation;

        }
        return detectionValues;
    }
    private void TelemetryUpdateCamera ( double[] detectionValues){
        //Giving the telemetry for the FTC Dashboard (the PC "Driver Station")
        // from the array of CameraDetection
        dashboardTelemetry.addData("April Tag id", id);

        dashboardTelemetry.addData("xDistance = ", x);
        dashboardTelemetry.addData("yDistance = ", y);
        dashboardTelemetry.addData("zDistance = ", z);

        dashboardTelemetry.addData("Yaw = ", yaw);
        dashboardTelemetry.addData("Pitch = ", roll);
        dashboardTelemetry.addData("Roll = ", pitch);

        dashboardTelemetry.addData("Range = ", range);
        dashboardTelemetry.addData("Bearing = ", bearing);
        dashboardTelemetry.addData("Elevation = ", roll);

        telemetry.addData("xDistance ", x);
        telemetry.addData("yDistance ", y);
        telemetry.addData("zDistance ", z);
        telemetry.addLine();
        telemetry.addData("Yaw", yaw);
        telemetry.addData("Pitch ",pitch);
        telemetry.addData("Roll ", roll);
        telemetry.addLine();
        telemetry.addData("Range ", range);
        telemetry.addData("Bearing ", bearing);
        telemetry.addData("Elevation ", elevation);
        telemetry.addLine();
        telemetry.addData("April Tag ", id);

        //Updating the position of the robot in the canvas by using
        // the values of xDistance and yDistance
        dashboardTelemetry.update();
        telemetry.update();
    }

}
