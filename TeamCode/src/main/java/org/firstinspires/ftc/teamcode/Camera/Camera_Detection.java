package org.firstinspires.ftc.teamcode.Camera;

import android.graphics.Bitmap;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.atomic.AtomicReference;
public class Camera_Detection{
    public Camera_Stream streamProcessor = new Camera_Stream();
    public AprilTagProcessor detectionProcessor;
    public VisionPortal visionPortal;
    public double x;
    public double y;
    public double z;
    public double yaw;
    public double pitch;
    public double roll;
    public static double bearing;
    public static double range;
    public double elevation;
    public static double id;
    public static boolean detection;

    public static String artifactPos1;
    public static String artifactPos2;
    public static String artifactPos3;

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>
            (Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public Camera_Detection(HardwareMap hardwareMap) {
        // We create the builder with our desired building for the AprilTag processor
        // and the VisionPortal

        //Set the orientation of the camera in the robot
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -90, 0, 0);
        // Set the position of the camera in the robot
        Position cameraPosition = new Position(DistanceUnit.INCH,
                0, 0, 0, 0);
        detectionProcessor = new AprilTagProcessor.Builder()
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

        //Initializing the visionPortal and its building process
        visionPortal = new VisionPortal.Builder()
                //Create our Camera using the hardwareMap
                .setCamera(hardwareMap.get(WebcamName.class, "WebCam"))
                //We assign the aprilTagProcessor and visionProcessor (Used for Stream)
                .addProcessors(detectionProcessor, streamProcessor)
                //that we gonna use
                .setCameraResolution(new Size(1280,720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setLiveViewContainerId(R.id.cameraMonitorViewId)
                .setAutoStartStreamOnBuild(true)
                .build();
    }


    public void CameraDetection() {
        if (detectionProcessor.getDetections().isEmpty()) {
            detection = false;
        } else {
            //isDetecting() = aprilTagProcessor.getDetections().isEmpty();
            for (AprilTagDetection detection : detectionProcessor.getDetections()) {
                // We put the detection values into the detectionValues array
                id = detection.id;

                //Getting xDistance, yDistance and zDistance
                x = detection.ftcPose.x;
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
            detection = true;
        }
    }


    public static String[] artifactsObelisk = new String[3];



    public void getObelisk(){
        if (id == 21){
            artifactsObelisk[0] = "green";
            artifactsObelisk[1] = "purple";
            artifactsObelisk[2] = "purple";
        } else if (id == 22) {
            artifactsObelisk[0] = "purple";
            artifactsObelisk[1] = "green";
            artifactsObelisk[2] = "purple";
        } else if (id == 23) {
            artifactsObelisk[0] = "purple";
            artifactsObelisk[1] = "purple";
            artifactsObelisk[2] = "green";
        }
    }
}