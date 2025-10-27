package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Camera_Autonomous.cameraOrientation;
import static org.firstinspires.ftc.teamcode.Camera_Autonomous.cameraPosition;


import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
public class Camera_Data {
    public  Camera_Stream processor = new Camera_Stream();
    public AprilTagProcessor aprilTagProcessor;
    static double x;
    static double y;
    static double z;
    static double yaw;
    static double pitch;
    static double roll;
    static double bearing;
    static double range;
    static double elevation;
    static double id;
    static boolean detection;

    static boolean isDetecting;
    //Array that will contain pose values obtained in CameraDetections
    public AprilTagDetection currentDetections;
    static double[] detectionValues = {
            id, x, y, z, yaw, pitch, roll, range, bearing, elevation};

    public Camera_Data(HardwareMap hardwareMap) {
        // We create the builder with our desired building for the AprilTag processor
        // and the VisionPortal
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

        //Initializing the visionPortal and its building process
        VisionPortal visionPortal = new VisionPortal.Builder()
                //Create our Camera using the hardwareMap
                .setCamera(hardwareMap.get(WebcamName.class, "WebCam"))
                //We assign the aprilTagProcessor and visionProcessor (Used for Stream)
                .addProcessors(aprilTagProcessor,processor)
                //that we gonna use
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setLiveViewContainerId(R.id.cameraMonitorViewId)
                .setAutoStartStreamOnBuild(true)
                .build();
    }

    public void CameraDetection() {
        if(aprilTagProcessor.getDetections().isEmpty()) {
            detection = false;
        } else {
            //isDetecting() = aprilTagProcessor.getDetections().isEmpty();
            for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
                // We put the detection values into the detectionValues array
                //

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

//    public boolean isDetecting(){
//        detection.
//    }
}
//All this is used for streaming what sees the camera in the Ftc Dashboard =)