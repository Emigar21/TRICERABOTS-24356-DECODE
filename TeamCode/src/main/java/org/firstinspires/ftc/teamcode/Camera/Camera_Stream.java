package org.firstinspires.ftc.teamcode.Camera;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Autonomous.RANGE_HIGH;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Autonomous.RANGE_LOW;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.concurrent.atomic.AtomicReference;

public class Camera_Stream implements VisionProcessor , CameraStreamSource {
        //The Mat is what is used by the OpenCV for containing the binary mask
        // that belongs to RANGE_LOW / RANGE_HIGH.
        Mat peopleMask = new Mat();
        
        //It contain the frame color media (The Scalar is what used the OpenCV for
        // representing the values of the channel), AtomicReference avoid the
        // blocking through Threads without warming
        private final AtomicReference<Scalar> mean = new AtomicReference<>(new Scalar(0, 0, 0, 0));

        //It will contain the latest frame processed (Bitmap able us to process
        // digital images into a pixel grid)
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>
                (Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        //It contain the binary mask (show us where is located the things)
        private final AtomicReference<Bitmap> peopleMaskedFrame = new AtomicReference<>
                (Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        //The Continuation use is for: when the result is found, the function will send
        //the result secured and without difficult to its destination
        //Consumer<Bitmap> is used when the Dashboard want to use/consume the Bitmap
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            //This send the image to the Dashboard safety, continuation assures that the
            //image is send correctly and without corruption
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

        public Bitmap getMaskedFrameBitmap() {
            return peopleMaskedFrame.get();
        }

        public Bitmap getLastFrame() {
            return lastFrame.get();
        }

        //This return the calculated color media
        public Scalar getMeanColor() {
            return mean.get();
        }

        @Override
        //Let us specify how would look like the video
        public void init(int width, int height, CameraCalibration cameraCalibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }
        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            //Create the Bitmap and select its specifications
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
            //Convert Mat (binary mask) into a Bitmap (Pixel grid)
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            //Get the color media and saved it
            mean.set(Core.mean(frame));
            //This create the people mask that depending the results drop into RANGE_LOW OR
            //RANGE_HIGH changes the pixel to 255 and if not 0
            Core.inRange(frame, RANGE_LOW, RANGE_HIGH, peopleMask);
            //Create People mask Bitmap
            Bitmap b_peopleMask = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(peopleMask, b_peopleMask);
            peopleMaskedFrame.set(b_peopleMask);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {}
    }
