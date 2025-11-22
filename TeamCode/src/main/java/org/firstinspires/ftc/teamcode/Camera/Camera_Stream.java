package org.firstinspires.ftc.teamcode.Camera;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

public class Camera_Stream implements VisionProcessor, CameraStreamSource {
    /*It will contain the latest frame processed (Bitmap able us to process
    digital images into a pixel grid)*/
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>
            (Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    /*The Continuation use is for: when the result is found, the function will send
    the result secured and without difficult to its destination
    Consumer<Bitmap> is used when the Dashboard want to use/consume the Bitmap*/
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            /*This send the image to the Dashboard safety, continuation assures that the
            image is send correctly and without corruption*/
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
    //Let us see what sees the camera
    public Bitmap getLastFrame() {
        return lastFrame.get();
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
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {}
}
