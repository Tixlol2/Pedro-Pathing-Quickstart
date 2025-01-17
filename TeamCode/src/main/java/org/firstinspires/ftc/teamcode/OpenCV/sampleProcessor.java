package org.firstinspires.ftc.teamcode.OpenCV;

import android.graphics.Canvas;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
@Config
public class sampleProcessor implements VisionProcessor {

    public static Scalar lowYellow = new Scalar(20, 100, 100);
    public static Scalar highYellow = new Scalar(30, 255, 255);
    public static Scalar lowBlue = new Scalar(20, 100, 100);
    public static Scalar highBlue = new Scalar(30, 255, 255);
    public static Scalar lowRed1 = new Scalar(20, 100, 100);
    public static Scalar highRed1 = new Scalar(30, 255, 255);
    public static Scalar lowRed2 = new Scalar(20, 100, 100);
    public static Scalar highRed2 = new Scalar(30, 255, 255);
    public static Color color = Color.RED;

    private int width;
    private int height;
    private Position position;

    enum Position {
        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }

    enum Color {
        RED,
        BLUE,
        YELLOW


    }


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        //Creates new Mat that we will be editing from the input frame
        Mat mat = new Mat();
        //Transform to HSV color space
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);

        //Ensures that the mat is not broken in anyway
        if (mat.empty()){return frame;}

        //This is the HSV values we are comparing our image against
        //This is used to find the regions that we want
        //Currently Yellow

        //Prepares a new Mat that will hold our thresholded image
        Mat mask = new Mat();

        //Does all the processing and stores it in the mask Mat
        if (color == Color.YELLOW){
            Core.inRange(mat, lowYellow, highYellow, mask);
        } else if (color == Color.RED) {
            Core.inRange(mat, lowRed1, highRed1, mask);
        } else {
            Core.inRange(mat, lowBlue, highBlue, mask);
        }

        Mat edges = new Mat();
        Imgproc.Canny(mask, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        double left_x = 0.25 * width;
        double right_x = 0.75 * width;

        for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x < left_x)
                position = Position.LEFT;
            if (boundRect[i].x + boundRect[i].width > right_x)
                position = Position.RIGHT;
            if(boundRect[i].x > right_x && boundRect[i].x < left_x){
                position = Position.MIDDLE;
            }
            else{position = Position.NOT_FOUND;}

            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
        }







        return mat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Position getPosition(){
        return position;
    }
}
