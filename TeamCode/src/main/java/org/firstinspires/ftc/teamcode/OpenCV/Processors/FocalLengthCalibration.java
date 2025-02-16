package org.firstinspires.ftc.teamcode.OpenCV.Processors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Focal Length Calibration", group = "Calibration")
public class FocalLengthCalibration extends LinearOpMode {
    private OpenCvCamera camera;
    private CalibrationPipeline pipeline;

    // Known calibration values
    private static final double KNOWN_DISTANCE = 24.0; // Distance from camera in inches
    private static final double KNOWN_WIDTH = 1.5;    // Width of your prism in inches

    @Override
    public void runOpMode() {
        initializeCamera();

        waitForStart();

        while (opModeIsActive()) {
            if (pipeline.isObjectDetected()) {
                double objectWidthPixels = pipeline.getObjectWidth();
                double calculatedFocalLength = (objectWidthPixels * KNOWN_DISTANCE) / KNOWN_WIDTH;

                telemetry.addData("Object Width (px)", "%.2f", objectWidthPixels);
                telemetry.addData("Calculated Focal Length", "%.2f", calculatedFocalLength);
                telemetry.addData("Instructions", "Place object exactly 24 inches from camera");
                telemetry.addData("", "Make sure object is parallel to camera");
            } else {
                telemetry.addData("Status", "No object detected");
                telemetry.addData("Instructions", "Adjust HSV values if needed");
            }
            telemetry.update();
        }

        camera.closeCameraDevice();
    }

    private void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CalibrationPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }

    class CalibrationPipeline extends OpenCvPipeline {
        private volatile double objectWidth = 0;
        private volatile boolean objectDetected = false;

        // Adjust these HSV values to match your object's color
        private final Scalar lowerBound = new Scalar(0, 100, 100);   // Example for red
        private final Scalar upperBound = new Scalar(10, 255, 255);  // Example for red

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat mask = new Mat();
            Mat hierarchy = new Mat();

            // Convert to HSV
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Create mask
            Core.inRange(hsv, lowerBound, upperBound, mask);

            // Noise reduction
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            objectDetected = false;

            // Find largest contour
            if (!contours.isEmpty()) {
                double maxArea = 0;
                MatOfPoint largestContour = null;

                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > maxArea && area > 500) {
                        maxArea = area;
                        largestContour = contour;
                    }
                }

                if (largestContour != null) {
                    // Get bounding rectangle
                    Rect boundRect = Imgproc.boundingRect(largestContour);
                    objectWidth = boundRect.width;

                    // Draw rectangle
                    Imgproc.rectangle(input, boundRect, new Scalar(0, 255, 0), 2);

                    // Draw centerline
                    Point center = new Point(boundRect.x + boundRect.width/2, input.height()/2);
                    Imgproc.line(input, new Point(0, input.height()/2),
                            new Point(input.width(), input.height()/2),
                            new Scalar(255, 0, 0), 1);

                    objectDetected = true;
                }
            }

            hsv.release();
            mask.release();
            hierarchy.release();

            return input;
        }

        public double getObjectWidth() {
            return objectWidth;
        }

        public boolean isObjectDetected() {
            return objectDetected;
        }
    }
}