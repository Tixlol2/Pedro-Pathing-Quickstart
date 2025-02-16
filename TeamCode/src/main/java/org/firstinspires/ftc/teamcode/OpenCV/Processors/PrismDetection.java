package org.firstinspires.ftc.teamcode.OpenCV.Processors;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Prism Pose Detection", group = "Vision")
public class PrismDetection extends LinearOpMode {
    private OpenCvCamera camera;
    private PrismPosePipeline pipeline;

    // Physical dimensions of the prism in inches
    private static final double PRISM_LENGTH = 3.5;
    private static final double PRISM_WIDTH = 1.5;

    // Camera parameters
    private static final double FOCAL_LENGTH_PX = 578.272; // Replace with your calibrated value
    private static final double CAMERA_HEIGHT = 12.0; // Height of camera from ground in inches
    private static final double CAMERA_TILT = Math.toRadians(0); // Camera tilt angle from horizontal

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new PrismPosePipeline();
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

        waitForStart();

        while (opModeIsActive()) {
            if (pipeline.isPrismDetected()) {
                PrismPose pose = pipeline.getPrismPose();

                telemetry.addData("Distance (inches)", "%.2f", pose.distance);
                telemetry.addData("X Offset (inches)", "%.2f", pose.xOffset);
                telemetry.addData("Rotation (degrees)", "%.2f", pose.rotation);
            } else {
                telemetry.addData("Status", "No prism detected");
            }
            telemetry.update();
        }

        camera.closeCameraDevice();
    }

//    private void initializeCamera(hardwareMap hMap) {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        camera = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        pipeline = new PrismPosePipeline();
//        camera.setPipeline(pipeline);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Error", errorCode);
//            }
//        });
//    }

    static class PrismPose {
        double distance;   // Distance from camera to prism in inches
        double xOffset;    // Lateral offset from camera center in inches
        double rotation;   // Rotation angle in degrees

        PrismPose(double distance, double xOffset, double rotation) {
            this.distance = distance;
            this.xOffset = xOffset;
            this.rotation = rotation;
        }
    }

    class PrismPosePipeline extends OpenCvPipeline {
        private volatile PrismPose lastPose = null;
        private volatile boolean prismDetected = false;

        // Adjust these HSV values based on your prism's color
        private final Scalar upperBound = new Scalar(90, 60, 60);  // Example for red
        private final Scalar lowerBound = new Scalar(105, 255, 255); // Example for red

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat mask = new Mat();
            Mat hierarchy = new Mat();

            // Convert to HSV color space
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Create mask for color detection
            Core.inRange(hsv, lowerBound, upperBound, mask);

            // Noise reduction
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            prismDetected = false;

            if (!contours.isEmpty()) {
                // Find the largest contour
                MatOfPoint largestContour = findLargestContour(contours);

                if (largestContour != null) {
                    // Get the minimum area rectangle
                    MatOfPoint2f contour2f = new MatOfPoint2f();
                    largestContour.convertTo(contour2f, CvType.CV_32F);
                    RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

                    // Calculate pose parameters
                    double distance = calculateDistance(rotatedRect);
                    double xOffset = calculateXOffset(rotatedRect, input.width());
                    double rotation = calculateRotation(rotatedRect);

                    // Create new pose object
                    lastPose = new PrismPose(distance, xOffset, rotation);

                    // Draw visualization
                    drawPoseVisualization(input, rotatedRect, lastPose);

                    prismDetected = true;
                }
            }

            hsv.release();
            mask.release();
            hierarchy.release();

            return input;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largest = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea && area > 500) {
                    maxArea = area;
                    largest = contour;
                }
            }

            return largest;
        }

        private double calculateDistance(RotatedRect rect) {
            // Using the apparent height of the prism to calculate distance
            double apparentHeight = Math.min(rect.size.width, rect.size.height);
            double distance = (PRISM_WIDTH * FOCAL_LENGTH_PX) / apparentHeight;

            // Correct for camera tilt
            //distance = distance * Math.cos(CAMERA_TILT);

            return distance;
        }

        private double calculateXOffset(RotatedRect rect, double imageWidth) {
            // Calculate pixel offset from center
            double centerX = rect.center.x;
            double pixelOffset = centerX - (imageWidth / 2.0);

            // Convert to real-world units using similar triangles
            return (pixelOffset ) / FOCAL_LENGTH_PX;
        }

        private double calculateRotation(RotatedRect rect) {
            // Get the angle of the rotated rectangle
            double baseRatio = PRISM_WIDTH/PRISM_LENGTH;
            double angle = ((rect.size.height/rect.size.width - baseRatio)/ (1-baseRatio)) * 90;

            return angle;
        }

        private void drawPoseVisualization(Mat input, RotatedRect rect, PrismPose pose) {
            // Draw rotated rectangle
            Point[] vertices = new Point[4];
            rect.points(vertices);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            // Draw direction arrow
            Point center = rect.center;
            double radians = Math.toRadians(pose.rotation);
            Point arrowTip = new Point(
                    center.x + Math.cos(radians) * 50,
                    center.y + Math.sin(radians) * 50
            );
            Imgproc.arrowedLine(input, center, arrowTip, new Scalar(255, 0, 0), 2);

            // Draw distance and confidence text
            String info = String.format("%.1f\"", pose.distance);
            Imgproc.putText(input, info, new Point(center.x - 20, center.y - 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
        }

        public PrismPose getPrismPose() {
            return lastPose;
        }

        public boolean isPrismDetected() {
            return prismDetected;
        }
    }
}