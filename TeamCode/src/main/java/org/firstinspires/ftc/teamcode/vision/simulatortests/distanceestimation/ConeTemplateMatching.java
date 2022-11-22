package org.firstinspires.ftc.teamcode.vision.simulatortests.distanceestimation;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


public class ConeTemplateMatching extends OpenCvPipeline {


    public final String filePath = "/sdcard/FIRST/output-onlineimagetools.png";

    public final double CAMERA_FOV = CameraConstants.fovXDeg;

    public final double CUBE_WIDTH = 4;

    Telemetry t;
    ElapsedTime frameTimer = new ElapsedTime();

    public Scalar lowerBound = new Scalar(82.2, 94.9, 83.6); // new Scalar(25.5, 80.8, 131.8);
    public Scalar upperBound = new Scalar(255, 255, 255);// new Scalar(46.8, 255, 255);

    private Mat hsvMat       = new Mat();
    private Mat blurredMat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat contourMat = new Mat();

    private final ArrayList<MatOfPoint> listOfContours = new ArrayList<>();

    private int stageNum = 2;

    public ConeTemplateMatching() {

    }

    private enum Stage
    {
        FILTERED,
        BLURRED,
        CONTOURS
    }

    Stage[] stages = Stage.values();

    /*
     Mat kernel1 = Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, new Size(3, 3));
     Mat kernel2 = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(3, 3));
     Mat kernel3 = Imgproc.getStructuringElement(Imgproc.MORPH_CLOSE, new Size(3, 3));
 */
    Size blurSize = new Size(10, 10);

    private double degreesError = 0;

    public ConeTemplateMatching(Telemetry t) {
        this.t = t;
    }

    public static double truncate(double value, int decimalpoint) {

        // Using the pow() method
        value = value * Math.pow(10, decimalpoint);
        value = Math.floor(value);
        value = value / Math.pow(10, decimalpoint);

        return value;
    }

    @Override
    public void onViewportTapped()
    {

        int nextStageNum = stageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;


    }
    @Override
    public Mat processFrame(Mat input) {

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));

        // Mat kernel1 = Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, new Size(5, 5));
        // Mat kernel2 = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(5, 5));
        // Mat kernel3 = Imgproc.get(Imgproc.MORPH_CLOSE, new Size(3, 3));

        frameTimer.reset();

        // t.addLine(input.size().width + " x " + input.size().height);
        hsvMat.release();
        thresholdMat.release();
        contourMat.release();
        listOfContours.clear();

        if (this.t != null) {
            t.addData("Release Time (ms): ", frameTimer.milliseconds());
        }

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);

        if (this.t != null) {
            t.addData("Threshold Time (ms): ", frameTimer.milliseconds());
        }

        // Imgproc.morphologyEx(thresholdMat, thresholdMat, Imgproc.MORPH_ERODE, kernel1);
        // Imgproc.morphologyEx(thresholdMat, thresholdMat, Imgproc.MORPH_DILATE, kernel2);
        // Imgproc.morphologyEx(thresholdMat, thresholdMat, Imgproc.MORPH_CLOSE, new MatOfDouble(4, 4));


        Imgproc.threshold(input, thresholdMat, 110, 255, Imgproc.THRESH_BINARY_INV);
        if (this.t != null) {
            t.addData("Post Processing Time (ms): ", frameTimer.milliseconds());
        }
        // Imgproc.GaussianBlur(thresholdMat, blurredMat, new Size(11, 11), 0, 0);
        Imgproc.findContours(thresholdMat, listOfContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        input.copyTo(contourMat);
        // Imgproc.drawContours(contourMat, listOfContours, -1, new Scalar(0, 0, 255), 2, 8);

        if (this.t != null) {
            t.addData("Math Time (ms): ", frameTimer.milliseconds());
        }
        MatOfPoint largestContour = new MatOfPoint();
        double largestContourArea = -1;

        if (listOfContours.size() > 0) {

            for (int i = 0; i < listOfContours.size(); i++) {

                MatOfPoint currentContour = listOfContours.get(i);
                double currentContourArea = Imgproc.contourArea(currentContour);

                if (currentContourArea > largestContourArea) {
                    largestContour = currentContour;
                    largestContourArea = currentContourArea;
                }

                Rect boundingBox = Imgproc.boundingRect(currentContour);
                Imgproc.rectangle(contourMat, boundingBox, new Scalar(255, 255 , 255));

            }

            Rect boundingBox = Imgproc.boundingRect(largestContour);
            Imgproc.rectangle(contourMat, boundingBox, new Scalar(15, 255, 80));

            int centerXCoordinate = boundingBox.x + boundingBox.width / 2;
            int centerYCoordinate = boundingBox.y + boundingBox.height / 2;

            Imgproc.circle(contourMat, new Point(centerXCoordinate, centerYCoordinate), 3, new Scalar(255, 0, 0));

            double conversionPixelsToDegrees = CAMERA_FOV / input.size().width;

            double linearDegreesErrorX = -(centerXCoordinate - (input.size().width / 2)) * conversionPixelsToDegrees;
            double curvedDegreesErrorX = -Math.toDegrees(Math.atan2((centerXCoordinate - (input.size().width / 2)), CameraConstants.fx));
            double curvedDegreesErrorY = -Math.toDegrees(Math.atan2((centerYCoordinate - (input.size().height / 2)), CameraConstants.fy));

            double ratioX = CUBE_WIDTH / boundingBox.width;
            double ratioY = CUBE_WIDTH / boundingBox.width;

            double depthX = ratioX * CameraConstants.fx;
            double depthY = ratioY * CameraConstants.fy;

            double rayDistance = Math.hypot(depthX, depthY); // true distance

            // <ignore>

            double cosDistance  = Math.cos(Math.toRadians(curvedDegreesErrorY));
            double hypotenuseY = cosDistance == 0 ? 0 : rayDistance / cosDistance - rayDistance; // have angle and adj, need hyp
            double hypotenuseX = Math.abs(rayDistance * Math.tan(Math.toRadians(curvedDegreesErrorX))); // have angle and adj, need opp

            double distanceToCamera = Math.cbrt(Math.pow(hypotenuseX, 3)  + Math.pow(hypotenuseY, 3) + Math.pow(rayDistance, 3)); // inaccurate
            // </ignore>

            this.degreesError = curvedDegreesErrorX;

            if (this.t != null) {
                t.addData("X Degrees Error: ", curvedDegreesErrorX);
                t.addData("Y Degrees Error: ", curvedDegreesErrorY);

                t.addData("Depth (X): ", depthX);
                t.addData("Depth (Y): ", depthY);

                t.addData("Ray Distance: ", rayDistance);

                t.addData("Hypotenuse Y: ", hypotenuseY);
                t.addData("Hypotenuse X: ", hypotenuseX);
                t.addData("Distance: ", distanceToCamera);
            }



            String distanceToConeString =
                    "Distance To Cone: " +
                            String.valueOf(
                                    ConeDetectionTesting.truncate(rayDistance, 4)
                            ) + " in.";
            String yDegreeErrorString =
                    "Horizontal Angle To Cone: " +
                            String.valueOf(
                                    ConeDetectionTesting.truncate(
                                            curvedDegreesErrorX, 4)
                            ) + " Degrees";

            String xDegreeErrorString =
                    "Vertical Angle To Cone: " +
                            String.valueOf(
                                    ConeDetectionTesting.truncate(curvedDegreesErrorX, 4)
                            ) + " Degrees";

            Point string1Position = new Point(10, 20);
            Point string2Position = new Point(10, 40);
            Point string3Position = new Point(10, 60);

            Scalar stringColor = new Scalar(255, 0, 0);

            Imgproc.putText(contourMat, distanceToConeString, string1Position, 1, 1, stringColor);
            Imgproc.putText(contourMat, yDegreeErrorString, string2Position, 1, 1, stringColor );
            Imgproc.putText(contourMat, xDegreeErrorString, string3Position, 1, 1, stringColor);




            // add text to contour Mat
            // Add the rayDistance, curvedXError, curvedYError
        }


        if (this.t != null) {
            t.addLine("Contours: " + listOfContours.size());
            t.addData("Frame Output (ms):", frameTimer.milliseconds());
            t.addLine("dt (ms): " + frameTimer.milliseconds());
            t.update();
        }




        switch (stages[stageNum]) {
            case FILTERED:
                return thresholdMat;
            case BLURRED:
                return blurredMat;
            case CONTOURS:
                return contourMat;
        }

        return contourMat;
    }

    public double getDegreesError() {
        return this.degreesError;
    }

}