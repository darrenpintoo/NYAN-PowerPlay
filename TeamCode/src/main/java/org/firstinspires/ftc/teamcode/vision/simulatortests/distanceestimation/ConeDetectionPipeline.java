package org.firstinspires.ftc.teamcode.vision.simulatortests.distanceestimation;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Algorithm;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


public class ConeDetectionPipeline extends OpenCvPipeline {


    public final double CAMERA_FOV = CameraConstants.fovXDeg;


    private final double CUBE_WIDTH = 4;

    Telemetry t;
    ElapsedTime frameTimer = new ElapsedTime();

    public Scalar lowerBound = new Scalar(82.2, 94.9, 83.6); // new Scalar(25.5, 80.8, 131.8);
    public Scalar upperBound = new Scalar(223.8, 168.6, 165.8);// new Scalar(46.8, 255, 255);

    private Mat hsvMat       = new Mat();
    private Mat blurredMat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat contourMat = new Mat();

    private final ArrayList<MatOfPoint> listOfContours = new ArrayList<>();

    private int stageNum = 0;

    private enum Stage
    {
        FILTERED,
        BLURRED,
        CONTOURS
    }

    Stage[] stages = Stage.values();

    Mat kernel1 = Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, new Size(3, 3));
    Mat kernel2 = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(3, 3));

    Size blurSize = new Size(10, 10);

    public ConeDetectionPipeline(Telemetry t) {
        this.t = t;
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
        frameTimer.reset();

        // t.addLine(input.size().width + " x " + input.size().height);
        hsvMat.release();
        thresholdMat.release();
        contourMat.release();
        listOfContours.clear();

        t.addData("Release Time: ", frameTimer.milliseconds());


        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvMat, lowerBound, upperBound, thresholdMat);
        t.addData("Threshold Time: ", frameTimer.milliseconds());


        Imgproc.morphologyEx(thresholdMat, thresholdMat, Imgproc.MORPH_ERODE, kernel1);
        Imgproc.morphologyEx(thresholdMat, thresholdMat, Imgproc.MORPH_DILATE, kernel2);

        Imgproc.blur(thresholdMat, blurredMat, blurSize);

        t.addData("Post Processing Time: ", frameTimer.milliseconds());
        // Imgproc.GaussianBlur(thresholdMat, blurredMat, new Size(11, 11), 0, 0);
        Imgproc.findContours(blurredMat, listOfContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        input.copyTo(contourMat);
        // Imgproc.drawContours(contourMat, listOfContours, -1, new Scalar(0, 0, 255), 2, 8);

        t.addData("Math Time: ", frameTimer.milliseconds());

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
            Imgproc.rectangle(contourMat, boundingBox, new Scalar(0, 255 , 255));

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
            double hypotenuseY = rayDistance / Math.cos(Math.toRadians(curvedDegreesErrorY)); // have angle and adj, need hyp
            double hypotenuseX = Math.abs(rayDistance * Math.tan(Math.toRadians(curvedDegreesErrorX))); // have angle and adj, need opp

            double distanceToCamera = Math.cbrt(Math.pow(hypotenuseX, 3)  + Math.pow(hypotenuseY, 3) + Math.pow(rayDistance, 3)); // inaccurate
            // </ignore>


            if (this.t != null) {
                t.addData("X Degrees Error: ", curvedDegreesErrorX);
                t.addData("Y Degrees Error: ", curvedDegreesErrorY);

                t.addData("Depth (X): ", depthX);
                t.addData("Depth (Y): ", depthY);

                t.addData("Ray Distance: ", rayDistance);

                t.addData("Hypotenuse Y: ", hypotenuseY);
                t.addData("Hypotenuse X: ", hypotenuseX);
            }
            // t.addData("Distance: ", distanceToCamera);


        }


        t.addLine("Contours: " + listOfContours.size());
        t.addLine("dt (s): " + frameTimer.milliseconds());
        t.update();




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

}