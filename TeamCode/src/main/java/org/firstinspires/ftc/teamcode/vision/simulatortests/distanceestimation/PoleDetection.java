package org.firstinspires.ftc.teamcode.vision.simulatortests.distanceestimation;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


public class PoleDetection extends OpenCvPipeline {

    Telemetry t;

    public Scalar lowerBound = new Scalar(14.2, 17, 213.9); // new Scalar(170, 137.4, 80.8); // ;
    public Scalar upperBound = new Scalar(32.6, 109.1, 255); // new Scalar(255, 255, 102); // ;

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

    public PoleDetection(Telemetry t) {
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

        long startComputationTime = System.nanoTime();
        hsvMat.release();
        thresholdMat.release();
        contourMat.release();
        listOfContours.clear();


        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);


        Core.inRange(hsvMat, lowerBound, upperBound, thresholdMat);

        Imgproc.blur(thresholdMat, blurredMat, new Size(10, 10));
        // Imgproc.GaussianBlur(thresholdMat, blurredMat, new Size(11, 11), 0, 0);
        Imgproc.findContours(blurredMat, listOfContours, new Mat(), Imgproc.RETR_TREE , Imgproc.CHAIN_APPROX_SIMPLE);

        input.copyTo(contourMat);
        // Imgproc.drawContours(contourMat, listOfContours, -1, new Scalar(0, 0, 255), 2, 8);

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

            }

            Rect boundingBox = Imgproc.boundingRect(largestContour);
            Imgproc.rectangle(contourMat, boundingBox, new Scalar(0, 0, 255));


            t.addLine("Area of largest: " + largestContourArea);

            t.addLine("Dimension: " + boundingBox.width + " by " + boundingBox.height);
            t.addLine("Ratio: " + (double) boundingBox.width/boundingBox.height);


        }


        t.addLine("Number of contours: " + listOfContours.size());
        t.addLine("End Computation Time: " + (System.nanoTime() - startComputationTime));
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