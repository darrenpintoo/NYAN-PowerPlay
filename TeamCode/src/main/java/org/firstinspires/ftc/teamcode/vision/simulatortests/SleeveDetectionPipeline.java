package org.firstinspires.ftc.teamcode.vision.simulatortests;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetectionPipeline extends OpenCvPipeline {

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private static final Point SLEEVE_TOP_LEFT_ANCHOR_POINT = new Point(145, 168);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    // Lower and upper boundaries for colors
    public static Scalar lowerMagenta = new Scalar(0, 0, 0);
    public static Scalar upperMagenta = new Scalar(255, 255, 255);
    public static Scalar lowerYellow  = new Scalar(0, 0, 0);
    public static Scalar upperYellow = new Scalar(255, 255, 255);
    public static Scalar lowerGreen = new Scalar(0, 0, 0);
    public static Scalar upperGreen = new Scalar(255, 255, 255);

    // Color definitions
    private final Scalar MAGENTA = new Scalar(255, 0, 255);
    private final Scalar GREEN    = new Scalar(0, 255, 255);
    private final Scalar YELLOW  = new Scalar(255, 255, 0);

    // Percent and mat definitions
    private double magentaPercent;
    private double yellowPercent;
    private double greenPercent;

    private Mat magentaMatrix = new Mat();
    private Mat yellowMatrix = new Mat();
    private Mat greenMatrix = new Mat();

    private Mat blurredMatrix = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point(SLEEVE_TOP_LEFT_ANCHOR_POINT.x, SLEEVE_TOP_LEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(SLEEVE_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH, SLEEVE_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    private int stageNum = 0;

    private enum Stage
    {
        BLURRED,
        MAGENTA,
        YELLOW,
        GREEN,
        INPUT
    }

    Stage[] stages = Stage.values();

    Telemetry t;

    public SleeveDetectionPipeline(Telemetry t) {
        this.t = t;
    }
    public SleeveDetectionPipeline() {}

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

        // Memory cleanup
        blurredMatrix.release();
        yellowMatrix.release();
        greenMatrix.release();
        magentaMatrix.release();

        // Noise reduction
        Imgproc.cvtColor(input, blurredMatrix, Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(blurredMatrix, blurredMatrix, new Size(5, 5));
        // blurredMatrix = blurredMatrix.submat(new Rect(sleeve_pointA, sleeve_pointB));

        // Apply Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatrix, blurredMatrix, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMatrix, lowerMagenta, upperMagenta, magentaMatrix);
        Core.inRange(blurredMatrix, lowerYellow, upperYellow, yellowMatrix);
        Core.inRange(blurredMatrix, lowerGreen, upperGreen, greenMatrix);

        // Gets color specific values
        magentaPercent = Core.countNonZero(magentaMatrix);
        yellowPercent = Core.countNonZero(yellowMatrix);
        greenPercent = Core.countNonZero(greenMatrix);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(magentaPercent, Math.max(greenPercent, yellowPercent));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == yellowPercent) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
            Imgproc.rectangle(
                    yellowMatrix,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        } else if (maxPercent == greenPercent) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    GREEN,
                    2
            );
            Imgproc.rectangle(
                    greenMatrix,
                    sleeve_pointA,
                    sleeve_pointB,
                    GREEN,
                    2
            );
        } else if (maxPercent == magentaPercent) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
            Imgproc.rectangle(
                    magentaMatrix,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        }


        if (this.t != null) {
            t.addData("Current Stage Index: ", stageNum);
            t.update();
        }

        switch (stages[stageNum]) {
            case BLURRED:
                return blurredMatrix;
            case MAGENTA:
                return magentaMatrix;
            case YELLOW:
                return yellowMatrix;
            case GREEN:
                return greenMatrix;
            case INPUT:
                return input;
        }

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}