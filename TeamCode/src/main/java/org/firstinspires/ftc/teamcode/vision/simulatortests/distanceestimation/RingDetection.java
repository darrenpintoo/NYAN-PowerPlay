package org.firstinspires.ftc.teamcode.vision.simulatortests.distanceestimation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Disabled
public class RingDetection {// extends OpenCvPipeline {
    /*
    private static final Mat EMPTY_MAT = new Mat();

    public static Scalar HSV_LOWER = new Scalar(6, 100, 50);
    public static Scalar HSV_UPPER = new Scalar(20, 256, 240);

    public static double MIN_AREA = 100;
    public static double MIN_CIRCULARITY = 0.5;
    public static double MIN_INERTIA_RATIO = 0.06;
    public static double MIN_CONVEXITY = 0.9;

    public static double RING_HEIGHT = 1.0;

    // This point is used to adjust
    public static Point3 relPoint = new Point3();
    public static Mat RVEC = new MatOfDouble(2.035399804462858, 1.041272664535304, -0.5618792670272742);
    public static Mat TVEC = new MatOfDouble(-11.36860632737538, -1.7354899166739, 31.81319062907322);
    public static CalibrationParameters CALIB_PARAMS = new CalibrationParameters(1010.7, 1010.7,
            638.2174071943463, 340.2355228022204, 0.2126095014133333, -1.001796829192392,
            0.000504850246603286, -0.0001913578573509387, 1.419425306492814);

    public static int DENOISING_ITERS = 3;

    public static int stageNum = 0;

    private Mat STRUCT_ELEMENT;

    private List<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    private MatOfPoint2f contour2f = new MatOfPoint2f();
    private MatOfInt hullIndices = new MatOfInt();
    private MatOfPoint convexHull = new MatOfPoint();
    private MatOfPoint2f undistortedPoints = new MatOfPoint2f();

    private Mat rotation = new Mat(3, 3, CvType.CV_64FC1);
    private Mat uvPoint = new Mat(3, 1, CvType.CV_64FC1);
    private Mat lhs = new Mat();
    private Mat rhs = new Mat();
    private Mat pointMat = new Mat();

    private Mat hsvMat = new Mat();
    private Mat rangeMat = new Mat();
    private Mat denoisedMat = new Mat();
    private Mat outputMat = new Mat();

    private Telemetry telemetry;

    public RingLocalizer(Telemetry telemetry) {
        super();
        this.telemetry = telemetry;
        STRUCT_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    }

    @Override
    public void onViewportTapped() {
        stageNum++;
    }

    private double calculateCircularity(MatOfPoint contour, double area) {
        contour2f.fromArray(contour.toArray());
        double perimeter = Imgproc.arcLength(contour2f, true);
        return 4 * Math.PI * area / (perimeter * perimeter);
    }

    private double calculateInertiaRatio(Moments moments) {
        double denominator = Math.hypot(2 * moments.mu11, moments.mu20 - moments.mu02);
        double epsilon = 1e-2;

        if (denominator > epsilon) {
            double cosmin = (moments.mu20 - moments.mu02) / denominator;
            double sinmin = 2 * moments.mu11 / denominator;
            double cosmax = -cosmin;
            double sinmax = -sinmin;
            double component = 0.5 * (moments.mu20 + moments.mu02);
            double imin = component - 0.5 * (moments.mu20 - moments.mu02) * cosmin - moments.mu11 * sinmin;
            double imax = component - 0.5 * (moments.mu20 - moments.mu02) * cosmax - moments.mu11 * sinmax;
            return imin / imax;
        } else {
            return 1;
        }
    }

    private double calculateConvexity(MatOfPoint contour, double area) {
        Imgproc.convexHull(contour, hullIndices);
        Point[] contourArray = contour.toArray();
        convexHull.fromList(
                hullIndices.toList().stream().map(idx -> contourArray[idx]).collect(Collectors.toList()));
        double hullArea = Imgproc.contourArea(convexHull);
        return area / hullArea;
    }

    private Point calculateCentroid(Moments moments) {
        double x = moments.m10 / moments.m00;
        double y = moments.m01 / moments.m00;
        return new Point(x, y);
    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(outputMat);

        contours.clear();

        // convert to hsv
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        // hsv filtering
        Core.inRange(hsvMat, HSV_LOWER, HSV_UPPER, rangeMat);

        // basic binary mat denoising
        Imgproc.erode(rangeMat, denoisedMat, STRUCT_ELEMENT, new Point(-1, -1), DENOISING_ITERS);
        Imgproc.dilate(denoisedMat, denoisedMat, STRUCT_ELEMENT, new Point(-1, -1), DENOISING_ITERS);

        // find contours in binary mat
        Imgproc.findContours(denoisedMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(outputMat, contours, -1, new Scalar(0, 255, 0), 3);

        // perform per-contour analysis
        for (MatOfPoint contour : contours) {
            // initial criterion, bbox height >= width
            Rect boundingBox = Imgproc.boundingRect(contour);

            if (boundingBox.width < boundingBox.height) {
                continue;
            }

            // further criteria based on
            // https://github.com/opencv/opencv/blob/68d15fc62edad980f1ffa15ee478438335f39cc3/modules/features2d/src/blobdetector.cpp
            // explaination at https://learnopencv.com/blob-detection-using-opencv-python-c/
            Moments moments = Imgproc.moments(contour);

            double area = moments.m00;

            if (area < MIN_AREA) {
                continue;
            }

            double circularity = calculateCircularity(contour, area);

            if (circularity < MIN_CIRCULARITY) {
                continue;
            }

            double inertiaRatio = calculateInertiaRatio(moments);

            if (inertiaRatio < MIN_INERTIA_RATIO) {
                continue;
            }

            double convexity = calculateConvexity(contour, area);

            if (convexity < MIN_CONVEXITY) {
                continue;
            }

            Point centroid = calculateCentroid(moments);

            // draw centroid
            Imgproc.circle(outputMat, centroid, 6, new Scalar(0, 0, 255), Imgproc.FILLED);

            // undistort centroid based on camera parameters and reproject with identity matrix
            Mat cameraMatrix = CALIB_PARAMS.getCameraMatrix();
            MatOfDouble distCoeffs = CALIB_PARAMS.getDistCoeffs();

            Calib3d.undistortPoints(new MatOfPoint2f(centroid), undistortedPoints, cameraMatrix, distCoeffs);
            Point undistortedPoint = undistortedPoints.toArray()[0];

            // rotation matrix from rotation vector
            Calib3d.Rodrigues(RVEC, rotation);

            // this based on the projection math in https://stackoverflow.com/questions/12299870 (the question)
            uvPoint.put(0, 0, new double[] { undistortedPoint.x, undistortedPoint.y, 1 });

            // camera matrix not involved as undistortPoints reprojects with identity camera matrix
            Core.gemm(rotation.inv(), uvPoint, 1, EMPTY_MAT, 0, lhs);
            Core.gemm(rotation.inv(), TVEC, 1, EMPTY_MAT, 0, rhs);

            double s = (RING_HEIGHT / 2 + rhs.get(2, 0)[0]) / lhs.get(2, 0)[0];
            Core.scaleAdd(uvPoint, -s, TVEC, pointMat);
            Core.gemm(rotation.inv(), pointMat, -1, EMPTY_MAT, 0, pointMat);

            Imgproc.drawContours(outputMat, Collections.singletonList(contour), -1, new Scalar(255, 0, 0), 3);

            double[] buff = new double[3];
            pointMat.get(0, 0, buff);
            Point3 point = new Point3(buff);
            Point3 adjustedPoint = new Point3(point.x + relPoint.x, point.y + relPoint.y, point.z + relPoint.z);

            telemetry.addData("point", point);
            telemetry.update();
        }

        Mat[] mats = new Mat[] { outputMat, denoisedMat, input };
        return mats[stageNum % mats.length];
    }

     */
}