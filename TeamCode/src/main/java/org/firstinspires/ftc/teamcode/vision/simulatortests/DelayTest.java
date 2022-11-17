package org.firstinspires.ftc.teamcode.vision.simulatortests;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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


public class DelayTest extends OpenCvPipeline {

    Telemetry t;

    public DelayTest(Telemetry t) {
        this.t = t;
    }

    ElapsedTime timer = new ElapsedTime();

    @Override
    public Mat processFrame(Mat input) {

        this.t.addData("Current Frame Time: ", this.timer.seconds());
        this.t.update();

        return input;
    }
}