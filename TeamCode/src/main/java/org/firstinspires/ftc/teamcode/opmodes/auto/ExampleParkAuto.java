package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.EncoderDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.vision.simulatortests.SleeveDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * Example teleop code for a basic mecanum drive
 */
@TeleOp(name = "Example Park Auto")
public class ExampleParkAuto extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    SleeveDetectionPipeline sleeveDetection = new SleeveDetectionPipeline();
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetectionPipeline();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        // scan sleeve

        // Initialize the robot

        waitForStart();

        // Notify subsystems before loop
        robot.postInit();

        if (isStopRequested()) return;

        EncoderDrive robotDrivetrain = new EncoderDrive();

        SleeveDetectionPipeline.ParkingPosition parkPosition = sleeveDetection.getPosition();

        robotDrivetrain.driveForwardFromInchesBB(20);

        switch (parkPosition) {
            case LEFT:
                robotDrivetrain.turnToAbsoluteAngle(90);
                robotDrivetrain.driveForwardFromInchesBB(20);
                break;
            case RIGHT:
                robotDrivetrain.turnToAbsoluteAngle(270);
                robotDrivetrain.driveForwardFromInchesBB(20);
                break;
            case CENTER:
                break;
        }

        // robot.drivetrain.enableAntiTip();


    }
}
