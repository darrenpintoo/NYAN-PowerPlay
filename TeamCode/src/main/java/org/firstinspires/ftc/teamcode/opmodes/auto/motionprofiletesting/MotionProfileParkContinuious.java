package org.firstinspires.ftc.teamcode.opmodes.auto.motionprofiletesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.EncoderDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MotionProfilingDrive;
import org.firstinspires.ftc.teamcode.vision.simulatortests.ApriltagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

/**
 * Example teleop code for a basic mecanum drive
 */
@Disabled
@Autonomous(name = "Motion Profile Park Continuious")
public class MotionProfileParkContinuious extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    ApriltagDetectionPipeline sleeveDetection;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);

/*        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new ApriltagDetectionPipeline();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getParkingPosition());
            telemetry.update();
        }*/

        // scan sleeve

        // Initialize the robot

        waitForStart();

        // Notify subsystems before loop
        robot.postInit();

        if (isStopRequested()) return;

        MotionProfilingDrive robotDrivetrain = new MotionProfilingDrive(this, telemetry);
        EncoderDrive robotDrivetrainE = new EncoderDrive(this, telemetry);

        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // robotDrivetrain.turnToIMUAngle(Math.toRadians(180));



        while (opModeIsActive()) {
            robotDrivetrain.driveForward(40);
            robotDrivetrainE.turnToIMUAngle(Math.toRadians(180));
            robotDrivetrain.driveForward(40);
            robotDrivetrainE.turnToIMUAngle(0);
        }
//        robotDrivetrain.turnToIMUAngle(Math.toRadians(180));
        // robot.drivetrain.enableAntiTip();


    }
}
