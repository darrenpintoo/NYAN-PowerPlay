package org.firstinspires.ftc.teamcode.opmodes.auto.lifttesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.robot.movement.EncoderDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MotionProfilingDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.simulatortests.ApriltagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.simulatortests.ParkingPosition;
import org.openftc.easyopencv.OpenCvCamera;

/**
 * Example teleop code for a basic mecanum drive
 */
@Autonomous(name = "FFLift Test")
@Config
@Disabled
public class FFLiftTesting extends LinearOpMode {

    public static double liftFF = 0;
    public static int state = 0;
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
        });*/

/*        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getParkingPosition());
            telemetry.update();
        }*/

        // scan sleeve

        // Initialize the robot

        waitForStart();

        // Notify subsystems before loop
        robot.postInit();

        if (isStopRequested()) return;

        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        while (opModeIsActive())  {
            telemetry.update();

            switch (state) {
                case (0): {
                    robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.DEFAULT);
                }
                case (1) : {
                    robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.LOW_JUNCTION);
                }
                case (2) : {
                    robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.MIDDLE_JUNCTION);
                }
                case (3) : {
                    robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.HIGH_JUNCTION);
                }
            }

            robot.lift.driveLiftFromGamepad(
                    0,
                    FFLiftTesting.liftFF
                    );
            robot.update();
        }
    }
}
