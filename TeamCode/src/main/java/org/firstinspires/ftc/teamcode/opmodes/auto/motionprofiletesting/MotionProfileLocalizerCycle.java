package org.firstinspires.ftc.teamcode.opmodes.auto.motionprofiletesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utilities.localizer.RoadrunnerLocalizer;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.EncoderDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MotionProfileLocalizerLineDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MotionProfileLocalizerLineDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MotionProfilingDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.simulatortests.ApriltagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.simulatortests.ParkingPosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * Example teleop code for a basic mecanum drive
 */
@Autonomous(name = "Motion Profile Localizer Cycle Left")
public class MotionProfileLocalizerCycle extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    ApriltagDetectionPipeline sleeveDetection;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);
/*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
/*
        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getParkingPosition());
            telemetry.update();
        }*/

        // scan sleeve

        // Initialize the robot

        robot.claw.setClawState(Claw.ClawStates.CLOSED);
        robot.claw.disableAutoClose();
        waitForStart();

        // Notify subsystems before loop
        robot.postInit();

        if (isStopRequested()) return;

        // robot.drivetrain.enableAntiTip();
        MotionProfileLocalizerLineDrive motionProfileLocalizerLineDrive = new MotionProfileLocalizerLineDrive(this, telemetry);
        MotionProfilingDrive motionProfilingDrive = new MotionProfilingDrive(this, telemetry);
        EncoderDrive robotDrivetrainE = new EncoderDrive(this, telemetry);

        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        ParkingPosition parkPosition = ParkingPosition.CENTER;
        robot.localizer.setPoseEstimate(new Pose2d(-34, -58, Math.toRadians(-180)));
        robot.internalIMU.setHeadingOffset(Math.toRadians(-180));
/*
        robot.update();
        robot.pause(2);
        motionProfileLocalizerLineDrive.forwardX(50);
        motionProfileLocalizerLineDrive.strafeY(10);
        motionProfileLocalizerLineDrive.strafeYToPoseLinearHeading(new Pose2d(50, -70, Math.toRadians(90)));
*/

        motionProfileLocalizerLineDrive.forwardXToPose(new Pose2d(-18, -36, Math.toRadians(-180)));
        robotDrivetrainE.turnToIMUAngle(Math.toRadians(-90));
        motionProfileLocalizerLineDrive.strafeYToPoseLinearHeading(new Pose2d(-18, -28, Math.toRadians(-90)));
        // Place Cone
        motionProfileLocalizerLineDrive.strafeYToPoseLinearHeading(new Pose2d(-18, -36, Math.toRadians(-90)));
        motionProfileLocalizerLineDrive.strafeYToPoseLinearHeading(new Pose2d(23, -36, Math.toRadians(-90)));
        motionProfileLocalizerLineDrive.strafeYToPoseLinearHeading(new Pose2d(18, -36, Math.toRadians(-90)));
        robotDrivetrainE.turnToIMUAngle(Math.toRadians(90));
        motionProfileLocalizerLineDrive.strafeYToPoseLinearHeading(new Pose2d(18, -58, Math.toRadians(-90)));
        // Pick up Cone
        motionProfileLocalizerLineDrive.strafeYToPoseLinearHeading(new Pose2d(18, -36, Math.toRadians(-90)));
        robotDrivetrainE.turnToIMUAngle(-Math.toRadians(135));
        motionProfilingDrive.driveForward(20);
        // Place Cone
        motionProfilingDrive.driveForward(-20);
        robotDrivetrainE.turnToIMUAngle(Math.toRadians(90));
        motionProfileLocalizerLineDrive.strafeYToPoseLinearHeading(new Pose2d(18, -58, Math.toRadians(-90)));
        // Pick up Cone
        motionProfileLocalizerLineDrive.strafeYToPoseLinearHeading(new Pose2d(18, -36, Math.toRadians(-90)));
        motionProfilingDrive.driveForward(20);
        // Place Cone
        motionProfilingDrive.driveForward(-20);
        // Park


/*
        robot.pause(0.5);
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.GROUND_JUNCTION);
        robot.pause(0.5);
        motionProfileLocalizerLineDrive.forwardX(-16);
        robotDrivetrainE.turnToIMUAngle(-Math.toRadians(90));
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.LOW_JUNCTION);
        robot.pause(0.25);
        motionProfileLocalizerLineDrive.strafeY( 8);
        robot.pause(0.15);
        robot.lift.setOffset(-5);
        robot.pause(0.5);
        robot.claw.setClawState(Claw.ClawStates.OPENED);
        robot.pause(0.25);
        robot.lift.setOffset(0);
        robot.pause(0.25);
        robot.claw.setClawState(Claw.ClawStates.CLOSED);
        motionProfileLocalizerLineDrive.strafeY(-8);
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.DEFAULT);
        robotDrivetrainE.turnToIMUAngle(Math.toRadians(180));
        motionProfileLocalizerLineDrive.forwardX(41);
        robot.claw.setClawState(Claw.ClawStates.OPENED);
        motionProfileLocalizerLineDrive.forwardX(-5);
        robot.claw.setClawState(Claw.ClawStates.SLIGHTLY_OPENED);
        robotDrivetrainE.turnToIMUAngle(Math.toRadians(90));
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.LOW_JUNCTION);
        motionProfileLocalizerLineDrive.strafeY(29);
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.GROUND_JUNCTION);
        robot.lift.incrementOffset(5);
        robot.pause(0.5);
        robot.claw.setClawState(Claw.ClawStates.CLOSED);
        robot.pause(0.5);
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.LOW_JUNCTION);
        motionProfileLocalizerLineDrive.strafeY(-40);
        robotDrivetrainE.turnToIMUAngle(Math.toRadians(0));
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.MIDDLE_JUNCTION);
        motionProfileLocalizerLineDrive.forwardX(8);
        robot.pause(0.1);
        robot.lift.setOffset(-10);
        robot.pause(0.5);
        robot.claw.setClawState(Claw.ClawStates.OPENED);
        robot.pause(0.1);
        robot.lift.setOffset(0);
        robot.pause(0.25);
        robot.claw.setClawState(Claw.ClawStates.SLIGHTLY_OPENED);
        motionProfileLocalizerLineDrive.forwardX(-8);
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.DEFAULT);
        robotDrivetrainE.turnToIMUAngle(Math.toRadians(90));*/
/*        switch (parkPosition) {
            case LEFT:
                motionProfileLocalizerLineDrive.forwardX(30);
                break;
            case RIGHT:
                motionProfileLocalizerLineDrive.forwardX(-15);
                break;
            case CENTER:
                motionProfileLocalizerLineDrive.forwardX(10);
                break;
        }*/

//        motionProfileLocalizerLineDrive.turnToIMUAngle(Math.toRadians(180));
        // robot.drivetrain.enableAntiTip();

        robot.persistData();

    }
}
