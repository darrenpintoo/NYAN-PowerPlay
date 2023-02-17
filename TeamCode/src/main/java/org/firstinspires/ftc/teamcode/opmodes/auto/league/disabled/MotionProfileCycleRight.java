package org.firstinspires.ftc.teamcode.opmodes.auto.league.disabled;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.EncoderDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
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
@Autonomous(name = "Motion Profile Cycle Right")
@Disabled
public class MotionProfileCycleRight extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    ApriltagDetectionPipeline sleeveDetection;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);

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
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getParkingPosition());
            telemetry.update();
        }

        // scan sleeve

        // Initialize the robot

        robot.claw.setClawState(Claw.ClawStates.CLOSED);
        robot.claw.disableAutoClose();
        robot.claw.onCyclePassed(); // REMOVE THIS LINE IF IT CAUSES WEIRD BEHAVIOR
        waitForStart();

        // Notify subsystems before loop
        robot.postInit();

        if (isStopRequested()) return;

        // robot.drivetrain.enableAntiTip();
        MotionProfilingDrive robotDrivetrain = new MotionProfilingDrive(this, telemetry);
        EncoderDrive robotDrivetrainE = new EncoderDrive(this, telemetry);

        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ParkingPosition parkPosition = sleeveDetection.getParkingPosition();

        camera.stopStreaming();

        // robotDrivetrain.turnToIMUAngle(Math.toRadians(180));

        DriveConstants.MAX_VELOCITY = 35;
        DriveConstants.MAX_ACCELERATION = 30;

        robot.claw.setClawState(Claw.ClawStates.CLOSED);
        robot.pause(0.5);
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.GROUND_JUNCTION);
        robot.pause(0.5);
        robotDrivetrain.driveForward(-16);
        robotDrivetrainE.turnToIMUAngle(Math.toRadians(90));
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.LOW_JUNCTION);
        robot.pause(0.25);
        robotDrivetrain.driveForward(8);
        robot.pause(0.15);
        robot.lift.setOffset(-5);
        robot.pause(0.5);
        robot.claw.setClawState(Claw.ClawStates.OPENED);
        robot.pause(0.25);
        robot.lift.setOffset(0);
        robot.pause(0.25);
        robot.claw.setClawState(Claw.ClawStates.CLOSED);
        robotDrivetrain.driveForward(-8);
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.DEFAULT);
        robotDrivetrainE.turnToIMUAngle(-Math.toRadians(180));
        robotDrivetrain.driveForward(41);
        robot.claw.setClawState(Claw.ClawStates.OPENED);
        robotDrivetrain.driveForward(-5);
        robot.claw.setClawState(Claw.ClawStates.SLIGHTLY_OPENED);
        robotDrivetrainE.turnToIMUAngle(-Math.toRadians(90));
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.DEFAULT);
        robot.lift.setOffset(8);
        robotDrivetrain.driveForward(32);
        robot.lift.setOffset(4);
        robot.pause(0.5);
        robot.claw.setClawState(Claw.ClawStates.CLOSED);
        robot.pause(0.5);
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.LOW_JUNCTION);
        robotDrivetrain.driveForward(-40);
        robotDrivetrainE.turnToIMUAngle(-Math.toRadians(0));
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.MIDDLE_JUNCTION);
        robotDrivetrain.driveForward(12);
        robot.pause(0.5);
        robot.lift.setOffset(-7);
        robot.pause(0.5);
        robot.claw.setClawState(Claw.ClawStates.OPENED);
        robot.pause(0.1);
        robot.lift.setOffset(0);
        robot.pause(0.25);
        robot.claw.setClawState(Claw.ClawStates.SLIGHTLY_OPENED);
        robotDrivetrain.driveForward(-9);
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.DEFAULT);
        robotDrivetrainE.turnToIMUAngle(-Math.toRadians(-90));

/*        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.DEFAULT);
        robotDrivetrainE.turnToIMUAngle(Math.toRadians(90));
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.LOW_JUNCTION);
        robotDrivetrain.driveForward(40);
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.GROUND_JUNCTION);
        robot.lift.incrementOffset(5);
        robot.pause(0.5);
        robot.claw.setClawState(Claw.ClawStates.CLOSED);
        robot.pause(0.5);
        robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.LOW_JUNCTION);
        robotDrivetrain.driveForward(-15);
        robotDrivetrainE.turnToIMUAngle(Math.toRadians(0));
        robotDrivetrain.driveForward(9);
        robot.pause(0.25);
        robot.claw.setClawState(Claw.ClawStates.OPENED);
        robot.pause(0.5);
        robot.claw.setClawState(Claw.ClawStates.SLIGHTLY_OPENED);
        robotDrivetrain.driveForward(-9);
        robotDrivetrainE.turnToIMUAngle(-Math.toRadians(90));
        robotDrivetrainE.strafeRightFromBB(10);*/

        // sleep(5000);

        switch (parkPosition) {
            case LEFT:
                robotDrivetrain.driveForward(-35);
                break;
            case RIGHT:
                robotDrivetrain.driveForward(15);
                break;
            case CENTER:
                robotDrivetrain.driveForward(-10);
                break;
        }


//        robotDrivetrain.turnToIMUAngle(Math.toRadians(180));
        // robot.drivetrain.enableAntiTip();

        robot.persistData();

    }
}
