package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utilities.robot.PersistentData;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Lift;

/**
 * Example teleop code for a basic mecanum drive
 */
@TeleOp(name = "Main Mecanum Drive")
@Config
public class MainMecanumDrive extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    public static double F = 0;
    @Override
    public void runOpMode() {

        // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        // Initialize the robot
        robot.init(hardwareMap, telemetry);

        waitForStart();

        if (PersistentData.startPose != null) {
            robot.localizer.setPoseEstimate(PersistentData.startPose);
            robot.internalIMU.setHeadingOffset(PersistentData.startPose.getHeading());
        }
        // Notify subsystems before loop
        robot.postInit();

        if (isStopRequested()) return;

        // Initialize variables for loop
        Gamepad currentFrameGamepad1 = new Gamepad();
        Gamepad currentFrameGamepad2 = new  Gamepad();

        Gamepad previousFrameGamepad1 = new Gamepad();
        Gamepad previousFrameGamepad2 = new Gamepad();

        // robot.drivetrain.enableAntiTip();

        // Declare state variables
        boolean intakeOn = false;
        boolean intakeDirection = false;

        boolean robotCentric = true;

        boolean fieldCentricAutomatedTurning = false;

        robot.update();
        robot.claw.enableAutoClose();
        robot.drivetrain.disableAntiTip();

        while(opModeIsActive()) {

            // Retain information about the previous frame's gamepad
            try {
                previousFrameGamepad1.copy(currentFrameGamepad1);
                previousFrameGamepad2.copy(currentFrameGamepad2);

                currentFrameGamepad1.copy(gamepad1);
                currentFrameGamepad2.copy(gamepad2);
            } catch (RobotCoreException ignored) {

            }

            // Handle Intake State
/*            if (currentFrameGamepad1.a != previousFrameGamepad1.a && currentFrameGamepad1.a) {
                intakeOn = !intakeOn;
                intakeDirection = false;
            }

            if (currentFrameGamepad1.b != previousFrameGamepad1.b && currentFrameGamepad1.b) {
                intakeOn = !intakeOn;
                intakeDirection = true;
            }*/

/*           robot.drivetrain.robotCentricDriveFromGamepad(
                    0,
                    0,
                    F
            );*/
            if (currentFrameGamepad1.right_trigger > 0.2 && previousFrameGamepad1.right_trigger < 0.2) {
                intakeOn = !intakeOn;
                intakeDirection = false;
            }

            if (currentFrameGamepad1.left_trigger > 0.2 && previousFrameGamepad1.left_trigger < 0.2) {
                intakeOn = !intakeOn;
                intakeDirection = true;
            }

            // Handle Claw State
            if (currentFrameGamepad2.b) {
                robot.claw.setClawState(Claw.ClawStates.CLOSED);
            } else if (currentFrameGamepad2.a) {
                robot.claw.setClawState(Claw.ClawStates.OPENED);
            } else if (currentFrameGamepad2.y) {
                robot.claw.setClawState(Claw.ClawStates.SLIGHTLY_OPENED);
            }

            if (currentFrameGamepad1.y && previousFrameGamepad1.y != currentFrameGamepad1.y) {
                robotCentric = !robotCentric;
            }

            if (currentFrameGamepad1.x && previousFrameGamepad1.x != currentFrameGamepad1.x) {
                fieldCentricAutomatedTurning = !fieldCentricAutomatedTurning;
            }
/*
            // Handle Drivetrain
            if (fieldCentricAutomatedTurning) {
                robot.drivetrain.fieldCentricRotationPIDFromGamepad(
                        currentFrameGamepad1.left_stick_y,
                        currentFrameGamepad1.left_stick_x,
                        currentFrameGamepad1.right_stick_y,
                        currentFrameGamepad1.right_stick_x
                );
            } else {
                if (robotCentric) {
                    robot.drivetrain.robotCentricDriveFromGamepad(
                            currentFrameGamepad1.left_stick_y * 0.75,
                            currentFrameGamepad1.left_stick_x * 0.75,
                            currentFrameGamepad1.right_stick_x * 0.5
                    );
                } else {
                    robot.drivetrain.fieldCentricDriveFromGamepad(
                            currentFrameGamepad1.left_stick_y * 0.5,
                            currentFrameGamepad1.left_stick_x * 0.5,
                            currentFrameGamepad1.right_stick_x * 0.5
                    );
                }
            }*/

            robot.drivetrain.robotCentricDriveFromGamepad(
                    currentFrameGamepad1.left_stick_y,
                    currentFrameGamepad1.left_stick_x,
                    currentFrameGamepad1.right_stick_x * 0.55
            );

            // Handle Manual Lift State
            robot.lift.driveLiftFromGamepad(
                    currentFrameGamepad2.left_trigger,
                    currentFrameGamepad2.right_trigger
            );

            if (currentFrameGamepad2.dpad_up) {
                robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.HIGH_JUNCTION);
            } else if (currentFrameGamepad2.dpad_left) {
                robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.MIDDLE_JUNCTION);
            } else if (currentFrameGamepad2.dpad_right) {
                robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.LOW_JUNCTION);
            } else if (currentFrameGamepad2.dpad_down) {
                robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.GROUND_JUNCTION);
            } else if (currentFrameGamepad2.x) {
                robot.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.DEFAULT);
            }

            if (currentFrameGamepad2.right_bumper && previousFrameGamepad2.right_bumper != currentFrameGamepad2.right_bumper) {
                robot.lift.incrementOffset(1);
            } else if (currentFrameGamepad2.left_bumper && previousFrameGamepad2.left_bumper != currentFrameGamepad2.left_bumper) {
                robot.lift.incrementOffset(-1);
            }

            if (currentFrameGamepad2.right_stick_button && previousFrameGamepad2.right_stick_button != currentFrameGamepad2.right_stick_button) {
                robot.lift.resetEncoderPosition();
            }


/*
            if (currentFrameGamepad1.x) {
                robot.drivetrain.disableAntiTip();
            }*/

            robot.drivetrain.setWeightedDrivePower(1 - gamepad1.left_trigger);

            double frameTime = robot.update();

            telemetry.addData("Robot Tilt : ", robot.internalIMU.getCurrentFrameTilt());
            telemetry.addData("Frame Time: ", frameTime);
            telemetry.addData("Refresh Rate: ", (frameTime != 0) ? (1000 / frameTime) : "inf");
            telemetry.addData("Increment offset: ", robot.lift.getOffset());
            // telemetry.addData("IMU orientation: ", robot.internalIMU.getCurrentFrameOrientation());
            // telemetry.addData("CCW IMU orientation: ", robot.internalIMU.getCurrentFrameHeadingCCW());
            // telemetry.addData("CW IMU orientation: ", robot.internalIMU.getCurrentFrameHeadingCW());
            // telemetry.addData("Robot Tilt Acceleration y: ", robot.internalIMU.getCurrentFrameVelocity().yRotationRate);
            // telemetry.addData("Joystick Orientation: ", Math.atan2(-currentFrameGamepad1.right_stick_x, -currentFrameGamepad1.right_stick_y));
            // telemetry.addData(
            //        "Updated Joystick Orientation: ",
            //        Math.atan2(currentFrameGamepad1.right_stick_y, currentFrameGamepad1.right_stick_x) - Math.PI / 2
            // );
            // ^ https://www.desmos.com/calculator/jp45vcfcbt



            telemetry.update();


        }

        robot.clearPersistData();
    }

}
