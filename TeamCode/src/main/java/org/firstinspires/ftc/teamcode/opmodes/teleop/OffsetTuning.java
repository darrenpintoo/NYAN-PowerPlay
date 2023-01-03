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
@TeleOp(name = "Offset Tuning")
@Config
public class OffsetTuning extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    public static int OFFSET = 0;
    public static double F = 0;

    @Override
    public void runOpMode() {

        // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        // Initialize the robot
        robot.init(hardwareMap, telemetry);

        waitForStart();

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

            robot.lift.setOffset(OFFSET);
/*
            if (currentFrameGamepad1.x) {
                robot.drivetrain.disableAntiTip();
            }*/

            robot.drivetrain.setWeightedDrivePower(1 - gamepad1.left_trigger);

            double frameTime = robot.update();

            telemetry.addData("Robot Tilt : ", robot.internalIMU.getCurrentFrameTilt());
            telemetry.addData("Frame Time: ", frameTime);
            telemetry.addData("Refresh Rate: ", (frameTime != 0) ? (1000 / frameTime) : "inf");
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
    }
}
