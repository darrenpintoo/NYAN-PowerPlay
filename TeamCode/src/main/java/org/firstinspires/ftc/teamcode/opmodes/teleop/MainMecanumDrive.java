package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;

/**
 * Example teleop code for a basic mecanum drive
 */
@TeleOp(name = "Main Mecanum Drive")
public class MainMecanumDrive extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    GeneralPIDController pid = new GeneralPIDController(5, 0, 2, 0);

    @Override
    public void runOpMode() {

        // Initialize the robot
        robot.init(hardwareMap, telemetry);

        waitForStart();

        // Notify subsystems before loop
        robot.postInit();

        if (isStopRequested()) return;

        // Initialize variables for loop
        Gamepad currentFrameGamepad1 = new Gamepad();
        Gamepad currentFrameGamepad2 = new Gamepad();

        Gamepad previousFrameGamepad1 = new Gamepad();
        Gamepad previousFrameGamepad2 = new Gamepad();

        // robot.drivetrain.enableAntiTip();

        boolean intakeOn = false;
        boolean intakeDirection = false;
        robot.update();

        while(opModeIsActive()) {

            // Retain information about the previous frame's gamepad
            try {
                previousFrameGamepad1.copy(currentFrameGamepad1);
                previousFrameGamepad2.copy(currentFrameGamepad2);

                currentFrameGamepad1.copy(gamepad1);
                currentFrameGamepad2.copy(gamepad2);
            } catch (RobotCoreException ignored) {

            }

            telemetry.addData("IMU orientation: ", robot.internalIMU.getCurrentFrameOrientation());
            telemetry.addData("CCW IMU orientation: ", robot.internalIMU.getCurrentFrameHeadingCCW());

            telemetry.addData("CW IMU orientation: ", robot.internalIMU.getCurrentFrameHeadingCW());
            telemetry.addData("Robot Tilt : ", robot.internalIMU.getCurrentFrameTilt());
            telemetry.addData("Time to get: ", robot.internalIMU.getCurrentFrameOrientation().acquisitionTime);
            telemetry.addData("Robot Tilt Acceleration y: ", robot.internalIMU.getCurrentFrameVelocity().yRotationRate);

            telemetry.addData("Joystick Orientation: ", Math.atan2(-currentFrameGamepad1.right_stick_x, -currentFrameGamepad1.right_stick_y));
            telemetry.addData(
                    "Updated Joystick Orientation: ",
                    Math.atan2(currentFrameGamepad1.right_stick_y, currentFrameGamepad1.right_stick_x) - Math.PI / 2
            );
            // ^ https://www.desmos.com/calculator/jp45vcfcbt
            telemetry.update();


            if (currentFrameGamepad1.a != previousFrameGamepad1.a && currentFrameGamepad1.a) {
                intakeOn = !intakeOn;
                intakeDirection = false;
            }

            if (currentFrameGamepad1.b != previousFrameGamepad1.b && currentFrameGamepad1.b) {
                intakeOn = !intakeOn;
                intakeDirection = true;
            }

            if (intakeOn) {
                robot.intake.enableIntakeMotor(intakeDirection);
            } else {
                robot.intake.disableIntakeMotor();
            }


            robot.drivetrain.fieldCentricDriveFromGamepad(
                    currentFrameGamepad1.left_stick_y,
                    currentFrameGamepad1.left_stick_x,
                    currentFrameGamepad1.right_stick_x
            );
            /* robot.drivetrain.fieldCentricDriveFromGamepad(
                    currentFrameGamepad1.left_stick_y,
                    currentFrameGamepad1.left_stick_x,
                    currentFrameGamepad1.right_stick_x
            ); */

            double frameTime = robot.update();

            telemetry.addData("Frame Time: ", frameTime);
            telemetry.addData("Refresh Rate: ", (frameTime != 0) ? (1 / frameTime) : "inf");
        }
    }
}
