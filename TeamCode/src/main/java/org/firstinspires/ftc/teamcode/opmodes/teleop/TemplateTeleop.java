package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;

/**
 * Example teleop code for a basic mecanum drive
 */
@TeleOp(name = "Template Teleop")
public class TemplateTeleop extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    private boolean fieldCentric = false;

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

            telemetry.addData("Field Centric: ", fieldCentric);

            telemetry.update();

            if (currentFrameGamepad1.a != previousFrameGamepad1.a && currentFrameGamepad1.a) {
                fieldCentric = !fieldCentric;
            }

            if (fieldCentric) {
                robot.drivetrain.fieldCentricDriveFromGamepad(
                        currentFrameGamepad1.left_stick_y,
                        currentFrameGamepad1.left_stick_x,
                        currentFrameGamepad1.right_stick_x
                );
            } else {
                robot.drivetrain.robotCentricDriveFromGamepad(
                        currentFrameGamepad1.left_stick_y,
                        currentFrameGamepad1.left_stick_x,
                        currentFrameGamepad1.right_stick_x
                );
            }

            double frameTime = robot.update();

            telemetry.addData("Frame Time: ", frameTime);
        }
    }
}
