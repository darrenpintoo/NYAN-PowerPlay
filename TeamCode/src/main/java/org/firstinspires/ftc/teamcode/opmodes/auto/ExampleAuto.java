package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.EncoderDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.vision.simulatortests.SleeveDetectionPipeline;

/**
 * Example teleop code for a basic mecanum drive
 */
@TeleOp(name = "Example Auto")
public class ExampleAuto extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    @Override
    public void runOpMode() {


        SleeveDetectionPipeline pipeline = new SleeveDetectionPipeline(telemetry);
        // scan sleeve

        // Initialize the robot
        robot.init(hardwareMap, telemetry);

        waitForStart();

        // Notify subsystems before loop
        robot.postInit();

        if (isStopRequested()) return;

        EncoderDrive robotDrivetrain = new EncoderDrive();

        SleeveDetectionPipeline.ParkingPosition parkPosition = pipeline.getPosition();

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
