package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;

import java.util.ArrayList;

@TeleOp(name = "Friction Test")
public class FrictionFinder extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        RobotEx robot = RobotEx.getInstance();

        robot.init(hardwareMap,telemetry);

        waitForStart();

        robot.postInit();
        MotorGroup<DcMotorEx> drivetrainMotors = robot.drivetrain.getDrivetrainMotorGroup();
        double currentDriveTrainPower = 0;
        double lastPos = drivetrainMotors.getAveragePosition();



        while (lastPos == drivetrainMotors.getAveragePosition() && !isStopRequested()) {
            lastPos = drivetrainMotors.getAveragePosition();
            currentDriveTrainPower += 0.01;
            drivetrainMotors.setPower(currentDriveTrainPower);
            robot.pause(0.1);
            telemetry.addData("MotorPower", currentDriveTrainPower);
            telemetry.update();

        }

        while (opModeIsActive()){
            telemetry.addData("MotorPower", currentDriveTrainPower);
            telemetry.update();
        }



    }
}
