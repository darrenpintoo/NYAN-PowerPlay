package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Robot Drivetrain
 */
public class Intake implements Subsystem {

    private DcMotorEx rightMotor;
    private DcMotorEx leftMotor;

    private final int ON_MOTOR_POWER = 1;

    private Telemetry telemetry;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightIntakeMotor");
        this.leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftIntakeMotor");

        // todo: figure out the directions
        rightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {

    }

    public void enableIntakeMotor(boolean directionReversed) {

        int intakeDirection = directionReversed ? -1 : 1;

        telemetry.addLine("Intake on");
        this.rightMotor.setPower(ON_MOTOR_POWER * intakeDirection);
        this.leftMotor.setPower(ON_MOTOR_POWER * intakeDirection);
        telemetry.addData("Intake Power: ", this.rightMotor.getPower());
        telemetry.addData("Intake Power: ", this.leftMotor.getPower());

    }

    public void disableIntakeMotor() {
        telemetry.addLine("Intake off");
        this.rightMotor.setPower(0);
        this.leftMotor.setPower(0);
    }
}