package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;

@Config
public class Lift implements Subsystem {

    public enum LIFT_POSITIONS {
        DEFAULT,
        GROUND_JUNCTION,
        LOW_JUNCTION,
        MIDDLE_JUNCTION,
        HIGH_JUNCTION
    }

    public enum LIFT_STATES {
        DEFAULT,
        RETRACTING,
        EXTENDING,
        EXTENDED
    }

    private final double GAMEPAD_THRESHOLD = 0.1;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public DcMotorEx leftLiftMotor;
    public DcMotorEx rightLiftMotor;

    private MotorGroup<DcMotorEx> liftMotors;

    private double currentFrameOutput = 0;

    LIFT_POSITIONS currentLiftTargetPosition = LIFT_POSITIONS.DEFAULT;
    LIFT_STATES currentLiftState = LIFT_STATES.DEFAULT;

    GeneralPIDController liftPID = new GeneralPIDController(kP, kI, kD, kF);

    private Telemetry telemetry;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        this.leftLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftLiftMotor");
        this.rightLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightLiftMotor");


        // todo: figure out the directions
        rightLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotorEx.Direction.FORWARD);

        liftMotors = new MotorGroup<>(leftLiftMotor, rightLiftMotor);

        liftMotors.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {
        telemetry.addData("Left Lift Pos: ", this.leftLiftMotor.getCurrentPosition());
        telemetry.addData("Right Lift Pos: ", this.rightLiftMotor.getCurrentPosition());

        liftPID.updateCoefficients(kP, kI, kD, kF);

        int targetPosition = this.getEncoderPositionFromLevel(this.currentLiftTargetPosition);

        if (this.currentFrameOutput == 0) {
            currentFrameOutput = liftPID.getOutputFromError(targetPosition, this.liftMotors.getAveragePosition());
        } else {
            this.currentFrameOutput += kF;
        }

        this.liftMotors.setPower(currentFrameOutput);
        /*
        switch (this.currentLiftState) {
            case DEFAULT:
            case EXTENDING:
            case EXTENDED:
            case RETRACTING:
        }

         */
/*        liftPID.updateCoefficients(kP, kI, kD, kF);

        int targetPosition = this.getEncoderPositionFromLevel(currentLiftPosition);
        double currentFrameOutput = liftPID.getOutputFromError(targetPosition, this.liftMotors.getAveragePosition());

        this.liftMotors.setPower(currentFrameOutput);*/

        this.currentFrameOutput = 0;

    }

    public void driveLiftFromGamepad(double leftTrigger, double rightTrigger) {
        if (leftTrigger > GAMEPAD_THRESHOLD) {
             this.currentFrameOutput = -leftTrigger;
        } else if (rightTrigger > GAMEPAD_THRESHOLD) {
            this.currentFrameOutput = rightTrigger;
        }
    }

    public int getEncoderPositionFromLevel(LIFT_POSITIONS currentLiftPosition) {
        switch (currentLiftPosition) {
            case DEFAULT:
                return 0;
            case GROUND_JUNCTION:
                return 100;
            case LOW_JUNCTION:
                return 200;
            case MIDDLE_JUNCTION:
                return 300;
            case HIGH_JUNCTION:
                return 400;

        }

        return 0;
    }
}
