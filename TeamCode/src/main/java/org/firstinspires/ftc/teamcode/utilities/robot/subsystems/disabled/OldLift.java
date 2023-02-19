package org.firstinspires.ftc.teamcode.utilities.robot.subsystems.disabled;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Subsystem;

@Config
public class OldLift implements Subsystem {

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

    public static double kP = 0.002;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF1 = 0.1;
    public static double kF2 = 0.15;

    public static int OFFSET_INCREASE = 70;// 80;
    public static int AT_POSITION_THRESHOLD = 75;
    public static int AT_VELOCITY_THRESHOLD = 10;

    public static int GROUND_HEIGHT = 100;
    public static int LOW_HEIGHT = 750;
    public static int MIDDLE_HEIGHT = 1225;
    public static int HIGH_HEIGHT = 1700;

    public DcMotorEx leftLiftMotor;
    public DcMotorEx rightLiftMotor;

    private MotorGroup<DcMotorEx> liftMotors;

    private double previousFramePosition = 0;
    private double currentFrameOutput = 0;
    private int offset = 0;

    private boolean liftAtTarget = false;

    LIFT_POSITIONS currentLiftTargetPosition = LIFT_POSITIONS.DEFAULT;
    LIFT_POSITIONS lastLiftTargetPosition = LIFT_POSITIONS.DEFAULT;

    LIFT_STATES currentLiftState = LIFT_STATES.DEFAULT;

    GeneralPIDController liftPID = new GeneralPIDController(kP, kI, kD, 0);

    private Telemetry telemetry;

    private ElapsedTime velocityTimer = new ElapsedTime();

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        this.leftLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftLiftMotor");
        this.rightLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightLiftMotor");

        // todo: figure out the directions
        rightLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        liftMotors = new MotorGroup<>(rightLiftMotor, leftLiftMotor);

        liftMotors.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.telemetry = telemetry;
    }

    @Override
    public void onOpmodeStarted() {
        this.currentLiftTargetPosition = LIFT_POSITIONS.DEFAULT;
        this.currentLiftState = LIFT_STATES.DEFAULT;
    }

    @Override
    public void onCyclePassed() {
/*        telemetry.addData("Left Lift Pos: ", this.leftLiftMotor.getCurrentPosition());
        telemetry.addData("Right Lift Pos: ", this.rightLiftMotor.getCurrentPosition());*/

        liftPID.updateCoefficients(kP, kI, kD, 0);

        double currentPosition = this.liftMotors.getAveragePosition();

        int targetPosition = this.getEncoderPositionFromLevel(this.currentLiftTargetPosition) + offset * OFFSET_INCREASE;

        if (this.currentFrameOutput == 0) {
            currentFrameOutput = liftPID.getOutputFromError(targetPosition, currentPosition);

            if (
                    Math.abs(targetPosition - currentPosition) < AT_POSITION_THRESHOLD &&
                    ((currentPosition - this.previousFramePosition) / this.velocityTimer.seconds()) < AT_VELOCITY_THRESHOLD
            ) {
                this.liftAtTarget = true;
            } else {
                this.liftAtTarget = false;
            }
        } else {
            this.liftAtTarget = false;
            this.currentFrameOutput = this.currentFrameOutput / 2;
        }

        double kF = MathHelper.lerp(kF1, kF2, currentPosition / this.getEncoderPositionFromLevel(LIFT_POSITIONS.HIGH_JUNCTION));

        currentFrameOutput += kF * RobotEx.getInstance().getPowerMultiple();
        this.liftMotors.setPower(MathHelper.clamp(currentFrameOutput, -1, 1));

        telemetry.addData("Output: ", currentFrameOutput);

        this.currentFrameOutput = 0;
        this.lastLiftTargetPosition = this.currentLiftTargetPosition;
        this.previousFramePosition = currentPosition;

        telemetry.addData("Lift Pos: ", currentPosition);

        this.velocityTimer.reset();
        // telemetry.addData("Target Position: ", targetPosition);
    }

    public void driveLiftFromGamepad(double leftTrigger, double rightTrigger) {
        if (leftTrigger > GAMEPAD_THRESHOLD) {
            this.currentFrameOutput = -leftTrigger;
        } else if (rightTrigger > GAMEPAD_THRESHOLD) {
            this.currentFrameOutput = rightTrigger;
        } else {
            this.currentFrameOutput = 0;
        }

    }

    public void setCurrentLiftTargetPosition(LIFT_POSITIONS targetListPosition) {
        this.resetOffset();
        this.currentLiftTargetPosition = targetListPosition;
    }

    public int getEncoderPositionFromLevel(LIFT_POSITIONS currentLiftPosition) {
        switch (currentLiftPosition) {
            case DEFAULT:
                return 0;
            case GROUND_JUNCTION:
                return GROUND_HEIGHT;
            case LOW_JUNCTION:
                return LOW_HEIGHT;
            case MIDDLE_JUNCTION:
                return MIDDLE_HEIGHT;
            case HIGH_JUNCTION:
                return HIGH_HEIGHT;

        }

        return 0;
    }

    public LIFT_POSITIONS getCurrentLiftTarget() {
        return this.currentLiftTargetPosition;
    }

    public void setOffset(int offset) {
        this.offset = offset;
    }

    public void incrementOffset(int offsetIncrease) {

        if (this.currentLiftTargetPosition != LIFT_POSITIONS.DEFAULT) {
            this.offset += offsetIncrease;
        } else {
            if (offsetIncrease > 0) {
                this.offset += offsetIncrease;
            } else if (this.getOffset() > 0) {
                this.offset += offsetIncrease;
            }
        }
    }

    public void incrementOffset() {
        this.incrementOffset(1);
    }

    public void resetOffset() {
        this.offset = 0;
    }

    public int getOffset() {
        return this.offset;
    }

    public void resetEncoderPosition() {
        this.liftMotors.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotors.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean checkAtTarget() {
        return this.liftAtTarget;
    }

    public void yieldTillAtPosition() {
        RobotEx robot = RobotEx.getInstance();

        while (!this.liftAtTarget) {
            robot.update();
        }
    }
}
