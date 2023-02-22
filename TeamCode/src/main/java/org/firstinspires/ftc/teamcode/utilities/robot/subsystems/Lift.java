package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
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
        ACTIVE,
        WAITING
    }

    public enum PROFILE_TYPE {
        DOWN, UP
    }

    private final double GAMEPAD_THRESHOLD = 0.05;

    public static double upvMax = 2000;
    public static double upaMax = 4000;
    public static double upkV = 0.0003;
    public static double upkA = 0.000025;

    public static double upkP = 0.01;
    public static double upkI = 0;
    public static double upkD = 0;
    public static double upkF1 = 0.25;
    public static double upkF2 = 0.35;

    public static double downvMax = 4000;
    public static double downaMax = 3000;
    public static double downkV = 0.0001;
    public static double downkA = 0.00005;

    public static double downkP = 0.001;
    public static double downkI = 0;
    public static double downkD = 0;
    public static double downkF1 = 0.05;
    public static double downkF2 = 0.05;

    public static int OFFSET_INCREASE = 70;// 80;
    public static int AT_POSITION_THRESHOLD = 75;
    public static int AT_VELOCITY_THRESHOLD = 10;

    public static int GROUND_HEIGHT = 100;
    public static int LOW_HEIGHT = 750;
    public static int MIDDLE_HEIGHT = 1225;
    public static int HIGH_HEIGHT = 1650;

    public DcMotorEx leftLiftMotor;
    public DcMotorEx rightLiftMotor;

    private MotorGroup<DcMotorEx> liftMotors;

    private double previousFramePosition = 0;
    private double currentFrameOutput = 0;
    private int offset = 0;

    private boolean liftAtTarget = false;

    LIFT_POSITIONS currentLiftTargetPosition = LIFT_POSITIONS.DEFAULT;
    LIFT_POSITIONS lastLiftTargetPosition = LIFT_POSITIONS.DEFAULT;

    LIFT_STATES currentLiftState = LIFT_STATES.ACTIVE;

    GeneralPIDController upLiftPID = new GeneralPIDController(upkP, upkI, upkD, 0);
    GeneralPIDController downLiftPID = new GeneralPIDController(downkP, downkI, downkD, 0);

    private MotionProfile currentMotionProfile = new MotionProfile(0, 0, upvMax, upaMax);
    PROFILE_TYPE profileType = PROFILE_TYPE.UP;

    private Telemetry telemetry;

    private ElapsedTime velocityTimer = new ElapsedTime();
    private ElapsedTime motionProfileTimer = new ElapsedTime();

    private ClawExtension clawExtension;
    private ClawRotation clawRotation;

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
        this.currentLiftState = LIFT_STATES.ACTIVE;

        this.clawExtension = RobotEx.getInstance().clawExtension;
        this.clawRotation = RobotEx.getInstance().clawRotation;
    }

    @Override
    public void onCyclePassed() {
        upLiftPID.updateCoefficients(upkP, upkI, upkD, 0);
        downLiftPID.updateCoefficients(downkP, downkI, downkD, 0);

        double currentMotionProfileTime = this.motionProfileTimer.seconds();

        double currentPosition = this.liftMotors.getAveragePosition();

        double targetPosition = this.currentMotionProfile.getPositionFromTime(currentMotionProfileTime);
        double targetVelocity = this.currentMotionProfile.getVelocityFromTime(currentMotionProfileTime);
        double targetAcceleration = this.currentMotionProfile.getAccelerationFromTime(currentMotionProfileTime);

        telemetry.addData("target Position: ", targetPosition);
        telemetry.addData("target Velocity: ", targetVelocity);
        telemetry.addData("target Acceleration: ", targetAcceleration);

        telemetry.addData("Current Position: ", currentPosition);
        telemetry.addData("Target pos: ", targetPosition);
        double feedforward = 0;
        double feedback = 0;
        double kF = 0;

        if (this.profileType == PROFILE_TYPE.UP) {
            kF = MathHelper.lerp(upkF1, upkF2, Math.min(currentPosition / this.getEncoderPositionFromLevel(LIFT_POSITIONS.HIGH_JUNCTION), 1));
            feedforward = targetAcceleration * upkA + targetVelocity * upkV + kF;
            feedback = upLiftPID.getOutputFromError(
                    targetPosition - currentPosition
            );
        } else {
            kF = MathHelper.lerp(downkF1, downkF2, Math.min(currentPosition / this.getEncoderPositionFromLevel(LIFT_POSITIONS.HIGH_JUNCTION), 1));
            feedforward = targetAcceleration * downkA + targetVelocity * downkV + kF;
            feedback = downLiftPID.getOutputFromError(
                    targetPosition - currentPosition
            );
        }

        telemetry.addData("kF: ", kF);
        telemetry.addData("feed forward: ", feedforward);
        telemetry.addData("feed back: ", feedback);
        currentFrameOutput = feedforward + feedback;

        if (this.getTargetPosition() == 0 && currentPosition < 200) {
            currentFrameOutput = -0.1 * currentPosition / 100;
        }
/*        if (this.currentFrameOutput == 0) {
            currentFrameOutput = liftPID.getOutputFromError(targetPosition, currentPosition);

            this.liftAtTarget = Math.abs(targetPosition - currentPosition) < AT_POSITION_THRESHOLD &&
                    ((currentPosition - this.previousFramePosition) / this.velocityTimer.seconds()) < AT_VELOCITY_THRESHOLD;
        } else {
            this.liftAtTarget = false;
        }*/
        if (this.currentLiftState == LIFT_STATES.WAITING) {
            if (this.clawExtension.isAtPosition() && this.clawRotation.atPosition()) {
                this.currentLiftState = LIFT_STATES.ACTIVE;
                motionProfileTimer.reset();
            } else {
                return;
            }
        }
        this.liftMotors.setPower(MathHelper.clamp(currentFrameOutput, -1, 1));

        telemetry.addData("output: ", currentFrameOutput);
        telemetry.addData("Type: ", this.profileType.toString());


        this.currentFrameOutput = 0;
        this.lastLiftTargetPosition = this.currentLiftTargetPosition;
        this.previousFramePosition = currentPosition;

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

        if (getEncoderPositionFromLevel(targetListPosition) == this.getTargetPosition()) {
            return;
        }

        if (targetListPosition == LIFT_POSITIONS.DEFAULT) {
            this.clawRotation.setCurrentState(ClawRotation.rotationState.DEFAULT);
            this.clawExtension.setCurrentExtensionState(ClawExtension.ExtensionState.DEFAULT);
            this.currentLiftState = LIFT_STATES.WAITING;
        } else {
            this.currentLiftState = LIFT_STATES.ACTIVE;
        }
        this.resetOffset();
        this.currentLiftTargetPosition = targetListPosition;

        this.updateMotionProfile();
    }

    public void updateMotionProfile() {

        if (this.previousFramePosition > this.getTargetPosition()) {
            this.currentMotionProfile = new MotionProfile(
                    this.previousFramePosition,
                    this.getTargetPosition(),
                    downvMax,
                    downaMax
            );

            this.profileType = PROFILE_TYPE.DOWN;
        } else {
            this.currentMotionProfile = new MotionProfile(
                    this.previousFramePosition,
                    this.getTargetPosition(),
                    upvMax,
                    upaMax
            );
            this.profileType = PROFILE_TYPE.UP;
        }

        this.motionProfileTimer.reset();

    }

    public int getTargetPosition() {
        return this.getEncoderPositionFromLevel(this.currentLiftTargetPosition) + offset * OFFSET_INCREASE;
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

        this.updateMotionProfile();
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
