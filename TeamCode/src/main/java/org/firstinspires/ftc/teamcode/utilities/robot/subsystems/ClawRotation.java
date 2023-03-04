package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;

@Config
public class ClawRotation implements Subsystem {

    public enum rotationState {
        LEFT,
        DEFAULT,
        RIGHT
    }

    public enum movementState {
        ACTIVE,
        WAITING
    }
    public static double vMax = 3;
    public static double aMax = 1;
    public static double leftPosition = 0.83;
    public static double defaultPosition = 0.51;
    public static double rightPosition = 0.16;

    public static double MOVEMENT_THRESHOLD = 0.3;

    public Servo rotationServo;

    private double previousFramePosition = defaultPosition;
    public rotationState currentState = rotationState.DEFAULT;
    public movementState currentMovementState = movementState.ACTIVE;

    Telemetry telemetry;

    MotionProfile currentMovementProfile = new MotionProfile(defaultPosition, defaultPosition, vMax, aMax);
    ElapsedTime profileTimer = new ElapsedTime();

    ClawExtension extension;

    RobotEx robot;
    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.rotationServo = hardwareMap.get(Servo.class, "extensionRotationServo");
        this.extension = RobotEx.getInstance().clawExtension;
        this.telemetry = telemetry;
        this.robot = RobotEx.getInstance();
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {


        if (this.currentMovementState == movementState.ACTIVE) {
            double currentFramePosition = this.getPosition();
            this.rotationServo.setPosition(currentFramePosition);
            this.previousFramePosition = currentFramePosition;
        } else {
            if (this.extension.isAtPosition()) {
                this.currentMovementState = movementState.ACTIVE;
                this.profileTimer.reset();
            }
        }
        // telemetry.addData("Current rotation Position: ", this.rotationServo.getPosition());

    }

    public double getServoPositionFromState(rotationState state) {
        switch (state) {
            case LEFT:
                return leftPosition;
            case DEFAULT:
                return defaultPosition;
            case RIGHT:
                return rightPosition;
        }

        return defaultPosition;
    }

    public void handleRotationFromGamepad(double y, double x) {
        if (-y > MOVEMENT_THRESHOLD) {
            this.setCurrentState(rotationState.DEFAULT);
        } else if (x < -MOVEMENT_THRESHOLD) {
            this.setCurrentState(rotationState.LEFT);
        } else if (x > MOVEMENT_THRESHOLD) {
            this.setCurrentState(rotationState.RIGHT);
        }
    }

    public void setAngle(double position) {

        position = position / 180;
        position += 0.5;
        this.currentMovementProfile = new MotionProfile(
                this.getPosition(),
                MathHelper.lerp(leftPosition, rightPosition, position),
                vMax,
                aMax
        );

        this.currentMovementState = movementState.WAITING;
        this.currentState = rotationState.RIGHT;
        this.extension.setCurrentExtensionState(ClawExtension.ExtensionState.DEFAULT);
    }

    public void setCurrentState(rotationState newRotationState) {

        if (newRotationState == currentState) {
            return;
        }

        this.currentMovementProfile = new MotionProfile(
                this.getPosition(),
                this.getServoPositionFromState(newRotationState),
                vMax,
                aMax
        );

        this.currentMovementState = movementState.WAITING;
        this.extension.setCurrentExtensionState(ClawExtension.ExtensionState.DEFAULT);
        this.currentState = newRotationState;

    }

    public double getPosition() {
        return this.currentMovementProfile.getPositionFromTime(this.profileTimer.seconds());
    }

    public boolean atPosition() {
        return this.currentMovementState != movementState.WAITING && this.profileTimer.seconds() > this.currentMovementProfile.getDuration();
    }

    public rotationState getTargetPosition() {
        return this.currentState;
    }

    public void yieldTillAtPosition() {
        while (!this.atPosition()) {
            this.robot.update();
        }
    }
}
