package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;

@Config
public class ClawExtension implements Subsystem {

    public enum ExtensionState {
        ACTIVE,
        MID,
        DEFAULT
    }

    public static double aMax = 2.5;
    public static double vMax = 5;
    public static double activeServoPosition = 0.4;
    public static double midServoPosition = 0.68;
    public static double defaultServoPosition = 0.74;

    public Servo extensionServo;

    public ExtensionState currentExtensionState = ExtensionState.DEFAULT;

    private MotionProfile currentMovementProfile = new MotionProfile(defaultServoPosition, defaultServoPosition, vMax, aMax);

    public ElapsedTime profileTimer = new ElapsedTime();

    RobotEx robot;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.extensionServo = hardwareMap.get(Servo.class, "extensionServo");
        this.robot = RobotEx.getInstance();
    }

    @Override
    public void onOpmodeStarted() {
    }

    @Override
    public void onCyclePassed() {
        this.extensionServo.setPosition(this.getPosition());
    }

    public double getServoPositionFromState(ExtensionState newState) {
        switch (newState) {
            case DEFAULT:
                return defaultServoPosition;
            case ACTIVE:
                return activeServoPosition;
            case MID:
                return midServoPosition;
            default:
                return defaultServoPosition;
        }
    }

    public void setCurrentExtensionState(ExtensionState newExtensionState) {

        if (newExtensionState == this.currentExtensionState) {
            return;
        }

        this.currentMovementProfile = new MotionProfile(
                this.getPosition(),
                this.getServoPositionFromState(newExtensionState),
                vMax,
                aMax
        );
        this.profileTimer.reset();
        this.currentExtensionState = newExtensionState;

    }

    public double getPosition() {
        return this.currentMovementProfile.getPositionFromTime(this.profileTimer.seconds());
    }

    public boolean isAtPosition() {
        return this.profileTimer.seconds() > this.currentMovementProfile.getDuration();
    }

    public void yieldTillAtPosition() {
        while (!this.isAtPosition()) {
            this.robot.update();
        }
    }
}
