package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ClawRotation implements Subsystem {

    public enum rotationState {
        LEFT,
        DEFAULT,
        RIGHT
    }
    public static double leftPosition = 0.8;
    public static double defaultPosition = 0.5;
    public static double rightPosition = 0.3;

    public static double MOVEMENT_THRESHOLD = 0.3;

    public Servo rotationServo;

    public rotationState currentState = rotationState.DEFAULT;

    Telemetry telemetry;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.rotationServo = hardwareMap.get(Servo.class, "extensionRotationServo");
        this.telemetry = telemetry;
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {
        this.rotationServo.setPosition(this.getServoPositionFromState(currentState));

        telemetry.addData("Current Tilt Position: ", this.rotationServo.getPosition());
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

    public void setCurrentState(rotationState newRotationState) {
        this.currentState = newRotationState;
    }

    public void handleRotationFromGamepad(double y, double x) {
        if (y > MOVEMENT_THRESHOLD) {
            this.setCurrentState(rotationState.DEFAULT);
        } else if (x < -MOVEMENT_THRESHOLD) {
            this.setCurrentState(rotationState.LEFT);
        } else if (x > MOVEMENT_THRESHOLD) {
            this.setCurrentState(rotationState.RIGHT);
        }
    }
}
