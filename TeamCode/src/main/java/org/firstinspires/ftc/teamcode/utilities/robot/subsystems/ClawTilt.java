package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ClawTilt implements Subsystem {

    public enum tiltState {
        ACTIVE,
        DEFAULT
    }
    public static double activePosition = 0.1;
    public static double defaultPosition = 0.2;

    public Servo tiltServo;

    public tiltState currentState = tiltState.DEFAULT;

    Telemetry telemetry;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        this.telemetry = telemetry;
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {
        this.tiltServo.setPosition(this.getServoPositionFromState(currentState));

        telemetry.addData("Current Tilt Position: ", this.tiltServo.getPosition());
    }

    public double getServoPositionFromState(tiltState state) {
        switch (state) {
            case ACTIVE:
                return activePosition;
            case DEFAULT:
                return defaultPosition;
        }

        return activePosition;
    }

    public void setCurrentState(tiltState newTiltState) {
        this.currentState = newTiltState;
    }
}
