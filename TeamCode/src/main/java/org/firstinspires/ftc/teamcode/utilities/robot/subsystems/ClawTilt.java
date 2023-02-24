package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;

@Config
public class ClawTilt implements Subsystem {

    public enum tiltState {
        ACTIVE,
        DEFAULT
    }

    public static double vMax = 1000;
    public static double aMax = 40;
    public static double activePosition = 0.1;
    public static double defaultPosition = 0.2;

    public Servo tiltServo;

    public tiltState currentState = tiltState.DEFAULT;
    private MotionProfile currentTiltProfile = new MotionProfile(defaultPosition, defaultPosition, vMax, aMax);
    private ElapsedTime profileTimer = new ElapsedTime();
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
        this.tiltServo.setPosition(this.getServoPosition());

        // telemetry.addData("Current Tilt Position: ", this.getServoPosition());
    }

    public double getServoPosition() {
        return this.currentTiltProfile.getPositionFromTime(this.profileTimer.seconds());
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

        this.currentTiltProfile = new MotionProfile(
                this.getServoPosition(),
                this.getServoPositionFromState(newTiltState),
                vMax,
                aMax
        );
        this.profileTimer.reset();
    }
}
