package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ClawExtension implements Subsystem {

    enum ExtensionState {
        ACTIVE,
        DEFAULT
    }
    public static double activeServoPosition = 0.4;
    public static double defaultServoPosition = 0.74;

    public Servo extensionServo;

    public ExtensionState currentExtensionState = ExtensionState.ACTIVE;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.extensionServo = hardwareMap.get(Servo.class, "extensionServo");
    }

    @Override
    public void onOpmodeStarted() {
    }

    @Override
    public void onCyclePassed() {
        this.extensionServo.setPosition(this.getServoPositionFromState(this.currentExtensionState));
    }

    public double getServoPositionFromState(ExtensionState newState) {
        switch (newState) {
            case DEFAULT:
                return defaultServoPosition;
            case ACTIVE:
                return activeServoPosition;
            default:
                return defaultServoPosition;
        }
    }

}
