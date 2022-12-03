package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Deprecated
public class ClawExtension implements Subsystem {

    final double JOYSTICK_THRESHOLD = 0.1;

    private CRServo extensionServo;

    private double currentFrameOutput = 0;

    Telemetry t;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.extensionServo = hardwareMap.get(CRServo.class, "clawExtensionServo");

        this.t = telemetry;
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {
        this.extensionServo.setPower(this.currentFrameOutput);
    }

    public void driveLiftFromGamepad(double joystick) {
        if (Math.abs(joystick) > JOYSTICK_THRESHOLD) {
            this.currentFrameOutput = joystick;
        } else {
            this.currentFrameOutput = 0;
        }
    }
}
