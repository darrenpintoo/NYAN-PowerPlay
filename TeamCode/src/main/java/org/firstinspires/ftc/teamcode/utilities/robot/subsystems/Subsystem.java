package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Subsystem template
 */
public interface Subsystem {

    void onInit(HardwareMap hardwareMap, Telemetry telemetry);

    void onOpmodeStarted();

    void onCyclePassed();

}
