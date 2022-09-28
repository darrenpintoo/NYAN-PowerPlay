package org.firstinspires.ftc.teamcode.utilities.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Subsystem;

import java.util.ArrayList;

public class Robot {
    private static Robot robotInstance = null;

    private static final Subsystem[] robotSubsystems = new Subsystem[] {

    };

    private Robot() {
        if (Robot.robotInstance != null) {
            throw new IllegalStateException("Robot already instantiated");
        }
    }

    public static Robot getInstance() {
        if (robotInstance == null) {
            Robot.robotInstance = new Robot();
        }

        return Robot.robotInstance;
    }

    public static void init(HardwareMap hardwareMap) {
        for (Subsystem subsystem : Robot.robotSubsystems) {
            subsystem.onInit(hardwareMap);
        }
    }

    public static void postInit() {
        for (Subsystem subsystem : Robot.robotSubsystems) {
            subsystem.onOpmodeStarted();
        }
    }

    public static void update() {
        for (Subsystem subsystem : Robot.robotSubsystems) {
            subsystem.onCyclePassed();
        }
    }

}