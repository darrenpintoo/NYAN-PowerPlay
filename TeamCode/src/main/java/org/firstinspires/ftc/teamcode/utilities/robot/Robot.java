package org.firstinspires.ftc.teamcode.utilities.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Subsystem;

import java.util.ArrayList;

public class Robot {
    private static Robot robotInstance = null;

    public final InternalIMU internalIMU = InternalIMU.getInstance();

    public final Drivetrain drivetrain = new Drivetrain();

    private final Subsystem[] robotSubsystems = new Subsystem[] {
            drivetrain,
            internalIMU
    };

    private Robot() {
        if (Robot.robotInstance != null) {
            throw new IllegalStateException("Robot already instantiated");
        }
    }

    public static Robot getInstance() {
        if (Robot.robotInstance == null) {
            Robot.robotInstance = new Robot();
        }

        return Robot.robotInstance;
    }

    public void init(HardwareMap hardwareMap) {
        for (Subsystem subsystem : this.robotSubsystems) {
            subsystem.onInit(hardwareMap);
        }
    }

    public void postInit() {
        for (Subsystem subsystem : this.robotSubsystems) {
            subsystem.onOpmodeStarted();
        }
    }

    public void update() {
        for (Subsystem subsystem : this.robotSubsystems) {
            subsystem.onCyclePassed();
        }
    }

}