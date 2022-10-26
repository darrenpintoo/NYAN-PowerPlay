package org.firstinspires.ftc.teamcode.utilities.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Subsystem;

public class RobotEx {
    private static RobotEx robotInstance = null;

    public final InternalIMU internalIMU = InternalIMU.getInstance();

    public Drivetrain drivetrain = new Drivetrain();
    public final Intake intake = new Intake();

    private final ElapsedTime frameTimer = new ElapsedTime();

    private final Subsystem[] robotSubsystems = new Subsystem[] {
            intake,
            drivetrain,
            internalIMU
    };

    private RobotEx() {
        if (RobotEx.robotInstance != null) {
            throw new IllegalStateException("Robot already instantiated");
        }
    }

    public static RobotEx getInstance() {
        if (RobotEx.robotInstance == null) {
            RobotEx.robotInstance = new RobotEx();
        }

        return RobotEx.robotInstance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        for (Subsystem subsystem : this.robotSubsystems) {
            subsystem.onInit(hardwareMap, telemetry);
        }

        telemetry.update();
    }

    public void postInit() {
        for (Subsystem subsystem : this.robotSubsystems) {
            subsystem.onOpmodeStarted();
        }
    }

    public double update() {
        for (Subsystem subsystem : this.robotSubsystems) {
            subsystem.onCyclePassed();
        }

        double frameTime = frameTimer.milliseconds();
        frameTimer.reset();
        return frameTime;
    }


}