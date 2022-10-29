package org.firstinspires.ftc.teamcode.utilities.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LynxModuleMeta;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Subsystem;

import java.util.List;

public class RobotEx {
    private static RobotEx robotInstance = null;

    // List<LynxModule> allHubs;

    public InternalIMU internalIMU = InternalIMU.getInstance();

    public Drivetrain drivetrain = new Drivetrain();
    public Intake intake = new Intake();

    public Lift lift = new Lift();

    private final ElapsedTime frameTimer = new ElapsedTime();

    private final Subsystem[] robotSubsystems = new Subsystem[] {
            internalIMU,
            intake,
           drivetrain,
            lift

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
/*
        this.allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub: allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }*/

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

/*        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }*/

        for (Subsystem subsystem : this.robotSubsystems) {
            subsystem.onCyclePassed();
        }

        double frameTime = frameTimer.milliseconds();
        frameTimer.reset();
        return frameTime;
    }


}