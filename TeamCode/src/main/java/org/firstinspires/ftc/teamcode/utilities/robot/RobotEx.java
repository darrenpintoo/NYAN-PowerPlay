package org.firstinspires.ftc.teamcode.utilities.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LynxModuleMeta;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.localizer.BaseLocalizer;
import org.firstinspires.ftc.teamcode.utilities.localizer.IMUEncoderLocalizer;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.ClawExtension;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Subsystem;

import java.util.List;

public class RobotEx {
    private static RobotEx robotInstance = null;

    List<LynxModule> allHubs;

    public InternalIMU internalIMU = InternalIMU.getInstance();

    public Drivetrain drivetrain = new Drivetrain();
    // public Intake intake = new Intake();
    public Lift lift = new Lift();
    public Claw claw = new Claw();

    public VoltageSensor voltageSensor;

    private final ElapsedTime frameTimer = new ElapsedTime();

    private final Subsystem[] robotSubsystems = new Subsystem[] {
            internalIMU,
            // intake,
            drivetrain,
            lift,
            claw//,
           //  clawExtension
    };

    Telemetry telemetry;

    public BaseLocalizer localizer;

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

        this.allHubs = hardwareMap.getAll(LynxModule.class);
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        
        this.localizer = new IMUEncoderLocalizer(drivetrain, internalIMU, telemetry);
        this.localizer.setPoseEstimation(new Pose(0, 0, Math.PI / 2));

        for (LynxModule hub: allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for (Subsystem subsystem : this.robotSubsystems) {
            subsystem.onInit(hardwareMap, telemetry);
        }

        telemetry.update();

        this.telemetry = telemetry;
    }

    public void postInit() {
        for (Subsystem subsystem : this.robotSubsystems) {
            subsystem.onOpmodeStarted();
        }
    }

    public double update() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        for (Subsystem subsystem : this.robotSubsystems) {
            subsystem.onCyclePassed();
        }

        this.localizer.updatePose();
        Pose currentPose = this.localizer.getDisplacement();
        telemetry.addData("X: ", currentPose.getX());
        telemetry.addData("Y: ", currentPose.getY());
        telemetry.addData("Heading: ", currentPose.getHeading());
        double frameTime = frameTimer.milliseconds();
        frameTimer.reset();

        return frameTime;
    }

    public void pause(double seconds) {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.seconds() < seconds) {
            this.update();
        }
    }

    public void persistData() {
        PersistentData.startPose = new Pose(0, 0, this.internalIMU.getCurrentFrameHeadingCCW());
    }

    public double getVoltage() {
        return this.voltageSensor.getVoltage();
    }

    public double getPowerMultiple() {
        return 12 / this.getVoltage();
    }


}