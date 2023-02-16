package org.firstinspires.ftc.teamcode.utilities.robot;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LynxModuleMeta;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utilities.localizer.BaseLocalizer;
import org.firstinspires.ftc.teamcode.utilities.localizer.IMUEncoderLocalizer;
import org.firstinspires.ftc.teamcode.utilities.localizer.RoadrunnerLocalizer;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.ClawExtension;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.ClawTilt;
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
    public Lift lift = new Lift();
    public Claw claw = new Claw();
    public ClawTilt clawTilt = new ClawTilt();
    public ClawExtension clawExtension = new ClawExtension();

    public VoltageSensor voltageSensor;

    private final ElapsedTime frameTimer = new ElapsedTime();

    private final Subsystem[] robotSubsystems = new Subsystem[] {
            internalIMU,
            drivetrain,
            lift,
            claw,
            clawTilt,
            clawExtension
    };

    Telemetry telemetry;

    public RoadrunnerLocalizer localizer;

    private double voltageCompensator = 12;

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
        this.voltageCompensator = this.voltageSensor.getVoltage();

        this.localizer = new RoadrunnerLocalizer(drivetrain, internalIMU, telemetry);

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

    @SuppressLint("")
    public double update() {

        double hubCurrent = 0;

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
            hubCurrent += hub.getCurrent(CurrentUnit.AMPS);
        }

        int i = 1;

        for (Subsystem subsystem : this.robotSubsystems) {

            subsystem.onCyclePassed();

        }

        this.localizer.update();

        Pose2d currentPose = this.localizer.getPoseEstimate();
        telemetry.addData("X: ", currentPose.getX());
        telemetry.addData("Y: ", currentPose.getY());
        telemetry.addData("Heading: ", currentPose.getHeading());
        telemetry.addData("Current Draw: ", hubCurrent);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#4CAF50");
        drawRobot(fieldOverlay, currentPose);


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
        PersistentData.startPose = this.localizer.getPoseEstimate();
    }

    public double getVoltage() {
        return this.voltageSensor.getVoltage();
    }

    public double getPowerMultiple() {
        return 12 / this.voltageCompensator;
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
    public void clearPersistData() {
        PersistentData.startPose = new Pose2d();
    }
}