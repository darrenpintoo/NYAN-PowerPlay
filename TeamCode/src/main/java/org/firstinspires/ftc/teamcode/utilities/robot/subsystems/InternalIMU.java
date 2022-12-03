package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.RobotOrientation;

/**
 * Robot IMU
 */
public class InternalIMU implements Subsystem {

    public static double TILT_THRESHOLD = Math.toRadians(20);
    public static double YAW_THRESHOLD = Math.toRadians(20);

    private final boolean DISABLE_VELOCITY_TRACKER = true;
    private static InternalIMU imuInstance = null;
    private BNO055IMU internalIMU;

    private Orientation currentFrameOrientation = new Orientation();
    private RobotOrientation currentFrameRobotOrientation = new RobotOrientation();

    private AngularVelocity currentFrameVelocity;

    private Orientation lastFrameOrientation = new Orientation();
    private RobotOrientation lastFrameRobotOrientation = new RobotOrientation();

    public RobotOrientation startFrameRobotOrientation = new RobotOrientation();

    private double startTilt = 0;

    // private Drivetrain drivetrain;

    private Telemetry telemetry;

    private InternalIMU() {
        if (InternalIMU.imuInstance != null) {
            throw new IllegalStateException("Robot already instantiated");
        }
    }

    public static InternalIMU getInstance() {
        if (InternalIMU.imuInstance == null) {
            InternalIMU.imuInstance = new InternalIMU();
        }

        return InternalIMU.imuInstance;
    }
    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        this.internalIMU = hardwareMap.get(BNO055IMU.class, "imu");
        this.internalIMU.initialize(parameters);
        this.telemetry = telemetry;

        telemetry.addData("Finished Initializing", this.internalIMU);
        telemetry.update();
    }

    @Override
    public void onOpmodeStarted() {
        this.onCyclePassed();
        this.startTilt = this.getCurrentFrameTilt();
        this.startFrameRobotOrientation = this.getCurrentFrameRobotOrientation();
    }

    @Override
    public void onCyclePassed() {

        this.lastFrameOrientation = this.currentFrameOrientation;
        this.lastFrameRobotOrientation = new RobotOrientation(this.lastFrameOrientation);

        this.currentFrameOrientation = internalIMU.getAngularOrientation();
        this.currentFrameRobotOrientation = new RobotOrientation(this.currentFrameOrientation);

        this.currentFrameVelocity = this.DISABLE_VELOCITY_TRACKER ? null : internalIMU.getAngularVelocity();
    }

    @Deprecated
    public Orientation getCurrentFrameOrientation() {
        return this.currentFrameOrientation;
    }

    @Deprecated
    public double getPreviousFrameHeadingCCW() {
        return -this.getPreviousFrameHeadingCW();
    }

    @Deprecated
    public double getPreviousFrameHeadingCW() {
        return this.lastFrameOrientation.firstAngle;
    }

    public double getCurrentFrameHeadingCW() {
        return this.currentFrameOrientation.firstAngle;
    }

    public AngularVelocity getCurrentFrameVelocity() {
        return this.currentFrameVelocity;
    }

    public double getCurrentFrameTiltVelocity() {
        return this.getCurrentFrameVelocity().yRotationRate;
    }

    public double getCurrentFrameTilt() {
        return this.currentFrameOrientation.secondAngle;
    }

    public double getCurrentFrameHeadingCCW() {
        return -this.getCurrentFrameHeadingCW();
    }

    public RobotOrientation getCurrentFrameRobotOrientation() {
        return this.currentFrameRobotOrientation;
    }

    public RobotOrientation getLastFrameRobotOrientation() {
        return this.lastFrameRobotOrientation;
    }

    public RobotOrientation getStartFrameRobotOrientation() {
        return this.startFrameRobotOrientation;
    }
    public boolean isRobotTilted() {
        return Math.abs(this.getCurrentFrameTilt() - this.startTilt) > 0.05;
    }
}
