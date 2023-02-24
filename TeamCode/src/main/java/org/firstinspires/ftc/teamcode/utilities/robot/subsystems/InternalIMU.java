package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.RobotAngularVelocity;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.RobotOrientation;

/**
 * Robot IMU
 */
public class InternalIMU implements Subsystem {

    public static double TILT_THRESHOLD = Math.toRadians(10);
    public static double YAW_THRESHOLD = Math.toRadians(10);

    private boolean trackAngularVelocity = true;
    private static InternalIMU imuInstance = null;
    private BNO055IMU internalIMU;

    private Orientation currentFrameOrientation = new Orientation();
    private RobotOrientation currentFrameRobotOrientation = new RobotOrientation();

    private AngularVelocity currentFrameVelocity = new AngularVelocity();
    private RobotAngularVelocity currentFrameRobotVelocity = new RobotAngularVelocity();

    private Orientation lastFrameOrientation = new Orientation();
    private RobotOrientation lastFrameRobotOrientation = new RobotOrientation();

    private AngularVelocity lastFrameVelocity = new AngularVelocity();
    private RobotAngularVelocity lastFrameRobotVelocity = new RobotAngularVelocity();

    public RobotOrientation startFrameRobotOrientation = new RobotOrientation();

    private double startTilt = 0;
    private double headingOffset = 0;

    private boolean enabledHeadingOffset = false;

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

        if (trackAngularVelocity) {
            this.lastFrameVelocity = this.currentFrameVelocity;
            this.lastFrameRobotVelocity = new RobotAngularVelocity(this.lastFrameVelocity);

            this.currentFrameVelocity = internalIMU.getAngularVelocity();
            this.currentFrameRobotVelocity = new RobotAngularVelocity(this.currentFrameVelocity);

        }
    }

    @Deprecated
    public Orientation getCurrentFrameOrientation() {
        return this.currentFrameOrientation;
    }

    @Deprecated AngularVelocity getLastFrameVelocity() {
        return this.lastFrameVelocity;
    }

    @Deprecated
    public double getPreviousFrameHeadingCCW() {
        return this.enabledHeadingOffset ? -this.getPreviousFrameHeadingCW() + this.headingOffset : -this.getPreviousFrameHeadingCW();
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
        return this.enabledHeadingOffset ? -this.getCurrentFrameHeadingCW() + this.headingOffset : -this.getCurrentFrameHeadingCW();
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

    public RobotAngularVelocity getCurrentFrameRobotVelocity() {
        return this.currentFrameRobotVelocity;
    }

    public RobotAngularVelocity getLastFrameRobotVelocity() {
        return this.currentFrameRobotVelocity;
    }

    public boolean isRobotTilted() {
        return Math.abs(this.getCurrentFrameTilt() - this.startTilt) > 0.05;
    }

    public void trackAngularVelocity() {
        this.trackAngularVelocity = true;
    }

    public void stopAngularVelocityTracking() {
        this.trackAngularVelocity = false;
    }

    public void setHeadingOffset(double headingOffset) {
        this.headingOffset = headingOffset;
    }

    public void enableHeadingOffsetCorrection() {
        this.enabledHeadingOffset = true;
    }

    public void destroy() {
        InternalIMU.imuInstance = null;
    }
}
