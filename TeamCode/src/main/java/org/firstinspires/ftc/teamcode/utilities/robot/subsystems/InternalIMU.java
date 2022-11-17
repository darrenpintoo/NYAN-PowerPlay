package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;

/**
 * Robot IMU
 */
public class InternalIMU implements Subsystem {

    private final boolean DISABLE_VELOCITY_TRACKER = true;
    private static InternalIMU imuInstance = null;
    private BNO055IMU internalIMU;

    private Orientation currentFrameOrientation = new Orientation() ;
    private AngularVelocity currentFrameVelocity;

    private Orientation lastFrameOrientation = new Orientation();

    private double absoluteOrientation;

    private double startTilt = 0;
    // private Drivetrain drivetrain;

    private Telemetry telemetry;

    private InternalIMU() {
        if (InternalIMU.imuInstance != null) {
            throw new IllegalStateException("Robot already instantiated");
        }

        // drivetrain = RobotEx.getInstance().drivetrain;
        absoluteOrientation = 0;
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
    }

    @Override
    public void onCyclePassed() {

        double changeInHeading = 0;

        this.lastFrameOrientation = this.currentFrameOrientation;

        double previousFrameHeading = this.getPreviousFrameHeadingCCW();

        this.currentFrameOrientation = internalIMU.getAngularOrientation();
        this.currentFrameVelocity = this.DISABLE_VELOCITY_TRACKER ? null : internalIMU.getAngularVelocity();

        double currentFrameHeading = this.getCurrentFrameHeadingCCW();


/*        if (this.drivetrain.getTurnDirection() == Drivetrain.TurnDirection.LEFT) {
            if (currentFrameHeading < previousFrameHeading) {
                this.absoluteOrientation += (180 - previousFrameHeading) + (-180 - currentFrameHeading);
            } else {
                this.absoluteOrientation += currentFrameHeading - previousFrameHeading;
            }
        } else if (this.drivetrain.getTurnDirection() == Drivetrain.TurnDirection.RIGHT) {
            if (currentFrameHeading > previousFrameHeading) {
                 this.absoluteOrientation += (-180 - previousFrameHeading) + (180 - currentFrameHeading);
            } else {
                this.absoluteOrientation += currentFrameHeading - previousFrameHeading;
            }
        }*/
    }

    public Orientation getCurrentFrameOrientation() {
        return this.currentFrameOrientation;
    }
    public double getPreviousFrameHeadingCCW() {
        return -this.getPreviousFrameHeadingCW();
    }

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

    public boolean isRobotTilted() {
        return Math.abs(this.getCurrentFrameTilt() - this.startTilt) > 0.1;
    }

    public double getAbsoluteOrientation() {
        return this.absoluteOrientation;
    }
}
