package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.statehandling.Debounce;
import org.firstinspires.ftc.teamcode.utilities.statehandling.DebounceObject;

@Config
public class Claw implements Subsystem {


    Debounce debounce = new Debounce(
            new DebounceObject(ClawStates.OPEN_REQUESTED.toString(), 1000)
    );

    Lift lift;
    ClawTilt clawTilt;

    public static double TIME_THRESHOLD = 0.25;
    public static int RED_THRESHOLD = 50;
    public static int BLUE_THRESHOLD = 50;
    public static double DISTANCE_THRESHOLD = 3;
    public static double vMax = 100;
    public static double aMax = 100;

    public enum ClawStates {
        OPENED, CLOSED, SLIGHTLY_OPENED, OPEN_REQUESTED
    }

    public enum ClawPositions {
        OPEN, CLOSE, SLIGHTLY_OPENED
    }

    public Servo clawGrabberServo;
    public ColorRangeSensor clawColorSensor;

    private Telemetry telemetry;

    //need to tune still
    public static double openPosition = 0.38;
    public static double closePosition = 0;
    public static double slightlyOpenPosition = 0.25;

    private boolean enableAutoClose = true;

    private ClawStates currentClawState = ClawStates.OPENED;
    private double previousClawPosition = openPosition;

    private int currentFrameBlue;
    private int currentFrameRed;

    private double currentDistance;

    private boolean coneInClawLast = false;
    private boolean requestedLift = false;

    private ElapsedTime requestedTime = new ElapsedTime();

    private ElapsedTime profileTime = new ElapsedTime();
    private MotionProfile clawProfile = new MotionProfile(openPosition, openPosition, vMax, aMax);

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        this.clawGrabberServo = hardwareMap.get(Servo.class, "clawGrabber");
        this.clawColorSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
        this.telemetry = telemetry;

        // this.onCyclePassed();
    }

    @Override
    public void onOpmodeStarted() {
        this.lift = RobotEx.getInstance().lift;
        this.clawTilt = RobotEx.getInstance().clawTilt;
        this.onCyclePassed();
    }

    @Override
    public void onCyclePassed() {

        this.cacheCurrentFrameValues();

        boolean coneInClaw = this.checkConeInClaw();

        telemetry.addData("Red: ", this.currentFrameRed);
        telemetry.addData("Blue: ", this.currentFrameBlue);
        telemetry.addData("Distance: ", this.currentDistance);

        telemetry.addData("Current Claw State: ", this.currentClawState.toString());
        if (this.enableAutoClose) {

            if (this.currentClawState == ClawStates.OPEN_REQUESTED) {
                if (debounce.check(ClawStates.OPEN_REQUESTED.toString())) {
                    this.setClawState(ClawStates.OPENED);
                }
            } else if (this.checkConeInClaw() && this.currentClawState != ClawStates.CLOSED) {
                // telemetry.addLine("Closing claw");
                this.setClawState(ClawStates.CLOSED);

                if (this.checkConeInClaw() != this.coneInClawLast) {
                    this.requestedLift = true;
                    this.requestedTime.reset();
                }
            } else if (!coneInClaw) {
                this.setClawState(ClawStates.OPENED);
                this.requestedLift = false;
            }

            if (this.requestedLift && this.requestedTime.seconds() > TIME_THRESHOLD) {
                if (this.lift.getCurrentLiftTarget() == Lift.LIFT_POSITIONS.DEFAULT) {
                    this.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.GROUND_JUNCTION);
                }


                this.requestedLift = false;
            }
        }

        if (this.lift.getCurrentLiftTarget() != Lift.LIFT_POSITIONS.DEFAULT && this.lift.getCurrentLiftTarget() != Lift.LIFT_POSITIONS.GROUND_JUNCTION) {
            if (this.currentClawState == ClawStates.CLOSED && coneInClaw) {
                if (this.getServoPosition() < closePosition+0.1) {
                    this.clawTilt.setCurrentState(ClawTilt.tiltState.ACTIVE);
                }
            }
        } else {
            this.clawTilt.setCurrentState(ClawTilt.tiltState.DEFAULT);
        }
        this.coneInClawLast = this.checkConeInClaw();
        clawGrabberServo.setPosition(getServoPosition(this.currentClawState));
    }

   public void setClawState(ClawStates newClawState) {

        if (this.currentClawState == ClawStates.OPEN_REQUESTED) {
            if (!debounce.checkAndUpdate(ClawStates.OPEN_REQUESTED.toString())) {
                return;
            } else {
                this.currentClawState = ClawStates.OPENED;
            }
        }

        switch (newClawState) {
            case OPENED:
                if (this.currentClawState == ClawStates.CLOSED && this.checkConeInClaw()) {
                    debounce.reset(ClawStates.OPEN_REQUESTED.toString());
                    this.currentClawState = ClawStates.OPEN_REQUESTED;
                } else {
                    this.currentClawState = newClawState;
                }
                break;
            case CLOSED:
                this.currentClawState = ClawStates.CLOSED;
                break;
            case OPEN_REQUESTED:
                this.currentClawState = newClawState;
                debounce.reset(ClawStates.OPEN_REQUESTED.toString());
                break;
        }
       createProfile();
    }

    private void createProfile() {
        this.clawProfile = new MotionProfile(
                this.getServoPosition(),
                this.getServoPosition(this.currentClawState),
                vMax,
                aMax
        );
        this.profileTime.reset();
    }

    public double getServoPosition() {
        return this.clawProfile.getPositionFromTime(this.profileTime.seconds());
    }

    public double getServoPosition(ClawPositions clawPosition) {
        switch (clawPosition) {
            case OPEN:
                return openPosition;
            case CLOSE:
                return closePosition;
            case SLIGHTLY_OPENED:
                return slightlyOpenPosition;
            default:
                this.telemetry.addLine("Error with Claw");
        }

        return openPosition;
    }

    public double getServoPosition(ClawStates clawState) {
        if (clawState == ClawStates.CLOSED) {
            return this.getServoPosition(ClawPositions.CLOSE);
        } else if (clawState == ClawStates.SLIGHTLY_OPENED) {
            return this.getServoPosition(ClawPositions.SLIGHTLY_OPENED);
        }

        return this.getServoPosition(ClawPositions.OPEN);
    }

    public void cacheCurrentFrameValues() {
        this.currentFrameBlue = this.getBlueColorFromI2C1();
        this.currentFrameRed = this.getRedColorFromI2C1();
        this.currentDistance = this.getDistanceFromI2C();
    }

    private int getRedColorFromI2C1() {
        return this.clawColorSensor.red();
    }

    private int getBlueColorFromI2C1() {
        return this.clawColorSensor.blue();
    }

    public int getBlueColor() {
        return this.currentFrameBlue;
    }

    public int getRedColor() {
        return this.currentFrameRed;
    }

    public boolean checkConeInClaw() {
        return (this.checkBlueConeInClaw() || this.checkRedConeInClaw()) && this.checkDistanceFromClaw();
    }

    public boolean checkRedConeInClaw() {
        return this.getRedColor() > Claw.RED_THRESHOLD ;
    }

    public boolean checkBlueConeInClaw() {
        return this.getBlueColor() > Claw.BLUE_THRESHOLD;
    }

    public boolean checkDistanceFromClaw() {
        return this.currentDistance < DISTANCE_THRESHOLD;
    }

    public double getDistanceFromI2C() {
        return this.clawColorSensor.getDistance(DistanceUnit.INCH);
    }

    public void enableAutoClose() {
        this.enableAutoClose = true;
    }

    public void disableAutoClose() {
        this.enableAutoClose = false;
    }

    public void setServoPosition(double position) {
        this.clawGrabberServo.setPosition(position);
    }
}


