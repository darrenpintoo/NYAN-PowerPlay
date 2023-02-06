package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.statehandling.Debounce;
import org.firstinspires.ftc.teamcode.utilities.statehandling.DebounceObject;

@Config
public class Claw implements Subsystem {


    Debounce debounce = new Debounce(
            new DebounceObject("OPEN_REQUESTED", 2000)
    );

    Lift lift;

    public static double TIME_THRESHOLD = 0.25;
    public static int RED_THRESHOLD = 30;
    public static int RED_THRESHOLD2 = 100;
    public static int BLUE_THRESHOLD = 30;
    public static int BLUE_THRESHOLD2 = 100;


    public enum ClawStates {
        OPENED, CLOSED, SLIGHTLY_OPENED
    }

    public enum ClawPositions {
        OPEN, CLOSE, SLIGHTLY_OPENED
    }

    public Servo clawGrabberServo;
    public ColorRangeSensor clawColorSensor;
    public ColorRangeSensor clawColorSensor2;

    private Telemetry telemetry;

    //need to tune still
    public static double openPosition = 0.38;
    public static double closePosition = 0;
    public static double slightlyOpenPosition = 0.2;

    private boolean enableAutoClose = true;

    private ClawStates currentClawState = ClawStates.OPENED;

    private int currentFrameBlue;
    private int currentFrameRed;

    private double currentDistance;

    private boolean coneInClawLast = false;
    private boolean requestedLift = false;

    private ElapsedTime requestedTime = new ElapsedTime();

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        this.clawGrabberServo = hardwareMap.get(Servo.class, "clawGrabber");
        this.clawColorSensor = hardwareMap.get(ColorRangeSensor.class, "ConeDetector");
        this.clawColorSensor2 = hardwareMap.get(ColorRangeSensor.class, "ConeDetector2");
        this.telemetry = telemetry;

        // this.onCyclePassed();
    }

    @Override
    public void onOpmodeStarted() {
        this.lift = RobotEx.getInstance().lift;
        this.onCyclePassed();
    }

    @Override
    public void onCyclePassed() {


        this.cacheCurrentFrameColors();

        telemetry.addData("Red: ", this.currentFrameRed);
        telemetry.addData("Blue: ", this.currentFrameBlue);

        // this.currentDistance = this.getDistanceFromI2C();

/*
        this.telemetry.addData("Claw -> Red: ", this.getRedColor());
        this.telemetry.addData("Claw -> Blue: ", this.getBlueColor());
        // this.telemetry.addData("Claw -> Distance: ", this.currentDistance);

        this.telemetry.addData("Claw Servo Pos: ", this.clawGrabberServo.getPosition());
        this.telemetry.addData("Cone in grabber: ", this.checkConeInClaw());
*/

        if (this.enableAutoClose) {
            if (this.checkConeInClaw() != this.coneInClawLast && this.checkConeInClaw() && this.currentClawState != ClawStates.CLOSED) {
                this.setClawState(ClawStates.CLOSED);

                this.requestedLift = true;
                this.requestedTime.reset();
            }

            if (!this.checkConeInClaw()) {
                this.requestedLift = false;
            }
            if (this.requestedLift && this.requestedTime.seconds() > TIME_THRESHOLD) {
                if (this.lift.getCurrentLiftTarget() == Lift.LIFT_POSITIONS.DEFAULT) {
                    this.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.GROUND_JUNCTION);
                }

                this.requestedLift = false;
            }
/*            if (this.lift.getCurrentLiftTarget() != Lift.LIFT_POSITIONS.DEFAULT || this.lift.getOffset() != 0) {
                return;
            }


            if (this.currentClawState == ClawStates.CLOSED) {
                if (this.requestedLift && this.requestedTime.seconds() > TIME_THRESHOLD) {
                    this.lift.setCurrentLiftTargetPosition(Lift.LIFT_POSITIONS.GROUND_JUNCTION);
                }
            } else {
                this.cacheCurrentFrameColors();

                if (this.checkConeInClaw()) {
                    this.setClawState(ClawStates.CLOSED);

                    this.requestedLift = true;
                    this.requestedTime.reset();
                }
            }*/
        }
        this.coneInClawLast = this.checkConeInClaw();

        /*
        switch (this.currentClawState) {
            case OPENED:
                telemetry.addLine("opened");
                if (this.checkConeInClaw()) {
                    telemetry.addLine("Cone in grabber");
                    this.setClawStateOverride(ClawStates.CLOSED);
                }
                break;
            case CLOSED:
                telemetry.addLine("closed");
                if (!this.checkRedConeInClaw()) {
                    this.setClawStateOverride(ClawStates.OPENED);
                    this.debounce.update("OPEN_REQUESTED");
                }
                break;
            case OPEN_REQUESTED:
                telemetry.addLine("open request");
                if (this.debounce.check("OPEN_REQUESTED")) {
                    this.setClawStateOverride(ClawStates.OPENED);
                }
                break;
        }

         */

        clawGrabberServo.setPosition(getServoPosition(this.currentClawState));
    }

    public void setClawState(ClawStates newClawState) {
        this.currentClawState = newClawState;

        if (newClawState != ClawStates.CLOSED) {
            this.requestedLift = false;
        }
    }

/*    public void setClawState(ClawStates newClawState) {
        switch (newClawState) {
            case OPENED:
                if (this.currentClawState == ClawStates.CLOSED) {
                    this.currentClawState = ClawStates.OPEN_REQUESTED;
                } else {
                    this.currentClawState = newClawState;
                }
                break;
            case CLOSED:
                if (!(this.currentClawState == ClawStates.OPEN_REQUESTED)) {
                    this.currentClawState = ClawStates.CLOSED;
                } else {
                    this.currentClawState = newClawState;
                }
                break;
            case OPEN_REQUESTED:
                this.currentClawState = newClawState;
                break;
        }
    }*/

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

    public void cacheCurrentFrameColors() {
        this.currentFrameBlue = this.getBlueColorFromI2C1() / 2;
        this.currentFrameRed = this.getRedColorFromI2C1() / 2;
    }

    private int getRedColorFromI2C1() {
        return this.clawColorSensor.red();
    }

    private int getBlueColorFromI2C1() {
        return this.clawColorSensor.blue();
    }

    private int getRedColorFromI2C2() {
        return this.clawColorSensor2.red();
    }

    private int getBlueColorFromI2C2() {
        return this.clawColorSensor2.blue();
    }

    public int getBlueColor() {
        return this.currentFrameBlue;
    }

    public int getRedColor() {
        return this.currentFrameRed;
    }

    public boolean checkConeInClaw() {
        return this.checkBlueConeInClaw() || this.checkRedConeInClaw();
    }

    public boolean checkRedConeInClaw() {
        return this.getRedColor() < Claw.RED_THRESHOLD || this.getRedColor() > RED_THRESHOLD2;
    }

    public boolean checkBlueConeInClaw() {
        return this.getBlueColor() < Claw.BLUE_THRESHOLD || this.getBlueColor() > BLUE_THRESHOLD2;
    }

    public boolean checkDistanceFromClaw() {
        return this.currentDistance < 4;
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


