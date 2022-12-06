package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.statehandling.Debounce;
import org.firstinspires.ftc.teamcode.utilities.statehandling.DebounceObject;

@Config
public class Claw implements Subsystem {


    Debounce debounce = new Debounce(
            new DebounceObject("OPEN_REQUESTED", 2000)
    );

    Lift lift;

    public static double TIME_THRESHOLD = 1;
    public static int RED_THRESHOLD = 100;
    public static int BLUE_THRESHOLD = 100;

    public enum ClawStates {
        OPENED, CLOSED, OPEN_REQUESTED
    }

    public enum ClawPositions {
        OPEN, CLOSE
    }

    public Servo clawGrabberServo;
    public ColorRangeSensor clawColorSensor;

    private Telemetry telemetry;

    //need to tune still
    public static double openPosition = 0.3;
    public static double closePosition = 0.7;

    private ClawStates currentClawState = ClawStates.OPENED;

    private int currentFrameBlue;
    private int currentFrameRed;

    private boolean coneInClawLast = false;
    private boolean requestedLift = false;

    private ElapsedTime requestedTime = new ElapsedTime();

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        this.clawGrabberServo = hardwareMap.get(Servo.class, "clawGrabber");
        this.clawColorSensor = hardwareMap.get(ColorRangeSensor.class, "ConeDetector");

        this.telemetry = telemetry;
    }

    @Override
    public void onOpmodeStarted() {
        this.lift = RobotEx.getInstance().lift;
    }

    @Override
    public void onCyclePassed() {


        this.currentFrameBlue = this.getBlueColorFromI2C();
        this.currentFrameRed = this.getRedColorFromI2C();

        this.telemetry.addData("Claw -> Red: ", this.getRedColor());
        this.telemetry.addData("Claw -> Blue: ", this.getBlueColor());


        this.telemetry.addData("Claw Servo Pos: ", this.clawGrabberServo.getPosition());
        this.telemetry.addData("Cone in grabber: ", this.checkConeInClaw());

        if (this.checkConeInClaw() != this.coneInClawLast && this.checkConeInClaw()) {
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
            default:
                this.telemetry.addLine("Error with Claw");
        }

        return openPosition;
    }

    public double getServoPosition(ClawStates clawState) {
        if (clawState == ClawStates.CLOSED) {
            return this.getServoPosition(ClawPositions.CLOSE);
        }

        return this.getServoPosition(ClawPositions.OPEN);
    }


    private int getRedColorFromI2C() {
        return this.clawColorSensor.red();
    }

    private int getBlueColorFromI2C() {
        return this.clawColorSensor.blue();
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
        return this.getRedColor() > Claw.RED_THRESHOLD;
    }

    public boolean checkBlueConeInClaw() {
        return this.getBlueColor() > Claw.BLUE_THRESHOLD;
    }
}


