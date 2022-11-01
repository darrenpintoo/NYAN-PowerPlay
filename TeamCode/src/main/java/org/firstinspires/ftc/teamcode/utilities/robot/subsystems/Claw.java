package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;

@Config
public class Claw implements Subsystem {


    public enum ClawPositions {
        OPEN, CLOSE
    }

    public Servo clawGrabberServo;

    private Telemetry telemetry;

    //need to tune still
    public static double openPosition = 0.5;
    public static double closePosition = 0.4;

    private double currentServoPosition = openPosition;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        this.clawGrabberServo = hardwareMap.get(Servo.class, "clawGrabber");

        this.telemetry = telemetry;
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {
        // Set actual servo obj position here '.setPosition()'
        clawGrabberServo.setPosition(currentServoPosition);
    }

    public void setServoPosition(ClawPositions targetClawPosition) {
        // set variable currentServoPosition here
        currentServoPosition = getServoPosition(targetClawPosition);
    }

    public double getServoPosition(ClawPositions clawPosition) {
        switch (clawPosition) {
            case OPEN:
                return openPosition;
            case CLOSE:
                return closePosition;
        }

        return 0;
    }
}


