package org.firstinspires.ftc.teamcode.utilities.robot.extensions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import java.util.Arrays;
import java.util.List;

public class MotorGroup<T extends DcMotor> {

    List<T> motors;
    int size;

    @SafeVarargs
    public MotorGroup(T... varargMotors) {
        this.motors = Arrays.asList(varargMotors);

        this.size = varargMotors.length;
    }

    public void setPower(double power) {
        for (T motor : motors) {
            motor.setPower(power);
        }
    }

    public int[] getPositions() {

        int[] positionArray = new int[this.size];

        for (int i = 0; i < this.size; i++) {
            positionArray[i] = motors.get(i).getCurrentPosition();
        }

        return positionArray;
    }

    public int getAveragePosition() {

        int[] positionArray = this.getPositions();

        int sum = 0;

        for (int position : positionArray) {
            sum += position;
        }

        return Math.floorDiv(sum, this.size) ;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior newBehavior) {
        for (T motor : motors) {
            motor.setZeroPowerBehavior(newBehavior);
        }
    }

    public void setRunMode(DcMotor.RunMode mode) {
        for (T motor : motors) {
            motor.setMode(mode);
        }
    }
}
