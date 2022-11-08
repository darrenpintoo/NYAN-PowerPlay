package org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler;

public class MotionProfileTestCommandLine {

    public static void main(String[] args) {

        MotionProfile motionProfile = new MotionProfile(0, 10, 1, 1);

        double duration = motionProfile.getDuration();

        System.out.println(duration);

        final double PARTITIONS = 5000;

        for (double t = 0; t < duration; t += (duration / PARTITIONS)) {

            System.out.print(
                    "Time: " + t +
                            " Position: " + motionProfile.getPositionFromTime(t) +
                            " Velocity: " + motionProfile.getVelocityFromTime(t) +
                            " Acceleration: " + motionProfile.getAccelerationFromTime(t) + "\n"
            );
        }

    }
}
