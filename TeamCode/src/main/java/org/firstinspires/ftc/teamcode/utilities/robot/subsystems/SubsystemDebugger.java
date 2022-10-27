package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class SubsystemDebugger implements Subsystem {

    Telemetry telemetry;

    protected SubsystemDebugger(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    protected Telemetry getTelemetryInstance() {
        return this.telemetry;
    }
}
