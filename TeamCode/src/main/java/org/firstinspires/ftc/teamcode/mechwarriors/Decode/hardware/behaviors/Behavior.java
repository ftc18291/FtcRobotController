package org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Behavior {
    protected String name;
    public Telemetry telemetry;
    protected boolean isDone = false;

    //public Behavior(String name) {
    //      this.name = name;
    // }

    public abstract void start();

    public abstract void run();

    public String getName() {
        return name;
    }
    public boolean isDone() {
        return isDone;
    }
}
