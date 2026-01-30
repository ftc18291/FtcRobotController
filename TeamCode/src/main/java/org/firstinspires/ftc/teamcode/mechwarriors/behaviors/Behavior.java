package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Behavior {
    String name;
    Telemetry telemetry;
    boolean isDone = false;

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
