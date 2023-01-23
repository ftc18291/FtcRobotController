package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ConcurrentActions extends Behavior {
    List<Behavior> actions = new ArrayList<>();

    public ConcurrentActions(Telemetry telemetry, Behavior... behaviors) {
        this.telemetry = telemetry;
        actions = Arrays.asList(behaviors);
    }

    @Override
    public void start() {
        for (Behavior behavior : actions) {
            behavior.start();
        }
        run();
    }

    @Override
    public void run() {
        boolean notDone = true;
        for (Behavior behavior : actions) {
            if (!behavior.isDone()) {
                behavior.run();
                notDone = true;
            }
            isDone = !notDone;
        }
    }
}
