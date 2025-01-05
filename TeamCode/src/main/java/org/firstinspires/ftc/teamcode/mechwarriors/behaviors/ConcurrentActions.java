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
            //telemetry.addData("Starting concurrent action", behavior.name);
            behavior.start();
        }
        run();
    }

    @Override
    public void run() {
        boolean allBehaviorsAreDone = true;
        for (Behavior behavior : actions) {
            //telemetry.addData("Running concurrent action", behavior.name);
            if (!behavior.isDone()) {
                //telemetry.addLine("Behavior running done");
                allBehaviorsAreDone = false;
                behavior.run();
            }
            telemetry.addLine("==============================");
        }
        if (allBehaviorsAreDone) {
            //telemetry.addLine("All concurrent actions done");
            this.isDone = true;
        }
    }
}
