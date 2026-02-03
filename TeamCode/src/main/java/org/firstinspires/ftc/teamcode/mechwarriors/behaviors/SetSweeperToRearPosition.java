package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactIntaker;

public class SetSweeperToRearPosition extends Behavior {

    Telemetry telemetry;
    ArtifactIntaker artifactIntaker;

    public SetSweeperToRearPosition(Telemetry telemetry, ArtifactIntaker artifactIntaker) {
        this.telemetry = telemetry;
        this.artifactIntaker = artifactIntaker;
    }

    @Override
    public void start() {
       artifactIntaker.setSweeperToRearPosition();
    }

    @Override
    public void run() {
        isDone = true;
    }
}