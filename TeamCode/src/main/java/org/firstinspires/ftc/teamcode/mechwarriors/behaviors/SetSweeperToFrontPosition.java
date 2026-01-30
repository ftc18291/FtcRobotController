package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactIntaker;

public class SetSweeperToFrontPosition extends Behavior {

    Telemetry telemetry;
    ArtifactIntaker artifactIntaker;

    public SetSweeperToFrontPosition(Telemetry telemetry, ArtifactIntaker artifactIntaker) {
        this.telemetry = telemetry;
        this.artifactIntaker = artifactIntaker;
    }

    @Override
    public void start() {
        artifactIntaker.setSweeperToFrontPosition();
    }
    @Override
    public void run() {
            isDone = true;
    }
}
