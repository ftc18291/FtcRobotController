package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorter;

public class RotateArtifactSorter extends Behavior {

    Telemetry telemetry;
    ArtifactSorter artifactSorter;

    public RotateArtifactSorter(Telemetry telemetry, ArtifactSorter artifactSorter) {
        this.telemetry = telemetry;
        this.artifactSorter = artifactSorter;
    }

    @Override
    public void start() {
        artifactSorter.rotateOneSlot();
        run();
    }

    @Override
    public void run() {
        if (!artifactSorter.isBusy()) {
            isDone = true;
        }
    }
}
