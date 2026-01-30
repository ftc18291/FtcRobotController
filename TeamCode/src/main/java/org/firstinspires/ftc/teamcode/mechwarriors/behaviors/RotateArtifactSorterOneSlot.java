package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorter;

public class RotateArtifactSorterOneSlot extends Behavior {

    Telemetry telemetry;
    ArtifactSorter artifactSorter;

    public RotateArtifactSorterOneSlot(Telemetry telemetry, ArtifactSorter artifactSorter) {
        this.telemetry = telemetry;
        this.artifactSorter = artifactSorter;

        this.name = "Rotate Artifact Sorter One Slot";
    }

    @Override
    public void start() {
        artifactSorter.rotateOneSlot();
    }

    @Override
    public void run() {
        if (!artifactSorter.isBusy()) {
            isDone = true;
        }
    }
}
