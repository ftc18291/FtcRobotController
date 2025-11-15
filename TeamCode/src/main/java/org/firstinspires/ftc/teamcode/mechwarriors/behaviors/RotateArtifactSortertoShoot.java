package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorter;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorterMode;

public class RotateArtifactSortertoShoot extends Behavior {

    Telemetry telemetry;
    ArtifactSorter artifactSorter;

    public RotateArtifactSortertoShoot(Telemetry telemetry, ArtifactSorter artifactSorter) {
        this.telemetry = telemetry;
        this.artifactSorter = artifactSorter;
    }

    @Override
    public void start() {
        artifactSorter.setArtifactSorterMode(ArtifactSorterMode.LAUNCH);
        run();
    }

    @Override
    public void run() {
        if (!artifactSorter.isBusy()) {
            isDone = true;
        }
    }
}
