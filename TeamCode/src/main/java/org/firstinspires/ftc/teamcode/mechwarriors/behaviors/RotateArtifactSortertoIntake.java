package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorter;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorterMode;

public class RotateArtifactSortertoIntake extends Behavior {

    Telemetry telemetry;
    ArtifactSorter artifactSorter;

    public RotateArtifactSortertoIntake(Telemetry telemetry, ArtifactSorter artifactSorter) {
        this.telemetry = telemetry;
        this.artifactSorter = artifactSorter;
    }

    @Override
    public void start() {
        artifactSorter.setArtifactSorterMode(ArtifactSorterMode.INTAKE);
        run();
    }

    @Override
    public void run() {
        if (!artifactSorter.isBusy()) {
            isDone = true;
        }
    }
}
