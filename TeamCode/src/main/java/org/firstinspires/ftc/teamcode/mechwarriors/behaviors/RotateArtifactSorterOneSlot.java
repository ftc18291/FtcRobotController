package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorter;

public class RotateArtifactSorterOneSlot extends Behavior {

    Telemetry telemetry;
    ArtifactSorter artifactSorter;

    ElapsedTime timer;

    public RotateArtifactSorterOneSlot(Telemetry telemetry, ArtifactSorter artifactSorter) {
        this.telemetry = telemetry;
        this.artifactSorter = artifactSorter;

        this.name = "Rotate Artifact Sorter One Slot";
        timer = new ElapsedTime();
        timer.startTime();
    }

    @Override
    public void start() {
        artifactSorter.rotateOneSlot();
        timer.reset();
    }

    @Override
    public void run() {
        if (timer.milliseconds() > 300 && !artifactSorter.isBusy()) {
            isDone = true;
        }
    }
}
