package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactIntaker;

public class StopIntakeMotor extends Behavior {

    ArtifactIntaker artifactIntaker;

    public StopIntakeMotor(ArtifactIntaker artifactIntaker) {
        this.artifactIntaker = artifactIntaker;
    }

    @Override
    public void start() {
        artifactIntaker.stopIntakeMotor();
    }

    @Override
    public void run() {
        isDone = true;
    }
}
