package org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors;

import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.ArtifactIntaker;

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
