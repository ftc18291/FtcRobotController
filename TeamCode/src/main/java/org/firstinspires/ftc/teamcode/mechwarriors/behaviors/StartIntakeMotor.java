package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactIntaker;

public class StartIntakeMotor extends Behavior {

    ArtifactIntaker artifactIntaker;

    public StartIntakeMotor(ArtifactIntaker artifactIntaker) {
        this.artifactIntaker = artifactIntaker;
    }

    @Override
    public void start() {
        artifactIntaker.runIntakeMotor();
    }

    @Override
    public void run() {
        isDone = true;
    }
}
