package org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors;

import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.ArtifactIntaker;

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
