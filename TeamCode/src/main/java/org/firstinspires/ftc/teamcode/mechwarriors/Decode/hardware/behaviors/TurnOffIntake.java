package org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.ArtifactIntaker;

public class TurnOffIntake extends Behavior {

    Telemetry telemetry;
    ArtifactIntaker artifactIntaker;

    public TurnOffIntake(Telemetry telemetry, ArtifactIntaker artifactIntaker) {
        this.telemetry = telemetry;
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
