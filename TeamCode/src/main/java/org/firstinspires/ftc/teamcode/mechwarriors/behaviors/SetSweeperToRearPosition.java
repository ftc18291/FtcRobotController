package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactIntaker;

public class SetSweeperToRearPosition extends Behavior {

    Telemetry telemetry;
    ArtifactIntaker artifactIntaker;

    ElapsedTime timer;

    AnalogInput sweeperPosition;

    public SetSweeperToRearPosition(Telemetry telemetry, ArtifactIntaker artifactIntaker) {
        this.telemetry = telemetry;
        this.artifactIntaker = artifactIntaker;
        timer = new ElapsedTime();
        timer.startTime();
    }

    @Override
    public void start() {
        artifactIntaker.setSweeperToRearPosition();
        timer.reset();
    }

    @Override
    public void run() {
        if (timer.milliseconds() >= 800) {
            isDone = true;
        }
    }
}