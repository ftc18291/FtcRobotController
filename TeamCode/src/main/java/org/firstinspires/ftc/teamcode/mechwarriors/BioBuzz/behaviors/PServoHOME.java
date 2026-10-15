package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.PServo;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors.Behavior;

public class PServoHOME extends Behavior {
    Telemetry telemetry;
    PServo pServo;

    public PServoHOME(Telemetry telemetry, PServo pServo) {
        this.telemetry = telemetry;
        this.pServo = pServo;
    }

    @Override
    public void start() {
        pServo.setPositionHOME();
    }

    @Override
    public void run() {
        isDone = true;
    }
}
