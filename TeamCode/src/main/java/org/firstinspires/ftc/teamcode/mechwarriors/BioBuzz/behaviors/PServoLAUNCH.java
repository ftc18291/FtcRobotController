package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.PServo;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors.Behavior;

public class PServoLAUNCH extends Behavior {
    Telemetry telemetry;
    PServo pnServo;

    public PServoLAUNCH(Telemetry telemetry, PServo pnServo) {
        this.telemetry = telemetry;
        this.pnServo = pnServo;
    }

    @Override
    public void start() {
        pnServo.setPositionLAUNCH();
    }

    @Override
    public void run() {
        isDone = true;
    }
}
