package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.PNLauncher;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.PNServo;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors.Behavior;

public class PNServoLAUNCH extends Behavior {
    Telemetry telemetry;
    PNServo pnServo;

    public PNServoLAUNCH(Telemetry telemetry, PNServo pnServo) {
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
