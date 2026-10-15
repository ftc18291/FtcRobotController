package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.NServo;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.PServo;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors.Behavior;

public class NServoLAUNCH extends Behavior {
    Telemetry telemetry;
    NServo nServo;

    public NServoLAUNCH(Telemetry telemetry, NServo nServo) {
        this.telemetry = telemetry;
        this.nServo = nServo;
    }

    @Override
    public void start() {
        nServo.setPositionLAUNCH();
    }

    @Override
    public void run() {
        isDone = true;
    }
}
