package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.NLauncher;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.PLauncher;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors.Behavior;


public class NLauncherOff extends Behavior {
    Telemetry telemetry;
    NLauncher nLauncher;


    public NLauncherOff(Telemetry telemetry, NLauncher nLauncher) {
        this.telemetry = telemetry;
        this.nLauncher = nLauncher;
    }

    @Override
    public void start() {
        nLauncher.stopNLauncher();
    }

    @Override
    public void run() {
        isDone = true;
    }
}

