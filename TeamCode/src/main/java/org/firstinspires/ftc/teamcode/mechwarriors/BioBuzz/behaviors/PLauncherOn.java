package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.PLauncher;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors.Behavior;

public class PLauncherOn extends Behavior {
    Telemetry telemetry;
    PLauncher pLauncher;


    public PLauncherOn(Telemetry telemetry, PLauncher pnLauncher) {
        this.telemetry = telemetry;
        this.pLauncher = pnLauncher;
    }

    @Override
    public void start() {
        pLauncher.startPLauncher();
    }

    @Override
    public void run() {
        isDone = true;
    }
}
