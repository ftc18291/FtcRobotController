package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.PNLauncher;

public class PNLauncherOn extends Behavior {
    Telemetry telemetry;
    PNLauncher pnLauncher;


    public PNLauncherOn(Telemetry telemetry, PNLauncher pnLauncher) {
        this.telemetry = telemetry;
        this.pnLauncher = pnLauncher;
    }

    @Override
    public void start() {
        pnLauncher.startPNLauncher();
    }

    @Override
    public void run() {

    }
}
