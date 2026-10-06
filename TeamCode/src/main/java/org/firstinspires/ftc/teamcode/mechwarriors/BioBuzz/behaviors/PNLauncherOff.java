package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.PNLauncher;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors.Behavior;


    public class PNLauncherOff extends Behavior {
        Telemetry telemetry;
        PNLauncher pnLauncher;


        public PNLauncherOff(Telemetry telemetry, PNLauncher pnLauncher) {
            this.telemetry = telemetry;
            this.pnLauncher = pnLauncher;
        }

        @Override
        public void start() {
            pnLauncher.stopPNLauncher();
        }

        @Override
        public void run() {

        }
    }

