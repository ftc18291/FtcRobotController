package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.PLauncher;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors.Behavior;


    public class PLauncherOff extends Behavior {
        Telemetry telemetry;
        PLauncher pnLauncher;


        public PLauncherOff(Telemetry telemetry, PLauncher pnLauncher) {
            this.telemetry = telemetry;
            this.pnLauncher = pnLauncher;
        }

        @Override
        public void start() {
            pnLauncher.stopPNLauncher();
        }

        @Override
        public void run() {
            isDone = true;
        }
    }

