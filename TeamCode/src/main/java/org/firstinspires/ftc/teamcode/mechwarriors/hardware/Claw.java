package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Claw {
    void open();
    void close();
    void up();
    void down();
    void stop();
    void toggleOpen(boolean buttonPressed, Telemetry telemetry);
}
