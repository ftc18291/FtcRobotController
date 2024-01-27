package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;

public class OpenSweepers extends Behavior {
    Robot robot;

    public OpenSweepers(Telemetry telemetry, Robot robot) {
        this.robot = robot;
        this.name = "Open Sweepers";
        this.telemetry = telemetry;
    }

    @Override
    public void start() {
        robot.openSweepers();
        this.run();
    }

    @Override
    public void run() {
        this.isDone = true;
    }
}
