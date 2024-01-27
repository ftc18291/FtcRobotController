package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;

public class CloseSweepers extends Behavior {
    Robot robot;

    public CloseSweepers(Telemetry telemetry, Robot robot) {
        this.robot = robot;
        this.name = "Close Sweepers";
        this.telemetry = telemetry;
    }

    @Override
    public void start() {
        robot.closeSweepers();
        this.run();
    }

    @Override
    public void run() {
        this.isDone = true;
    }
}
