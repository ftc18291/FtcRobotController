package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.LiftHeight;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;

public class RaiseLift extends Behavior {
    Robot robot;
    LiftHeight junctionType;

    public RaiseLift(Telemetry telemetry, Robot robot, LiftHeight junctionType) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.junctionType = junctionType;
        this.name = "Raise Lift = [Junction Type: " + junctionType + "]";
    }



    @Override
    public void start() {
        run();
    }

    @Override
    public void run() {
        //this.telemetry.addData("lift ticks", robot.getLift().getLiftTicks());
        if (robot.getLift().getLiftTicks() < junctionType.getTicks()) {
            robot.getLift().liftArmUp();
        } else {
            robot.getLift().liftArmStop();
            this.isDone = true;
        }
    }
}
