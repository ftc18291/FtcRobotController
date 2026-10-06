package org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.Miscellaneous.LiftHeight;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.LinearSlideLiftPID;

public class SetLiftHeight extends Behavior {
    LinearSlideLiftPID lift;
    LiftHeight liftHeight;

    public SetLiftHeight(Telemetry telemetry, LinearSlideLiftPID lift, LiftHeight liftHeight) {
        this.telemetry = telemetry;
        this.lift = lift;
        this.liftHeight = liftHeight;
        this.name = "Set Lift = [liftHeight: " + liftHeight + "]";
    }

    @Override
    public void start() {
        run();
    }

    @Override
    public void run() {
        lift.setPosition(liftHeight);
        telemetry.addData("Lift current", lift.getMotorCurrent());

        if (!lift.isMoving()) {
            this.isDone = true;
        }
    }
}
