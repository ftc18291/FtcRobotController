package org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.Miscellaneous.ClawArmPosition;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.ClawArmPID;

public class SetClawArmPosition extends Behavior {
    ClawArmPID clawArm;
    ClawArmPosition clawArmPosition;

    public SetClawArmPosition(Telemetry telemetry, ClawArmPID clawArm, ClawArmPosition clawArmPosition) {
        this.telemetry = telemetry;
        this.clawArm = clawArm;
        this.clawArmPosition = clawArmPosition;
        this.name = "Claw Arm Position = [clawArmPosition: " + clawArmPosition + "]";
    }

    @Override
    public void start() {
        run();
    }

    @Override
    public void run() {
        clawArm.setPosition(clawArmPosition);
        telemetry.addData("Lift current", clawArm.getMotorCurrent());

        if (!clawArm.isMoving()) {
            this.isDone = true;
        }
    }
}
