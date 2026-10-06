package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.IntakeSpinner;
import org.firstinspires.ftc.teamcode.mechwarriors.Decode.hardware.behaviors.Behavior;

public class IntakeSpinnerOn extends Behavior {
    Telemetry telemetry;
    IntakeSpinner intakeSpinner;

    public IntakeSpinnerOn(Telemetry telemetry, IntakeSpinner intakeSpinner) {
        this.telemetry = telemetry;
        this.intakeSpinner = intakeSpinner;
    }

    @Override
    public void start() {
        intakeSpinner.runIntakeSpinner();
    }

    @Override
    public void run() {isDone = true;}
}
