package org.firstinspires.ftc.teamcode.mechwarriors;


public enum ClawArmPosition {

    STOW(0),
    SCORE(150),
    REACH(350),
    PICKUP(495);

    private final int ticks;

    ClawArmPosition(final int ticks) {
        this.ticks = ticks;
    }

    public int getTicks() {
        return ticks;
    }
}