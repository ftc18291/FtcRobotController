package org.firstinspires.ftc.teamcode.mechwarriors;


public enum ClawArmPosition {

    STOW(0),
    PARK(75),
    SCORE(170),
    REACH(350),
    PICKUP(440);

    private final int ticks;

    ClawArmPosition(final int ticks) {
        this.ticks = ticks;
    }

    public int getTicks() {
        return ticks;
    }
}