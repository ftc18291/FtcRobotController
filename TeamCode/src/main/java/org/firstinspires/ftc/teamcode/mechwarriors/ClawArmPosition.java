package org.firstinspires.ftc.teamcode.mechwarriors;


public enum ClawArmPosition {

    STOW(0),
    SCORE(180),
    REACH(350),
    PICKUP(505);

    private final int ticks;

    ClawArmPosition(final int ticks) {
        this.ticks = ticks;
    }

    public int getTicks() {
        return ticks;
    }
}