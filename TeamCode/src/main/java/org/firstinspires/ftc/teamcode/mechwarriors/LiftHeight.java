package org.firstinspires.ftc.teamcode.mechwarriors;


public enum LiftHeight {

    GROUND(0),    // 0"
    TRAVEL(400),  // ~3"
    LOW(2750), // 23.5"
    HIGH(7500);   // 33.5"

    private final int ticks;

    LiftHeight(final int ticks) {
        this.ticks = ticks;
    }

    public int getTicks() {
        return ticks;
    }
}