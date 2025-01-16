package org.firstinspires.ftc.teamcode.mechwarriors;


public enum LiftHeight {

    BOTTOM(0),    // 0"
    RETRIEVE(400),  // ~3"
    LOW(1900), // 23.5"
    HIGH(3900);   // 33.5"

    private final int ticks;

    LiftHeight(final int ticks) {
        this.ticks = ticks;
    }

    public int getTicks() {
        return ticks;
    }
}