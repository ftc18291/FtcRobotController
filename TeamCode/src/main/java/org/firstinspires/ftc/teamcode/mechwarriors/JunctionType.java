package org.firstinspires.ftc.teamcode.mechwarriors;


public enum JunctionType {

    GROUND(0),    // 0"
    TRAVEL(400),  // ~3"
    LOW(800),    // 13.5"
    MEDIUM(1600), // 23.5"
    HIGH(2460);   // 33.5"

    private final int ticks;

    JunctionType(final int ticks) {
        this.ticks = ticks;
    }

    public int getTicks() {
        return ticks;
    }
}