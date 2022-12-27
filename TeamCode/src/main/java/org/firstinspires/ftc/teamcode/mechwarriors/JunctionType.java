package org.firstinspires.ftc.teamcode.mechwarriors;


public enum JunctionType {

    GROUND(0),    // 0"
    TRAVEL(400),  // ~3"
    LOW(2000),    // 13.5"
    MEDIUM(3300), // 23.5"
    HIGH(4000);   // 33.5"

    private final int ticks;

    JunctionType(final int ticks) {
        this.ticks = ticks;
    }

    public int getTicks() {
        return ticks;
    }
}