package org.firstinspires.ftc.teamcode.mechwarriors;

public class Utilities {

    public final static double MILLIMETERS_PER_INCH = 25.4;

    /**
     * Can be used to "soften" the controls
     *
     * @param input
     * @return
     */
    public static double squareInputWithSign(double input) {
        double output = input * input;
        if (input < 0) {
            output = output * -1;
        }
        return output;
    }
}
