package org.firstinspires.ftc.teamcode.mechwarriors;

public class Utilities {

    public final static double MILLIMETERS_PER_INCH = 25.4;

    /**
     * Can be used to "soften" the controls.
     *
     * @param input
     * @return
     */
    public static double squareInputWithSign(double input) {
        //System.out.println("Input: " + input); test
        double positiveInput = Math.abs(input);
        double output = positiveInput * positiveInput * positiveInput;
        if (input < 0) {
            output = output * -1;
        }
        //System.out.println("Output: " + output);
        return output;
    }
}
