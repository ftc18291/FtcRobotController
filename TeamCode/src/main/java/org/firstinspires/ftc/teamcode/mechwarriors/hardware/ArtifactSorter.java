package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.*;

public class ArtifactSorter {

    public static final double TICKS_PER_ROTATION = 587.04;
    public static final double TICKS_PER_SLOT = TICKS_PER_ROTATION / 3;
    public static final double MOVE_TO_INTAKE_POSITION_TICKS = 55;
    public static final double MOVE_TO_LAUNCH_POSITION_TICKS = TICKS_PER_SLOT - MOVE_TO_INTAKE_POSITION_TICKS;

    static final int NUMBER_OF_SLOTS = 3;

    public static double MOTOR_SPEED = 0.3;
    public static double NEW_P = 20;
    public static double NEW_I = 10;
    public static double NEW_D = 0.1;

    int position = 0;

    private ArtifactSorterMode artifactSorterMode = ArtifactSorterMode.LAUNCH;

    private double currentTicksTarget = 0;
    boolean rotating = false;
    public DcMotorEx sorterMotor;
    DigitalChannel sorterTouchSensor;



    boolean isBusy = false;

    public ArtifactSorter(HardwareMap hardwareMap) {

        sorterMotor = hardwareMap.get(DcMotorEx.class, "sorterMotor");
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, 0);
        sorterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        sorterMotor.setTargetPosition(0);
        sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sorterTouchSensor = hardwareMap.get(DigitalChannel.class, "sorterTouchSensor");
    }

    public void init() {
        sorterMotor.setPower(MOTOR_SPEED);
        sorterMotor.setTargetPosition((int) currentTicksTarget);
    }

    /**
     * Returns if the sorter is currently busy.
     * @return true if busy, else false
     */
    public boolean isBusy() {
        // If the sorter motor position is within +/- 2 ticks of the target, then we are done
        return currentTicksTarget >= sorterMotor.getCurrentPosition() - 2 &&
                currentTicksTarget <= sorterMotor.getCurrentPosition() + 2;
    }

    public void rotateOneSlot() {
        position++;
        if (position >= NUMBER_OF_SLOTS) {
            position = 0;
        }
        currentTicksTarget = currentTicksTarget + TICKS_PER_SLOT;
        sorterMotor.setTargetPosition((int) currentTicksTarget);
    }

    public void rotateBackOneSlot() {
        position--;
        if (position == 0) {
            position = 2;
        }
        currentTicksTarget = currentTicksTarget - TICKS_PER_SLOT;
        sorterMotor.setTargetPosition((int) currentTicksTarget);
    }

    public double getCurrentTicksTarget() {
        return currentTicksTarget;
    }

    public ArtifactSorterMode getArtifactSorterMode() {
        return artifactSorterMode;
    }

    public void setArtifactSorterMode(ArtifactSorterMode mode) {
        if (artifactSorterMode == mode) {
            // do nothing
        } else {
            artifactSorterMode = mode;
            if (artifactSorterMode == ArtifactSorterMode.INTAKE) {
                currentTicksTarget = currentTicksTarget + MOVE_TO_INTAKE_POSITION_TICKS;
            } else if (artifactSorterMode == ArtifactSorterMode.LAUNCH) {
                currentTicksTarget = currentTicksTarget + MOVE_TO_LAUNCH_POSITION_TICKS;
            }
            sorterMotor.setTargetPosition((int) currentTicksTarget);
        }
    }

    public void rotateToPosition(int newPosition) {
        this.position = newPosition;
        if (newPosition >= 0 && newPosition < NUMBER_OF_SLOTS) {
            sorterMotor.setPower(1);
            rotating = true;
        }
    }

    public void process() {
        if (rotating && sorterTouchSensor.getState()) {
            sorterMotor.setPower(0.2);
        } else {
            sorterMotor.setPower(0);
            rotating = false;
        }
    }

}
