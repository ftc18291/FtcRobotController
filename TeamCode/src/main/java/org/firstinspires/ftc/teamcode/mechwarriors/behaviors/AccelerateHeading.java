package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

public class AccelerateHeading extends Behavior {

    private MechRobot robot;
    public static final double MINIMUM_SPEED = 0.15;
    public static final double MAXIMUM_SPEED = 1.0;
    public static final int ACCELERATION_TIME_MS = 3000;

    private int heading;
    private int desiredTicks;
    private double currentSpeed = 0.0;
    private double targetSpeed;
    private boolean resetMotors;

    private ElapsedTime timer;

    private int accelerationTicks;
    private double calculatedSpeed;
    private AccelerationMode accelerationMode;

    public enum AccelerationMode {
        ACCELERATE, DECELERATE, COAST
    }

    public AccelerateHeading(MechRobot robot, Telemetry telemetry, int heading, int distance, double targetSpeed) {
        this.robot = robot;
        this.heading = heading;
        this.desiredTicks = distance;
        this.targetSpeed = targetSpeed;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.name = "AccelerateHeading: [heading: " + heading + "°] [distance: " + distance + " ] [ targetSpeed: " + targetSpeed + "]";

        accelerationMode = AccelerationMode.ACCELERATE;
        calculatedSpeed = MINIMUM_SPEED;
    }

    @Override
    public void start() {
        robot.resetMotorTicks();
        timer.reset();
        run();
    }

    @Override
    public void run() {
        System.out.println("motor ticks: " + robot.getDriveTicks());
        if (robot.getDriveTicks() < desiredTicks) {
            robot.mecanumDrive(0, calculateSpeed((int) robot.getDriveTicks()), 0);
        } else {
            robot.stop();
            isDone = true;
        }
    }

    @Override
    public boolean isDone() {
        return isDone;
    }

    private double calculateSpeed(int travelledTicks) {

        double elapsedTime = timer.milliseconds();

        System.out.print("Timer (ms): " + elapsedTime + ", travelledTicks: " + travelledTicks + ", desiredTicks: " + desiredTicks + ", Mode: " + accelerationMode);
        System.out.println(", currentSpeed: " + currentSpeed + ", targetSpeed: " + targetSpeed);

        if (accelerationMode == AccelerationMode.ACCELERATE &&
                (currentSpeed >= targetSpeed || travelledTicks > desiredTicks / 2)) {
            accelerationTicks = travelledTicks;
            System.out.println("accelerationTicks: " + accelerationTicks);
            accelerationMode = AccelerationMode.COAST;
            targetSpeed = currentSpeed; // reset target speed to current speed in case we did not make it to our full target speed
        }
        if (accelerationMode == AccelerationMode.COAST && travelledTicks >= desiredTicks - accelerationTicks) {
            timer.reset();
            elapsedTime = timer.milliseconds();
            accelerationMode = AccelerationMode.DECELERATE;
        }

        // 0 to 1.0 motor power
        // 0 to X ms (ACCELERATION_TIME_MS)
        calculatedSpeed = elapsedTime / ACCELERATION_TIME_MS;

        if (accelerationMode == AccelerationMode.ACCELERATE) {
            if (calculatedSpeed >= targetSpeed) {
                calculatedSpeed = targetSpeed;
            }
        } else if (accelerationMode == AccelerationMode.COAST) {
            calculatedSpeed = targetSpeed;
        } else if (accelerationMode == AccelerationMode.DECELERATE) {
            // calculate percentage of deceleration distance has been travelled
            // and apply this to the speed
            double remainingTicks = desiredTicks - travelledTicks;
            double powerRatio = remainingTicks / accelerationTicks;
            calculatedSpeed = targetSpeed * powerRatio;
            System.out.println("accelerationTicks: " + accelerationTicks + ", remainingTicks: " + remainingTicks + ", powerRatio: " + powerRatio);

            //calculatedSpeed = targetSpeed - calculatedSpeed;
            System.out.print("calculatedSpeed: " + calculatedSpeed + ", " + accelerationMode);

            if (calculatedSpeed < MINIMUM_SPEED) {
                calculatedSpeed = MINIMUM_SPEED;
            }
        }
        currentSpeed = calculatedSpeed;
        System.out.println(", final calculatedSpeed: " + calculatedSpeed);

        return calculatedSpeed;
    }

}
