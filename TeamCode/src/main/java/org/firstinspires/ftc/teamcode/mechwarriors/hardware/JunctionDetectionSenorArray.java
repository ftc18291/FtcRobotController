package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public class JunctionDetectionSenorArray {

    DistanceSensor leftDistanceSensor;
    DistanceSensor rightDistanceSensor;

    double averageLeftDistance = 0;
    double averageRightDistance = 0;

    List<Double> leftDistances = new ArrayList<>();
    List<Double> rightDistances = new ArrayList<>();

    public class DistanceData {
        public double left;
        public double right;

        public DistanceData(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    public JunctionDetectionSenorArray(HardwareMap hardwareMap) {
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistance");


    }

    public DistanceData detect() {
        double lastLeftDistance = leftDistanceSensor.getDistance(DistanceUnit.MM);
        if (leftDistances.size() == 10) {
            leftDistances.remove(0);
        }
        leftDistances.add(lastLeftDistance);

        double lastRightDistance = rightDistanceSensor.getDistance(DistanceUnit.MM);
        if (rightDistances.size() == 10) {
            rightDistances.remove(0);
        }
        rightDistances.add(lastRightDistance);

        averageLeftDistance = calculateAverage(leftDistances);
        averageRightDistance = calculateAverage(rightDistances);

        return new DistanceData(averageLeftDistance, averageRightDistance);
    }

    private double calculateAverage(List<Double> list) {
        double sum = 0.0;
        for (Double num : list) {
            sum += num;
        }
        return sum / list.size();
    }

    public String distancesToString() {
        return "leftDistance: " +
                averageLeftDistance +
                ", rightDistance: " +
                averageRightDistance;
    }
}
