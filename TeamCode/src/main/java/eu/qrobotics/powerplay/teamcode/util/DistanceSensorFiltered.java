package eu.qrobotics.powerplay.teamcode.util;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorFiltered {
    private Rev2mDistanceSensor sensor;
    private MovingStatistics sensorAverage = new MovingStatistics(16);
    private KalmanFilter sensorFilter = new KalmanFilter(0, 25);

    public DistanceSensorFiltered(Rev2mDistanceSensor sensor) {
        this.sensor = sensor;
    }

    public void reset() {
        sensorFilter.reset();
    }

    public double getDistance() {
        double reading = sensor.getDistance(DistanceUnit.MM);

        if(reading > 65000) {
            sensor.initialize();
        }

        sensorAverage.add(reading);

        return sensorFilter.update(reading);//, sensorAverage.getStandardDeviation() * sensorAverage.getStandardDeviation());
    }
}
