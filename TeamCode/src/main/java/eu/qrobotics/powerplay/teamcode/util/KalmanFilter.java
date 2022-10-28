package eu.qrobotics.powerplay.teamcode.util;

public class KalmanFilter {
    private final double initialState;
    private final double initialCovarianceGuess;
    private double x; // estimate
    private double p; // estimate error

    public KalmanFilter(double initialState, double initialCovarianceGuess) {
        this.initialState = initialState;
        this.initialCovarianceGuess = initialCovarianceGuess;
        reset();
    }

    public double update(double reading) {
        return update(reading, initialCovarianceGuess);
    }
    public double update(double reading, double readingError) {
        if(Double.isNaN(readingError)) {
            readingError = initialCovarianceGuess;
        }

        double K = p/(p + readingError);

        x = x + K * (reading - x);

        p = (1 - K) * p;

        return x;
    }

    public void reset() {
        this.x = initialState;
        this.p = initialCovarianceGuess;
    }
}
