package eu.qrobotics.powerplay.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class CachingServo extends ServoImpl {
    public CachingServo(ServoController controller, int portNumber) {
        super(controller, portNumber);
    }

    public CachingServo(ServoController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }

    private double lastPosition;

    @Override
    protected void internalSetPosition(double position) {
        if(position != lastPosition) {
            super.internalSetPosition(position);
            lastPosition = position;
        }
    }
}