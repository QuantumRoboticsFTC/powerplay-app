package eu.qrobotics.powerplay.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class CachingMotorEx extends DcMotorImplEx {
    public CachingMotorEx(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
    }

    public CachingMotorEx(DcMotorController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }

    public CachingMotorEx(DcMotorController controller, int portNumber, Direction direction, @NonNull MotorConfigurationType motorType) {
        super(controller, portNumber, direction, motorType);
    }

    private double lastPower;

    @Override
    protected void internalSetPower(double power) {
        if (power != lastPower) {
            super.internalSetPower(power);
            lastPower = power;
        }
    }
}