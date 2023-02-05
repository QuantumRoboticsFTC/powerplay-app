package eu.qrobotics.powerplay.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Elevator implements Subsystem {

    public static int THRESHOLD_DOWN = 20;
    public static int THRESHOLD_DOWN_LEVEL_1 = 10;
    public static int THRESHOLD_DOWN_LEVEL_2 = 20;
    public static int THRESHOLD_DOWN_LEVEL_3 = 45;
    public static int THRESHOLD = 32;
    public static int THRESHOLD_LEVEL_1 = 15;
    public static int THRESHOLD_LEVEL_2 = 60;
    public static int THRESHOLD_LEVEL_3 = 130;
    public static double DOWN_POWER_1 = -0.35;
    public static double DOWN_POWER_2 = -0.5;
    public static double DOWN_POWER_3 = -0.6;
    public static double DOWN_POWER_4 = -0.7;
    public static double HOLD_POWER = 0.09;
    public static double LEVEL_1_POWER = 0.4;
    public static double LEVEL_2_POWER = 0.7;
    public static double LEVEL_3_POWER = 0.95;
    public static double LEVEL_4_POWER = 0.95;

    public enum ElevatorMode {
        DISABLED,
        DOWN,
        UP,
        MANUAL
    }

    public enum TargetHeight {
        LOW(10) {
            @Override
            public TargetHeight previous() {
                return this;
            }
        },
        MID(300),
        HIGH(770){
            @Override
            public TargetHeight next() {
                return this;
            }
        };

        private final int encoderPosition;

        TargetHeight(int encoderPosition) {
            this.encoderPosition = encoderPosition;
        }

        public int getEncoderPosition() {
            return encoderPosition;
        }

        public TargetHeight previous() {
            return values()[ordinal() - 1];
        }

        public TargetHeight next() {
            return values()[ordinal() + 1];
        }
    }

    private int downPosition;
    private int lastEncoder;
    public double offsetPosition;
    public TargetHeight targetPosition;
    public double manualPower;

    public ElevatorMode elevatorMode;
    private DcMotorEx motorLeft, motorRight;
    private Robot robot;

    Elevator(HardwareMap hardwareMap) {
//        this.robot = robot;

        motorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");

//        motorLeft.setDirection(DcMotor.Direction.REVERSE);
//        motorRight.setDirection(DcMotor.Direction.REVERSE);

        //motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        downPosition = getRawEncoder();

        targetPosition = TargetHeight.LOW;
    }

    public int getRawEncoder() {
        return motorLeft.getCurrentPosition();
    }

    public int getEncoder() {
        lastEncoder = getRawEncoder() - downPosition;
        return lastEncoder;
    }

    public int getLastEncoder() {
        return lastEncoder;
    }

    public int getTargetEncoder() {
        if(elevatorMode == ElevatorMode.DOWN) {
            return 0;
        }
        return targetPosition.getEncoderPosition();
    }

    public int getDistanceLeft() {
        return getTargetEncoder() - lastEncoder + (int) offsetPosition;
    }

    public TargetHeight getTargetPosition() {
        return targetPosition;
    }

    private void setPower(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }
    public static boolean IS_DISABLED = false;
    @Override
    public void update() {
        if(IS_DISABLED) return;
        if (elevatorMode == ElevatorMode.DISABLED)
            return;

        if (elevatorMode == ElevatorMode.DOWN) {
            offsetPosition = 0;
            if (getEncoder() <= THRESHOLD_DOWN)
                setPower(0);
            else if(getEncoder() <= THRESHOLD_DOWN_LEVEL_1)
                setPower(DOWN_POWER_1);
            else if(getEncoder() <= THRESHOLD_DOWN_LEVEL_2)
                setPower(DOWN_POWER_2);
            else if(getEncoder() <= THRESHOLD_DOWN_LEVEL_3)
                setPower(DOWN_POWER_3);
            else
                setPower(DOWN_POWER_4);
        } else if (elevatorMode == ElevatorMode.UP) {
            int distanceLeft = targetPosition.getEncoderPosition() - getEncoder() + (int) offsetPosition;
            if (Math.abs(distanceLeft) <= THRESHOLD)
                setPower(HOLD_POWER + powerForHeight(getEncoder()) / 3);
            else if (Math.abs(distanceLeft) <= THRESHOLD_LEVEL_1)
                setPower(LEVEL_1_POWER * Math.signum(distanceLeft) + powerForHeight(getEncoder()));
            else if (Math.abs(distanceLeft) <= THRESHOLD_LEVEL_2)
                setPower(LEVEL_2_POWER * Math.signum(distanceLeft) + powerForHeight(getEncoder()));
            else if (Math.abs(distanceLeft) <= THRESHOLD_LEVEL_3)
                setPower(LEVEL_3_POWER * Math.signum(distanceLeft) + powerForHeight(getEncoder()));
            else
                setPower(LEVEL_4_POWER * Math.signum(distanceLeft) + powerForHeight(getEncoder()));
        } else {
            setPower(manualPower);
        }
    }

    private double powerForHeight(int encoder) {
        if(encoder > TargetHeight.LOW.getEncoderPosition())
            return 0.1;
        if(encoder > TargetHeight.MID.getEncoderPosition())
            return 0.15;
        return 0;
    }

}