package eu.qrobotics.powerplay.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import eu.qrobotics.powerplay.teamcode.hardware.CachingDcMotorEx;

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
        AUTOMATIC,
        MANUAL
    }

    public enum TargetHeight {
        GROUND(0),
        LOW(90) {
            @Override
            public TargetHeight previous() {
                return this;
            }
        },
        MID(380),
        HIGH(755){
            @Override
            public TargetHeight next() {
                return this;
            }
        },
        LOW_TILTED(0),
        MID_TILTED(380),
        HIGH_TILTED(740),
        AUTO_DROP_MID(320),
        AUTO_DROP(680);

        public final int encoderPosition;

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

    public static PIDCoefficients coef = new PIDCoefficients(0.012, 0.08, 0.0004);
    private PIDController pid = new PIDController(coef.p, coef.i, coef.d);
    public static double f1 = 0.07, f2 = 0.00007;

    private int downPosition;
    private int lastEncoder;
    public double offsetPosition;
    public TargetHeight targetPosition = TargetHeight.GROUND;
    public ElevatorMode elevatorMode = ElevatorMode.AUTOMATIC;
    public double manualPower;

    private CachingDcMotorEx motorLeft, motorRight;
    private Robot robot;

    Elevator(HardwareMap hardwareMap) {
        motorLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "elevatorLeft"));
        motorRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "elevatorRight"));

//        motorLeft.setDirection(DcMotor.Direction.REVERSE);
//        motorRight.setDirection(DcMotor.Direction.REVERSE);

        //motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        downPosition = getRawEncoder();

        targetPosition = TargetHeight.GROUND;
        elevatorMode = ElevatorMode.AUTOMATIC;
    }

    public int getRawEncoder() {
        return motorLeft.getCurrentPosition();
    }

    public int updateEncoder() {
        lastEncoder = getRawEncoder() - downPosition;
        return lastEncoder;
    }

    public int getEncoder() {
        return lastEncoder;
    }

    public int getLastEncoder() {
        return lastEncoder;
    }

    public int getTargetEncoder() {
        return targetPosition.getEncoderPosition();
    }

    public int getDistanceLeft() {
        return getTargetEncoder() - lastEncoder + (int)offsetPosition;
    }

    public TargetHeight getTargetPosition() {
        return targetPosition;
    }

    private void setPower(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public double leftPowah, rightPowah;
    private void getEnergy() {
        leftPowah = motorLeft.getCurrent(CurrentUnit.AMPS);
        rightPowah = motorRight.getCurrent(CurrentUnit.AMPS);
    }
    public static boolean IS_DISABLED = false;
    @Override
    public void update() {
        if(IS_DISABLED) return;
        if (elevatorMode == ElevatorMode.DISABLED)
            return;

        updateEncoder();
        getEnergy();

        if (elevatorMode == ElevatorMode.MANUAL) {
            setPower(manualPower);
        } else {
            int targetPos = targetPosition.encoderPosition + (int)offsetPosition;
            int current = motorLeft.getCurrentPosition();
            pid.setPID(coef.p, coef.i, coef.d);
            double ff = f1 + f2 * current;
            double power = pid.calculate(current, targetPos) + ff;
            setPower(power);
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