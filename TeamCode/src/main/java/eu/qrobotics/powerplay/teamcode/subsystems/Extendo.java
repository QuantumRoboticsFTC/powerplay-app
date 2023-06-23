package eu.qrobotics.powerplay.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import eu.qrobotics.powerplay.teamcode.hardware.CachingDcMotorEx;


@Config
public class Extendo implements Subsystem {

    public static double ZERO_BEHAVIOUR = -0.15;

    public static double THRESHOLD_DOWN = 1.5;
    public static double THRESHOLD_DOWN_LEVEL_1 = 0.75;
    public static double THRESHOLD_DOWN_LEVEL_2 = 1.5;
    public static double THRESHOLD_DOWN_LEVEL_3 = 2.5;
    public static double THRESHOLD = 0.5;
    public static double THRESHOLD_LEVEL_1 = 0.25;
    public static double THRESHOLD_LEVEL_2 = 2.5;
    public static double THRESHOLD_LEVEL_3 = 6.5;

    public static double DOWN_POWER_1 = -1;
    public static double DOWN_POWER_2 = -1;
    public static double DOWN_POWER_3 = -1;
    public static double DOWN_POWER_4 = -1;
    public static double HOLD_POWER = 0;
    public static double LEVEL_1_POWER = 0.4;
    public static double LEVEL_2_POWER = 0.5;
    public static double LEVEL_3_POWER = 0.75;
    public static double LEVEL_4_POWER = 1;

    public static double SPOOL_RADIUS = 1.5748;
    private static final double TICKS_PER_REV = 384.5;

    public double encoderTicksToInches(double ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public double inchesToEncoderTicks(double inches) {
        return inches * TICKS_PER_REV / Math.PI / 2 / SPOOL_RADIUS;
    }

    public enum ExtendoMode {
        DISABLED,
        AUTOMATIC,
        RETRACTED,
        UP,
        BRAKE,
        MANUAL
    }

    public enum TargetPosition {
        TRANSFER(0),
        AUTO_CONE5(1.5) { // 1.6
            @Override
            public TargetPosition previous() {
                return this;
            }
        },
        AUTO_CONE4(1.1) { // 0.9
            @Override
            public TargetPosition previous() {
                return this;
            }
        },
        AUTO_CONE3(0.7) { // 0.1
            @Override
            public TargetPosition previous() {
                return this;
            }
        },
        AUTO_CONE2(0.4) { // -0.2
            @Override
            public TargetPosition previous() {
                return this;
            }
        },
        AUTO_CONE1(0) { // -0.8
            @Override
            public TargetPosition previous() {
                return this;
            }
        },
        AUTO_ROTATE_MID(-0.55),
        AUTO_ROTATE_HIGH(-0.8),
        CUSTOM(0) {
            @Override
            public TargetPosition previous() {
                return this;
            }
        };

        private final double offset;

        TargetPosition(double offset) {
            this.offset = offset;
        }

        public TargetPosition previous() {
            return values()[ordinal() - 1];
        }

        public TargetPosition next() {
            return values()[ordinal() + 1];
        }
    }

    public Vector2d targetVector2d = new Vector2d(0, 0);
    private double downPosition;
    public static double transferDelta = 0;
    public double teleopDeltaTicks = 0; // substation stuff
    private double lastEncoder;
    public double offsetPosition;
    public TargetPosition targetPosition = TargetPosition.TRANSFER;
    public double targetLength;
    public double manualPower;

    public ExtendoMode extendoMode;
    private CachingDcMotorEx motor;
    private Robot robot;

    public static PIDCoefficients coef = new PIDCoefficients(0.0075, 0.001, 0.0002);
    private PIDController pid = new PIDController(coef.p, coef.i, coef.d);
    public static double ff = 0.002;
    public static double TRANSFER_THRESHOLD = 0.001;

    public double extendoLimitTicks = downPosition + 1635;
    public static double extendoLimitDelta = 140;

    public double powah;

    double lastMotorPower;

    Extendo(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        motor.setDirection(DcMotor.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        downPosition = getRawEncoder();
    }

    public double motorPos() {
        return motor.getCurrentPosition();
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    private int getRawEncoder() {
        return motor.getCurrentPosition();
    }

    private double updateEncoder() {
        lastEncoder = getRawEncoder() - downPosition;
        return lastEncoder;
    }

    public void updateTargetLength() {
        targetLength = calculateTargetLength(targetVector2d);
    }

    public double getEncoder() {
        return lastEncoder;
    }

    public double getTargetLength() {
        if (targetPosition == TargetPosition.TRANSFER) {
            return downPosition - transferDelta;
        }
        return targetLength + targetPosition.offset;
    }

    public double getCurrentLength() {
        return encoderTicksToInches(getEncoder());
    }

    public double getDistanceLeft() {
        double distance = getTargetLength() + teleopDeltaTicks - getCurrentLength();
        distance = Math.abs(distance) + offsetPosition;
        return distance;
    }

    private void setPower(double power) {
        if (extendoLimitTicks - getEncoder() <= extendoLimitDelta &&
            power > 0) {
            motor.setPower(0);
            return;
        }
        motor.setPower(power);
    }
    public static boolean IS_DISABLED = false;

    public double signum(double val) {
        return (val != 0) ? Math.signum(val) : +1;
    }

    @Override
    public void update() {
//        powah = motor.getCurrent(CurrentUnit.AMPS);

        if(IS_DISABLED) return;
        if (extendoMode == ExtendoMode.DISABLED)
            return;

        updateEncoder();
        if (targetPosition == TargetPosition.AUTO_CONE1 ||
            targetPosition == TargetPosition.AUTO_CONE2 ||
            targetPosition == TargetPosition.AUTO_CONE3 ||
            targetPosition == TargetPosition.AUTO_CONE4 ||
            targetPosition == TargetPosition.AUTO_CONE5 ||
            targetPosition == TargetPosition.AUTO_ROTATE_HIGH) {
            updateTargetLength();
        }

        if (extendoMode == ExtendoMode.BRAKE) {
            setPower(ZERO_BEHAVIOUR);
        } else if (extendoMode == ExtendoMode.AUTOMATIC) {
            double targetPos = inchesToEncoderTicks(getTargetLength() + teleopDeltaTicks);
            double current = getEncoder();
            pid.setPID(coef.p, coef.i, coef.d);
            double power = pid.calculate(current, targetPos);
            power = power + ff * signum(power);
//            if (targetPosition == TargetPosition.TRANSFER && power <= TRANSFER_THRESHOLD) {
//                extendoMode = ExtendoMode.BRAKE;
//            }
            setPower(power);
        } else if (extendoMode == ExtendoMode.RETRACTED) {
            offsetPosition = 0;
            if (getCurrentLength() <= THRESHOLD_DOWN) {
                setPower(ZERO_BEHAVIOUR);
                extendoMode = ExtendoMode.BRAKE;
            } else if(getCurrentLength() <= THRESHOLD_DOWN_LEVEL_1)
                setPower(DOWN_POWER_1);
            else if(getCurrentLength() <= THRESHOLD_DOWN_LEVEL_2)
                setPower(DOWN_POWER_2);
            else if(getCurrentLength() <= THRESHOLD_DOWN_LEVEL_3)
                setPower(DOWN_POWER_3);
            else
                setPower(DOWN_POWER_4);
        } else if (extendoMode == ExtendoMode.UP) {
            double distanceLeft = getDistanceLeft();
            if (Math.abs(distanceLeft) <= THRESHOLD)
                setPower(HOLD_POWER);
            else if (Math.abs(distanceLeft) <= THRESHOLD_LEVEL_1)
                setPower(LEVEL_1_POWER * Math.signum(distanceLeft));
            else if (Math.abs(distanceLeft) <= THRESHOLD_LEVEL_2)
                setPower(LEVEL_2_POWER * Math.signum(distanceLeft));
            else if (Math.abs(distanceLeft) <= THRESHOLD_LEVEL_3)
                setPower(LEVEL_3_POWER * Math.signum(distanceLeft));
            else
                setPower(LEVEL_4_POWER * Math.signum(distanceLeft));
        } else {
            setPower(manualPower);
        }
    }

    public double calculateTargetLength(Vector2d targetPosition) {
        return robot.drive.getPoseEstimate().vec().distTo(targetPosition) - 18;
    }
}