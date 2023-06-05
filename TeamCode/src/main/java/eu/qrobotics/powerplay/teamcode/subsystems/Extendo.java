package eu.qrobotics.powerplay.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static double DOWN_ZERO_BEHAVIOUR = -0.1;

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

    private static double encoderTicksToInches(double ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public enum ExtendoMode {
        DISABLED,
        RETRACTED,
        UP,
        BRAKE,
        MANUAL
    }

    public enum ManualSpeedMode {
        FAST,
        SLOW
    }

    public enum TargetCone {
        AUTO_CONE5(1.6) {
            @Override
            public TargetCone previous() {
                return this;
            }
        },
        AUTO_CONE4(0.9) {
            @Override
            public TargetCone previous() {
                return this;
            }
        },
        AUTO_CONE3(0.1) {
            @Override
            public TargetCone previous() {
                return this;
            }
        },
        AUTO_CONE2(-0.2) {
            @Override
            public TargetCone previous() {
                return this;
            }
        },
        AUTO_CONE1(-0.8) {
            @Override
            public TargetCone previous() {
                return this;
            }
        };

        private final double offset;

        TargetCone(double offset) {
            this.offset = offset;
        }

        public TargetCone previous() {
            return values()[ordinal() - 1];
        }

        public TargetCone next() {
            return values()[ordinal() + 1];
        }
    }

    private int downPosition;
    private int lastEncoder;
    public double offsetPosition;
    public TargetCone targetCone;
    public double targetLength;
    public double manualPower;

    public ExtendoMode extendoMode;
    public ManualSpeedMode manualSpeedMode;
    private CachingDcMotorEx motor;
    private Robot robot;

    public int extendoLimitTicks = 1635;
    public static int extendoLimitDelta = 100;

    public double powah;

//    public static PIDCoefficients coef = new PIDCoefficients(0, 0, 0);
//    private PIDController pid = new PIDController(coef.p, coef.i, coef.d);

    Extendo(HardwareMap hardwareMap, Robot robot) {
        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor"));

        motor.setDirection(DcMotor.Direction.REVERSE);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        downPosition = getRawEncoder();

        //targetPosition = TargetHeight.AUTO;
    }

    private int getRawEncoder() {
        return motor.getCurrentPosition();
    }

    private int updateEncoder() {
        lastEncoder = getRawEncoder() - downPosition;
        return lastEncoder;
    }

    public int getEncoder() {
        return lastEncoder;
    }

    public double getTargetLength() {
        if (extendoMode == ExtendoMode.RETRACTED) {
            return 0;
        }
        return targetLength + targetCone.offset;
    }

    public double getCurrentLength() {
        return encoderTicksToInches(getEncoder());
    }

    public double getDistanceLeft() {
        return getTargetLength() - getCurrentLength() + offsetPosition;
    }

    private void setPower(double power) {
        motor.setPower(power);
    }
    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
//        powah = motor.getCurrent(CurrentUnit.AMPS);

        if(IS_DISABLED) return;
        if (extendoMode == ExtendoMode.DISABLED)
            return;

        updateEncoder();

        if (extendoMode == ExtendoMode.BRAKE) {
            setPower(ZERO_BEHAVIOUR);
        } else if (extendoMode == ExtendoMode.RETRACTED) {
            offsetPosition = 0;
            if (getCurrentLength() <= THRESHOLD_DOWN) {
                setPower(DOWN_ZERO_BEHAVIOUR);
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
//            switch (manualSpeedMode) {
//                case FAST:
//                    setPower(manualPower * FAST_SPEED_MULTIPLIER);
//                case SLOW:
//                    setPower(manualPower * SLOW_SPEED_MULTIPLIER);
//            }
            setPower(manualPower);
        }
    }

    public double calculateTargetLength(Vector2d targetPosition) {
        return robot.drive.getPoseEstimate().vec().distTo(targetPosition) - 18.5;
    }
}