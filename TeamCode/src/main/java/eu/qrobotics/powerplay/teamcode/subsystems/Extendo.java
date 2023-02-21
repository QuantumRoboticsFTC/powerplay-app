package eu.qrobotics.powerplay.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import eu.qrobotics.powerplay.teamcode.hardware.CachingDcMotorEx;

@Config
public class Extendo implements Subsystem {

    public static int THRESHOLD_DOWN = 15;
    public static int THRESHOLD_DOWN_LEVEL_1 = 30;
    public static int THRESHOLD_DOWN_LEVEL_2 = 60;
    public static int THRESHOLD_DOWN_LEVEL_3 = 100;
    public static int THRESHOLD = 20;
    public static int THRESHOLD_LEVEL_1 = 10;
    public static int THRESHOLD_LEVEL_2 = 100;
    public static int THRESHOLD_LEVEL_3 = 250;
    public static double DOWN_POWER_1 = -1;
    public static double DOWN_POWER_2 = -1;
    public static double DOWN_POWER_3 = -1;
    public static double DOWN_POWER_4 = -1;
    public static double HOLD_POWER = 0;
    public static double LEVEL_1_POWER = 0.4;
    public static double LEVEL_2_POWER = 0.5;
    public static double LEVEL_3_POWER = 0.75;
    public static double LEVEL_4_POWER = 1;

    public static double FAST_SPEED_MULTIPLIER = 1;
    public static double SLOW_SPEED_MULTIPLIER = 0.5;

    public enum ExtendoMode {
        DISABLED,
        RETRACTED,
        UP,
        MANUAL
    }

    public enum ManualSpeedMode {
        FAST,
        SLOW
    }

    public enum TargetHeight {
        AUTO_CONE5(553) {
            @Override
            public Extendo.TargetHeight previous() {
                return this;
            }
        },
        AUTO_CONE4(815) {
            @Override
            public Extendo.TargetHeight previous() {
                return this;
            }
        },
        AUTO_CONE3(815) {
            @Override
            public Extendo.TargetHeight previous() {
                return this;
            }
        },
        AUTO_CONE2(807) {
            @Override
            public Extendo.TargetHeight previous() {
                return this;
            }
        },
        AUTO_CONE1(805) {
            @Override
            public Extendo.TargetHeight previous() {
                return this;
            }
        },
        HIGH(750){
            @Override
            public Extendo.TargetHeight next() {
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

        public Extendo.TargetHeight previous() {
            return values()[ordinal() - 1];
        }

        public Extendo.TargetHeight next() {
            return values()[ordinal() + 1];
        }
    }

    private int downPosition;
    private int lastEncoder;
    public double offsetPosition;
    public TargetHeight targetPosition;
    public double manualPower;

    public ExtendoMode extendoMode;
    public ManualSpeedMode manualSpeedMode;
    private CachingDcMotorEx motor;
    private Robot robot;

    Extendo(HardwareMap hardwareMap) {

        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor"));

        motor.setDirection(DcMotor.Direction.REVERSE);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        downPosition = getRawEncoder();

        //targetPosition = TargetHeight.AUTO;
    }

    public int getRawEncoder() {
        return motor.getCurrentPosition();
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
        if(extendoMode == ExtendoMode.RETRACTED) {
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
        motor.setPower(power);
    }
    public static boolean IS_DISABLED = false;
    @Override
    public void update() {
        if(IS_DISABLED) return;
        if (extendoMode == ExtendoMode.DISABLED)
            return;

        updateEncoder();

        if (extendoMode == ExtendoMode.RETRACTED) {
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
        } else if (extendoMode == ExtendoMode.UP) {
            int distanceLeft = targetPosition.getEncoderPosition() - getEncoder() + (int) offsetPosition;
            if (Math.abs(distanceLeft) <= THRESHOLD)
                setPower(HOLD_POWER );
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


//    public void nextStone() {
//        targetPosition = targetPosition.next();
//    }
//
//    public void previousStone() {
//        targetPosition = targetPosition.previous();
//    }
}