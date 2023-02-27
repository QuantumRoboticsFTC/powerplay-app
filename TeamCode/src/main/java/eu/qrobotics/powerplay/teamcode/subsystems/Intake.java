package eu.qrobotics.powerplay.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import eu.qrobotics.powerplay.teamcode.hardware.CachingServo;

@Config
public class Intake implements Subsystem {


    public enum ArmPosition {
        LOW_POLE {
            @Override
            public ArmPosition previous() {
                return this;
            }
        },
        CONE_1,
        CONE_2,
        CONE_3,
        CONE_4,
        CONE_5 {
            @Override
            public ArmPosition next() {
                return this;
            }
        },
        TRANSFER{
            @Override
            public ArmPosition previous() {
                return this;
            }
            @Override
            public ArmPosition next() {
                return this;
            }
        },
        VERTICAL{
            @Override
            public ArmPosition previous() {
                return this;
            }
            @Override
            public ArmPosition next() {
                return this;
            }
        },
        AUTOPARK{
            @Override
            public ArmPosition previous() {
                return this;
            }
            @Override
            public ArmPosition next() {
                return this;
            }
        },
        FULL_1,
        FULL_0;

        public ArmPosition previous() {
            return values()[ordinal() - 1];
        }

        public ArmPosition next() {
            return values()[ordinal() + 1];
        }
    }

    public enum ArmRotate {
        PARALLEL,
        PARALLEL_CONE5,
        PARALLEL_CONE4,
        STRAIGHT,
        TRANSFER,
        LOW_POLE,
        LOW_POLE_DROP
    }

    public enum ClawMode {
        OPEN,
        CLOSED
    }

    public static double ARM_LOW_POLE_POSITION = 0.63;
    public static double ARM_CONE_1_POSITION = 0.96;
    public static double ARM_CONE_2_POSITION = 0.92;
    public static double ARM_CONE_3_POSITION = 0.87;
    public static double ARM_CONE_4_POSITION = 0.83;
    public static double ARM_CONE_5_POSITION = 0.78;
    public static double ARM_VERTICAL_POSITION = 0.7;
    public static double ARM_TRANSFER_POSITION = 0.8;
    public static double ARM_AUTOPARK_POSITION = 0.6;

    public static double ROTATE_PARALLEL_POSITION = 0.73;
    public static double ROTATE_STRAIGHT_POSITION = 0.47;
    public static double ROTATE_TRANSFER_POSITION = 0.225;
    public static double ROTATE_PARALLEL_CONE5_POSITION = 0.45;
    public static double ROTATE_PARALLEL_CONE4_POSITION = 0.71;
    public static double ROTATE_LOW_POLE_POSITION = 0.53;
    public static double ROTATE_LOW_POLE_DROP_POSITION = 0.6;

    public static double CLAW_OPEN_POSITION = 0.44;
    public static double CLAW_CLOSED_POSITION = 0.67;

    public static double SENSOR_TRESHOLD = 40;
    public static double armServoOffset = -0.02;

    public ArmPosition armPosition;
    public ArmRotate armRotate;
    public ClawMode clawMode;

    private CachingServo intakeArmServoLeft;
    private CachingServo intakeArmServoRight;
    private CachingServo intakeRotateServo;
    private CachingServo intakeClawServo;

    private ColorRangeSensor intakeSensor;

    public Intake(HardwareMap hardwareMap, Robot robot) {
        intakeArmServoLeft = new CachingServo(hardwareMap.get(Servo.class, "intakeArmServoLeft"));
        intakeArmServoRight = new CachingServo(hardwareMap.get(Servo.class, "intakeArmServoRight"));
        intakeRotateServo = new CachingServo(hardwareMap.get(Servo.class, "intakeRotateServo"));
        intakeClawServo = new CachingServo(hardwareMap.get(Servo.class, "intakeClawServo"));
        intakeSensor = hardwareMap.get(ColorRangeSensor.class, "intakeSensor");

        intakeArmServoRight.setDirection(Servo.Direction.REVERSE);

//        armPosition = ArmPosition.TRANSFER;
//        clawMode = ClawMode.OPEN;
//        armRotate = ArmRotate.TRANSFER;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;
        switch (armPosition) {
            case LOW_POLE:
                intakeArmServoLeft.setPosition(ARM_LOW_POLE_POSITION);
                intakeArmServoRight.setPosition(ARM_LOW_POLE_POSITION + armServoOffset);
                break;
            case CONE_1:
                intakeArmServoLeft.setPosition(ARM_CONE_1_POSITION);
                intakeArmServoRight.setPosition(ARM_CONE_1_POSITION + armServoOffset);
                break;
            case CONE_2:
                intakeArmServoLeft.setPosition(ARM_CONE_2_POSITION);
                intakeArmServoRight.setPosition(ARM_CONE_2_POSITION + armServoOffset);
                break;
            case CONE_3:
                intakeArmServoLeft.setPosition(ARM_CONE_3_POSITION);
                intakeArmServoRight.setPosition(ARM_CONE_3_POSITION + armServoOffset);
                break;
            case CONE_4:
                intakeArmServoLeft.setPosition(ARM_CONE_4_POSITION);
                intakeArmServoRight.setPosition(ARM_CONE_4_POSITION + armServoOffset);
                break;
            case CONE_5:
                intakeArmServoLeft.setPosition(ARM_CONE_5_POSITION);
                intakeArmServoRight.setPosition(ARM_CONE_5_POSITION + armServoOffset);
                break;
            case TRANSFER:
                intakeArmServoLeft.setPosition(ARM_TRANSFER_POSITION);
                intakeArmServoRight.setPosition(ARM_TRANSFER_POSITION + armServoOffset);
                break;
            case VERTICAL:
                intakeArmServoLeft.setPosition(ARM_VERTICAL_POSITION);
                intakeArmServoRight.setPosition(ARM_VERTICAL_POSITION + armServoOffset);
                break;
            case AUTOPARK:
                intakeArmServoLeft.setPosition(ARM_AUTOPARK_POSITION);
                intakeArmServoRight.setPosition(ARM_AUTOPARK_POSITION + armServoOffset);
                break;
            case FULL_1:
                intakeArmServoLeft.setPosition(1);
                intakeArmServoRight.setPosition(1);
            case FULL_0:
                intakeArmServoLeft.setPosition(0);
                intakeArmServoRight.setPosition(0);
            default:
                break;

        }
        switch (armRotate) {
            case PARALLEL:
                intakeRotateServo.setPosition(ROTATE_PARALLEL_POSITION);
                break;
            case TRANSFER:
                intakeRotateServo.setPosition(ROTATE_TRANSFER_POSITION);
                break;
            case STRAIGHT:
                intakeRotateServo.setPosition(ROTATE_STRAIGHT_POSITION);
                break;
            case PARALLEL_CONE5:
                intakeRotateServo.setPosition(ROTATE_PARALLEL_CONE5_POSITION);
                break;
            case PARALLEL_CONE4:
                intakeRotateServo.setPosition(ROTATE_PARALLEL_CONE4_POSITION);
                break;
            case LOW_POLE:
                intakeRotateServo.setPosition(ROTATE_LOW_POLE_POSITION);
                break;
            case LOW_POLE_DROP:
                intakeRotateServo.setPosition(ROTATE_LOW_POLE_DROP_POSITION);
                break;
            default:
                break;
        }
        switch (clawMode) {
            case OPEN:
                intakeClawServo.setPosition(CLAW_OPEN_POSITION);
                break;
            case CLOSED:
                intakeClawServo.setPosition(CLAW_CLOSED_POSITION);
                break;
            default:
                break;
        }
    }
    public boolean hasCone() {
        return intakeSensor.getDistance(DistanceUnit.MM) < SENSOR_TRESHOLD;
    }
}
