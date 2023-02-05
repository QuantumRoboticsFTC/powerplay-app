package eu.qrobotics.powerplay.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake implements Subsystem {


    public enum ArmPosition {
        CONE_1 {
            @Override
            public ArmPosition previous() {
                return this;
            }
        },
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
        };

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
        TRANSFER
    }

    public enum ClawMode {
        OPEN,
        CLOSE
    }

    public static double ARM_CONE_1_POSITION = 0.95;
    public static double ARM_CONE_2_POSITION = 0.9;
    public static double ARM_CONE_3_POSITION = 0.85;
    public static double ARM_CONE_4_POSITION = 0.80;
    public static double ARM_CONE_5_POSITION = 0.76;
    public static double ARM_TRANSFER_POSITION = 0.79;

    public static double ROTATE_PARALLEL_POSITION = 0.73;
    public static double ROTATE_STRAIGHT_POSITION = 0.4;
    public static double ROTATE_TRANSFER_POSITION = 0.24;
    public static double ROTATE_PARALLEL_CONE5_POSITION = 0.45;
    public static double ROTATE_PARALLEL_CONE4_POSITION = 0.71;

    public static double CLAW_OPEN_POSITION = 0.4;
    public static double CLAW_CLOSED_POSITION = 0.61;

    public ArmPosition armPosition;
    public ArmRotate armRotate;
    public ClawMode clawMode;

    private Servo intakeArmServoLeft;
    private Servo intakeArmServoRight;
    private Servo intakeRotateServo;
    private Servo intakeClawServo;

    public Intake(HardwareMap hardwareMap, Robot robot) {
        intakeArmServoLeft = hardwareMap.get(Servo.class, "intakeArmServoLeft");
        intakeArmServoRight = hardwareMap.get(Servo.class, "intakeArmServoRight");
        intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");

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
            case CONE_1:
                intakeArmServoLeft.setPosition(ARM_CONE_1_POSITION);
                intakeArmServoRight.setPosition(ARM_CONE_1_POSITION);
                break;
            case CONE_2:
                intakeArmServoLeft.setPosition(ARM_CONE_2_POSITION);
                intakeArmServoRight.setPosition(ARM_CONE_2_POSITION);
                break;
            case CONE_3:
                intakeArmServoLeft.setPosition(ARM_CONE_3_POSITION);
                intakeArmServoRight.setPosition(ARM_CONE_3_POSITION);
                break;
            case CONE_4:
                intakeArmServoLeft.setPosition(ARM_CONE_4_POSITION);
                intakeArmServoRight.setPosition(ARM_CONE_4_POSITION);
                break;
            case CONE_5:
                intakeArmServoLeft.setPosition(ARM_CONE_5_POSITION);
                intakeArmServoRight.setPosition(ARM_CONE_5_POSITION);
                break;
            case TRANSFER:
                intakeArmServoLeft.setPosition(ARM_TRANSFER_POSITION);
                intakeArmServoRight.setPosition(ARM_TRANSFER_POSITION);
                break;
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
            case PARALLEL_CONE4:
                intakeRotateServo.setPosition(ROTATE_PARALLEL_CONE4_POSITION);
            default:
                break;
        }
        switch (clawMode) {
            case OPEN:
                intakeClawServo.setPosition(CLAW_OPEN_POSITION);
                break;
            case CLOSE:
                intakeClawServo.setPosition(CLAW_CLOSED_POSITION);
                break;
            default:
                break;
        }
    }
}