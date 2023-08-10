package eu.qrobotics.powerplay.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import eu.qrobotics.powerplay.teamcode.hardware.CachingServo;

@Config
public class Intake implements Subsystem {

    public enum ArmPosition {
        LOW_POLE {
            @Override
            public ArmPosition previous() {
                return TRANSFER;
            }

            @Override
            public ArmPosition next() {
                return CONE_1;
            }
        },
        LOW_POLE_WHEN_IN_TRANSFER {
            @Override
            public ArmPosition previous() {
                return TRANSFER;
            }
        },
        CONE_1,
        CONE_2,
        CONE_3,
        CONE_4,
        CONE_5,
        TRANSFER{
            @Override
            public ArmPosition next() {
                return LOW_POLE_WHEN_IN_TRANSFER;
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
        STRAIGHT,
        TRANSFER,
        LOW_POLE,
        LOW_POLE_DROP
    }

    public enum ClawMode {
        OPEN,
        CLOSED
    }

    public static double ARM_LOW_POLE_POSITION = 0.65;
    public static double ARM_CONE_1_POSITION = 0.99;
    public static double ARM_CONE_2_POSITION = 0.96;
    public static double ARM_CONE_3_POSITION = 0.91;
    public static double ARM_CONE_4_POSITION = 0.86;
    public static double ARM_CONE_5_POSITION = 0.81;
    public static double ARM_VERTICAL_POSITION = 0.76;
    public static double ARM_TRANSFER_POSITION = 0.807;
    public static double ARM_AUTOPARK_POSITION = 0.85;

    public static double ROTATE_PARALLEL_POSITION = 0.88;
    public static double ROTATE_STRAIGHT_POSITION = 0.36;
    public static double ROTATE_TRANSFER_POSITION = 0.135;
    public static double ROTATE_PARALLEL_CONE5_POSITION = 0.24;
    public static double ROTATE_PARALLEL_CONE4_POSITION = 0.5;
    public static double ROTATE_LOW_POLE_POSITION = 0.56;
    public static double ROTATE_LOW_POLE_DROP_POSITION = 0.81;

    public static double CLAW_OPEN_POSITION = 0.92;
    public static double CLAW_CLOSED_POSITION = 0.62;

    public static double SENSOR_TRESHOLD = 40;
    public static double armServoOffset = -0.02;

    public static double sum = 0;
    public static int counter = 0;
    public static double sensorDist = 0;
    public ElapsedTime sensorTimer = new ElapsedTime(0);

    public ArmPosition armPosition;
    public ArmRotate armRotate;
    public ClawMode clawMode;

    private Servo intakeArmServoLeft;
    private Servo intakeArmServoRight;
    private Servo intakeRotateServo;
    private Servo intakeClawServo;

    private ColorRangeSensor intakeSensor;
    public DistanceSensor distanceSensor;

    private Robot robot;

    public Intake(HardwareMap hardwareMap, Robot _robot) {
        intakeArmServoLeft = hardwareMap.get(Servo.class, "intakeArmServoLeft");
        intakeArmServoRight = hardwareMap.get(Servo.class, "intakeArmServoRight");
        intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");
        intakeSensor = hardwareMap.get(ColorRangeSensor.class, "intakeSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        intakeArmServoRight.setDirection(Servo.Direction.REVERSE);

        robot = _robot;

//        armPosition = ArmPosition.TRANSFER;
//        clawMode = ClawMode.OPEN;
//        armRotate = ArmRotate.TRANSFER;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;

        if (sensorTimer.seconds() > 0.1) {
            sensorTimer.reset();
            sensorDist = sum / counter;
            sum = 0;
            counter = 0;
        }
        sum += distanceSensor.getDistance(DistanceUnit.INCH);
        counter++;

        switch (armPosition) {
            case LOW_POLE_WHEN_IN_TRANSFER:
                intakeArmServoLeft.setPosition(ARM_TRANSFER_POSITION);
                intakeArmServoRight.setPosition(ARM_TRANSFER_POSITION + armServoOffset);
                break;
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
                if (robot.intake.armRotate != ArmRotate.TRANSFER)
                    intakeClawServo.setPosition(CLAW_CLOSED_POSITION);
                break;
            default:
                break;
        }
    }

    public double sensorDistance() {
        return sensorDist;
    }

    public boolean hasCone() {
        return intakeSensor.getDistance(DistanceUnit.MM) < SENSOR_TRESHOLD;
    }
}
