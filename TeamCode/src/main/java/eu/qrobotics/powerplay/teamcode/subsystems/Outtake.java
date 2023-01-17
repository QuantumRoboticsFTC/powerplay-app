package eu.qrobotics.powerplay.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake implements Subsystem {

    public enum TurretMode {
        TRANSFER,
        SCORE
    }

    public enum TurretPosition {
        LEFT,
        RIGHT,
        CENTER,
        MANUAL
    }

    public enum ArmPosition {
        TRANSFER,
        SCORE,
        PUSH,
        UP,
        MANUAL
    }

    public enum ClawMode {
        OPEN,
        CLOSE
    }

    public TurretMode turretMode;
    public TurretPosition turretPosition;
    public ArmPosition armPosition;
    public ClawMode clawMode;

    public static double SERVO_OFFSET = 0;

    public static double MANUAL_OFFSET = 0;
    public static double ARM_MANUAL_OFFSET = 0;

    public static double TURRET_LEFT_POSITION = 0.05;
    public static double TURRET_RIGHT_POSITION = 0.95;
    public static double TURRET_CENTER_POSITION = 0.5;

    public static double ARM_TRANSFER_POSITION = 0.26;
    public static double ARM_UP_POSITION = 0.60;
    public static double ARM_SCORE_POSITION = 0.95;
    public static double ARM_PUSH_POSITION = 0.95;

    public static double CLAW_OPEN_POSITION = 0.5;
    public static double CLAW_CLOSE_POSITION = 0.18;

    private Servo turretServoLeft;
    private Servo turretServoRight;
    private Servo outtakeArmServoLeft;
    private Servo outtakeArmServoRight;
    private Servo outtakeClawServo;

    public double turretManualPosition = 0.5;

    public Outtake(HardwareMap hardwareMap) {
        turretServoLeft = hardwareMap.get(Servo.class, "turretServoLeft");
        turretServoRight = hardwareMap.get(Servo.class, "turretServoRight");
        outtakeArmServoLeft = hardwareMap.get(Servo.class, "outtakeArmServoLeft");
        outtakeArmServoRight = hardwareMap.get(Servo.class, "outtakeArmServoRight");
        outtakeClawServo = hardwareMap.get(Servo.class, "outtakeClawServo");

        outtakeArmServoRight.setDirection(Servo.Direction.REVERSE);

        clawMode = ClawMode.OPEN;
        armPosition = ArmPosition.TRANSFER;
        turretMode = TurretMode.SCORE;
        turretPosition = TurretPosition.CENTER;
    }

    public static boolean IS_DISABLED = false;
    @Override
    public void update() {
        if(IS_DISABLED) return;
        switch (turretMode) {
            case SCORE:
                switch (turretPosition) {
                    case LEFT:
                        turretManualPosition = TURRET_LEFT_POSITION;
                        turretServoLeft.setPosition(TURRET_LEFT_POSITION + SERVO_OFFSET);
                        turretServoRight.setPosition(TURRET_LEFT_POSITION);
                        break;
                    case RIGHT:
                        turretManualPosition = TURRET_RIGHT_POSITION;
                        turretServoLeft.setPosition(TURRET_RIGHT_POSITION + SERVO_OFFSET);
                        turretServoRight.setPosition(TURRET_RIGHT_POSITION);
                        break;
                    case CENTER:
                        turretManualPosition = TURRET_CENTER_POSITION;
                        turretServoLeft.setPosition(TURRET_CENTER_POSITION + SERVO_OFFSET);
                        turretServoRight.setPosition(TURRET_CENTER_POSITION);
                        break;
                    case MANUAL:
                        turretServoLeft.setPosition(MANUAL_OFFSET + SERVO_OFFSET);
                        turretServoRight.setPosition(MANUAL_OFFSET);
                }
                break;
            case TRANSFER:
                turretManualPosition = TURRET_CENTER_POSITION;
                turretServoLeft.setPosition(TURRET_CENTER_POSITION + SERVO_OFFSET);
                turretServoRight.setPosition(TURRET_CENTER_POSITION);
                break;

        }

        switch (armPosition) {
            case TRANSFER:
                outtakeArmServoLeft.setPosition(ARM_TRANSFER_POSITION);
                outtakeArmServoRight.setPosition(ARM_TRANSFER_POSITION);
                break;
            case SCORE:
                outtakeArmServoLeft.setPosition(ARM_SCORE_POSITION);
                outtakeArmServoRight.setPosition(ARM_SCORE_POSITION);
                break;
            case PUSH:
                outtakeArmServoLeft.setPosition(ARM_PUSH_POSITION);
                outtakeArmServoRight.setPosition(ARM_PUSH_POSITION);
                break;
            case UP:
                outtakeArmServoLeft.setPosition(ARM_UP_POSITION);
                outtakeArmServoRight.setPosition(ARM_UP_POSITION);
                break;
            case MANUAL:
                outtakeArmServoLeft.setPosition(ARM_MANUAL_OFFSET);
                outtakeArmServoRight.setPosition(ARM_MANUAL_OFFSET);
                break;

        }

        switch (clawMode) {
            case OPEN:
                outtakeClawServo.setPosition(CLAW_OPEN_POSITION);
                break;
            case CLOSE:
                outtakeClawServo.setPosition(CLAW_CLOSE_POSITION);
                break;
        }
    }
    public double getTurretPosition () {
        switch (turretPosition) {
            case LEFT:
                return TURRET_LEFT_POSITION;

            case RIGHT:
                return TURRET_RIGHT_POSITION;

            case CENTER:
                return TURRET_CENTER_POSITION;

            default:
                return TURRET_CENTER_POSITION;
        }
    }
    public double getArmPosition () {
        switch (armPosition) {
            case TRANSFER:
                return ARM_TRANSFER_POSITION;
            case SCORE:
                return ARM_SCORE_POSITION;
            case PUSH:
                return ARM_PUSH_POSITION;
            case UP:
                return ARM_UP_POSITION;
            default:
                return ARM_UP_POSITION;
        }
    }
}
