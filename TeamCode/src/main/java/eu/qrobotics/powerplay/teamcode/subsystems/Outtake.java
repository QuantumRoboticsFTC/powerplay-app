package eu.qrobotics.powerplay.teamcode.subsystems;

import android.graphics.Point;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Vector;

import eu.qrobotics.powerplay.teamcode.hardware.CachingServo;

@Config
public class Outtake implements Subsystem {
    public static final Vector2d OUTTAKE_ROBOT_POS = new Vector2d(0, 5.6);
    public static final Vector2d OUTTAKE_AUTO_HIGH_POS = new Vector2d(0, -24);
    public static final Vector2d OUTTAKE_AUTO_PRELOAD_POS = new Vector2d(-24, 0);

    public enum TurretMode {
        TRANSFER,
        SCORE
    }

    public enum TurretPosition {
        LEFT,
        RIGHT,
        CENTER,
        MANUAL,
        AUTO_LEFT_SCORE,
        AUTO_RIGHT_SCORE
    }

    public enum ArmPosition {
        TRANSFER,
        SCORE,
        SCORE_TILTED,
        PUSH,
        AUTO_INIT,
        UP,
        MANUAL
    }

    public enum ClawMode {
        OPEN,
        CLOSED
    }

    public enum AlignerMode {
        DEPLOYED,
        RETRACTED
    }

    public TurretMode turretMode;
    public TurretPosition turretPosition;
    public ArmPosition armPosition;
    public ClawMode clawMode;

    public AlignerMode alignerMode;

    public static double SERVO_OFFSET = 0;

    public double manualOffset = 0;

    public double armManualOffset = 0;

    public static double TURRET_LEFT_POSITION = 0.02;
    public static double TURRET_RIGHT_POSITION = 0.92;
    public static double TURRET_CENTER_POSITION = 0.45;

    public static double TURRET_LEFT_AUTO_SCORE_POSITION = 0.18;
    public static double TURRET_RIGHT_AUTO_SCORE_POSITION = 0.92;

    public static double ARM_TRANSFER_POSITION = 0.28;
    public static double ARM_UP_POSITION = 0.62;
    public static double ARM_AUTO_INIT_POSITION = 0.4;
    public static double ARM_SCORE_POSITION = 0.93;
    public static double ARM_SCORE_TILTED_POSITION = 0.87;
    public static double ARM_PUSH_POSITION = 0.95;

    public static double CLAW_OPEN_POSITION = 0.305;
    public static double CLAW_CLOSE_POSITION = 0.18;

    public static double ALIGNER_RETRACTED_POSITION = 0.55;
    public static double ALIGNER_DEPLOYED_POSITION = 0.47;

    public CachingServo turretServoLeft;
    private CachingServo turretServoRight;
    private CachingServo outtakeArmServoLeft;
    private CachingServo outtakeArmServoRight;
    private CachingServo outtakeClawServo;
    private CachingServo outtakeAlignerServo;

    private ColorRangeSensor outtakeSensor;

    public double turretManualPosition = 0.5;

    private Robot robot;

    public Outtake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        turretServoLeft = new CachingServo(hardwareMap.get(Servo.class, "turretServoLeft"));
        turretServoRight = new CachingServo(hardwareMap.get(Servo.class, "turretServoRight"));
        outtakeArmServoLeft = new CachingServo(hardwareMap.get(Servo.class, "outtakeArmServoLeft"));
        outtakeArmServoRight = new CachingServo(hardwareMap.get(Servo.class, "outtakeArmServoRight"));
        outtakeClawServo = new CachingServo(hardwareMap.get(Servo.class, "outtakeClawServo"));
        outtakeAlignerServo = new CachingServo(hardwareMap.get(Servo.class, "outtakeAlignerServo"));
        outtakeSensor = hardwareMap.get(ColorRangeSensor.class, "outtakeSensor");

        outtakeArmServoRight.setDirection(Servo.Direction.REVERSE);
        turretServoLeft.setDirection(Servo.Direction.REVERSE);
        turretServoRight.setDirection(Servo.Direction.REVERSE);

        clawMode = ClawMode.OPEN;
        armPosition = ArmPosition.TRANSFER;
        turretMode = TurretMode.TRANSFER;
        turretPosition = TurretPosition.CENTER;
        alignerMode = AlignerMode.RETRACTED;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;
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
                    case AUTO_LEFT_SCORE:
                        turretManualPosition = TURRET_LEFT_AUTO_SCORE_POSITION;
                        turretServoLeft.setPosition(TURRET_LEFT_AUTO_SCORE_POSITION + SERVO_OFFSET);
                        turretServoRight.setPosition(TURRET_LEFT_AUTO_SCORE_POSITION);
                        break;
                    case AUTO_RIGHT_SCORE:
                        turretManualPosition = TURRET_RIGHT_AUTO_SCORE_POSITION;
                        turretServoLeft.setPosition(TURRET_RIGHT_AUTO_SCORE_POSITION + SERVO_OFFSET);
                        turretServoRight.setPosition(TURRET_RIGHT_AUTO_SCORE_POSITION);
                        break;
                    case MANUAL:
                        turretServoLeft.setPosition(approximateToThreeDecimals(manualOffset) + SERVO_OFFSET);
                        turretServoRight.setPosition(approximateToThreeDecimals(manualOffset));
                        break;
                    default:
                        break;
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
            case SCORE_TILTED:
                outtakeArmServoLeft.setPosition(ARM_SCORE_TILTED_POSITION);
                outtakeArmServoRight.setPosition(ARM_SCORE_TILTED_POSITION);
                break;
            case PUSH:
                outtakeArmServoLeft.setPosition(ARM_PUSH_POSITION);
                outtakeArmServoRight.setPosition(ARM_PUSH_POSITION);
                break;
            case UP:
                outtakeArmServoLeft.setPosition(ARM_UP_POSITION);
                outtakeArmServoRight.setPosition(ARM_UP_POSITION);
                break;
            case AUTO_INIT:
                outtakeArmServoLeft.setPosition(ARM_AUTO_INIT_POSITION);
                outtakeArmServoRight.setPosition(ARM_AUTO_INIT_POSITION);
                break;
            case MANUAL:
                outtakeArmServoLeft.setPosition(armManualOffset);
                outtakeArmServoRight.setPosition(armManualOffset);
                break;
            default:
                break;

        }

        switch (clawMode) {
            case OPEN:
                outtakeClawServo.setPosition(CLAW_OPEN_POSITION);
                break;
            case CLOSED:
                outtakeClawServo.setPosition(CLAW_CLOSE_POSITION);
                break;
            default:
                break;
        }
        switch (alignerMode) {
            case DEPLOYED:
                outtakeAlignerServo.setPosition(ALIGNER_DEPLOYED_POSITION);
                break;
            case RETRACTED:
                outtakeAlignerServo.setPosition(ALIGNER_RETRACTED_POSITION);
                break;
            default:
                break;
        }
    }

    public double getTurretPosition() {
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

    public double getArmPosition() {
        switch (armPosition) {
            case TRANSFER:
                return ARM_TRANSFER_POSITION;
            case SCORE:
                return ARM_SCORE_POSITION;
            case PUSH:
                return ARM_PUSH_POSITION;
            case UP:
                return ARM_UP_POSITION;
            case AUTO_INIT:
                return ARM_AUTO_INIT_POSITION;
            case SCORE_TILTED:
                return ARM_SCORE_TILTED_POSITION;
            default:
                return ARM_UP_POSITION;
        }
    }


    public boolean hasCone() {
        return outtakeSensor.getDistance(DistanceUnit.MM) < 28;
    }

    public static double approximateToThreeDecimals(double d) {
        return Math.round(d * 1000.0) / 1000.0;
    }

    public static Pose2d TURRET_ROBOT_POSE = new Pose2d(-5.51, 0, Math.toRadians(180));
    public static double TURRET_ARM_LENGTH = 11.789;

    public double getTargetTurretAngle(Vector2d targetPosition) {
        Pose2d robotPose = robot.drive.getPoseEstimate();
        Pose2d turretWorldPose = new Pose2d(robotPose.vec().plus(TURRET_ROBOT_POSE.vec().rotated(robotPose.getHeading())), robotPose.getHeading() + TURRET_ROBOT_POSE.getHeading());

        double turretAngle = targetPosition.minus(turretWorldPose.vec()).angle() - turretWorldPose.getHeading();
        while(turretAngle > Math.PI)
            turretAngle -= 2 * Math.PI;
        while(turretAngle < -Math.PI)
            turretAngle += 2 * Math.PI;
        return turretAngle;
    }

    public double getTargetArmAngle(Vector2d targetPosition) {
        Pose2d robotPose = robot.drive.getPoseEstimate();
        Pose2d turretWorldPose = new Pose2d(robotPose.vec().plus(TURRET_ROBOT_POSE.vec().rotated(robotPose.getHeading())), robotPose.getHeading() + TURRET_ROBOT_POSE.getHeading());

        double armAngle = Math.acos(targetPosition.distTo(turretWorldPose.vec()) / TURRET_ARM_LENGTH);
        while(armAngle > Math.PI)
            armAngle -= 2 * Math.PI;
        while(armAngle < -Math.PI)
            armAngle += 2 * Math.PI;
        return armAngle;
    }



    public double getTargetTurretServoPosition(Vector2d targetPosition) {
        double turretAngle = getTargetTurretAngle(targetPosition);

        // gearing: 4/6
        // 270 servo = 180 turret

        // pos (x + 0.5 - TURRET_CENTER) = x * 180 deg - 90 deg
        // x = (turretAngle + 90)/180 - 0.5 + TURRET_CENTER

        return (turretAngle + Math.toRadians(90))/Math.toRadians(180) - 0.5 + TURRET_CENTER_POSITION;
    }

    public double getTargetArmServoPosition(Vector2d targetPosition) {
        double turretAngle = Math.PI / 2 - getTargetArmAngle(targetPosition);

        // vertical = 0.652
        // pos (x-0.652) = (x-0.652)*270

        return turretAngle/Math.toRadians(270) + 0.652;
    }
}
