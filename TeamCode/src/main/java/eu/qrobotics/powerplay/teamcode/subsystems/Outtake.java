package eu.qrobotics.powerplay.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import eu.qrobotics.powerplay.teamcode.hardware.CachingServo;

@Config
public class Outtake implements Subsystem {

    public enum TurretMode {
        TRANSFER,
        SCORE,
        FOLLOWING
    }

    public enum TurretPosition {
        LEFT,
        RIGHT,
        CENTER,
        MANUAL,
        AUTO_LEFT_SCORE,
        AUTO_RIGHT_SCORE,
    }

    public enum ArmPosition {
        TRANSFER,
        SCORE,
        SCORE_TILTED,
        SCORE_VERY_DOWN,
        PUSH,
        AUTO_INIT,
        FOLLOWING,
        UP,
        AUTO_VERTICAL,
        MANUAL
    }

    public enum ClawMode {
        OPEN,
        CLOSED
    }

    public enum AlignerMode {
        DEPLOYED,
        RETRACTED,
        AUTO_PROBLEM
    }

    public TurretMode turretMode;
    public TurretPosition turretPosition;
    public ArmPosition armPosition;
    public ClawMode clawMode;
    public AlignerMode alignerMode;

    public double manualOffset = 0;
    public double armManualOffset = 0;

    public Vector2d followingPosition = new Vector2d(0, 0);

    public static double TURRET_LEFT_POSITION = 0.4;
    public static double TURRET_RIGHT_POSITION = 1;
    public static double TURRET_CENTER_POSITION = 0.745;

    public static double TURRET_LEFT_AUTO_SCORE_POSITION = 0.4;
    public static double TURRET_RIGHT_AUTO_SCORE_POSITION = 1;

    public static double ARM_TRANSFER_POSITION = 0.1;
    public static double ARM_UP_POSITION = 0.485;
    public static double ARM_AUTO_INIT_POSITION = 0.22;
    public static double ARM_SCORE_VERY_DOWN_POS  = 0.87;
    public static double ARM_SCORE_POSITION = 0.83;
    public static double ARM_SCORE_TILTED_POSITION = 0.76;
    public static double ARM_PUSH_POSITION = 0.64;
    public static double ARM_AUTO_VERTICAL_POSITION = 0.59;

    public static double CLAW_OPEN_POSITION = 0.67;
    public static double CLAW_CLOSE_POSITION = 0.56;

    public static double ALIGNER_RETRACTED_POSITION = 1;
    public static double ALIGNER_AUTO_PROB_POSITION = 0.84;
    public static double ALIGNER_DEPLOYED_POSITION = 0.65;

    private CachingServo turretServo;
    private CachingServo armServoLeft;
    private CachingServo armServoRight;
    private CachingServo clawServo;
    private CachingServo alignerServo;

    private ColorRangeSensor outtakeSensor;

    public double turretManualPosition = TURRET_CENTER_POSITION;
    public double armManualPosition = ARM_UP_POSITION;

    public boolean isScoring;
    public boolean alignerActive; // is aligner active at next cycle

    private Robot robot;

    public Outtake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        turretServo = new CachingServo(hardwareMap.get(Servo.class, "turretServo"));
        armServoLeft = new CachingServo(hardwareMap.get(Servo.class, "outtakeArmServoLeft"));
        armServoRight = new CachingServo(hardwareMap.get(Servo.class, "outtakeArmServoRight"));
        clawServo = new CachingServo(hardwareMap.get(Servo.class, "outtakeClawServo"));
        alignerServo = new CachingServo(hardwareMap.get(Servo.class, "outtakeAlignerServo"));

        armServoRight.setDirection(Servo.Direction.REVERSE);
        turretServo.setDirection(Servo.Direction.REVERSE);

        clawMode = ClawMode.OPEN;
        armPosition = ArmPosition.TRANSFER;
        turretMode = TurretMode.TRANSFER;
        turretPosition = TurretPosition.CENTER;
        alignerMode = AlignerMode.RETRACTED;

        isScoring = false;
        alignerActive = true;
    }

    public double getTurretServoPosition() {
        return turretServo.getPosition();
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
                        turretServo.setPosition(TURRET_LEFT_POSITION);
                        break;
                    case RIGHT:
                        turretManualPosition = TURRET_RIGHT_POSITION;
                        turretServo.setPosition(TURRET_RIGHT_POSITION);
                        break;
                    case CENTER:
                        turretManualPosition = TURRET_CENTER_POSITION;
                        turretServo.setPosition(TURRET_CENTER_POSITION);
                        break;
                    case AUTO_LEFT_SCORE:
                        turretManualPosition = TURRET_LEFT_AUTO_SCORE_POSITION;
                        turretServo.setPosition(TURRET_LEFT_AUTO_SCORE_POSITION);
                        break;
                    case AUTO_RIGHT_SCORE:
                        turretManualPosition = TURRET_RIGHT_AUTO_SCORE_POSITION;
                        turretServo.setPosition(TURRET_RIGHT_AUTO_SCORE_POSITION);
                        break;
                    case MANUAL:
                        turretServo.setPosition(approximateToThreeDecimals(manualOffset));
                        break;
                    default:
                        break;
                }
                break;
            case TRANSFER:
                turretManualPosition = TURRET_CENTER_POSITION;
                turretServo.setPosition(TURRET_CENTER_POSITION);
                break;
            case FOLLOWING:
                // set turret position to follow the target
                turretManualPosition = Range.clip(getTargetTurretServoPosition(followingPosition), 0, 1);
                turretServo.setPosition(turretManualPosition);
                break;
        }

        switch (armPosition) {
            case FOLLOWING:
                // set arm to follow the target
                armManualPosition = Range.clip(getTargetArmServoPosition(followingPosition), 0, 1);
                armServoLeft.setPosition(armManualPosition);
                armServoRight.setPosition(armManualPosition);
                break;
            case TRANSFER:
                armServoLeft.setPosition(ARM_TRANSFER_POSITION);
                armServoRight.setPosition(ARM_TRANSFER_POSITION);
                break;
            case SCORE_VERY_DOWN:
                armServoLeft.setPosition(ARM_SCORE_VERY_DOWN_POS);
                armServoRight.setPosition(ARM_SCORE_VERY_DOWN_POS);
                break;
            case SCORE:
                armServoLeft.setPosition(ARM_SCORE_POSITION);
                armServoRight.setPosition(ARM_SCORE_POSITION);
                break;
            case SCORE_TILTED:
                armServoLeft.setPosition(ARM_SCORE_TILTED_POSITION);
                armServoRight.setPosition(ARM_SCORE_TILTED_POSITION);
                break;
            case PUSH:
                armServoLeft.setPosition(ARM_PUSH_POSITION);
                armServoRight.setPosition(ARM_PUSH_POSITION);
                break;
            case UP:
                armServoLeft.setPosition(ARM_UP_POSITION);
                armServoRight.setPosition(ARM_UP_POSITION);
                break;
            case AUTO_INIT:
                armServoLeft.setPosition(ARM_AUTO_INIT_POSITION);
                armServoRight.setPosition(ARM_AUTO_INIT_POSITION);
                break;
            case AUTO_VERTICAL:
                armServoLeft.setPosition(ARM_AUTO_VERTICAL_POSITION);
                armServoRight.setPosition(ARM_AUTO_VERTICAL_POSITION);
                break;
            case MANUAL:
                armServoLeft.setPosition(armManualOffset);
                armServoRight.setPosition(armManualOffset);
                break;
            default:
                break;
        }
        //}

        switch (clawMode) {
            case OPEN:
                clawServo.setPosition(CLAW_OPEN_POSITION);
                break;
            case CLOSED:
                clawServo.setPosition(CLAW_CLOSE_POSITION);
                break;
            default:
                break;
        }
        switch (alignerMode) {
            case DEPLOYED:
                alignerServo.setPosition(ALIGNER_DEPLOYED_POSITION);
                break;
            case RETRACTED:
                alignerServo.setPosition(ALIGNER_RETRACTED_POSITION);
                break;
            case AUTO_PROBLEM:
                alignerServo.setPosition(ALIGNER_AUTO_PROB_POSITION);
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

    public static double approximateToThreeDecimals(double d) {
        return Math.round(d * 1000.0) / 1000.0;
    }

    public static Pose2d TURRET_ROBOT_POSE = new Pose2d(-5.51, 0, Math.toRadians(180));
    public static double TURRET_ARM_LENGTH = 11.789;
    public double bulanVar = 0;

    public double getTargetTurretAngle(Vector2d targetPosition) {
        Pose2d robotPose = robot.drive.getPoseEstimate();
        Pose2d turretWorldPose = new Pose2d(robotPose.vec().plus(TURRET_ROBOT_POSE.vec().rotated(robotPose.getHeading())), robotPose.getHeading() + TURRET_ROBOT_POSE.getHeading());

        double turretAngle = targetPosition.minus(turretWorldPose.vec()).angle() - turretWorldPose.getHeading();
        while(turretAngle > Math.PI)
            turretAngle -= 2 * Math.PI;
        while(turretAngle < -Math.PI)
            turretAngle += 2 * Math.PI;
        return turretAngle
                + Math.toRadians(bulanVar) * Math.signum(turretAngle);
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

        // gearing: 1:1
        // 300 servo = 300 turret

        // pos (x + 0.5 - TURRET_CENTER) = x * 300 deg - 150 deg
        // x = (turretAngle + 150)/300 - 0.5 + TURRET_CENTER

        return (turretAngle + Math.toRadians(150))/Math.toRadians(300) - 0.5 + TURRET_CENTER_POSITION;
    }

    public double getTargetArmServoPosition(Vector2d targetPosition) {
        double turretAngle = Math.PI / 2 - getTargetArmAngle(targetPosition);

        // vertical (UP) = ARM_UP_POSITION
        // pos (x-UP) = (x-UP)*270

        return turretAngle/Math.toRadians(270) + ARM_UP_POSITION;
    }
}
