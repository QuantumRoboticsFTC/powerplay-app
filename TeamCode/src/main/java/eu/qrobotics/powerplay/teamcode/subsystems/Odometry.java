package eu.qrobotics.powerplay.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import eu.qrobotics.powerplay.teamcode.util.Encoder;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class Odometry extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.689; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 11.1;
    public static double FORWARD_OFFSET = -5.1;

    public static Pose2d LEFT_POSE = new Pose2d(0,  LATERAL_DISTANCE / 2, 0);
    public static Pose2d RIGHT_POSE = new Pose2d(0, -LATERAL_DISTANCE / 2, 0);
    public static Pose2d REAR_POSE = new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90));

    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER = 1;

    private Encoder leftEncoder, rightEncoder, rearEncoder;

    public Odometry(HardwareMap hardwareMap) {
        super(Arrays.asList(
                LEFT_POSE, // left
                RIGHT_POSE, // right
                REAR_POSE // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        rearEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));

        rightEncoder.setDirection(Encoder.Direction.REVERSE);
//        rearEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * Y_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rearEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * Y_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rearEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}