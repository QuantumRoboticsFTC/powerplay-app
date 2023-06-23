package eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories;

import static eu.qrobotics.powerplay.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static eu.qrobotics.powerplay.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.List;

public class TrajectoriesLeftMidCtd {
    public static Pose2d START_POSE = new Pose2d(-37, -65, Math.toRadians(270));

    private static Pose2d getTrajectorySequenceEndPose(List<Trajectory> trajectories) {
        if(trajectories.size() == 0)
            return START_POSE;
        return trajectories.get(trajectories.size() - 1).end();
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent, TrajectoryVelocityConstraint velocityConstraint, TrajectoryAccelerationConstraint accelerationConstraint) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, velocityConstraint, accelerationConstraint);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(Pose2d pose, double startTangent) {
        return new TrajectoryBuilder(pose, startTangent, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT);
    }

    public static List<Trajectory> getTrajectories(int readFromCamera) {
        List<Trajectory> trajectories = new ArrayList<>();

        // go to preload high
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineTo(new Vector2d(-34, -35))
                .splineToSplineHeading(new Pose2d(-31.25, -13.25, Math.toRadians(245)), Math.toRadians(90))
                .build()
        );

//        // go to mid
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(245), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-29, -11, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(-38.5, -16, Math.toRadians(176.5)), Math.toRadians(15))
                .build()
        );

//      park
        if (readFromCamera == 3) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(176.5), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(-34, -14))
                    .splineToSplineHeading(new Pose2d(-12, -12, Math.toRadians(270)), Math.toRadians(90))
                    .build()
            );
        } else if (readFromCamera == 2 || readFromCamera == -1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(176.5), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToLinearHeading(new Pose2d(-39, -14, Math.toRadians(270)))
                    .build()
            );
        } else if (readFromCamera == 1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(176.5), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(-40, -15))
                    .splineToSplineHeading(new Pose2d(-58, -13, Math.toRadians(270)), Math.toRadians(180))
                    .build()
            );
        }

        return trajectories;
    }
}