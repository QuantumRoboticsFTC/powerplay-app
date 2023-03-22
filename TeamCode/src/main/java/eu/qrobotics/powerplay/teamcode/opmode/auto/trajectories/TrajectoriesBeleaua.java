package eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories;

import static eu.qrobotics.powerplay.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static eu.qrobotics.powerplay.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;
import static eu.qrobotics.powerplay.teamcode.subsystems.DriveConstants.NORMAL_ACCEL_CONSTRAINT;
import static eu.qrobotics.powerplay.teamcode.subsystems.DriveConstants.NORMAL_VEL_CONSTRAINT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.List;

public class TrajectoriesBeleaua {

    public static Pose2d START_POSE = new Pose2d(-36, -65, Math.toRadians(270));

    public static int CYCLE_COUNT = 7;

    private static Pose2d getTrajectorySequenceEndPose(List<Trajectory> trajectories) {
        if(trajectories.size() == 0)
            return START_POSE;
        return trajectories.get(trajectories.size() - 1).end();
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent, TrajectoryVelocityConstraint velocityConstraint, TrajectoryAccelerationConstraint accelerationConstraint) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, velocityConstraint, accelerationConstraint);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(Pose2d pose, double startTangent) {
        return new TrajectoryBuilder(pose, startTangent, NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT);
    }

    public static List<Trajectory> getTrajectories(int readFromCamera) {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-36, -28, Math.toRadians(270)))
                .splineToSplineHeading(new Pose2d(-36, -8, Math.toRadians(183)), Math.toRadians(90))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(183), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(0)))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(36, -12))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(36, -8, Math.toRadians(357)))
                .build()
        );

//         park
        if (readFromCamera == 3) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(357), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                    .lineToLinearHeading(new Pose2d(11.5, -12, Math.toRadians(0)))
                    .build()
            );
        } else if (readFromCamera == 2 || readFromCamera == -1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(357), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                    .lineToLinearHeading(new Pose2d(38, -12, Math.toRadians(0)))
                    .build()
            );
        } else if (readFromCamera == 1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(357), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                    .lineToLinearHeading(new Pose2d(62, -12, Math.toRadians(0)))
                    .build()
            );
        }

        return trajectories;
    }
}
