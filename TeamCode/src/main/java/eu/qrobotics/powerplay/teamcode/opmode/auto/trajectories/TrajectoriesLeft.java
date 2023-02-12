package eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.List;

import static eu.qrobotics.powerplay.teamcode.subsystems.DriveConstants.*;

public class TrajectoriesLeft {
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
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(Pose2d pose, double startTangent) {
        return new TrajectoryBuilder(pose, startTangent, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT);
    }

    public static List<Trajectory> getTrajectories(int readFromCamera) {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-36, -28, Math.toRadians(270)))
                .splineToSplineHeading(new Pose2d(-36, -13.5, Math.toRadians(225)), Math.toRadians(90))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(225), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .splineToLinearHeading(new Pose2d(-40, -14, Math.toRadians(180)), Math.toRadians(270))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-8, -15))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-32, -14))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-8, -15))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-32, -14))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-8, -15))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-32, -14))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-8, -15))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-32, -14))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-8, -15))
                .build()
        );

//         park
        if (readFromCamera == 3) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToLinearHeading(new Pose2d(-14, -12, Math.toRadians(180)))
                    .build()
            );
        } else if (readFromCamera == 2 || readFromCamera == -1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToLinearHeading(new Pose2d(-38, -14, Math.toRadians(180)))
                    .build()
            );
        } else if (readFromCamera == 1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToLinearHeading(new Pose2d(-62, -14, Math.toRadians(180)))
                    .build()
            );
        }

        return trajectories;
    }
}