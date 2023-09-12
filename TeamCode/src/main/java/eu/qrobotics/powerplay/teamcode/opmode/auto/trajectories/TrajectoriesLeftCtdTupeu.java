//package eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories;
//
//import static eu.qrobotics.powerplay.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
//import static eu.qrobotics.powerplay.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class TrajectoriesLeftCtdTupeu {
//    public static Pose2d START_POSE = new Pose2d(-37, -65, Math.toRadians(270));
//
//    private static Pose2d getTrajectorySequenceEndPose(List<Trajectory> trajectories) {
//        if(trajectories.size() == 0)
//            return START_POSE;
//        return trajectories.get(trajectories.size() - 1).end();
//    }
//
//    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent, TrajectoryVelocityConstraint velocityConstraint, TrajectoryAccelerationConstraint accelerationConstraint) {
//        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, velocityConstraint, accelerationConstraint);
//    }
//
//    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent) {
//        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT);
//    }
//
//    private static TrajectoryBuilder makeTrajectoryBuilder(Pose2d pose, double startTangent) {
//        return new TrajectoryBuilder(pose, startTangent, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT);
//    }
//
//    public static List<Trajectory> getTrajectories(int readFromCamera) {
//        List<Trajectory> trajectories = new ArrayList<>();
//        double angle = 187;
//
//        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
//                .lineTo(new Vector2d(-34, -12))
//                .splineToSplineHeading(new Pose2d(-38, -1.3, Math.toRadians(180)), Math.toRadians(90))
//                .build()
//        );
//
//        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
//                .lineTo(new Vector2d(-38, -5))
//                .splineToSplineHeading(new Pose2d(-39.5, -10, Math.toRadians(angle)), Math.toRadians(95))
//                .build()
//        );
//
////      park
//        if (readFromCamera == 3) {
//            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(angle), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
//                    .splineToSplineHeading(new Pose2d(-10, -12, Math.toRadians(280)), Math.toRadians(10))
//                    .build()
//            );
//        } else if (readFromCamera == 2 || readFromCamera == -1) {
//            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(angle), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
//                    .lineTo(new Vector2d(-35, -15))
//                    .build()
//            );
//        } else if (readFromCamera == 1) {
//            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(angle), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
//                    .lineToSplineHeading(new Pose2d(-60, -15, Math.toRadians(180)))
//                    .build()
//            );
//        }
//
//        return trajectories;
//    }
//}