package eu.qrobotics.powerplay.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

import eu.qrobotics.powerplay.teamcode.AprilTagDetectionPipeline;
import eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories.TrajectoriesBeleaua;
import eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories.TrajectoriesLeft;
import eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories.TrajectoriesLeftMid;
import eu.qrobotics.powerplay.teamcode.subsystems.Elevator;
import eu.qrobotics.powerplay.teamcode.subsystems.Extendo;
import eu.qrobotics.powerplay.teamcode.subsystems.Intake;
import eu.qrobotics.powerplay.teamcode.subsystems.Outtake;
import eu.qrobotics.powerplay.teamcode.subsystems.Robot;

@Config
@Autonomous
public class BeleauaMTI extends LinearOpMode {
    public static double ELEVATOR_THRESHOLD = 2;
    public static double EXTENDO_THRESHOLD = 0.38;
    public static Vector2d CONE_STACK_LEFT = new Vector2d(-69.5, -12);

    public static Vector2d CONE_STACK_RIGHT = new Vector2d(70, -12);

    public static Vector2d JUNCTION_LEFT = new Vector2d(-24, -1);

    public static Vector2d JUNCTION_RIGHT = new Vector2d(24, -1);

    private ElapsedTime transferTimer = new ElapsedTime(0);

    public void cycle (Robot robot, int stackPosition, Vector2d JUNCTION, Vector2d CONE_STACK) {
        robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.outtake.armPosition = Outtake.ArmPosition.UP;
        robot.sleep(0.15);

        // Set turret target position

        robot.outtake.followingPosition = JUNCTION;
        robot.outtake.turretMode = Outtake.TurretMode.FOLLOWING;
        robot.elevator.targetPosition = Elevator.TargetHeight.HIGH;
//        while (robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
//            robot.sleep(0.01);
//        }
        // Wait for da elevator shit
        // Move intake outside of the robot
//        robot.intake.clawMode = Intake.ClawMode.CLOSED;
        robot.outtake.armPosition = Outtake.ArmPosition.PUSH;
        robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
        robot.intake.armPosition = getintakeArmPosition(stackPosition);
        robot.sleep(0.1);

        robot.intake.clawMode = Intake.ClawMode.OPEN;
        robot.extendo.targetCone = getExtendoLevel(stackPosition);
        robot.extendo.targetLength = robot.extendo.calculateTargetLength(CONE_STACK);
        robot.extendo.extendoMode = Extendo.ExtendoMode.UP;
        if(stackPosition == 6) {
            robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
        }
        while (robot.extendo.getDistanceLeft() > EXTENDO_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.extendo.targetLength = robot.extendo.calculateTargetLength(CONE_STACK);
            telemetry.addData("extendo target", robot.extendo.getTargetLength());
            telemetry.addData("extendo actual", robot.extendo.getCurrentLength());
            telemetry.update();
            robot.sleep(0.01);
        }
//        telemetry.addData("outtake x", Math.toDegrees(robot.outtake.getTargetTurretAngle(OUTTAKE_AUTO_PRELOAD_POS)));
//        telemetry.addData("outtake servo pops", robot.outtake.getTargetTurretServoPosition(OUTTAKE_AUTO_PRELOAD_POS));
//        telemetry.addData("outtake servo pops actual", robot.outtake.turretServoLeft.getPosition());
//        telemetry.update();

        robot.outtake.clawMode = Outtake.ClawMode.OPEN;
        robot.sleep(0.15);

        // Collect da sheet
        robot.intake.clawMode = Intake.ClawMode.CLOSED;
        robot.sleep(0.2);
        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
        robot.intake.armPosition = Intake.ArmPosition.CONE_5; // go a little bit :sus: so that you go down when transfering instead of going up

        robot.outtake.turretMode = Outtake.TurretMode.SCORE;
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
        robot.outtake.alignerMode = Outtake.AlignerMode.RETRACTED;
        robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;

        while (robot.elevator.getDistanceLeft() > 200 && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.3);

        robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
        robot.sleep(0.1);
        robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
        while (robot.extendo.getDistanceLeft() > EXTENDO_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }

        robot.sleep(0.3);
        robot.intake.clawMode = Intake.ClawMode.OPEN;
        robot.sleep(0.3);
        robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
    }

    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoriesLeftMid.START_POSE);
        robot.elevator.targetPosition = Elevator.TargetHeight.HIGH;
        robot.extendo.targetCone = Extendo.TargetCone.AUTO_CONE5;
        robot.outtake.turretMode = Outtake.TurretMode.SCORE;
        robot.outtake.armPosition = Outtake.ArmPosition.AUTO_INIT;
        robot.start();

        int readFromCamera = -1;

        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        // Tag ID 1,2,3 from the 36h11 family
        /*EDIT IF NEEDED!!!*/

        int LEFT = 1;
        int MIDDLE = 2;
        int RIGHT = 3;

        AprilTagDetection tagOfInterest = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // :salute:
            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) {
                robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
            }

            telemetry.addLine("Press a on gamepad 1 to close claw");

            //region Camera shit

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        if (tag.id == LEFT) {
                            readFromCamera = 1;
                        }
                        if (tag.id == MIDDLE) {
                            readFromCamera = 2;
                        }
                        if (tag.id == RIGHT) {
                            readFromCamera = 3;
                        }
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    telemetry.addData("tele1 ", tagOfInterest.id);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        telemetry.addData("tele1 ", tagOfInterest.id);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    telemetry.addData("telemetry ", tagOfInterest.id);
                }

            }

            //endregion Camera Shit

            telemetry.update();
        }

        if (isStopRequested()) {
            robot.stop();
            return;
        }

        camera.closeCameraDeviceAsync(() -> {});

        List<Trajectory> trajectories = TrajectoriesBeleaua.getTrajectories(readFromCamera);
        telemetry.addData("camera tag", readFromCamera);
        telemetry.update();

        // PRELOAD
        // Go to preload junction
        robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
        robot.sleep(0.1);

        // Put elevator up
        robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;

        robot.drive.followTrajectory(trajectories.get(0));

        // Wait for da elevator shit
        while (robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
        robot.intake.armPosition = Intake.ArmPosition.CONE_5;


        // Maybe put the teeth aligner
        // robot.outtake.alignerMode = Outtake.AlignerMode.DEPLOYED
        // Wait for moving to stop
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }

        // Fucking inertia
        robot.sleep(0.2);

        robot.outtake.followingPosition = JUNCTION_LEFT;
        robot.outtake.turretMode = Outtake.TurretMode.FOLLOWING;

        for(int i=1;i<=5;++i) {
            cycle(robot, i, JUNCTION_LEFT, CONE_STACK_LEFT);
        }

        robot.drive.followTrajectory(trajectories.get(1));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.drive.followTrajectory(trajectories.get(2));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.drive.followTrajectory(trajectories.get(3));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        for(int i=1;i<=6;++i) {
            cycle(robot, i, JUNCTION_RIGHT, CONE_STACK_RIGHT);
        }


        robot.sleep(0.2);
//        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
//        robot.outtake.turretMode = Outtake.TurretMode.TRANSFER;
//        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
//        robot.intake.armPosition = Intake.ArmPosition.AUTOPARK;
//        robot.drive.followTrajectory(trajectories.get(4));
//        robot.sleep(0.6);
//        robot.outtake.armPosition = Outtake.ArmPosition.AUTO_INIT;
//        robot.sleep(0.4);

        robot.stop();
    }

    private Extendo.TargetCone getExtendoLevel(int i) {
        Extendo.TargetCone targetExtendo;
        switch (i) {
            case 1:
                targetExtendo = Extendo.TargetCone.AUTO_CONE5;
                break;
            case 2:
                targetExtendo = Extendo.TargetCone.AUTO_CONE4;
                break;
            case 3:
                targetExtendo = Extendo.TargetCone.AUTO_CONE3;
                break;
            case 4:
                targetExtendo = Extendo.TargetCone.AUTO_CONE2;
                break;
            case 5:
                targetExtendo = Extendo.TargetCone.AUTO_CONE1;
                break;
            default:
                targetExtendo = Extendo.TargetCone.AUTO_CONE1;
        }
        return targetExtendo;
    }

    private Intake.ArmPosition getintakeArmPosition(int i) {
        Intake.ArmPosition targetPosition;
        switch (i) {
            case 1:
                targetPosition = Intake.ArmPosition.CONE_5;
                break;
            case 2:
                targetPosition = Intake.ArmPosition.CONE_4;
                break;
            case 3:
                targetPosition = Intake.ArmPosition.CONE_3;
                break;
            case 4:
                targetPosition = Intake.ArmPosition.CONE_2;
                break;
            case 5:
                targetPosition = Intake.ArmPosition.CONE_1;
                break;
            case 6:
                targetPosition = Intake.ArmPosition.VERTICAL;
                break;
            default:
                targetPosition = Intake.ArmPosition.CONE_1;
        }
        return targetPosition;
    }
}