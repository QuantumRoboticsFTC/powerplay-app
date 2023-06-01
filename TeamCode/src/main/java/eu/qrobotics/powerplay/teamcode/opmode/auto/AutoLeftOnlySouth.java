package eu.qrobotics.powerplay.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

import eu.qrobotics.powerplay.teamcode.AprilTagDetectionPipeline;
import eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories.TrajectoriesLeftOnlySouth;
import eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories.TrajectoriesPastLeft;
import eu.qrobotics.powerplay.teamcode.subsystems.Elevator;
import eu.qrobotics.powerplay.teamcode.subsystems.Extendo;
import eu.qrobotics.powerplay.teamcode.subsystems.Intake;
import eu.qrobotics.powerplay.teamcode.subsystems.Outtake;
import eu.qrobotics.powerplay.teamcode.subsystems.Robot;

@Config
@Autonomous
public class AutoLeftOnlySouth extends LinearOpMode {
    public static double ELEVATOR_THRESHOLD = 2;
    public static double EXTENDO_THRESHOLD = 0.38;
    public static Vector2d CONE_STACK = new Vector2d(-70, -12);
    public static final Vector2d OUTTAKE_AUTO_HIGH_POS = new Vector2d(0, -24);

    private ElapsedTime transferTimer = new ElapsedTime(0);

    void getConeFromStack(Robot robot, int i) {
        robot.extendo.targetCone = getExtendoLevel(i); /// CHANGE CONE NR
        robot.intake.armPosition = getintakeArmPosition(i); /// CHANGE CONE NR
        robot.intake.armRotate = Intake.ArmRotate.PARALLEL;/// CHANGE CONE NR

//        robot.extendo.targetLength = robot.extendo.calculateTargetLength(CONE_STACK);
        robot.extendo.targetLength = 56;
        robot.intake.clawMode = Intake.ClawMode.OPEN;
        robot.extendo.extendoMode = Extendo.ExtendoMode.UP;
        while (robot.extendo.getDistanceLeft() > EXTENDO_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.extendo.targetLength = robot.extendo.calculateTargetLength(CONE_STACK);
            telemetry.addData("extendo target", robot.extendo.getTargetLength());
            telemetry.addData("extendo actual", robot.extendo.getCurrentLength());
            telemetry.update();
            robot.sleep(0.01);
        }
        // outtake transfer
        robot.outtake.turretMode = Outtake.TurretMode.SCORE;
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
        robot.sleep(0.1);

        robot.intake.clawMode = Intake.ClawMode.CLOSED;
        robot.sleep(0.2);
        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
        robot.intake.armPosition = Intake.ArmPosition.CONE_5; // go a little bit :sus: so that you go down when transfering instead of going up
        robot.sleep(0.4);
        robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
        robot.sleep(0.1);
        robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
        transferTimer.reset();
        while (robot.extendo.getCurrentLength() > EXTENDO_THRESHOLD && opModeIsActive() && !isStopRequested() && transferTimer.seconds() < 1.5) {
            robot.sleep(0.01);
        }
        robot.sleep(0.25);
        robot.intake.clawMode = Intake.ClawMode.OPEN;
        robot.sleep(0.25);
        robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
    }

    void score(Robot robot, int i) {
        robot.elevator.targetPosition = Elevator.TargetHeight.HIGH;
        robot.elevator.elevatorMode = Elevator.ElevatorMode.AUTOMATIC;
        robot.sleep(0.1);

        robot.outtake.armPosition = Outtake.ArmPosition.UP;
        robot.sleep(0.3);

        robot.outtake.turretPosition = Outtake.TurretPosition.AUTO_LEFT_SCORE;
        robot.sleep(0.1);

        robot.outtake.armPosition = Outtake.ArmPosition.SCORE;
        robot.sleep(0.1);

        robot.outtake.turretMode = Outtake.TurretMode.FOLLOWING;
        robot.outtake.followingPosition = OUTTAKE_AUTO_HIGH_POS;
//            robot.outtake.alignerMode = Outtake.AlignerMode.DEPLOYED;
        robot.sleep(0.5);

        telemetry.addData("outtake x", Math.toDegrees(robot.outtake.getTargetTurretAngle(OUTTAKE_AUTO_HIGH_POS)));
        telemetry.update();
        robot.outtake.clawMode = Outtake.ClawMode.OPEN;
        robot.sleep(0.2);
//            robot.outtake.alignerMode = Outtake.AlignerMode.RETRACTED;
        robot.outtake.armPosition = Outtake.ArmPosition.UP;
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.elevator.targetPosition = Elevator.TargetHeight.GROUND;
        robot.elevator.elevatorMode = Elevator.ElevatorMode.AUTOMATIC;
        robot.sleep(0.1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoriesLeftOnlySouth.START_POSE);
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

        List<Trajectory> trajectories = TrajectoriesPastLeft.getTrajectories(readFromCamera);
        telemetry.addData("camera tag", readFromCamera);
        telemetry.update();

        robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
        robot.drive.followTrajectory(trajectories.get(0));

        robot.sleep(0.1);

        robot.drive.followTrajectory(trajectories.get(1));

        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.2);

        score(robot, 1);
        for (int i = 1; i <= 5; i++) {
            getConeFromStack(robot, i);
            score(robot, i);
        }

        robot.sleep(0.2);
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.outtake.turretMode = Outtake.TurretMode.TRANSFER;
        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
        robot.intake.armPosition = Intake.ArmPosition.AUTOPARK;
        robot.drive.followTrajectory(trajectories.get(2));
        robot.sleep(0.6);
        robot.outtake.armPosition = Outtake.ArmPosition.AUTO_INIT;
        robot.sleep(0.4);

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