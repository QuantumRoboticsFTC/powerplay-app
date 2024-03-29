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
import eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories.TrajectoriesRightContested;
import eu.qrobotics.powerplay.teamcode.subsystems.Elevator;
import eu.qrobotics.powerplay.teamcode.subsystems.Extendo;
import eu.qrobotics.powerplay.teamcode.subsystems.Intake;
import eu.qrobotics.powerplay.teamcode.subsystems.Outtake;
import eu.qrobotics.powerplay.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "#6 AutoRightContested")
public class AutoRightContested extends LinearOpMode {
    public static double EXTENDO_THRESHOLD = 0.3;
    public static Vector2d CONE_STACK = new Vector2d(72, -12);
    public static Vector2d PRE_CONE_STACK = new Vector2d(68, -12);
    public static final Vector2d OUTTAKE_AUTO_HIGH_POS = new Vector2d(24, 0);

    private ElapsedTime transferTimer = new ElapsedTime(0);

    int useCamera(Robot robot) {
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
            return -10;
        }

        camera.closeCameraDeviceAsync(() -> {});
        return readFromCamera;
    }

    // grija cand intra sa fie turretPosition pe CETNER
    void cycle(Robot robot, int i) {
        // OUTTAKE
        robot.outtake.armPosition = Outtake.ArmPosition.AUTO_VERTICAL;
        robot.elevator.scoringPosition = Elevator.TargetHeight.HIGH;
        robot.elevator.isScoring = true;
        robot.sleep(0.07);

        robot.intake.armPosition = getintakeArmPosition(i);
        robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
        if (i == 1) {
            robot.intake.clawMode = Intake.ClawMode.OPEN;
        }
        robot.sleep(0.1);
        if (i < 6) {
            robot.extendo.targetVector2d = PRE_CONE_STACK;
            robot.extendo.targetPosition = getExtendoLevel(i);
            robot.extendo.extendoMode = Extendo.ExtendoMode.AUTOMATIC;
        }
        robot.sleep(0.35);

        robot.outtake.turretMode = Outtake.TurretMode.FOLLOWING;
        robot.outtake.followingPosition = OUTTAKE_AUTO_HIGH_POS;
        robot.sleep(0.75);

        robot.outtake.armPosition = Outtake.ArmPosition.PUSH;
        robot.sleep(0.15);
        if (i > 1) {
            robot.sleep(0.2);
        }
        robot.outtake.armPosition = Outtake.ArmPosition.SCORE_VERY_DOWN;
        robot.sleep(0.2);

        robot.outtake.clawMode = Outtake.ClawMode.OPEN;
        robot.outtake.alignerMode = Outtake.AlignerMode.AUTO_PROBLEM;
        robot.sleep(0.2);

        robot.outtake.armPosition = Outtake.ArmPosition.AUTO_VERTICAL;
        robot.sleep(0.15);

        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.outtake.turretMode = Outtake.TurretMode.TRANSFER;
        robot.sleep(0.2);
        robot.elevator.isScoring = false;
        robot.elevator.targetPosition = Elevator.TargetHeight.GROUND;

        robot.sleep(0.2);
        robot.outtake.alignerMode = Outtake.AlignerMode.RETRACTED;

        if (i == 6) {
            robot.outtake.armPosition = Outtake.ArmPosition.AUTO_VERTICAL;
            return;
        }

        robot.extendo.extendoMode = Extendo.ExtendoMode.AUTOMATIC;
        robot.extendo.targetVector2d = CONE_STACK;
        while (robot.extendo.getDistanceLeft() > EXTENDO_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extendo target", robot.extendo.getTargetLength());
            telemetry.addData("extendo actual", robot.extendo.getCurrentLength());
            telemetry.update();
            robot.sleep(0.01);
        }
        robot.sleep(0.1);
        robot.intake.clawMode = Intake.ClawMode.CLOSED;
        robot.sleep(0.2);

        robot.extendo.manualPower = Extendo.autonomousGoBackAfterStack;
        robot.extendo.extendoMode = Extendo.ExtendoMode.MANUAL;
        robot.sleep(0.15);

        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
        robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
        robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
        robot.sleep(0.2);

        robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
        robot.extendo.targetPosition = Extendo.TargetPosition.TRANSFER;
        transferTimer.reset();
        while (robot.extendo.getCurrentLength() > EXTENDO_THRESHOLD && opModeIsActive() && !isStopRequested() && transferTimer.seconds() < 1.5) {
            robot.sleep(0.01);
        }
        robot.sleep(0.1);
        robot.intake.clawMode = Intake.ClawMode.OPEN;
        robot.sleep(0.2);
        robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
        robot.sleep(0.1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoriesRightContested.START_POSE);
        robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
        robot.elevator.isScoring = false;
        robot.elevator.scoringPosition = Elevator.TargetHeight.HIGH;
        robot.extendo.targetPosition = Extendo.TargetPosition.AUTO_CONE5;
        robot.extendo.targetVector2d = CONE_STACK;
        robot.outtake.turretMode = Outtake.TurretMode.SCORE;
        robot.outtake.armPosition = Outtake.ArmPosition.AUTO_INIT;
        robot.outtake.bulanVar = 10;
        robot.start();
//        PhotonCore.enable();

        int readFromCamera = useCamera(robot);

        if (readFromCamera == -10) {
            return;
        }

        List<Trajectory> trajectories = TrajectoriesRightContested.getTrajectories(readFromCamera);
        telemetry.addData("camera tag", readFromCamera);
        telemetry.update();

        robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
        robot.drive.followTrajectory(trajectories.get(0));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.2);

        for (int i = 1; i <= 6; i++) {
            cycle(robot, i);
        }

        robot.sleep(0.2);
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.outtake.turretMode = Outtake.TurretMode.TRANSFER;
        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
        robot.intake.armPosition = Intake.ArmPosition.AUTOPARK;
        robot.drive.followTrajectory(trajectories.get(1));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.3);
        robot.outtake.armPosition = Outtake.ArmPosition.AUTO_INIT;
        robot.sleep(0.3);

        robot.stop();
    }

    private Extendo.TargetPosition getExtendoLevel(int i) {
        Extendo.TargetPosition targetExtendo;
        switch (i) {
            case 1:
                targetExtendo = Extendo.TargetPosition.AUTO_CONE5;
                break;
            case 2:
                targetExtendo = Extendo.TargetPosition.AUTO_CONE4;
                break;
            case 3:
                targetExtendo = Extendo.TargetPosition.AUTO_CONE3;
                break;
            case 4:
                targetExtendo = Extendo.TargetPosition.AUTO_CONE2;
                break;
            case 5:
                targetExtendo = Extendo.TargetPosition.AUTO_CONE1;
                break;
            default:
                targetExtendo = Extendo.TargetPosition.AUTO_CONE1;
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