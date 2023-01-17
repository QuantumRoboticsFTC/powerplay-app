package eu.qrobotics.powerplay.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import eu.qrobotics.powerplay.teamcode.AprilTagDetectionPipeline;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

import eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories.TrajectoriesLeft;
import eu.qrobotics.powerplay.teamcode.subsystems.Elevator;
import eu.qrobotics.powerplay.teamcode.subsystems.Extendo;
import eu.qrobotics.powerplay.teamcode.subsystems.Intake;
import eu.qrobotics.powerplay.teamcode.subsystems.Outtake;
import eu.qrobotics.powerplay.teamcode.subsystems.Robot;


@Config
@Autonomous
public class AutoLeft extends LinearOpMode {
//    private static final String VUFORIA_KEY =
//            "AZkUdjT/////AAABmXTO5InzOk5Yo1OlXRo+EFQkVF1qqCBnur0y1G+RBktOx7nVuzRvRaahrrHE0OJTAUwmyuPkGSbIFETtV9VN5Ezo8vTtN90u2lqAMZx5ZY5qWtTs+rm/2y4CctYrNhnxeme+qRNeRj6gKhUMa2FAVHr2qBtJK/CrZ0Ud/1vpLavIr+TrHuIjABcEXRyXBcdIaj5gw4EiVChCFrjv24qMiHuOq1pHOAbpTqe392045VnPLDlJ6bJKq0cNZ3TR86ccLGd2Pg0lnVLvf/qthVFRy8NASoyrgQkEU0P5WSC+8A3IlWPPEgG2LMu8FACw+6t1da+EiznSyu5dSW7UcAw5oHKpGgfxRV3pXmNJ3bn+AfMi";
//
//    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//    private static final String LABEL_FIRST_ELEMENT = "Quad";
//    private static final String LABEL_SECOND_ELEMENT = "Single";

//    private VuforiaLocalizer vuforia;
//    private TFObjectDetector tfod;

    //    public static Point TOP_LEFT = new Point(500, 250);
//    public static Point BOTTOM_RIGHT = new Point(775, 500);
    public static double ELEVATOR_THRESHOLD = 2;
    public static double EXTENDO_THRESHOLD = 5;

    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoriesLeft.START_POSE);
        robot.start();

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.openCameraDevice();
//        webcam.showFpsMeterOnViewport(true);
//        RingDetector ringDetector = new RingDetector(webcam, TOP_LEFT, BOTTOM_RIGHT);

//        webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
//        webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
//        FtcDashboard.getInstance().startCameraStream(webcam, 30);

//        RingDetector.Stack parkPosition = RingDetector.Stack.FOUR;

        int readFromCamera = -1;

        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        final double FEET_PER_METER = 3.28084;

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
//                    throw new Exception("");
            }
        });

//        telemetry.setMsTransmissionInterval(50);


        while (!isStarted() && !isStopRequested()) {
            robot.elevator.targetPosition = Elevator.TargetHeight.HIGH;
            robot.extendo.targetPosition = Extendo.TargetHeight.AUTO_CONE5;
            robot.outtake.turretMode = Outtake.TurretMode.SCORE;
            robot.intake.clawMode = Intake.ClawMode.OPEN;
            if (gamepad1.a) {
                robot.outtake.clawMode = Outtake.ClawMode.CLOSE;
            }
            telemetry.addLine("Press a on gamepad 1 to close claw");
//            telemetry.update();
//        webcam.stopStreaming();
//        resetStartTime();

//            while (!isStopRequested()) {
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

                telemetry.update();
//                sleep(20);
//            }
        }

        if (isStopRequested()) {
            robot.stop();
            return;
        }

        robot.outtake.clawMode = Outtake.ClawMode.CLOSE;
        robot.outtake.armPosition = Outtake.ArmPosition.UP;
        robot.intake.clawMode = Intake.ClawMode.OPEN;
        robot.intake.armPosition = Intake.ArmPosition.CONE_5;
        robot.intake.armRotate = Intake.ArmRotate.STRAIGHT;


        List<Trajectory> trajectories = TrajectoriesLeft.getTrajectories(readFromCamera);
        telemetry.addData("camera tag", readFromCamera);

        robot.drive.followTrajectorySync(trajectories.get(0));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.drive.followTrajectorySync(trajectories.get(1));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
        robot.outtake.turretPosition = Outtake.TurretPosition.RIGHT;
        while (robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.outtake.armPosition = Outtake.ArmPosition.UP;
        robot.sleep(1);
        robot.outtake.armPosition = Outtake.ArmPosition.SCORE;
        robot.sleep(1);
        robot.outtake.armPosition = Outtake.ArmPosition.PUSH;
        robot.sleep(0.2);
        robot.outtake.clawMode = Outtake.ClawMode.OPEN;
        robot.sleep(0.5);
        robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
        robot.sleep(0.5);
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.sleep(0.5);
        robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
        robot.sleep(0.3);

        /*robot.intake.armPosition = Intake.ArmPosition.CONE_5    ; // start intek
        robot.intake.armRotate = Intake.ArmRotate.PARALLEL_AUTO;
        robot.extendo.extendoMode = Extendo.ExtendoMode.UP;
        robot.outtake.armPosition = Outtake.ArmPosition.UP;
        robot.sleep(0.5);
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.sleep(0.5);
//            while (robot.extendo.getDistanceLeft() > EXTENDO_THRESHOLD && opModeIsActive() && !isStopRequested())
//                robot.sleep(0.01);
        robot.sleep(1);
        robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
        robot.outtake.clawMode = Outtake.ClawMode.OPEN;
        robot.intake.clawMode = Intake.ClawMode.CLOSE;
        robot.sleep(0.5);
        robot.intake.armRotate = Intake.ArmRotate.STRAIGHT;
        robot.sleep(0.5);
        robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
        robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
        while (robot.extendo.getDistanceLeft() > EXTENDO_THRESHOLD && opModeIsActive() && !isStopRequested())
            robot.sleep(0.01);

        robot.sleep(1);
        robot.outtake.clawMode = Outtake.ClawMode.CLOSE;
        robot.sleep(0.2);
        robot.intake.clawMode = Intake.ClawMode.OPEN;
        robot.sleep(0.5);
        robot.outtake.armPosition = Outtake.ArmPosition.UP;
        robot.sleep(0.5);
        robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
        robot.outtake.turretPosition = Outtake.TurretPosition.RIGHT;
        while (robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.outtake.armPosition = Outtake.ArmPosition.SCORE;
        robot.sleep(0.5);
        robot.outtake.armPosition = Outtake.ArmPosition.PUSH;
        robot.sleep(0.2);
        robot.outtake.clawMode = Outtake.ClawMode.OPEN;
        robot.sleep(0.5);
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.sleep(1);
        robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
        robot.sleep(0.3);
        robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
        robot.sleep(1);


        //2+1
        robot.extendo.targetPosition = Extendo.TargetHeight.AUTO_CONE4;
        robot.intake.armPosition = Intake.ArmPosition.CONE_4    ; // start intek
        robot.intake.armRotate = Intake.ArmRotate.PARALLEL_AUTO;
        robot.extendo.extendoMode = Extendo.ExtendoMode.UP;
        robot.outtake.armPosition = Outtake.ArmPosition.UP;
        robot.sleep(0.5);
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.sleep(0.5);
//            while (robot.extendo.getDistanceLeft() > EXTENDO_THRESHOLD && opModeIsActive() && !isStopRequested())
//                robot.sleep(0.01);
        robot.sleep(1);
        robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
        robot.outtake.clawMode = Outtake.ClawMode.OPEN;
        robot.intake.clawMode = Intake.ClawMode.CLOSE;
        robot.sleep(0.5);
        robot.intake.armRotate = Intake.ArmRotate.STRAIGHT;
        robot.sleep(0.5);
        robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
        robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
        while (robot.extendo.getDistanceLeft() > EXTENDO_THRESHOLD && opModeIsActive() && !isStopRequested())
            robot.sleep(0.01);

        robot.sleep(1);
        robot.outtake.clawMode = Outtake.ClawMode.CLOSE;
        robot.sleep(0.2);
        robot.intake.clawMode = Intake.ClawMode.OPEN;
        robot.sleep(0.5);
        robot.outtake.armPosition = Outtake.ArmPosition.UP;
        robot.sleep(0.5);
        robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
        robot.outtake.turretPosition = Outtake.TurretPosition.RIGHT;
        while (robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.outtake.armPosition = Outtake.ArmPosition.SCORE;
        robot.sleep(0.5);
        robot.outtake.armPosition = Outtake.ArmPosition.PUSH;
        robot.sleep(0.2);
        robot.outtake.clawMode = Outtake.ClawMode.OPEN;
        robot.sleep(0.5);
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.sleep(1);
        robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
        robot.sleep(0.3);
        robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;

        robot.sleep(0.2);*/

        // parcare
        robot.drive.followTrajectorySync(trajectories.get(2));

        robot.stop();
    }
}