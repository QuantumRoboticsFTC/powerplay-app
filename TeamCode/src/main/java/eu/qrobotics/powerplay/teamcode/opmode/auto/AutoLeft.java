package eu.qrobotics.powerplay.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static double EXTENDO_THRESHOLD = 15;

    private ElapsedTime transferTimer = new ElapsedTime(0);


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

        telemetry.setMsTransmissionInterval(50);


        while (!isStarted() && !isStopRequested()) {
            robot.elevator.targetPosition = Elevator.TargetHeight.HIGH;
            robot.extendo.targetPosition = Extendo.TargetHeight.AUTO_CONE5;
            robot.outtake.turretMode = Outtake.TurretMode.SCORE;
            robot.outtake.armPosition = Outtake.ArmPosition.AUTO_INIT;
//            robot.intake.clawMode = Intake.ClawMode.OPEN;
            if (gamepad1.a) {
                robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
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

//        robot.outtake.clawMode = Outtake.ClawMode.CLOSE;
//        robot.outtake.armPosition = Outtake.ArmPosition.UP;
//        robot.intake.clawMode = Intake.ClawMode.OPEN;
//        robot.intake.armPosition = Intake.ArmPosition.CONE_5;
//        robot.intake.armRotate = Intake.ArmRotate.STRAIGHT;


        List<Trajectory> trajectories = TrajectoriesLeft.getTrajectories(readFromCamera);
        telemetry.addData("camera tag", readFromCamera);

        // PRELOAD
        robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
        robot.drive.followTrajectory(trajectories.get(0));
        robot.outtake.armPosition = Outtake.ArmPosition.UP;
        robot.sleep(0.4);
        robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        while (robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.000001);
        }
//        robot.outtake.alignerMode = Outtake.AlignerMode.DEPLOYED;
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.2);
        robot.outtake.armPosition = Outtake.ArmPosition.SCORE;
        robot.intake.clawMode = Intake.ClawMode.CLOSED;
        robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
        robot.intake.armPosition = Intake.ArmPosition.CONE_5;
        robot.sleep(0.3);
        robot.outtake.clawMode = Outtake.ClawMode.OPEN;
        robot.elevator.targetPosition = Elevator.TargetHeight.AUTO_DROP;
        robot.outtake.armPosition = Outtake.ArmPosition.PUSH;
        robot.sleep(0.3);
        robot.outtake.alignerMode = Outtake.AlignerMode.RETRACTED;
        robot.drive.followTrajectorySync(trajectories.get(1));
        robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
        robot.elevator.targetPosition = Elevator.TargetHeight.HIGH;
        robot.sleep(0.2);
        robot.outtake.armPosition = Outtake.ArmPosition.UP;
        robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        int trajectoryIndex = 2;
        for(int i=1;i <= 5;i++){ // i = cycul
            /// extendo target switch
            robot.extendo.targetPosition = getExtendoLevel(i); /// /// CHANGE CONE NR

            robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
            if(i!=1) {
               robot.drive.followTrajectory(trajectories.get(trajectoryIndex++));
            }
            robot.intake.armRotate = Intake.ArmRotate.PARALLEL;


            robot.intake.armPosition = getintakeArmPosition(i); /// CHANGE CONE NR


            robot.intake.clawMode = Intake.ClawMode.OPEN;
            robot.extendo.extendoMode = Extendo.ExtendoMode.UP;
            while (robot.extendo.getDistanceLeft() > EXTENDO_THRESHOLD && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }
            robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
            robot.sleep(0.3);
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }
            robot.intake.clawMode = Intake.ClawMode.CLOSED;
            robot.sleep(0.2);
            robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
            robot.intake.armPosition = Intake.ArmPosition.CONE_5; // go a little bit :sus: so that you go down when transfering instead of going up
            robot.sleep(0.2);
            robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
//            robot.sleep(0.3);
            robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
            robot.drive.followTrajectory(trajectories.get(trajectoryIndex++));
            transferTimer.reset();
            while (robot.extendo.getEncoder() > EXTENDO_THRESHOLD && opModeIsActive() && !isStopRequested() && transferTimer.seconds() < 1.5) {
                robot.sleep(0.01);
            }
            robot.sleep(0.25);
            robot.intake.clawMode = Intake.ClawMode.OPEN;
            robot.sleep(0.15);
            robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;

            robot.intake.armPosition = getintakeArmPosition(i+1); /// CHANGE CONE NR
            robot.intake.armRotate = Intake.ArmRotate.PARALLEL;/// CHANGE CONE NR

            robot.outtake.armPosition = Outtake.ArmPosition.AUTO_INIT;
            robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
            robot.sleep(0.1);
            robot.outtake.armPosition = Outtake.ArmPosition.UP;
            robot.sleep(0.4);
            robot.outtake.turretPosition = Outtake.TurretPosition.AUTO_LEFT_SCORE;
            robot.sleep(0.35);
            while(robot.drive.isBusy() && opModeIsActive() && !isStopRequested()){
                robot.sleep(0.01);
            }
            robot.outtake.armPosition = Outtake.ArmPosition.SCORE;
            robot.sleep(0.3);
//            robot.outtake.alignerMode = Outtake.AlignerMode.DEPLOYED;
            robot.elevator.targetPosition = Elevator.TargetHeight.AUTO_DROP;
            robot.outtake.armPosition = Outtake.ArmPosition.PUSH;
            robot.sleep(0.3);
            robot.outtake.clawMode = Outtake.ClawMode.OPEN;
            robot.sleep(0.2);
//            robot.outtake.alignerMode = Outtake.AlignerMode.RETRACTED;
//            robot.sleep(0.5);
            robot.outtake.armPosition = Outtake.ArmPosition.UP;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
            robot.elevator.targetPosition = Elevator.TargetHeight.HIGH;
//            robot.sleep(0.3);
//            robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
//            robot.sleep(0.5);

//            robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
        }
        robot.sleep(0.3);
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
        robot.intake.armPosition = Intake.ArmPosition.AUTOPARK;
        robot.drive.followTrajectorySync(trajectories.get(trajectoryIndex++));


        robot.stop();
    }
    private Extendo.TargetHeight getExtendoLevel (int i) {
        Extendo.TargetHeight targetExtendo;
        switch (i) {
            case 1: targetExtendo = Extendo.TargetHeight.AUTO_CONE5;
                break;
            case 2: targetExtendo = Extendo.TargetHeight.AUTO_CONE4;
                break;
            case 3: targetExtendo = Extendo.TargetHeight.AUTO_CONE3;
                break;
            case 4: targetExtendo = Extendo.TargetHeight.AUTO_CONE2;
                break;
            case 5: targetExtendo = Extendo.TargetHeight.AUTO_CONE1;
                break;
            default: targetExtendo = Extendo.TargetHeight.AUTO_CONE1;
        }
        return targetExtendo;
    }
    private Intake.ArmPosition getintakeArmPosition (int i) {
        Intake.ArmPosition targetPosition;
        switch (i) {
            case 1: targetPosition = Intake.ArmPosition.CONE_5;
                break;
            case 2: targetPosition = Intake.ArmPosition.CONE_4;
                break;
            case 3: targetPosition = Intake.ArmPosition.CONE_3;
                break;
            case 4: targetPosition = Intake.ArmPosition.CONE_2;
                break;
            case 5: targetPosition = Intake.ArmPosition.CONE_1;
                break;
            case 6: targetPosition = Intake.ArmPosition.VERTICAL;
                break;
            default: targetPosition = Intake.ArmPosition.CONE_1;
        }
        return targetPosition;
    }
}