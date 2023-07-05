package eu.qrobotics.powerplay.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories.TrajectoriesLeftMid;
import eu.qrobotics.powerplay.teamcode.subsystems.Elevator;
import eu.qrobotics.powerplay.teamcode.subsystems.Extendo;
import eu.qrobotics.powerplay.teamcode.subsystems.Intake;
import eu.qrobotics.powerplay.teamcode.subsystems.Outtake;
import eu.qrobotics.powerplay.teamcode.subsystems.Robot;
import eu.qrobotics.powerplay.teamcode.util.StickyGamepad;

@TeleOp
public class DebugTeleOP extends OpMode {
    enum DriveMode {
        NORMAL,
        SLOW,
        SUPER_SLOW
    }

    private Elevator.ElevatorMode prevElevatorMode;
    private ElapsedTime turretCenterTimer = new ElapsedTime(0);
    private ElapsedTime outtakeGrabTimer = new ElapsedTime(0);
    private ElapsedTime finishTransferTimer = new ElapsedTime(0);
    private ElapsedTime toggledClawTimer = new ElapsedTime(0);

    // for some fed up reason the y needs to be positive? prob cuz dt is reversed but sussy
    private Pose2d SUBSTATION_CONE_LEFT_DT = new Pose2d(-12, -12, Math.toRadians(270));
    private Pose2d SUBSTATION_CONE_RIGHT_DT = new Pose2d(+12, -12, Math.toRadians(270));
    private Vector2d SUBSTATION_CONE_LEFT = new Vector2d(-12, -70);
    private Vector2d SUBSTATION_CONE_RIGHT = new Vector2d(+12, 70);

    private NanoClock loopTime;

    private boolean haveConeIntake = false;
    private boolean haveConeOuttake = false;
    private boolean pushOuttake = false;
    private boolean scoringWithIntake = false;
    private boolean intakeTransferStarted = false;

    Intake.ClawMode pastClawMode;

    Robot robot;
    DriveMode driveMode;
    StickyGamepad stickyGamepad1 = null;
    StickyGamepad stickyGamepad2 = null;
    MultipleTelemetry telemetry;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void init() {
        loopTime = NanoClock.system();

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(this, false);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        driveMode = DriveMode.NORMAL;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.log().add("Ready!");
        robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
        robot.intake.clawMode = Intake.ClawMode.OPEN;
        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
    }

    @Override
    public void start() {
        robot.start();
        robot.drive.setPoseEstimate(TrajectoriesLeftMid.START_POSE);
//        PhotonCore.enable();
    }

    @Override
    public void loop() {
        double timestamp = loopTime.seconds();

        stickyGamepad1.update();
        stickyGamepad2.update();

        boolean customCurve = false;
        if (scoringWithIntake &&
            robot.extendo.extendoMode == Extendo.ExtendoMode.RETRACTED) {
            customCurve = true;
        }

        //region Driver 1 controls
        if (!robot.drive.isBusy()) {
            switch (driveMode) {
                case NORMAL:
                    robot.drive.setMotorPowersFromGamepad(gamepad1, 1, customCurve);
                    break;
                case SLOW:
                    robot.drive.setMotorPowersFromGamepad(gamepad1, 0.7, customCurve);
                    break;
                case SUPER_SLOW:
                    robot.drive.setMotorPowersFromGamepad(gamepad1, 0.5, customCurve);
                    break;
            }

        }

        if (gamepad1.right_trigger > 0.1) {
            robot.extendo.extendoMode = Extendo.ExtendoMode.MANUAL;
            if (robot.extendo.extendoLimitTicks - robot.extendo.getEncoder() <= Extendo.extendoLimitDelta) {
                robot.extendo.manualPower = 0;
            } else {
                robot.extendo.manualPower = gamepad1.right_trigger;
            }

            if (robot.intake.armPosition == Intake.ArmPosition.TRANSFER) {
                robot.intake.armPosition = Intake.ArmPosition.CONE_1;
                robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
            }
            if (!haveConeIntake) {
                robot.intake.clawMode = Intake.ClawMode.OPEN;
            }
        } else if (gamepad1.left_trigger > 0.1) {
            robot.extendo.extendoMode = Extendo.ExtendoMode.MANUAL;
            robot.extendo.manualPower = -gamepad1.left_trigger * 0.85;
        }

        if (robot.extendo.extendoMode != Extendo.ExtendoMode.BRAKE &&
                robot.intake.armPosition == Intake.ArmPosition.LOW_POLE_WHEN_IN_TRANSFER) {
            robot.intake.armPosition = Intake.ArmPosition.LOW_POLE;
        }

        if (robot.extendo.extendoMode == Extendo.ExtendoMode.BRAKE &&
                robot.intake.armPosition == Intake.ArmPosition.LOW_POLE) {
            robot.intake.armPosition = Intake.ArmPosition.LOW_POLE_WHEN_IN_TRANSFER;
        }

        if (gamepad1.right_trigger <= 0.1 && gamepad1.left_trigger <= 0.1) {
            robot.extendo.manualPower = 0;
        }

        if (stickyGamepad1.left_bumper) {
            robot.intake.armPosition = robot.intake.armPosition.previous();
            if (robot.intake.armPosition == Intake.ArmPosition.LOW_POLE ||
                robot.intake.armPosition == Intake.ArmPosition.LOW_POLE_WHEN_IN_TRANSFER) {
                robot.intake.armRotate = Intake.ArmRotate.LOW_POLE;
            } else {
                robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
            }
        }
        if (stickyGamepad1.right_bumper) {
            robot.intake.armPosition = robot.intake.armPosition.next();
            if (robot.intake.armPosition == Intake.ArmPosition.LOW_POLE ||
                robot.intake.armPosition == Intake.ArmPosition.LOW_POLE_WHEN_IN_TRANSFER) {
                robot.intake.armRotate = Intake.ArmRotate.LOW_POLE;
            } else {
                robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
            }
        }

        if (!scoringWithIntake &&
            robot.intake.clawMode == Intake.ClawMode.CLOSED &&
            (robot.intake.armPosition == Intake.ArmPosition.CONE_1 ||
            robot.intake.armPosition == Intake.ArmPosition.CONE_2 ||
            robot.intake.armPosition == Intake.ArmPosition.CONE_3 ||
            robot.intake.armPosition == Intake.ArmPosition.CONE_4 ||
            robot.intake.armPosition == Intake.ArmPosition.CONE_5) &&
                (Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_y) > 0.1)) {
            retract();
        }

        // aici la close cand e pe low sa *censored* lowul
        if (stickyGamepad1.a) {
            pastClawMode = robot.intake.clawMode;
            toggledClawTimer.reset();
        }

        // ez backdoors backrooms DPAD region gen

        telemetry.addData("dt getX", robot.drive.getPoseEstimate().getX());
        telemetry.addData("dt getY", robot.drive.getPoseEstimate().getY());
        telemetry.addData("dt heading", robot.drive.getPoseEstimate().headingVec());

        if (stickyGamepad1.dpad_up) {
            switch (driveMode) {
                case SUPER_SLOW:
                    driveMode = DriveMode.NORMAL;
                    break;
                case NORMAL:
                    driveMode = DriveMode.SUPER_SLOW;
                    break;
            }
        }

        if (stickyGamepad1.dpad_down) {
            retract();
        }

        if (stickyGamepad1.b &&
            robot.intake.armPosition == Intake.ArmPosition.TRANSFER &&
            robot.outtake.armPosition == Outtake.ArmPosition.TRANSFER) {
            intakeTransferStarted = true;
        }

        //endregion
        // region Driver 2 controls

        if (stickyGamepad2.b) {
            intakeTransferStarted = true;
        }

        if (stickyGamepad2.dpad_up) {
            robot.elevator.scoringPosition = Elevator.TargetHeight.HIGH_TILTED;
            if (haveConeIntake) {
                intakeTransferStarted = true;
            }
        } else if (stickyGamepad2.dpad_left) {
            robot.elevator.scoringPosition = Elevator.TargetHeight.MID_TILTED;
            if (haveConeIntake) {
                intakeTransferStarted = true;
            }
        } else if (stickyGamepad2.dpad_down) {
            robot.elevator.scoringPosition = Elevator.TargetHeight.LOW_TILTED;
            if (haveConeIntake) {
                intakeTransferStarted = true;
            }
        }

        if (stickyGamepad2.dpad_right) {
            if (robot.outtake.isScoring) {
                if (robot.outtake.alignerMode == Outtake.AlignerMode.DEPLOYED)
                    robot.outtake.alignerMode = Outtake.AlignerMode.RETRACTED;
                else if (robot.outtake.alignerMode == Outtake.AlignerMode.RETRACTED)
                    robot.outtake.alignerMode = Outtake.AlignerMode.DEPLOYED;
            } else {
                robot.outtake.alignerActive = !robot.outtake.alignerActive;
            }
        }

        if (stickyGamepad2.right_bumper) {
            pushOuttake = true;
        } else if (stickyGamepad2.left_bumper) {
            if (robot.outtake.clawMode == Outtake.ClawMode.CLOSED) {
                robot.outtake.clawMode = Outtake.ClawMode.OPEN;
            }

            robot.outtake.isScoring = false;
            robot.elevator.isScoring = false;
            robot.outtake.alignerActive = true;
            robot.outtake.alignerMode = Outtake.AlignerMode.RETRACTED;
            robot.outtake.armPosition = Outtake.ArmPosition.SCORE;
            robot.outtake.turretMode = Outtake.TurretMode.TRANSFER;
            robot.outtake.clawMode = Outtake.ClawMode.OPEN;
            robot.elevator.targetPosition = Elevator.TargetHeight.GROUND;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.AUTOMATIC;
            turretCenterTimer.reset();
        }

        if(robot.elevator.targetPosition == Elevator.TargetHeight.LOW && robot.elevator.elevatorMode == Elevator.ElevatorMode.AUTOMATIC &&  robot.outtake.armPosition != Outtake.ArmPosition.MANUAL) {
            robot.outtake.armPosition = Outtake.ArmPosition.SCORE;
        }

        if (gamepad2.right_trigger > 0.1) {
            if (robot.elevator.elevatorMode != Elevator.ElevatorMode.MANUAL)
                prevElevatorMode = robot.elevator.elevatorMode;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
            robot.elevator.manualPower = gamepad2.right_trigger * 0.75;
        } else if (gamepad2.left_trigger > 0.1) {
            if (robot.elevator.elevatorMode != Elevator.ElevatorMode.MANUAL)
                prevElevatorMode = robot.elevator.elevatorMode;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
            robot.elevator.manualPower = gamepad2.left_trigger * (-0.07);
        } else {
            if (robot.elevator.elevatorMode == Elevator.ElevatorMode.MANUAL) {
                robot.elevator.offsetPosition = robot.elevator.getEncoder() - robot.elevator.targetPosition.getEncoderPosition();
                robot.elevator.elevatorMode = prevElevatorMode;
                prevElevatorMode = null;
            }
        }

        if (stickyGamepad2.y) {
            robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        }

        if (stickyGamepad2.a) {
            if (robot.outtake.isScoring) {
                haveConeOuttake = false;
            }
            robot.outtake.clawMode = Outtake.ClawMode.OPEN;
        }

        if (Math.abs(gamepad2.right_stick_x) > 0.1) {
            if (robot.outtake.turretPosition != Outtake.TurretPosition.MANUAL) {
                robot.outtake.manualOffset = robot.outtake.getTurretPosition();
            }
            if (robot.outtake.manualOffset - gamepad2.right_stick_x * 0.01 > 1)
                robot.outtake.manualOffset = 1;
            else if (robot.outtake.manualOffset - gamepad2.right_stick_x * 0.01 < -1)
                robot.outtake.manualOffset = -1;
            else
                robot.outtake.manualOffset -= gamepad2.right_stick_x * 0.01;
            robot.outtake.turretPosition = Outtake.TurretPosition.MANUAL;
        }

        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
            if (robot.outtake.armPosition != Outtake.ArmPosition.MANUAL) {
                robot.outtake.armManualOffset = robot.outtake.getArmPosition();
            }
            // TODO: robot.outtake.armManualOffset = Range.clip(robot.outtake.getTargetArmServoPosition(Outtake.OUTTAKE_AUTO_PRELOAD_POS), 0, 1);
            if (robot.outtake.armManualOffset + gamepad2.left_stick_y * 0.005 > 1)
                robot.outtake.armManualOffset = 1;
            else if (robot.outtake.manualOffset + gamepad2.left_stick_y * 0.005 < -1)
                robot.outtake.armManualOffset = -1;
            else
                robot.outtake.armManualOffset += gamepad2.left_stick_y * 0.005;
            robot.outtake.armPosition = Outtake.ArmPosition.MANUAL;
        }
        //endregion

        //region Update Timers
        updateOuttakeGrabTimer();
        updateTurretCenterTimer();
        updateToggleClawTimer();
        finishTransfer();
        pushOuttakeFunc();
        //endregion

        //region Telemetry

        telemetry.addData("turret servo pos", robot.outtake.getTurretServoPosition());

        telemetry.addData("elevator isScoring", robot.elevator.isScoring);

        telemetry.addData("elevator target position", robot.elevator.targetPosition.encoderPosition);
        telemetry.addData("current position", robot.elevator.getRawEncoder());

        telemetry.addData("aligner active", robot.outtake.alignerActive);
        telemetry.addData("isScoring", robot.outtake.isScoring);

        telemetry.addData("armPosition:", robot.intake.armPosition);
        telemetry.addData("armRotate:", robot.intake.armRotate);
        telemetry.addData("clawMode:", robot.intake.clawMode);

        telemetry.addData("Aligner Mode", robot.outtake.alignerMode);
        telemetry.addData("Elevator Encoder", robot.elevator.getEncoder());
        telemetry.addData("Extendo Length", robot.extendo.getCurrentLength());
        telemetry.addData("Extendo Mode", robot.extendo.extendoMode);
        telemetry.addData("Elevator Left power", robot.elevator.leftPowah);
        telemetry.addData("Elevator Right power", robot.elevator.rightPowah);

        telemetry.addData("extendo power", robot.extendo.powah);

        telemetry.addData("extendo encoders", robot.extendo.getEncoder());

        telemetry.addData("loop time", (loopTime.seconds() - timestamp) * 1000.0);

        addStatistics();
        telemetry.update();
        //endregion

//        TelemetryPacket packet = new TelemetryPacket();
//        Canvas fieldOverlay = packet.fieldOverlay();

        /*
        Turret drawing thing - but OUTTAKE_AUTO_PRELOAD_POS are dumb idfk i didnt want to delete

        Pose2d robotPose = robot.drive.getPoseEstimate();
        Pose2d turretWorldPose = new Pose2d(robotPose.vec().plus(TURRET_ROBOT_POSE.vec().rotated(robotPose.getHeading())), robotPose.getHeading() + TURRET_ROBOT_POSE.getHeading());
        double turretAngle = robot.outtake.getTargetTurretAngle(OUTTAKE_AUTO_PRELOAD_POS);
        Vector2d turretVector = new Vector2d(10, 0).rotated(turretAngle).rotated(turretWorldPose.getHeading());


        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, robotPose);
        fieldOverlay.setStroke("#4CAF50");
        fieldOverlay.fillCircle(Outtake.OUTTAKE_AUTO_PRELOAD_POS.getX(), Outtake.OUTTAKE_AUTO_PRELOAD_POS.getY(), 3);
        fieldOverlay.setStroke("#FF9800");
        fieldOverlay.strokeLine(turretWorldPose.getX(), turretWorldPose.getY(), turretWorldPose.getX() + turretVector.getX(), turretWorldPose.getY() + turretVector.getY());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
         */


    }

    private void retract() {
        if (robot.intake.clawMode == Intake.ClawMode.CLOSED) {
            scoringWithIntake = true;
        }
        robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
        robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
    }

    private void updateToggleClawTimer() {
        if (pastClawMode == Intake.ClawMode.OPEN) {
            if (toggledClawTimer.seconds() < 0.1) {
                robot.intake.clawMode = Intake.ClawMode.CLOSED;
                haveConeIntake = true;
            }

            if (0.35 < toggledClawTimer.seconds() && toggledClawTimer.seconds() < 0.4) {
                if (robot.intake.armPosition == Intake.ArmPosition.CONE_2 ||
                        robot.intake.armPosition == Intake.ArmPosition.CONE_3 ||
                        robot.intake.armPosition == Intake.ArmPosition.CONE_4 ||
                        robot.intake.armPosition == Intake.ArmPosition.CONE_5) {
                    robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
                }
            }
        } else {
            if (robot.intake.armRotate == Intake.ArmRotate.LOW_POLE ||
                robot.intake.armRotate == Intake.ArmRotate.LOW_POLE_DROP) {
                if (toggledClawTimer.seconds() < 0.15) {
                    robot.intake.armRotate = Intake.ArmRotate.LOW_POLE_DROP;
                }
                if (0.15 < toggledClawTimer.seconds() && toggledClawTimer.seconds() < 0.3) {
                    robot.intake.clawMode = Intake.ClawMode.OPEN;
                }
                if (0.3 < toggledClawTimer.seconds() && toggledClawTimer.seconds() < 0.4) {
                    robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
                    robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
                    robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
                    scoringWithIntake = false;
                }
            } else {
                if (toggledClawTimer.seconds() < 0.1) {
                    robot.intake.clawMode = Intake.ClawMode.OPEN;
                    if (robot.extendo.targetPosition == Extendo.TargetPosition.TRANSFER) {
                        scoringWithIntake = false;
                    }
                }
            }
        }
    }

    private void pushOuttakeFunc() {
        if (pushOuttake) {
            if (haveConeIntake) {
                intakeTransferStarted = true;
                return;
            }
            robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
            robot.outtake.isScoring = true;
            robot.elevator.isScoring = true;
            if (robot.outtake.alignerActive) {
                robot.outtake.alignerMode = Outtake.AlignerMode.DEPLOYED;
            }
            robot.elevator.elevatorMode = Elevator.ElevatorMode.AUTOMATIC;
            outtakeGrabTimer.reset();
            pushOuttake = false;
        }
    }

    private void finishTransfer() {
        if (intakeTransferStarted &&
            robot.extendo.extendoMode == Extendo.ExtendoMode.BRAKE &&
            finishTransferTimer.seconds() > 0.9) {
            finishTransferTimer.reset();
        }
        if (0.075 < finishTransferTimer.seconds() && finishTransferTimer.seconds() < 0.125) {
            robot.intake.clawMode = Intake.ClawMode.OPEN;
        }
        if (0.45 < finishTransferTimer.seconds() && finishTransferTimer.seconds() < 0.5) {
            robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
        }
        if (0.55 < finishTransferTimer.seconds() && finishTransferTimer.seconds() < 0.6) {
            robot.outtake.armPosition = Outtake.ArmPosition.UP;
            intakeTransferStarted = false;
            haveConeIntake = false;
            scoringWithIntake = false;
            haveConeOuttake = true;
        }
    }

    private void updateTurretCenterTimer() {
        if (0.1 < turretCenterTimer.seconds() && turretCenterTimer.seconds() < 0.2) {
            robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
            robot.elevator.targetPosition = Elevator.TargetHeight.GROUND;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.AUTOMATIC;
        }
    }

    private void updateOuttakeGrabTimer() {
        if (0.05 < outtakeGrabTimer.seconds() && outtakeGrabTimer.seconds() < 0.15) {
            robot.outtake.turretMode = Outtake.TurretMode.SCORE;
        }
        if (0.15 < outtakeGrabTimer.seconds() && outtakeGrabTimer.seconds() < 0.3) {
            robot.outtake.armPosition = Outtake.ArmPosition.SCORE_TILTED;
        }
    }

    private static String formatResults(MovingStatistics statistics) {
        return Misc.formatInvariant("μ = %.2fms, σ = %.2fms, err = %.3fms",
                statistics.getMean() * 1000,
                statistics.getStandardDeviation() * 1000,
                statistics.getStandardDeviation() / Math.sqrt(statistics.getCount()) * 1000);
    }

    private void addStatistics() {
        telemetry.addData("Top 250", formatResults(robot.top250));
        telemetry.addData("Top 100", formatResults(robot.top100));
        telemetry.addData("Top 10", formatResults(robot.top10));
    }

    @Override
    public void stop() {
        robot.stop();
    }
}