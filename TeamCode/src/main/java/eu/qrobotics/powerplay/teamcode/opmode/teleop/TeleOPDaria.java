//package eu.qrobotics.powerplay.teamcode.opmode.teleop;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.util.NanoClock;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.MovingStatistics;
//
//import org.firstinspires.ftc.robotcore.internal.system.Misc;
//
//import eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories.TrajectoriesPastLeft;
//import eu.qrobotics.powerplay.teamcode.subsystems.Elevator;
//import eu.qrobotics.powerplay.teamcode.subsystems.Extendo;
//import eu.qrobotics.powerplay.teamcode.subsystems.Intake;
//import eu.qrobotics.powerplay.teamcode.subsystems.Outtake;
//import eu.qrobotics.powerplay.teamcode.subsystems.Robot;
//import eu.qrobotics.powerplay.teamcode.util.StickyGamepad;
//
//@TeleOp
//public class TeleOPDaria extends OpMode {
//    enum DriveMode {
//        NORMAL,
//        SLOW,
//        SUPER_SLOW
//    }
//
//    private Elevator.ElevatorMode prevElevatorMode;
//    private ElapsedTime turretCenterTimer = new ElapsedTime(0);
//    private ElapsedTime outtakeGrabTimer = new ElapsedTime(0);
//    private ElapsedTime coneDropTimer = new ElapsedTime(0);
//
//    private NanoClock loopTime;
//
//    private boolean careAboutIntakeSensor = true;
//    private boolean startedTransfer = false;
//    private boolean intakeTransferStarted = false;
//
//    private boolean staccMode = false;
//
//    Robot robot;
//    DriveMode driveMode;
//    StickyGamepad stickyGamepad1 = null;
//    StickyGamepad stickyGamepad2 = null;
//    MultipleTelemetry telemetry;
//
//    private VoltageSensor batteryVoltageSensor;
//
//    @Override
//    public void init() {
//        loopTime = NanoClock.system();
//
//        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
//        robot = new Robot(this, false);
//        stickyGamepad1 = new StickyGamepad(gamepad1);
//        stickyGamepad2 = new StickyGamepad(gamepad2);
//        driveMode = DriveMode.NORMAL;
//
//        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
//
//        telemetry.log().add("Ready!");
//        robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
//        robot.intake.clawMode = Intake.ClawMode.OPEN;
//        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
//    }
//
//    @Override
//    public void start() {
//        robot.start();
//        robot.drive.setPoseEstimate(TrajectoriesPastLeft.START_POSE);
//    }
//
//    @Override
//    public void loop() {
//        double timestamp = loopTime.seconds();
//
//        stickyGamepad1.update();
//        stickyGamepad2.update();
//
//        //region Driver 1 controls
//        if (!robot.drive.isBusy()) {
//            switch (driveMode) {
//                case NORMAL:
//                    robot.drive.setMotorPowersFromGamepad(gamepad1, 1);
//                    break;
//                case SLOW:
//                    robot.drive.setMotorPowersFromGamepad(gamepad1, 0.7);
//                    break;
//                case SUPER_SLOW:
//                    robot.drive.setMotorPowersFromGamepad(gamepad1, 0.5);
//                    break;
//            }
//
//        }
//
//        if (stickyGamepad1.left_bumper) {
//            driveMode = DriveMode.SUPER_SLOW;
//        }
//        if (stickyGamepad1.right_bumper) {
//            driveMode = DriveMode.NORMAL;
//        }
//
//        if (gamepad1.right_trigger > 0.1) {
//            robot.extendo.extendoMode = Extendo.ExtendoMode.MANUAL;
//            if (robot.extendo.extendoLimitTicks - robot.extendo.getEncoder() <= Extendo.extendoLimitDelta) {
//                robot.extendo.manualPower = 0;
//            } else {
//                robot.extendo.manualPower = gamepad1.right_trigger;
//            }
//            if (robot.intake.armPosition != Intake.ArmPosition.CONE_1 &&
//                    robot.intake.armPosition != Intake.ArmPosition.CONE_2 &&
//                    robot.intake.armPosition != Intake.ArmPosition.CONE_3 &&
//                    robot.intake.armPosition != Intake.ArmPosition.CONE_4 &&
//                    robot.intake.armPosition != Intake.ArmPosition.CONE_5 &&
//                    robot.intake.armPosition != Intake.ArmPosition.LOW_POLE)
//                robot.intake.armPosition = Intake.ArmPosition.CONE_1;
//            if (robot.intake.armPosition != Intake.ArmPosition.LOW_POLE) {
//                robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
//                robot.intake.clawMode = Intake.ClawMode.OPEN;
//                careAboutIntakeSensor = true;
//                startedTransfer = false;
//            }
//        } else if (gamepad1.left_trigger > 0.1) {
//            robot.extendo.extendoMode = Extendo.ExtendoMode.MANUAL;
//            robot.extendo.manualPower = -gamepad1.left_trigger * 0.85;
//        }
//        if (careAboutIntakeSensor && robot.intake.armRotate != Intake.ArmRotate.TRANSFER && robot.intake.clawMode == Intake.ClawMode.OPEN) {
//            if (robot.intake.hasCone())
//                robot.intake.clawMode = Intake.ClawMode.CLOSED;
//        }
////        if(stickyGamepad1.y) {
////            if(robot.extendo.manualSpeedMode == Extendo.ManualSpeedMode.FAST)
////                robot.extendo.manualSpeedMode = Extendo.ManualSpeedMode.SLOW;
////            else
////                robot.extendo.manualSpeedMode = Extendo.ManualSpeedMode.FAST;
////        }
//
//        if (gamepad1.right_trigger <= 0.1 && gamepad1.left_trigger <= 0.1) {
//            robot.extendo.manualPower = 0;
//        }
//        if (stickyGamepad1.dpad_down) {
//            robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
//            if (!staccMode || (robot.intake.armPosition == Intake.ArmPosition.VERTICAL && robot.intake.armRotate == Intake.ArmRotate.STRAIGHT)) {
//                robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
//                robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
//                if (robot.intake.clawMode == Intake.ClawMode.CLOSED) {
//                    intakeTransferStarted = true;
//                }
//            } else {
//                robot.intake.armRotate = Intake.ArmRotate.STRAIGHT;
//                robot.intake.armPosition = Intake.ArmPosition.VERTICAL;
//            }
//        }
//        if (stickyGamepad1.dpad_left) {
//            robot.intake.armPosition = robot.intake.armPosition.previous();
//            if(robot.intake.armPosition == Intake.ArmPosition.LOW_POLE) {
//                if (robot.intake.armRotate == Intake.ArmRotate.LOW_POLE) {
//                    robot.intake.armRotate = Intake.ArmRotate.LOW_POLE_DROP;
//                } else {
//                    robot.intake.armRotate = Intake.ArmRotate.LOW_POLE;
//                }
//            } else
//                robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
//        }
//        if (stickyGamepad1.dpad_right) {
//            robot.intake.armPosition = robot.intake.armPosition.next();
//            if(robot.intake.armPosition == Intake.ArmPosition.LOW_POLE) {
//                if (robot.intake.armRotate == Intake.ArmRotate.LOW_POLE) {
//                    robot.intake.armRotate = Intake.ArmRotate.LOW_POLE_DROP;
//                } else {
//                    robot.intake.armRotate = Intake.ArmRotate.LOW_POLE;
//                }
//            } else
//                robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
//        }
//        if (stickyGamepad1.dpad_up) {
//            robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
//        }
//        if (stickyGamepad1.a) {
//            switch (robot.intake.clawMode) {
//                case OPEN:
//                    robot.intake.clawMode = Intake.ClawMode.CLOSED;
//                    break;
//                case CLOSED:
//                    robot.intake.clawMode = Intake.ClawMode.OPEN;
//                    break;
//            }
//        }
//
//        if (stickyGamepad1.back) {
//            if (robot.intake.armPosition == Intake.ArmPosition.FULL_0)
//                robot.intake.armPosition = Intake.ArmPosition.FULL_1;
//            else
//                robot.intake.armPosition = Intake.ArmPosition.FULL_0;
//        }
//        //endregion
//        // region Driver 2 controls
//
//        if (stickyGamepad2.back) {
//            staccMode = !staccMode;
//        }
//
////        if (robot.elevator.elevatorMode == Elevator.ElevatorMode.DOWN) {
//        if (stickyGamepad2.dpad_up) {
//            if (staccMode) {
//                robot.elevator.scoringPosition = Elevator.TargetHeight.HIGH;
//            } else {
//                robot.elevator.scoringPosition = Elevator.TargetHeight.HIGH_TILTED;
//            }
//        } else if (stickyGamepad2.dpad_left) {
//            if (staccMode) {
//                robot.elevator.scoringPosition = Elevator.TargetHeight.MID;
//            } else {
//                robot.elevator.scoringPosition = Elevator.TargetHeight.MID_TILTED;
//            }
//        } else if (stickyGamepad2.dpad_down) {
//            robot.elevator.scoringPosition = Elevator.TargetHeight.LOW_TILTED;
//        }
////        }
//        if (stickyGamepad2.dpad_right) {
//            if (robot.outtake.isScoring) {
//                if (robot.outtake.alignerMode == Outtake.AlignerMode.DEPLOYED)
//                    robot.outtake.alignerMode = Outtake.AlignerMode.RETRACTED;
//                else if (robot.outtake.alignerMode == Outtake.AlignerMode.RETRACTED)
//                    robot.outtake.alignerMode = Outtake.AlignerMode.DEPLOYED;
//            } else {
//                robot.outtake.alignerActive = !robot.outtake.alignerActive;
//            }
//        }
//        if (stickyGamepad2.right_bumper) {
//            robot.outtake.isScoring = true;
//            robot.elevator.isScoring = true;
//            if (robot.outtake.alignerActive) {
//                robot.outtake.alignerMode = Outtake.AlignerMode.DEPLOYED;
//            }
//            robot.elevator.elevatorMode = Elevator.ElevatorMode.AUTOMATIC;
//            startedTransfer = false;
//            outtakeGrabTimer.reset();
//        } else if (stickyGamepad2.left_bumper) {
//            robot.elevator.isScoring = false;
//            robot.outtake.isScoring = false;
//            robot.outtake.alignerActive = true;
//            robot.outtake.clawMode = Outtake.ClawMode.OPEN;
//            robot.sleep(0.3);
//            robot.outtake.alignerMode = Outtake.AlignerMode.RETRACTED;
//            robot.outtake.armPosition = Outtake.ArmPosition.UP;
//            robot.outtake.turretMode = Outtake.TurretMode.TRANSFER;
//            robot.elevator.targetPosition = Elevator.TargetHeight.GROUND;
//            robot.elevator.elevatorMode = Elevator.ElevatorMode.AUTOMATIC;
//            turretCenterTimer.reset();
//        }
////        if(robot.elevator.targetPosition == Elevator.TargetHeight.LOW && robot.elevator.elevatorMode == Elevator.ElevatorMode.AUTOMATIC &&  robot.outtake.armPosition != Outtake.ArmPosition.MANUAL) {
////            robot.outtake.armPosition = Outtake.ArmPosition.SCORE;
////        }
//
//        if (gamepad2.right_trigger > 0.1) {
//            if (robot.elevator.elevatorMode != Elevator.ElevatorMode.MANUAL)
//                prevElevatorMode = robot.elevator.elevatorMode;
//            robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
//            robot.elevator.manualPower = gamepad2.right_trigger * 0.75;
//        } else if (gamepad2.left_trigger > 0.1) {
//            if (robot.elevator.elevatorMode != Elevator.ElevatorMode.MANUAL)
//                prevElevatorMode = robot.elevator.elevatorMode;
//            robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
//            robot.elevator.manualPower = gamepad2.left_trigger * (-0.07);
//        } else {
//            if (robot.elevator.elevatorMode == Elevator.ElevatorMode.MANUAL) {
//                robot.elevator.offsetPosition = robot.elevator.getEncoder() - robot.elevator.targetPosition.getEncoderPosition();
//                robot.elevator.elevatorMode = prevElevatorMode;
//                prevElevatorMode = null;
//            }
//        }
//
//        if (stickyGamepad2.x) {
//            if(!staccMode)
//                robot.outtake.turretPosition = Outtake.TurretPosition.RIGHT;
//            else
//                robot.outtake.turretPosition = Outtake.TurretPosition.AUTO_RIGHT_SCORE;
//        } else if (stickyGamepad2.y) {
//            robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
//        } else if (stickyGamepad2.b) {
//            if(!staccMode)
//                robot.outtake.turretPosition = Outtake.TurretPosition.LEFT;
//            else
//                robot.outtake.turretPosition = Outtake.TurretPosition.AUTO_LEFT_SCORE;
//        }
//        if (stickyGamepad2.a) {
//            robot.outtake.clawMode = Outtake.ClawMode.OPEN;
//        }
//
//        if (Math.abs(gamepad2.right_stick_x) > 0.1) {
//            if (robot.outtake.turretPosition != Outtake.TurretPosition.MANUAL) {
//                robot.outtake.manualOffset = robot.outtake.getTurretPosition();
//            }
//            if (robot.outtake.manualOffset - gamepad2.right_stick_x * 0.01 > 1)
//                robot.outtake.manualOffset = 1;
//            else if (robot.outtake.manualOffset - gamepad2.right_stick_x * 0.01 < -1)
//                robot.outtake.manualOffset = -1;
//            else
//                robot.outtake.manualOffset -= gamepad2.right_stick_x * 0.01;
//            robot.outtake.turretPosition = Outtake.TurretPosition.MANUAL;
//        }
//
//        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
//            if (robot.outtake.armPosition != Outtake.ArmPosition.MANUAL) {
//                robot.outtake.armManualOffset = robot.outtake.getArmPosition();
//            }
//            // TODO: robot.outtake.armManualOffset = Range.clip(robot.outtake.getTargetArmServoPosition(Outtake.OUTTAKE_AUTO_PRELOAD_POS), 0, 1);
//            if (robot.outtake.armManualOffset + gamepad2.left_stick_y * 0.005 > 1)
//                robot.outtake.armManualOffset = 1;
//            else if (robot.outtake.manualOffset + gamepad2.left_stick_y * 0.005 < -1)
//                robot.outtake.armManualOffset = -1;
//            else
//                robot.outtake.armManualOffset += gamepad2.left_stick_y * 0.005;
//            robot.outtake.armPosition = Outtake.ArmPosition.MANUAL;
//        }
//        //endregion
//
//        //region Update Timers
//        updateConeDropTimer();
//        updateOuttakeGrabTimer();
//        updateTurretCenterTimer();
//        finishTransfer();
//        //endregion
//
//        //region Telemetry
//
//        // INTAKE TELEM
//        telemetry.addData("elevator isScoring", robot.elevator.isScoring);
//
//        telemetry.addData("elevator target position", robot.elevator.targetPosition.encoderPosition);
//        telemetry.addData("current position", robot.elevator.getRawEncoder());
//
//        telemetry.addData("aligner active", robot.outtake.alignerActive);
//        telemetry.addData("isScoring", robot.outtake.isScoring);
//
//        telemetry.addData("armPosition:", robot.intake.armPosition);
//        telemetry.addData("armRotate:", robot.intake.armRotate);
//        telemetry.addData("clawMode:", robot.intake.clawMode);
//
//        telemetry.addData("Aligner Mode", robot.outtake.alignerMode);
//        telemetry.addData("Elevator Encoder", robot.elevator.getEncoder());
//        telemetry.addData("Extendo Length", robot.extendo.getCurrentLength());
//        telemetry.addData("Extendo Mode", robot.extendo.extendoMode);
//        telemetry.addData("Elevator Left power", robot.elevator.leftPowah);
//        telemetry.addData("Elevator Right power", robot.elevator.rightPowah);
//
//        telemetry.addData("extendo power", robot.extendo.powah);
//
//        telemetry.addData("extendo encoders", robot.extendo.getEncoder());
//
//        telemetry.addData("loop time", (loopTime.seconds() - timestamp) * 1000.0);
//
//        addStatistics();
//        telemetry.update();
//        //endregion
//
////        TelemetryPacket packet = new TelemetryPacket();
////        Canvas fieldOverlay = packet.fieldOverlay();
//
//        /*
//        Turret drawing thing - but OUTTAKE_AUTO_PRELOAD_POS are dumb idfk i didnt want to delete
//
//        Pose2d robotPose = robot.drive.getPoseEstimate();
//        Pose2d turretWorldPose = new Pose2d(robotPose.vec().plus(TURRET_ROBOT_POSE.vec().rotated(robotPose.getHeading())), robotPose.getHeading() + TURRET_ROBOT_POSE.getHeading());
//        double turretAngle = robot.outtake.getTargetTurretAngle(OUTTAKE_AUTO_PRELOAD_POS);
//        Vector2d turretVector = new Vector2d(10, 0).rotated(turretAngle).rotated(turretWorldPose.getHeading());
//
//
//        fieldOverlay.setStroke("#3F51B5");
//        DashboardUtil.drawRobot(fieldOverlay, robotPose);
//        fieldOverlay.setStroke("#4CAF50");
//        fieldOverlay.fillCircle(Outtake.OUTTAKE_AUTO_PRELOAD_POS.getX(), Outtake.OUTTAKE_AUTO_PRELOAD_POS.getY(), 3);
//        fieldOverlay.setStroke("#FF9800");
//        fieldOverlay.strokeLine(turretWorldPose.getX(), turretWorldPose.getY(), turretWorldPose.getX() + turretVector.getX(), turretWorldPose.getY() + turretVector.getY());
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
//         */
//
//
//    }
//
//    private void finishTransfer() {
//        if (intakeTransferStarted && robot.extendo.extendoMode == Extendo.ExtendoMode.BRAKE) {
//            robot.sleep(0.1);
//            robot.intake.clawMode = Intake.ClawMode.OPEN;
//            robot.sleep(0.4);
//            robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
//            robot.sleep(0.15);
//            robot.outtake.armPosition = Outtake.ArmPosition.UP;
//            intakeTransferStarted = false;
//        }
//    }
//
//    private void updateTurretCenterTimer() {
//        if (0.1 < turretCenterTimer.seconds() && turretCenterTimer.seconds() < 0.2) {
//            robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
//            robot.elevator.targetPosition = Elevator.TargetHeight.GROUND;
//            robot.elevator.elevatorMode = Elevator.ElevatorMode.AUTOMATIC;
//        }
//    }
//
//    private void updateOuttakeGrabTimer() {
//        if (0.05 < outtakeGrabTimer.seconds() && outtakeGrabTimer.seconds() < 0.15) {
//            robot.outtake.turretMode = Outtake.TurretMode.SCORE;
//        }
//        if (0.15 < outtakeGrabTimer.seconds() && outtakeGrabTimer.seconds() < 0.3) {
//            if (staccMode) {
//                robot.outtake.armPosition = Outtake.ArmPosition.SCORE;
//            }
//            else {
//                robot.outtake.armPosition = Outtake.ArmPosition.SCORE_TILTED;
//            }
//        }
//    }
//
//    private void updateConeDropTimer() {
//        if (!startedTransfer && robot.intake.armPosition == Intake.ArmPosition.TRANSFER && robot.extendo.getCurrentLength() < 0.25) {
////            robot.intake.clawMode = Intake.ClawMode.OPEN;
//            coneDropTimer.reset();
//            startedTransfer = true;
//        }
//        if (0.3 < coneDropTimer.seconds() && coneDropTimer.seconds() < 0.45) {
//            robot.intake.clawMode = Intake.ClawMode.OPEN;
//        }
////        if (0.5 < coneDropTimer.seconds() && coneDropTimer.seconds() < 0.65) {
////            robot.outtake.armPosition= Outtake.ArmPosition.AUTO_INIT;
////        }
//        if (0.9 < coneDropTimer.seconds() && coneDropTimer.seconds() < 1.05) {
////            robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
//        }
//    }
//
//    private static String formatResults(MovingStatistics statistics) {
//        return Misc.formatInvariant("μ = %.2fms, σ = %.2fms, err = %.3fms",
//                statistics.getMean() * 1000,
//                statistics.getStandardDeviation() * 1000,
//                statistics.getStandardDeviation() / Math.sqrt(statistics.getCount()) * 1000);
//    }
//
//    private void addStatistics() {
//        telemetry.addData("Top 250", formatResults(robot.top250));
//        telemetry.addData("Top 100", formatResults(robot.top100));
//        telemetry.addData("Top 10", formatResults(robot.top10));
//    }
//
//    @Override
//    public void stop() {
//        robot.stop();
//    }
//}