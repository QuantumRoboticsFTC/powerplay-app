package eu.qrobotics.powerplay.teamcode.opmode.teleop;

import static eu.qrobotics.powerplay.teamcode.subsystems.Outtake.TURRET_ROBOT_POSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import eu.qrobotics.powerplay.teamcode.opmode.auto.trajectories.TrajectoriesLeft;
import eu.qrobotics.powerplay.teamcode.subsystems.Elevator;
import eu.qrobotics.powerplay.teamcode.subsystems.Extendo;
import eu.qrobotics.powerplay.teamcode.subsystems.Intake;
import eu.qrobotics.powerplay.teamcode.subsystems.Outtake;
import eu.qrobotics.powerplay.teamcode.subsystems.Robot;
import eu.qrobotics.powerplay.teamcode.util.DashboardUtil;
import eu.qrobotics.powerplay.teamcode.util.StickyGamepad;

@TeleOp
public class TeleOP extends OpMode {
    enum DriveMode {
        NORMAL,
        SLOW,
        SUPER_SLOW
    }

    public static final Vector2d OUTTAKE_HIGH_POS = new Vector2d(0, -24);

    public static double staccOffset = 0;
    public static double turretTarget = 0;

    public static double ELEVATOR_ARM_THRESHOLD = 15;

    private Elevator.ElevatorMode prevElevatorMode;
    private ElapsedTime turretCenterTimer = new ElapsedTime(0);
    private ElapsedTime outtakeGrabTimer = new ElapsedTime(0);
    private ElapsedTime coneDropTimer = new ElapsedTime(0);
    private ElapsedTime outtakeSensorTimer = new ElapsedTime(0);
    private ElapsedTime outtakeConeDropTimer = new ElapsedTime(0);

    private boolean careAboutIntakeSensor = true;
    private boolean startedTransfer = false;

    private boolean staccMode = false;

    Robot robot;
    DriveMode driveMode;
    StickyGamepad stickyGamepad1 = null;
    StickyGamepad stickyGamepad2 = null;
    MultipleTelemetry telemetry;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(this, false);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        driveMode = DriveMode.NORMAL;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.log().add("Ready!");
        robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
        robot.intake.clawMode = Intake.ClawMode.OPEN;
        robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
    }

    @Override
    public void start() {
        robot.start();
        robot.drive.setPoseEstimate(TrajectoriesLeft.START_POSE);
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        //region Driver 1 controls
        if (!robot.drive.isBusy()) {
            switch (driveMode) {
                case NORMAL:
                    robot.drive.setMotorPowersFromGamepad(gamepad1, 1);
                    break;
                case SLOW:
                    robot.drive.setMotorPowersFromGamepad(gamepad1, 0.7);
                    break;
                case SUPER_SLOW:
                    robot.drive.setMotorPowersFromGamepad(gamepad1, 0.5);
                    break;
            }

        }
        if (stickyGamepad1.left_bumper) {
            driveMode = DriveMode.SUPER_SLOW;
        }
        if (stickyGamepad1.right_bumper) {
            driveMode = DriveMode.NORMAL;
        }

        if (gamepad1.right_trigger > 0.1) {
            robot.extendo.extendoMode = Extendo.ExtendoMode.MANUAL;
            robot.extendo.manualPower = gamepad1.right_trigger * gamepad1.right_trigger;
            if (robot.intake.armPosition != Intake.ArmPosition.CONE_1 &&
                    robot.intake.armPosition != Intake.ArmPosition.CONE_2 &&
                    robot.intake.armPosition != Intake.ArmPosition.CONE_3 &&
                    robot.intake.armPosition != Intake.ArmPosition.CONE_4 &&
                    robot.intake.armPosition != Intake.ArmPosition.CONE_5 &&
                    robot.intake.armPosition != Intake.ArmPosition.LOW_POLE
            )
                robot.intake.armPosition = Intake.ArmPosition.CONE_1;
            if (robot.intake.armPosition != Intake.ArmPosition.LOW_POLE) {
                robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
                robot.intake.clawMode = Intake.ClawMode.OPEN;
                careAboutIntakeSensor = true;
                startedTransfer = false;
            }
            if (robot.outtake.getArmPosition() == Outtake.ARM_AUTO_INIT_POSITION && robot.elevator.elevatorMode != Elevator.ElevatorMode.UP) {
                robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
            }
        } else if (gamepad1.left_trigger > 0.1) {
            robot.extendo.extendoMode = Extendo.ExtendoMode.MANUAL;
            robot.extendo.manualPower = -gamepad1.left_trigger * 0.75;
        }
        if (careAboutIntakeSensor && robot.intake.armRotate != Intake.ArmRotate.TRANSFER && robot.intake.clawMode == Intake.ClawMode.OPEN) {
            if (robot.intake.hasCone())
                robot.intake.clawMode = Intake.ClawMode.CLOSED;
        }
//        if(stickyGamepad1.y) {
//            if(robot.extendo.manualSpeedMode == Extendo.ManualSpeedMode.FAST)
//                robot.extendo.manualSpeedMode = Extendo.ManualSpeedMode.SLOW;
//            else
//                robot.extendo.manualSpeedMode = Extendo.ManualSpeedMode.FAST;
//        }

        if (gamepad1.right_trigger <= 0.1 && gamepad1.left_trigger <= 0.1) {
            robot.extendo.manualPower = 0;
        }
        if (stickyGamepad1.dpad_down) {
            robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
            if (!staccMode || (robot.intake.armPosition == Intake.ArmPosition.VERTICAL && robot.intake.armRotate == Intake.ArmRotate.STRAIGHT)) {
                robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
                robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
            } else {
                robot.intake.armRotate = Intake.ArmRotate.STRAIGHT;
                robot.intake.armPosition = Intake.ArmPosition.VERTICAL;
            }
        }
        if (stickyGamepad1.dpad_left) {
            robot.intake.armPosition = robot.intake.armPosition.previous();
            if(robot.intake.armPosition == Intake.ArmPosition.LOW_POLE) {
                if (robot.intake.armRotate == Intake.ArmRotate.LOW_POLE) {
                    robot.intake.armRotate = Intake.ArmRotate.LOW_POLE_DROP;
                } else {
                    robot.intake.armRotate = Intake.ArmRotate.LOW_POLE;
                }
            } else
                robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
        }
        if (stickyGamepad1.dpad_right) {
            robot.intake.armPosition = robot.intake.armPosition.next();
            if(robot.intake.armPosition == Intake.ArmPosition.LOW_POLE) {
                if (robot.intake.armRotate == Intake.ArmRotate.LOW_POLE) {
                    robot.intake.armRotate = Intake.ArmRotate.LOW_POLE_DROP;
                } else {
                    robot.intake.armRotate = Intake.ArmRotate.LOW_POLE;
                }
            } else
                robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
        }
        if (stickyGamepad1.dpad_up) {
            robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
        }
        if (stickyGamepad1.a) {
            robot.intake.clawMode = Intake.ClawMode.CLOSED;
        }
        if (stickyGamepad1.b) {
            if (robot.intake.armPosition == Intake.ArmPosition.LOW_POLE) {

            }
            robot.intake.clawMode = Intake.ClawMode.OPEN;
            careAboutIntakeSensor = false;
        }

        if (stickyGamepad1.back) {
            if (robot.intake.armPosition == Intake.ArmPosition.FULL_0)
                robot.intake.armPosition = Intake.ArmPosition.FULL_1;
            else
                robot.intake.armPosition = Intake.ArmPosition.FULL_0;
        }
        //endregion
        // region Driver 2 controls

        if (stickyGamepad2.back) {
            staccMode = !staccMode;
        }

//        if (robot.elevator.elevatorMode == Elevator.ElevatorMode.DOWN) {
        if (stickyGamepad2.dpad_up) {
            if (staccMode) {
                robot.elevator.targetPosition = Elevator.TargetHeight.HIGH;
            } else {
                robot.elevator.targetPosition = Elevator.TargetHeight.HIGH_TILTED;
            }
        } else if (stickyGamepad2.dpad_left) {
            if (staccMode) {
                robot.elevator.targetPosition = Elevator.TargetHeight.MID;
            } else {
                robot.elevator.targetPosition = Elevator.TargetHeight.MID_TILTED;
            }
        } else if (stickyGamepad2.dpad_down) {
            robot.elevator.targetPosition = Elevator.TargetHeight.LOW;
        }
//        }
        if (stickyGamepad2.dpad_right) {
            if (robot.outtake.alignerMode == Outtake.AlignerMode.DEPLOYED)
                robot.outtake.alignerMode = Outtake.AlignerMode.RETRACTED;
            else if (robot.outtake.alignerMode == Outtake.AlignerMode.RETRACTED)
                robot.outtake.alignerMode = Outtake.AlignerMode.DEPLOYED;
        }
        if (stickyGamepad2.right_bumper) {
            robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
            robot.intake.clawMode = Intake.ClawMode.OPEN;
            robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
            startedTransfer = false;
            outtakeGrabTimer.reset();
        } else if (stickyGamepad2.left_bumper) {
            robot.outtake.alignerMode = Outtake.AlignerMode.RETRACTED;
            robot.outtake.armPosition = Outtake.ArmPosition.UP;
            robot.outtake.clawMode = Outtake.ClawMode.OPEN;
            robot.outtake.turretMode = Outtake.TurretMode.TRANSFER;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
            turretCenterTimer.reset();
        }
        if(robot.elevator.targetPosition == Elevator.TargetHeight.LOW && robot.elevator.elevatorMode == Elevator.ElevatorMode.UP &&  robot.outtake.armPosition != Outtake.ArmPosition.MANUAL) {
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
                if (prevElevatorMode == Elevator.ElevatorMode.DOWN) {
//                    robot.elevator.reset();
                } else {
                    robot.elevator.offsetPosition = robot.elevator.getEncoder() - robot.elevator.targetPosition.getEncoderPosition();
                }
                robot.elevator.elevatorMode = prevElevatorMode;
                prevElevatorMode = null;
            }
        }

        // in staccMode (stackMode), daca apesi butonu pune automat autoaim pe high-ul de aproape -> very cool big shusta
        if (staccMode && (stickyGamepad2.left_stick_button || stickyGamepad2.right_stick_button)) {
            turretTarget = Range.clip(robot.outtake.getTargetTurretServoPosition(OUTTAKE_HIGH_POS), 0, 1);
            robot.outtake.manualOffset = staccOffset + turretTarget;
            robot.outtake.turretPosition = Outtake.TurretPosition.MANUAL;
        }

        if (stickyGamepad2.x) {
            robot.outtake.turretPosition = Outtake.TurretPosition.RIGHT;
        } else if (stickyGamepad2.y) {
            robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        } else if (stickyGamepad2.b) {
            robot.outtake.turretPosition = Outtake.TurretPosition.LEFT;
        }
        if (stickyGamepad2.a) {
            robot.outtake.clawMode = Outtake.ClawMode.OPEN;
            outtakeConeDropTimer.reset();
        }

        if (Math.abs(gamepad2.right_stick_x) > 0.1) {
            if (staccMode) {
                if (robot.outtake.manualOffset - gamepad2.right_stick_x * 0.005 > 1)
                    staccOffset = 1 - turretTarget;
                else if (robot.outtake.manualOffset - gamepad2.right_stick_x * 0.005 < -1)
                    staccOffset = turretTarget - 1;
                else
                    staccOffset -= gamepad2.right_stick_x * 0.005;
                robot.outtake.turretPosition = Outtake.TurretPosition.MANUAL;
                robot.outtake.manualOffset = turretTarget + staccOffset;
            } else {
                if (robot.outtake.turretPosition != Outtake.TurretPosition.MANUAL) {
                    robot.outtake.manualOffset = robot.outtake.getTurretPosition();
                }
                if (robot.outtake.manualOffset - gamepad2.right_stick_x * 0.005 > 1)
                    robot.outtake.manualOffset = 1;
                else if (robot.outtake.manualOffset - gamepad2.right_stick_x * 0.005 < -1)
                    robot.outtake.manualOffset = -1;
                else
                    robot.outtake.manualOffset -= gamepad2.right_stick_x * 0.005;
                robot.outtake.turretPosition = Outtake.TurretPosition.MANUAL;
            }
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
        updateConeDropTimer();
        updateOuttakeSensorTimer();
        updateOuttakeGrabTimer();
        updateTurretCenterTimer();
        updateOuttakeConeDropTimer();
        //endregion

        //region Telemetry
        telemetry.addData("Elevator Encoder", robot.elevator.getEncoder());
        telemetry.addData("Elevator Target Encoder", robot.elevator.getTargetEncoder());
        telemetry.addData("Extendo Length", robot.extendo.getCurrentLength());
        telemetry.addData("Left power", robot.elevator.leftPowah);
        telemetry.addData("Right power", robot.elevator.rightPowah);
        addStatistics();
        telemetry.update();
        //endregion

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

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

    private void updateOuttakeConeDropTimer() {
        if (0.1 < outtakeConeDropTimer.seconds() && outtakeConeDropTimer.seconds() < 0.25) {
            robot.outtake.alignerMode = Outtake.AlignerMode.RETRACTED;
        }
    }

    private void updateTurretCenterTimer() {
        if (0.6 < turretCenterTimer.seconds() && turretCenterTimer.seconds() < 0.75) {
            robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
        }
    }

    private void updateOuttakeGrabTimer() {
        if (0.15 < outtakeGrabTimer.seconds() && outtakeGrabTimer.seconds() < 0.3) {
            robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
            robot.outtake.armPosition = Outtake.ArmPosition.UP;
            robot.intake.armPosition = Intake.ArmPosition.VERTICAL;
            robot.intake.armRotate = Intake.ArmRotate.STRAIGHT;
        }
        if (0.45 < outtakeGrabTimer.seconds() && outtakeGrabTimer.seconds() < 0.6) {
            robot.outtake.turretMode = Outtake.TurretMode.SCORE;
        }
        if (0.6 < outtakeGrabTimer.seconds() && outtakeGrabTimer.seconds() < 0.7) {
            if (staccMode) {
                robot.outtake.armPosition = Outtake.ArmPosition.SCORE;
            }
            else {
                robot.outtake.armPosition = Outtake.ArmPosition.SCORE_TILTED;
            }
        }
    }

    private void updateOuttakeSensorTimer() {
        if (startedTransfer)
            if (robot.outtake.hasCone())
                outtakeSensorTimer.reset();
//            robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
//        if (0.15 < outtakeSensorTimer.seconds() && outtakeSensorTimer.seconds() < 0.2) {
//            robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
//        }
    }

    private void updateConeDropTimer() {
        if (!startedTransfer && robot.intake.armPosition == Intake.ArmPosition.TRANSFER && robot.extendo.getCurrentLength() < 0.25) {
//            robot.intake.clawMode = Intake.ClawMode.OPEN;
            coneDropTimer.reset();
            startedTransfer = true;
        }
        if (0.3 < coneDropTimer.seconds() && coneDropTimer.seconds() < 0.45) {
            robot.intake.clawMode = Intake.ClawMode.OPEN;
        }
//        if (0.5 < coneDropTimer.seconds() && coneDropTimer.seconds() < 0.65) {
//            robot.outtake.armPosition= Outtake.ArmPosition.AUTO_INIT;
//        }
        if (0.9 < coneDropTimer.seconds() && coneDropTimer.seconds() < 1.05) {
            robot.outtake.clawMode = Outtake.ClawMode.CLOSED;
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