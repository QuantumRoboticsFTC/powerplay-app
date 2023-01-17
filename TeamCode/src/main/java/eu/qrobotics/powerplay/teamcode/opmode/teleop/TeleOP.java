package eu.qrobotics.powerplay.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public static double ELEVATOR_ARM_THRESHOLD = 15;

    private Elevator.ElevatorMode prevElevatorMode;
    private ElapsedTime turretCenterTimer = new ElapsedTime(0);
    private ElapsedTime grabTimer = new ElapsedTime(0);
    private ElapsedTime intakeGrabTimer = new ElapsedTime(0);

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
    }

    @Override
    public void start() {
        robot.start();
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
            robot.extendo.manualPower = gamepad1.right_trigger;
            robot.intake.armPosition = Intake.ArmPosition.CONE_1;
            robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
            robot.intake.clawMode = Intake.ClawMode.OPEN;
        } else if (gamepad1.left_trigger > 0.1) {
            robot.extendo.extendoMode = Extendo.ExtendoMode.MANUAL;
            robot.extendo.manualPower = -gamepad1.left_trigger * 0.75;
        }
//        if(stickyGamepad1.y) {
//            if(robot.extendo.manualSpeedMode == Extendo.ManualSpeedMode.FAST)
//                robot.extendo.manualSpeedMode = Extendo.ManualSpeedMode.SLOW;
//            else
//                robot.extendo.manualSpeedMode = Extendo.ManualSpeedMode.FAST;
//        }

        if(gamepad1.right_trigger <=0.1 && gamepad1.left_trigger <=0.1) {
            robot.extendo.manualPower = 0;
        }
        if (stickyGamepad1.dpad_down) {
            intakeGrabTimer.reset();
            robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
            robot.intake.armPosition = Intake.ArmPosition.TRANSFER;
        }
        if (stickyGamepad1.dpad_left) {
            robot.intake.armPosition = robot.intake.armPosition.previous();
        }
        if (stickyGamepad1.dpad_right) {
            robot.intake.armPosition = robot.intake.armPosition.next();
        }
        if (stickyGamepad1.dpad_up) {
            robot.intake.armRotate = Intake.ArmRotate.TRANSFER;
        }
        if (stickyGamepad1.a) {
            robot.intake.clawMode = Intake.ClawMode.CLOSE;
        }
        if (stickyGamepad1.b) {
            robot.intake.clawMode = Intake.ClawMode.OPEN;
        }
        //endregion

        // region Driver 2 controls
//        if (robot.elevator.elevatorMode == Elevator.ElevatorMode.DOWN) {
            if (stickyGamepad2.dpad_up) {
                robot.elevator.targetPosition = Elevator.TargetHeight.HIGH;
            } else if (stickyGamepad2.dpad_left) {
                robot.elevator.targetPosition = Elevator.TargetHeight.MID;
            } else if (stickyGamepad2.dpad_down) {
                robot.elevator.targetPosition = Elevator.TargetHeight.LOW;
            }
//        }

        if (stickyGamepad2.right_bumper) {
            robot.outtake.clawMode = Outtake.ClawMode.CLOSE;
            grabTimer.reset();
        } else if (stickyGamepad2.left_bumper) {
            robot.outtake.armPosition = Outtake.ArmPosition.UP;
            robot.outtake.clawMode = Outtake.ClawMode.OPEN;
            robot.outtake.turretMode = Outtake.TurretMode.TRANSFER;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
            turretCenterTimer.reset();
        }
        if (0.5 < grabTimer.seconds() && grabTimer.seconds() < 0.65) {
            robot.outtake.armPosition = Outtake.ArmPosition.UP;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
            robot.intake.clawMode = Intake.ClawMode.OPEN;
            robot.intake.armPosition = Intake.ArmPosition.CONE_1;
            robot.intake.armRotate = Intake.ArmRotate.PARALLEL;
        }
        if (0.9 < grabTimer.seconds() && grabTimer.seconds() < 1.05) {
            robot.outtake.turretMode = Outtake.TurretMode.SCORE;
        }
        if(robot.elevator.elevatorMode == Elevator.ElevatorMode.UP && robot.elevator.getDistanceLeft() < ELEVATOR_ARM_THRESHOLD && robot.outtake.armPosition != Outtake.ArmPosition.MANUAL) {
            robot.outtake.armPosition = Outtake.ArmPosition.SCORE;
        }
        if (0.6 < intakeGrabTimer.seconds() && intakeGrabTimer.seconds() < 0.75) {
            robot.extendo.extendoMode = Extendo.ExtendoMode.RETRACTED;
        }
        if (0.6 < turretCenterTimer.seconds() && turretCenterTimer.seconds() < 0.75) {
            robot.outtake.armPosition = Outtake.ArmPosition.TRANSFER;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
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

        if (stickyGamepad2.x && robot.outtake.armPosition != Outtake.ArmPosition.TRANSFER) {
            robot.outtake.turretMode = Outtake.TurretMode.SCORE;
            robot.outtake.turretPosition = Outtake.TurretPosition.RIGHT;
        } else if (stickyGamepad2.y && robot.outtake.armPosition != Outtake.ArmPosition.TRANSFER) {
            robot.outtake.turretMode = Outtake.TurretMode.SCORE;
            robot.outtake.turretPosition = Outtake.TurretPosition.CENTER;
        } else if (stickyGamepad2.b && robot.outtake.armPosition != Outtake.ArmPosition.TRANSFER) {
            robot.outtake.turretMode = Outtake.TurretMode.SCORE;
            robot.outtake.turretPosition = Outtake.TurretPosition.LEFT;
        }
        if (stickyGamepad2.a) {
            robot.outtake.clawMode = Outtake.ClawMode.OPEN;
        }
        if ( Math.abs(gamepad2.right_stick_x) > 0.1){
            if (robot.outtake.turretPosition != Outtake.TurretPosition.MANUAL) {
                robot.outtake.MANUAL_OFFSET = robot.outtake.getTurretPosition();
            }
            if (robot.outtake.MANUAL_OFFSET - gamepad2.right_stick_x * 0.005 > 1)
                robot.outtake.MANUAL_OFFSET = 1;
            else if (robot.outtake.MANUAL_OFFSET - gamepad2.right_stick_x * 0.005 < -1)
                robot.outtake.MANUAL_OFFSET = -1;
            else
                robot.outtake.MANUAL_OFFSET -= gamepad2.right_stick_x * 0.005;
            robot.outtake.turretPosition = Outtake.TurretPosition.MANUAL;
        }

        if (Math.abs(gamepad2.left_stick_y) > 0.1){
            if (robot.outtake.armPosition != Outtake.ArmPosition.MANUAL) {
                robot.outtake.ARM_MANUAL_OFFSET = robot.outtake.getArmPosition();
            }
            if (robot.outtake.ARM_MANUAL_OFFSET + gamepad2.left_stick_y * 0.005 > 1)
                robot.outtake.ARM_MANUAL_OFFSET = 1;
            else if (robot.outtake.MANUAL_OFFSET + gamepad2.left_stick_y * 0.005 < -1)
                robot.outtake.ARM_MANUAL_OFFSET = -1;
            else
                robot.outtake.ARM_MANUAL_OFFSET += gamepad2.left_stick_y * 0.005;
            robot.outtake.armPosition = Outtake.ArmPosition.MANUAL;
        }
        //endregion

        //region Telemetry

        telemetry.update();
        telemetry.addData("Elevator Encoder: ", robot.elevator.getEncoder());
        telemetry.addData("Elevator Target Encoder: ", robot.elevator.getTargetEncoder());
        telemetry.addData("Extendo Encoder: ", robot.extendo.getEncoder());
//        telemetry.addData("Extendo Target Encoder: ", robot.extendo.g());
        //endregion

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, robot.drive.getPoseEstimate());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);


    }

    @Override
    public void stop() {
        robot.stop();
    }
}