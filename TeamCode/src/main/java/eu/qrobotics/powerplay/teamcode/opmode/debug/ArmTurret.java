package eu.qrobotics.powerplay.teamcode.opmode.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import eu.qrobotics.powerplay.teamcode.opmode.teleop.DebugTeleOP;
import eu.qrobotics.powerplay.teamcode.subsystems.Outtake;
import eu.qrobotics.powerplay.teamcode.subsystems.Robot;
import eu.qrobotics.powerplay.teamcode.util.StickyGamepad;

@TeleOp(name = "Arm Turret Servo", group = "Debug")
public class ArmTurret extends OpMode {

    Robot robot;
    StickyGamepad stickyGamepad1 = null;
    StickyGamepad stickyGamepad2 = null;
    MultipleTelemetry telemetry;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(this, false);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        robot.drive.setPoseEstimate(new Pose2d(-14.75, -15.75, Math.toRadians(177.25)));
        robot.outtake.armPosition = Outtake.ArmPosition.AUTO_VERTICAL;
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        if (stickyGamepad1.a) {
            robot.outtake.turretMode = Outtake.TurretMode.TRANSFER;
        }

        if (stickyGamepad1.b) {
            robot.outtake.followingPosition = new Vector2d(0, -24);
            robot.outtake.turretMode = Outtake.TurretMode.FOLLOWING;
        }

        telemetry.update();
    }

}