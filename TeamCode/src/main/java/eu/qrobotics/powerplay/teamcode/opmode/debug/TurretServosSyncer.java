package eu.qrobotics.powerplay.teamcode.opmode.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import eu.qrobotics.powerplay.teamcode.util.StickyGamepad;

@TeleOp(name = "turet arm syncer", group = "Debug")
public class TurretServosSyncer extends OpMode {

    Servo leftServo = null;
    Servo rightServo = null;

    private StickyGamepad stickyGamepad = null;
    private double servoPos = 0.5;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        leftServo = hardwareMap.get(Servo.class, "outtakeArmServoLeft");
        rightServo = hardwareMap.get(Servo.class, "outtakeArmServoRight");

        rightServo.setDirection(Servo.Direction.REVERSE);

        stickyGamepad = new StickyGamepad(gamepad1);
    }

    @Override
    public void loop() {
        stickyGamepad.update();
        leftServo.setPosition(servoPos);
        rightServo.setPosition(servoPos);

        servoPos += gamepad1.left_stick_y * 0.001;
        telemetry.addData("serpos", servoPos);
        telemetry.addData("gamepad1 delta", gamepad1.left_stick_y * 0.1);
        telemetry.update();
    }
}
