package eu.qrobotics.powerplay.teamcode.opmode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Elevator Tester", group = "Debug")
@Disabled
public class ElevatorMode extends OpMode {

    public DcMotorEx motorLeft, motorRight;

    @Override
    public void init() {
        motorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");

        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.0) {
            motorLeft.setPower(gamepad1.right_trigger);
            motorRight.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.0) {
            motorLeft.setPower(-gamepad1.left_trigger);
            motorRight.setPower(-gamepad1.left_trigger);
        }

        telemetry.addData("Left pos", motorLeft.getCurrentPosition());
        telemetry.addData("Right pos", motorRight.getCurrentPosition());
        telemetry.addData("Left powah", motorLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right powah", motorRight.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}
