package eu.qrobotics.powerplay.teamcode.opmode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Outtake Sensor Test", group = "Debug")
public class OuttakeSensorTest extends OpMode {

    public ColorRangeSensor outtakeSensor;

    @Override
    public void init() {
        outtakeSensor = hardwareMap.get(ColorRangeSensor.class, "outtakeSensor");

        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {
        telemetry.addData("Red", outtakeSensor.red());
        telemetry.addData("Green", outtakeSensor.green());
        telemetry.addData("Blue", outtakeSensor.blue());
        telemetry.addData("Distance", outtakeSensor.getDistance(DistanceUnit.MM));
        telemetry.update();
    }
}
