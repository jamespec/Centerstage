package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Make it differenc

@TeleOp
public class ArmTest extends OpMode {
    int power  = 0;
    private DcMotor arm = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(power);
    }

    @Override
    public void loop() {

    }

}
