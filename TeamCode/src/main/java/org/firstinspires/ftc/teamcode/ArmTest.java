package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ArmTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");

        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Press start...");
        waitForStart();
        while(opModeIsActive()) {
            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
            double power = -gamepad1.left_stick_y / 5;

            telemetry.addData("Power: ", power);
            telemetry.update();
            arm.setPower( power );
        }
    }

}
