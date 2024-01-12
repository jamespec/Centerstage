package org.firstinspires.ftc.teamcode.x.previous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class ArmTest extends LinearOpMode {
    
    private DcMotorEx arm = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");

        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Press start...");
        waitForStart();
        while(opModeIsActive()) {
            double armSpeed = gamepad2.right_stick_y;

            telemetry.addData("Arm Spread", "%f", armSpeed);
            telemetry.update();

            arm.setPower( armSpeed );
        }
    }
}

        
    


