package org.firstinspires.ftc.teamcode.x.previous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp

public class ErrorFix extends LinearOpMode 
{
    @Override
    public void runOpMode() 
    {
        TwoMotorChassis twoMotorChassis = new TwoMotorChassis( hardwareMap, telemetry );

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) 
        {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            double angle = 0.0;
            if (Math.abs(y) >= Math.abs(x)) {
                if (y <= 0)
                    angle = 0.0;
                else
                    angle = 180;
            }
            else {
                if(x > 0) 
                    angle = 90;
                else
                    angle = -90;
            }
            
            if( gamepad1.a ) {
                twoMotorChassis.turn(angle, 1.0, 1.0);
                sleep(2000);
            }

            telemetry.addData("Angle", "%f", angle);
            telemetry.update();
        }
    }
}
