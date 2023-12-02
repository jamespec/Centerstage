package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@TeleOp
public class ArmTest extends LinearOpMode {
    
    private DcMotorEx arm = null;
    private Servo intake = null;
    
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        
        
        intake     = hardwareMap.get(Servo.class, "intake");
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        

        
    
        double powerArm = 0.0;

        int armPos;
        double intakePos;
        intakePos = 0;//0 start, -650 is backwards angle
        //armPos = arm.getCurrentPosition();
        
        armPos = 0;

        telemetry.addData("Status", "Press start...");
        waitForStart();
        while(opModeIsActive()) {
            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
            double power = gamepad1.left_stick_y / 7;
            armPos = arm.getCurrentPosition();

            intakePos = intake.getPosition();

            new Double (intakePos).toString();
            
            boolean setArmT = gamepad1.a;
            boolean setArmZ = gamepad1.b;
            boolean intakeDrop = gamepad1.x;
            boolean intakeGrab = gamepad1.y;
            
            //telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Motor Position", "%s", armPos);
            telemetry.addData("Intake Position", "%s", intakePos);
            telemetry.addData("Power", "%s", power);
            //telemetry.addData("Power: ", power);
            telemetry.addData("A: ", setArmT);
            telemetry.addData("B: ", setArmZ);
            //telemetry.addData("Intake", "%s", intakeValue);
            telemetry.update();
            arm.setPower( power );
            if (intakeGrab == true){
                intake.setPosition(0.25);
            }
            if (intakeDrop == true){
                intake.setPosition(0.0);
            }
            if (setArmT == true ){
                PIDCoefficients pidController = new PIDCoefficients(0.3,0.00001,-0.1);
                arm.setTargetPosition(-522);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.3);

                while(arm.isBusy()) {
                    sleep(1000);
                }
            }
            
            if (setArmZ == true ){//YOU MUST START THE ARM IN THE RIGHT POSITION(ON THE BACK)
                //setTimeout(arm.setPower(power), 700);

                PIDCoefficients pidController = new PIDCoefficients(0.02,0.0,0.0);
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.035);
                while(arm.isBusy()) {
                    sleep(1000);
                }
            }
        }
    }
}

        
    


