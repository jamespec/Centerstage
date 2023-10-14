package org.firstinspires.ftc.teamcode.x.previous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;

@Autonomous(name="DirectionAuto")
@Disabled
public class DirectionAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elevator = null;
    private Servo intake = null;
    private CRServo rise = null;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    //private VuforiaLocalizer vuforia;

    /**
     * is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "DO NOT RUN");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake     = hardwareMap.get(Servo.class, "intake");
        elevator   = hardwareMap.get(DcMotor.class, "elevator");
        rise     = hardwareMap.get(CRServo.class, "rise");
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start Autonomous");
        telemetry.update();
        waitForStart();
        runtime.reset();
        
  
        
        
        // Setup a variable for each drive wheel to save power level for telemetry
        
        //double leftPower = 0.2;
        //double rightPower = 0.2;
        //double risePower = 0.0;
        
        //whole elevator deploys
        //rise.setPower(-0.5);
        
        //grab starting cone
        //intake.setPosition(0.2);
        
        //sleep(3500);
        
        //move backwards cone
        move( leftDrive, rightDrive, -2500, 2500, 0.1 );
        
        
   
    }
    
    private void move( DcMotor leftDrive, DcMotor rightDrive, int leftTarget, int rightTarget, double power )
    {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setTargetPosition(leftTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while(leftDrive.isBusy() || rightDrive.isBusy()) {
            int rightPos = rightDrive.getCurrentPosition();
            int leftPos  = leftDrive.getCurrentPosition();
            telemetry.addData("LeftPos", "%d", leftPos);
            telemetry.addData("RightPos", "%d", rightPos);
            telemetry.update();
        }
        
        //leftDrive.setPower(0);
        //rightDrive.setPower(0);
    }
}

