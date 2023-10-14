package org.firstinspires.ftc.teamcode.x.previous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp
@Disabled
public class David extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elevator = null;
    private Servo intake = null;
    private CRServo rise = null;
    //private DistanceSensor distance = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake     = hardwareMap.get(Servo.class, "intake");
        elevator   = hardwareMap.get(DcMotor.class, "elevator");
        rise     = hardwareMap.get(CRServo.class, "rise");

        //leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double risePower = 0.0;
        //double intake;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        // double drive = -gamepad1.left_stick_y;
        // double turn  = -gamepad1.right_stick_x;
        // double speed =  gamepad1.right_stick_y;
    
        double drive =  Range.clip(-gamepad1.left_stick_x, -0.3, 0.3);
        
        double turn  =  Range.clip(-gamepad1.left_stick_y/2, -0.4, 0.4);
        
        double elevatorSpeed =  (gamepad2.left_stick_y);
        
        
        
        if( gamepad2.a )
            intake.setPosition(0.2);//grab
            
        if( gamepad2.b )
            intake.setPosition(0.6) ;//drop
            
        if( gamepad2.dpad_down )
            risePower = 0.5;
            
        if( gamepad2.dpad_up )
            risePower = -0.75;
            
        
        //double distanceMM = distance.getDistance(DistanceUnit.MM);
        
        // int r = color.red();
        // int g = color.green();
        // int b = color.blue();

        // if( touched ) {
        //     red.setState(false);
        //     green.setState(true);
        // }
        // else {
        //     red.setState(true);
        //     green.setState(false);
        // }

 
        leftPower    = Range.clip(drive + turn, -1.0, 1.0);
        rightPower   = Range.clip(drive - turn, -1.0, 1.0);

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        rise.setPower(risePower);
        
        //intake.setPower(intakePower);
        
        elevator.setPower(elevatorSpeed/2.5);
        
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        int rightPos = rightDrive.getCurrentPosition();
        int leftPos  = leftDrive.getCurrentPosition();
        telemetry.addData("LeftPos", "%d", leftPos);
        telemetry.addData("RightPos", "%d", rightPos);
        telemetry.update();
        //telemetry.addData("Wheel", "Wheel speed: (%.2f)", elevatorSpeed );
        //telemetry.addData("Distance", "In MM: (%.2f)", distanceMM );
        // telemetry.addData("Magnetic Limit", "Is Near: %b " , touched );
        // telemetry.addData("TouchSensor", "IsTouched: %b " , touched );
        // String rgb = r + ":" + g + ":" + b;
        // telemetry.addData("ColorSensor", "RBG: %s", rgb );
        
        
        
    }
    

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        
    }

}
