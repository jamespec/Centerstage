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
public class Deadzone extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elevator = null;
    private Servo intake = null;
    private CRServo rise = null;
    double power = 0;
        boolean aHit = false;
        boolean yHit = false;
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
        
        
        if( gamepad1.a && !aHit )
            power -= 0.01;
        
            
        if( gamepad1.y && !yHit )
            power += 0.01;
            
        aHit = gamepad1.a;
        yHit = gamepad1.y;
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        telemetry.addData("Power", "%f", power);
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
