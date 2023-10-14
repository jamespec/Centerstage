package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class Jim extends OpMode
{
    // A change to the file.

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor wheel = null;
    private CRServo elevator = null;
    private CRServo intake = null;
    //private DistanceSensor distance = null;
    private ColorSensor color = null;
    private TouchSensor touch = null;
    private DigitalChannel red = null;
    private DigitalChannel green = null;
    private DistanceSensor leftDistanceSensor = null;
    private DistanceSensor rightDistanceSensor = null;
    

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
        wheel      = hardwareMap.get(DcMotor.class, "wheel");
        elevator   = hardwareMap.get(CRServo.class, "elevator");
        intake     = hardwareMap.get(CRServo.class, "intake");
        // distance   = hardwareMap.get(DistanceSensor.class, "distance");
        // color      = hardwareMap.get(ColorSensor.class, "color");
        // touch      = hardwareMap.get(TouchSensor.class, "touch");
        // red        = hardwareMap.get(DigitalChannel.class, "red");
        // green      = hardwareMap.get(DigitalChannel.class, "green");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right_distance");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // red.setMode(DigitalChannel.Mode.OUTPUT);
        // red.setState(true);
        
        // green.setMode(DigitalChannel.Mode.OUTPUT);
        // green.setState(true);
    

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftDrive.setDirection(DcMotor.Direction.FORWARD);
        // rightDrive.setDirection(DcMotor.Direction.REVERSE);

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
        int colorCount=0;
        double leftPower;
        double rightPower;
        double intakePower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        // double drive = -gamepad1.left_stick_y;
        // double turn  = -gamepad1.right_stick_x;
        // double speed =  gamepad1.right_stick_y;
    
        double drive =  Range.clip(gamepad1.left_stick_y, -0.8, 0.8);
        double turn  =  Range.clip(-gamepad1.left_stick_x/2, -0.8, 0.8);
        double elevatorSpeed =  gamepad2.right_stick_y;
        double wheelSpeed = gamepad2.left_stick_y;
        //double distanceMM = distance.getDistance(DistanceUnit.MM);
        boolean touched   = touch.isPressed();
        
        
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

        if( gamepad2.a )
            intakePower = 1.0;
        else if( gamepad2.b )
            intakePower = -1.0;
        else
            intakePower = 0.0;
            
        leftPower    = Range.clip(drive + turn, -1.0, 1.0);
        rightPower   = Range.clip(drive - turn, -1.0, 1.0);

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        
        wheel.setPower(wheelSpeed/1);
        elevator.setPower(elevatorSpeed);
        intake.setPower(intakePower);
        
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Wheel", "Wheel speed: (%.2f)", elevatorSpeed );
        //telemetry.addData("Distance", "In MM: (%.2f)", distanceMM );
        telemetry.addData("Magnetic Limit", "Is Near: %b " , touched );
        telemetry.addData("TouchSensor", "IsTouched: %b " , touched );
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
