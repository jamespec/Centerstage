package org.firstinspires.ftc.teamcode.x.previous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name="JimAuto")
@Disabled
public class JimAuto extends LinearOpMode 
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private DcMotor wheel = null;
    private CRServo elevator = null;
    private CRServo intake   = null;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
//    "Ball",  // Don't look for ball
      "Cube",
      "Duck"
      // , "Marker" // Don't look for Market, whatever that is.
    };

    private static final String VUFORIA_KEY =
            "AROHLt//////AAABmRqtKHXnCETrrKry+MJXVcdqPPGOE6f4kvj9Kh5mhFwXKquVVjlrr+9T0G3ckUn7PEtQocoVIAwVfdY7y1cqkibeTI3IxRhf1taeZO+ovnE8T++Udbtqy8Y9sN9IwHbXus5AXTLp1s1jEZGB5thPT7rLACUDOouus47BeF8Ygyj5ygGSDIGEh0hWdwtF3G4zI1HUjPNtVakFVYQMqIaaUmv0FtTRUJP+2aBeEtETU5dmuq6eLZ76sHWarETv+lzUe1rOZM7fTNKfRGT/M6TZZXmnbOB2w45TM6nS5Z15vUjzuvX+L+Nxn+BeaAjmPpWk87gecXYnTyQ5bz+oOJtld5EkIMgu3DSyFEg46O374Ytr";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    //private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
     
    private TFObjectDetector tfod;
    @Override
    public void runOpMode() 
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        wheel      = hardwareMap.get(DcMotor.class, "wheel");
        elevator   = hardwareMap.get(CRServo.class, "elevator");
        intake     = hardwareMap.get(CRServo.class, "intake");
        int level = 1;
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            //tfod.setZoom(1.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();
        
        
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
              telemetry.addData("# Object Detected", updatedRecognitions.size());
              // step through the list of recognitions and display boundary info.
              int i = 0;
              for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                i++;
                if (recognition.getLeft() > 280) {
                    level = 3;
                }
                else {
                    level = 2;
                }
              }
            }
        }
       
        telemetry.addData("level: ", "(%d)", level);
        telemetry.update();

        double wheelPower = .80;
        
        //first step - Back up to Carousel
        move( leftDrive, rightDrive, -900, -900, 0.2 );

        // If level 3, top, raise elevator - slowly
        if (level == 3)
            elevator.setPower(-0.125);
        
        // Turn carousel    
        wheel.setPower(wheelPower);
        
        sleep(3000); //3 second wait
        wheel.setPower(0);
        // Duck deliveryed

        // Move back along wall to even with Hub        
        move( leftDrive, rightDrive, 2175, 2175, 0.2 );

        // TURN 90 degrees right
        move( leftDrive, rightDrive, 360, -360, 0.2 );
        
        // Back into wall to ensure 90 degrees
        move( leftDrive, rightDrive, -150, -150, 0.2 );

        // Set elevator for level 1 or 2, level three would already be in place    
        if (level == 1) 
        {
            telemetry.addData("Move elevator to level:", " %d", level);
            telemetry.update();
            elevator.setPower(-0.3);
            sleep(1500);
            // need a little power to hold in place
            elevator.setPower(-0.06);

            // Move to Hub
            move( leftDrive, rightDrive, 1170, 1170, 0.2 );
        }
        if (level == 2) 
        {
            telemetry.addData("Move elevator to level:", " %d", level);
            telemetry.update();
            elevator.setPower(-0.3);
            sleep(2000);
            elevator.setPower(-0.06);

            // Move to Hub
            move( leftDrive, rightDrive, 1150, 1150, 0.2 );
        }
        if (level == 3) {
            telemetry.addData("Move elevator to level:", " %d", level);
            telemetry.update();
            move( leftDrive, rightDrive, 1120, 1120, 0.2 );
        }

        intake.setPower(0.15);
        sleep(1000);
        intake.setPower(0.0);

        // Backup            
        //move( leftDrive, rightDrive, -275, -275, 0.2 );
        move( leftDrive, rightDrive, -1000, -1000, 0.2 );

        // Turn left - turning too much, not sure why
        move( leftDrive, rightDrive, -375, 375, 0.2 );

        // Drive into warehouse
        move( leftDrive, rightDrive, 3000, 3000, 0.5 );

        elevator.setPower(0.0);
    }

    private void move( DcMotorEx leftDrive, DcMotorEx rightDrive, int leftTarget, int rightTarget, double power )
    {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setTargetPosition(leftTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Velocity is in Ticks per second.
        // We receive power as a percent of full speed.
        double rpm = 2500 * power;
        leftDrive.setVelocity( rpm );
        rightDrive.setVelocity( rpm );
        // leftDrive.setPower(power);
        // rightDrive.setPower(power);

        while(leftDrive.isBusy() || rightDrive.isBusy()) {
            int rightPos = rightDrive.getCurrentPosition();
            int leftPos  = leftDrive.getCurrentPosition();
            telemetry.addData("LeftPos", "%d", leftPos);
            telemetry.addData("RightPos", "%d", rightPos);
            telemetry.update();
        }
        
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    

    // The next two routines are straight from the Concept samples supplied.
    
    private void initVuforia() 
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() 
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.7f;
//        tfodParameters.isModelTensorFlow2 = true;
//        tfodParameters.inputSize = 320;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    
}

