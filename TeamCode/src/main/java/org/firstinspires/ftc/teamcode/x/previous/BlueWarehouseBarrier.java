package org.firstinspires.ftc.teamcode.x.previous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name="BlueWarehouseBarrier")
@Disabled
public class BlueWarehouseBarrier extends LinearOpMode {
    
    TwoMotorChassis twoMotorChassis =null;

    private BNO055IMU imu;
    private Orientation angles; 
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor wheel = null;
    private CRServo elevator = null;
    private CRServo intake   = null;
    private DistanceSensor leftDistanceSensor = null;
    private DistanceSensor rightDistanceSensor = null;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck"
      // , "Marker"
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
    public void runOpMode() {
        telemetry.addData("Status", "DO NOT RUN");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        wheel      = hardwareMap.get(DcMotor.class, "wheel");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right_distance");
        elevator   = hardwareMap.get(CRServo.class, "elevator");
        intake     = hardwareMap.get(CRServo.class, "intake");
        int level = 1;
        
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Wait for the game to start (driver presses PLAY)
        initVuforia();
        initTfod();

        twoMotorChassis = new TwoMotorChassis( hardwareMap, telemetry );
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
        telemetry.addData(">", "Press Play to start Autonomous");
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
              for (Recognition recognition : updatedRecognitions) 
              {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                
                if(recognition.getBottom() < 300 )
                    continue;

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
   
        telemetry.addData("level", "(%d)", level);
        telemetry.update();
        
        
        //Supposed to know what level duck is going to
        // Setup a variable for each drive wheel to save power level for telemetry
        
        double leftPower = 0.2;
        double rightPower = 0.2;
        double wheelPower = .75;
        
        // ======================
        // == first step       ==
        // ======================
        twoMotorChassis.move( 0.0, -850, 0.2, 1.0 );

        // raise elevator - slow
        if (level == 3) {
            elevator.setPower(-0.2);
        }
        
       twoMotorChassis.turn(-90.0, 1.0);
        // TURN 90 degrees right
        
        //Back  into wall
       twoMotorChassis.move( -90, -250, 0.2, 2.5 );
    
        if (level == 1) {
            elevator.setPower(-0.3);
            sleep(1500);
            elevator.setPower(-0.0625);
            // Move to Hub
            twoMotorChassis.move( -90, 1155, 0.2, 1.0 );
        }
        
        if (level == 2) {
            elevator.setPower(-0.3);
            sleep(2000);
            elevator.setPower(-0.0625);
            // Move to Hub
            twoMotorChassis.move( -90, 1150, 0.2, 1.0);
        }
        
        if (level == 3) {
        twoMotorChassis.move( -90, 1120, 0.2, 1.0);
        }
    
        intake.setPower(0.15);
        sleep(1500);
        intake.setPower(0.0);
        
        // Backup            
        //chassis.move( -90, -1000, 0.2, 1.0 );
        elevator.setPower(0.2);

        twoMotorChassis.turn(0,3.0);
        // Turn left

        elevator.setPower(0.1);

        // Drive into warehouse
        twoMotorChassis.move( 0, 3000, 0.5, 1.0 );
        
        elevator.setPower(0.0);
        
    }
    
    private void initVuforia() {
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
    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//       tfodParameters.minResultConfidence = 0.8f;
//       tfodParameters.isModelTensorFlow2 = true;
//       tfodParameters.inputSize = 320;
//       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    
}

