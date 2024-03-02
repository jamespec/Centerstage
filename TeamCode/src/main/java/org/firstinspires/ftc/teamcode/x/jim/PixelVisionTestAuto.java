package org.firstinspires.ftc.teamcode.x.jim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Pixel Vision Test")
public class PixelVisionTestAuto extends OpMode {
   private PixelVisionProcessor visionProcessor;
   private VisionPortal visionPortal;

   @Override
   public void init() {
      visionProcessor = new PixelVisionProcessor();

      WebcamName web = hardwareMap.get(WebcamName.class, "Webcam 1");
      visionPortal = VisionPortal.easyCreateWithDefaults(web, visionProcessor);
   }

   @Override
   public void init_loop() {
   }

   @Override
   public void start() {
      visionPortal.stopStreaming();
   }

   @Override
   public void loop() {

   }
}
