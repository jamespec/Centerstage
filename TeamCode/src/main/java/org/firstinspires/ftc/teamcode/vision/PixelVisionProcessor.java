package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.inRange;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class PixelVisionProcessor implements VisionProcessor
{
    private final static Scalar minGreen = new Scalar(24, 44, 57, 0);
    private final static Scalar maxGreen = new Scalar(71, 159, 161, 255);
    private final static Scalar minYellow = new Scalar(13, 99, 126, 0);
    private final static Scalar maxYellow = new Scalar(25, 255, 233, 255);
    private final static Scalar minWhite = new Scalar(0, 0, 180, 0);
    private final static Scalar maxWhite = new Scalar(180, 27, 255, 255);
    private final static Scalar minPurple = new Scalar(131, 28, 85, 0);
    private final static Scalar maxPurple = new Scalar(157, 144, 216, 255);

    Point center;

    public Point getCenter()
    {
        return center;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        center = findLargestPixelCenter(hsvMat, minWhite, maxWhite);
        drawContoursAroundColorRange( hsvMat, frame, minWhite, maxWhite );

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    private static void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours)
    {
        Mat hierarchy = new Mat();
        contours.clear();

        int mode = (externalOnly ? Imgproc.RETR_EXTERNAL : Imgproc.RETR_LIST);
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    private static Point findLargestPixelCenter( Mat hsvMat, Scalar minColor, Scalar maxColor )
    {
        Mat colorMat = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Point center = new Point();
        float[] radius = new float[2];
        MatOfPoint2f mat2f = new MatOfPoint2f();
        float maxRadius = 0;
        Point result=null;

        inRange(hsvMat, minColor, maxColor, colorMat);

        findContours(colorMat, true, contours);
        for(int i=0; i<contours.size(); ++i ) {
            MatOfPoint contour = contours.get(i);
            contour.convertTo(mat2f, CvType.CV_32F);
            Imgproc.minEnclosingCircle(mat2f, center, radius);

            if( radius[0] > maxRadius ) {
                maxRadius = radius[0];
                result = center.clone();
            }
        }
        return result;
    }

    // Input is expected to be an HSV Frame from the video
    private static void drawContoursAroundColorRange( Mat hsvMat, Mat output, Scalar minColor, Scalar maxColor )
    {
        Mat colorMat = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Point center = new Point();
        float[] radius = new float[2];
        MatOfPoint2f mat2f = new MatOfPoint2f();

        inRange(hsvMat, minColor, maxColor, colorMat);

        findContours(colorMat, true, contours);
        for(int i=0; i<contours.size(); ++i ) {
            MatOfPoint contour = contours.get(i);
            contour.convertTo(mat2f, CvType.CV_32F);
            Imgproc.minEnclosingCircle(mat2f, center, radius);

            if( radius[0] > 20 ) {
                // draw the contour and center of the shape on the image
                Imgproc.drawContours(output, contours, i, new Scalar(0, 255, 0), 2);
                String s = "(" + center.x + ", " + center.y + ")";
                center.y += 50;
                Imgproc.putText(output, s, center, Imgproc.FONT_HERSHEY_PLAIN, 1.0, new Scalar(255, 255, 255), 2);
            }
        }
    }
}
