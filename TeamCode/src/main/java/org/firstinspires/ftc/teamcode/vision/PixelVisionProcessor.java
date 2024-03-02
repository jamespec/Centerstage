package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.inRange;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class PixelVisionProcessor implements VisionProcessor
{
    private final static Scalar minGreen = new Scalar(36, 0, 48, 0);
    private final static Scalar maxGreen = new Scalar(79, 153, 255, 255);
    private final static Scalar minYellow = new Scalar(13, 112, 158, 0);
    private final static Scalar maxYellow = new Scalar(25, 255, 255, 255);
    private final static Scalar minWhite = new Scalar(0, 0, 180, 0);
    private final static Scalar maxWhite = new Scalar(180, 27, 255, 255);
    private final static Scalar minPurple = new Scalar(131, 28, 71, 0);
    private final static Scalar maxPurple = new Scalar(157, 116, 255, 255);

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
        Mat colorMat = new Mat();

        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        blur(hsvMat, 4.0, hsvMat);

        inRange(hsvMat, minGreen, maxGreen, colorMat);
        Mat totalColorMat = colorMat.clone();

        inRange(hsvMat, minYellow, maxYellow, colorMat);
        Core.add(totalColorMat, colorMat, totalColorMat);
        inRange(hsvMat, minWhite, maxWhite, colorMat);
        Core.add(totalColorMat, colorMat, totalColorMat);
        inRange(hsvMat, minPurple, maxPurple, colorMat);
        Core.add(totalColorMat, colorMat, totalColorMat);

        List<MatOfPoint> contours = new ArrayList<>();
        findContours(totalColorMat, true, contours);
        center = findTargetCenter(contours, true, frame);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    private static void blur(Mat input, double doubleRadius, Mat output)
    {
        int radius = (int) (doubleRadius + 0.5);
        int kernelSize;

        kernelSize = 2 * radius + 1;
        Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
    }

    private static void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours)
    {
        Mat hierarchy = new Mat();
        contours.clear();

        int mode = (externalOnly ? Imgproc.RETR_EXTERNAL : Imgproc.RETR_LIST);
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    private static Point findTargetCenter( List<MatOfPoint> contours, boolean drawCandidates, Mat output )
    {
        Point target=null;
        double maxY=0;
        Point center = new Point();
        float[] radius = new float[2];
        MatOfPoint2f mat2f = new MatOfPoint2f();

        int contourCount=0;
        for(int i=0; i<contours.size(); ++i ) {
            MatOfPoint contour = contours.get(i);
            contour.convertTo(mat2f, CvType.CV_32F);
            Imgproc.minEnclosingCircle(mat2f, center, radius);

            if( radius[0] > 40 &&
                center.x > output.width() * 0.20 &&
                center.x < output.width() * 0.80 &&
                center.y > output.height() * 0.60 )
            {
                contourCount++;

                if( drawCandidates ) {
                    // draw the contour and center of the shape on the image
                    Imgproc.drawContours(output, contours, i, new Scalar(0, 255, 0), 2);
                    String s = String.format("(%3.0f, %3.0f) R: %3.0f", center.x, center.y, radius[0] );
                    center.x += 30;
                    Imgproc.putText(output, s, center, Imgproc.FONT_HERSHEY_PLAIN, 1.0, new Scalar(255, 255, 255), 2);
                }

                if( center.y > maxY ) {
                    maxY = center.y;
                    target = center.clone();
                }
            }
        }
        if( drawCandidates ) {
            String s = String.format("Contours: %d", contourCount);
            Imgproc.putText(output, s, new Point(100, 100), Imgproc.FONT_HERSHEY_PLAIN, 1.0, new Scalar(255, 255, 255), 1);
        }

        return target;
    }
}
