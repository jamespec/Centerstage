package org.firstinspires.ftc.teamcode;

import static org.opencv.core.Core.inRange;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


@Autonomous()
public class PixelVisionProcessor implements VisionProcessor {
    public Rect rectMiddle = new Rect(170, 220, 80, 80);
    public Rect rectRight = new Rect(500, 265, 80, 80);
    double satRectMiddle;
    double satRectRight;

    Selected selection = Selected.NONE;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();
    Mat hsvFilteredMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Step HSV_Threshold0:
        double[] hsvThresholdHue = {106.83453237410072, 121.63822525597271};
        double[] hsvThresholdSaturation = {119.24460431654676, 255.0};
        double[] hsvThresholdValue = {2.293165467625899, 61.356655290102395};
        // inRange(hsvMat,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),hsvFilteredMat);
        inRange(hsvMat,new Scalar(106.0,119.0,2.29), new Scalar(122.0,255.0,61.35),hsvFilteredMat);

        satRectMiddle = getAvgSaturation(hsvFilteredMat, rectMiddle);
        satRectRight = getAvgSaturation(hsvFilteredMat, rectRight);

        // If the Middle and Right Saturation are the same then the target is Left.
        if( Math.abs(satRectMiddle - satRectRight) > 10.0 ) {
            if (satRectMiddle < satRectRight)
                return Selected.MIDDLE;
            else
                return Selected.RIGHT;
        }

        return Selected.LEFT;
    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        selection = (Selected) userContext;
        switch (selection) {
            case MIDDLE:
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case LEFT:
            case NONE:
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
        }
    }

    public Selected getSelection() {
        return selection;
    }
    public double getSatRectMiddle() {
        return satRectMiddle;
    }

    public double getSatRectRight() {
        return satRectRight;
    }

    public enum Selected {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }
}
