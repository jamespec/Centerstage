package org.firstinspires.ftc.teamcode.x.previous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//Different

public class AutoPipeline extends OpenCvPipeline
{
    private static float[] target = {4.0f/8f,5.5f/8f};

    private final int rows;
    private final int cols;

    private final int[] hueRed = {0,255};
    private final int[] satRed = {0,255};
    private final int[] valRed = {0,255};

    private final int[] hueBlue = {0,255};
    private final int[] satBlue = {0,255};
    private final int[] valBlue = {0,255};

    public int valR = -1;
    public int valG = -1;
    public int valB = -1;

    List<MatOfPoint> contoursList = new ArrayList<>();

    public enum Stage
    {
        START,
        YELLOW,
        RED,
        BLUE,
    }

    public Stage stage = Stage.START;

    public AutoPipeline(int rows, int cols) {
        this.rows = rows;
        this.cols = cols;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        switch (stage) {
            case START:
                return processStart(input);
            case YELLOW:
                return processYellow(input);
            case RED:
                return processRed(input);
            case BLUE:
                return processBlue(input);
        }
        return null;
    }

    public Mat processStart(Mat input) {
        input = input.submat(new Rect(new Point(160, 240), new Point(480, 480)));
        contoursList.clear();
        double[] targetPix = input.get((int)(input.rows() * target[1]), (int)(input.cols() * target[0]));//gets value at circle
        valR = (int)targetPix[0];
        valG = (int)targetPix[1];
        valB = (int)targetPix[2];
        Point targetPoint = new Point((int)(input.cols()* target[0]), (int)(input.rows() * target[1]));
        Imgproc.circle(input, targetPoint,5, new Scalar( 255, 0, 0 ),1 );
        Imgproc.circle(input, targetPoint,25, new Scalar( 0, 255, 0 ),1 );
        return input;
    }

    public Mat processYellow(Mat input) {
        contoursList.clear();
        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         */
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(input, input, 2);
        Imgproc.threshold(input, input, 102, 255, Imgproc.THRESH_BINARY_INV);
        Imgproc.findContours(input, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        int numContoursFound = contoursList.size();
        Imgproc.drawContours(input, contoursList, -1, new Scalar(0, 0, 255), 3, 8);
        return input;
    }

    public Mat processRed(Mat input) {
        contoursList.clear();
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Core.inRange(input, new Scalar(hueRed[0], satRed[0], valRed[0]),
                new Scalar(hueRed[1], satRed[1], valRed[1]), input);
        Imgproc.findContours(input, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contoursList, -1, new Scalar(0, 255, 0), 3, 8);
        return input;
    }

    public Mat processBlue(Mat input) {
        contoursList.clear();
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Core.inRange(input, new Scalar(hueBlue[0], satBlue[0], valBlue[0]),
                new Scalar(hueBlue[1], satBlue[1], valBlue[1]), input);
        Imgproc.findContours(input, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contoursList, -1, new Scalar(0, 255, 0), 3, 8);
        return input;
    }

    public int getLocation() {
        if (valR > valG && valR > valB)
        {
            return 1;
        }

        if (valG > valR && valG > valB)
        {
            return 2;
        }

        return 3;
    }

    public double getPercentDiff() {
        if (contoursList.size() == 0) {
            return 0;
        }
        Rect max = new Rect();
        for (MatOfPoint contour: contoursList) {
            Rect r = Imgproc.boundingRect(contour);
            if (r.area() > max.area()) {
                max = r;
            }
        }
        return (max.x - (cols/2)) / cols * 100;
    }
}