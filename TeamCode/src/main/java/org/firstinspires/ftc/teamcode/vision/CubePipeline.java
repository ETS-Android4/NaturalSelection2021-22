package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class CubePipeline extends OpenCvPipeline {
    ArrayList<MatOfPoint> contours = new ArrayList<>();
    Point point;
    int contourIndex;
    class Fraction {
        private int numerator, denominator;

        Fraction(long a, long b) {
            numerator = (int) (a / gcd(a, b));
            denominator = (int) (b / gcd(a, b));
        }

        /**
         * @return the greatest common denominator
         */
        private long gcd(long a, long b) {
            return b == 0 ? a : gcd(b, a % b);
        }

        public int getNumerator() {
            return numerator;
        }

        public int getDenominator() {
            return denominator;
        }
    }

    // Camera Settings
    protected int imageWidth;
    protected int imageHeight;

    private double cameraPitchOffset;
    private double cameraYawOffset;

    private double fov;
    private double horizontalFocalLength;
    private double verticalFocalLength;

    public CubePipeline(double fov, double cameraPitchOffset, double cameraYawOffset) {
        super();
        this.fov = fov;
        this.cameraPitchOffset = cameraPitchOffset;
        this.cameraYawOffset = cameraYawOffset;
    }

    public CubePipeline(double fov) {
        this(fov, 0, 0);
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);

        imageWidth = mat.width();
        imageHeight = mat.height();

        // pinhole model calculations
        double diagonalView = Math.toRadians(this.fov);
        Fraction aspectFraction = new Fraction(this.imageWidth, this.imageHeight);
        int horizontalRatio = aspectFraction.getNumerator();
        int verticalRatio = aspectFraction.getDenominator();
        double diagonalAspect = Math.hypot(horizontalRatio, verticalRatio);
        double horizontalView = Math.atan(Math.tan(diagonalView / 2) * (horizontalRatio / diagonalAspect)) * 2;
        double verticalView = Math.atan(Math.tan(diagonalView / 2) * (verticalRatio / diagonalAspect)) * 2;
        horizontalFocalLength = this.imageWidth / (2 * Math.tan(horizontalView / 2));
        verticalFocalLength = this.imageHeight / (2 * Math.tan(verticalView / 2));
    }


    /**
     * @param offsetCenterX centerX
     */
    public Double calculateYaw(double offsetCenterX) {
        if (getCenterofRect(rect) != null){
            double targetCenterX = getCenterofRect(rect).x;

            return Math.toDegrees(
                    Math.atan((targetCenterX - offsetCenterX) / horizontalFocalLength)
            );
        }
        return null;
    }

    /**
     * @param offsetCenterY centerY
     */
    double offsetCenterY = 0;
    double targetCenterY = 0;
    public Double calculatePitch(double offsetCenterY) {
        if (getCenterofRect(rect) != null){
            targetCenterY = getCenterofRect(rect).y;
            return -Math.toDegrees(Math.atan((targetCenterY - offsetCenterY) / verticalFocalLength));
        }
        return null;
    }


    public Point getCenterofRect(Rect rect) {
        if (rect == null) {
            return null;
        }
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }

    Rect rect = null;
    @Override
    public Mat processFrame(Mat input) {
        Mat subMat = new Mat();
        Imgproc.cvtColor(input,subMat,Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(subMat,subMat,new Size(5,5),0);
        Scalar lower = new Scalar(13,187,215);
        Scalar upper = new Scalar(16,195,221);
        Core.inRange(subMat,lower,upper,subMat);
        Imgproc.morphologyEx(subMat,subMat,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));
        contours.clear();
        MatOfPoint contour = null;
        Imgproc.findContours(subMat,contours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint newContour = contours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(contour == null){
                    contour = newContour;
                    contourIndex=i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(contour)){
                    contour = newContour;
                    contourIndex=i;
                }
            }
        }
        try {
            rect = Imgproc.boundingRect(contours.get(contourIndex));
            Imgproc.drawContours(input,contours,contourIndex,new Scalar(0,255,255));
            if(rect.area()<200){
                rect = null;
            }
        }catch(Exception ignored){}
        if(rect !=null) {
            Imgproc.rectangle(input, rect, new Scalar(255, 0, 0));
            point = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
        }else {
            point = new Point(0, 0);
        }
        subMat.release();
        return input;
    }

    public Point getPoint() {
        return point;
    }

    double cameraHeight = 25.4;
    public Double getGroundDistance() {
            double groundDistance = Math.tan(Math.atan((targetCenterY - offsetCenterY) / verticalFocalLength)) * cameraHeight;
            return groundDistance;
        }
}
