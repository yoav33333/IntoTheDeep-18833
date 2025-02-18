package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.RotatedRect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline


class pixleRec : OpenCvPipeline() {
    var lowerRGBA: Scalar = Scalar(0.0, 0.0, 60.0, 151.0)
    var upperRGBA: Scalar = Scalar(85.0, 98.0, 255.0, 255.0)
    private val rgbaBinaryMat = Mat()

    private val contours = ArrayList<MatOfPoint>()
    private val hierarchy = Mat()

    var minArea: Int = 1000
    var maxArea: Int = 50000000
    private val contoursByArea = ArrayList<MatOfPoint>()

    private val contoursByArea2f = MatOfPoint2f()
    private val contoursByAreaRotRects = ArrayList<RotatedRect>()

    var lineColor: Scalar = Scalar(0.0, 0.0, 255.0, 0.0)
    var lineThickness: Int = 7

    private val inputRotRects = Mat()

    override fun processFrame(input: Mat): Mat {
        Core.inRange(input, lowerRGBA, upperRGBA, rgbaBinaryMat)

        contours.clear()
        hierarchy.release()
        Imgproc.findContours(
            rgbaBinaryMat,
            contours,
            hierarchy,
            Imgproc.RETR_EXTERNAL,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        contoursByArea.clear()
        for (contour in contours) {
            val area = Imgproc.contourArea(contour)
            if ((area >= minArea) && (area <= maxArea)) {
                contoursByArea.add(contour)
            }
        }

        contoursByAreaRotRects.clear()
        for (points in contoursByArea) {
            contoursByArea2f.release()
            points.convertTo(contoursByArea2f, CvType.CV_32F)

            contoursByAreaRotRects.add(Imgproc.minAreaRect(contoursByArea2f))
        }

        input.copyTo(inputRotRects)
        for (rect in contoursByAreaRotRects) {
            if (rect != null) {
                val rectPoints = arrayOfNulls<Point>(4)
                rect.points(rectPoints)
                val matOfPoint = MatOfPoint(*rectPoints)

                Imgproc.polylines(inputRotRects, listOf(matOfPoint), true, lineColor, lineThickness)
            }
        }

        return inputRotRects
    }
}
