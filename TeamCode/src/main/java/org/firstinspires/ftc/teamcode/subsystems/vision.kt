package org.firstinspires.ftc.teamcode.subsystems

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

/**
 * This class defines a vision pipeline for detecting and tracking objects using OpenCV.
 */
class vision : OpenCvPipeline() {

    // Define color range for object detection (RGBA)
    var lowerRGBA: Scalar = Scalar(9.0, 41.0, 0.0, 0.0)
    var upperRGBA: Scalar = Scalar(69.0, 151.0, 35.0, 255.0)
    private val rgbaBinaryMat = Mat() // Binary mat to store the result of color thresholding

    // Contours and hierarchy for detected objects
    private val contours = ArrayList<MatOfPoint>()
    private val hierarchy = Mat()

    // Properties for drawing the crosshair
    var lineColor: Scalar = Scalar(0.0, 255.0, 0.0, 0.0) // Green color
    var lineThickness: Int = 3

    // Crosshair points and image
    private val crosshair = ArrayList<MatOfPoint>()
    private val crosshairImage = Mat()
    var crosshairSize: Int = 0 // Size of the crosshair

    // Properties for drawing the rotated rectangles
    var lineColor1: Scalar = Scalar(0.0, 255.0, 0.0, 0.0) // Green color
    var lineThickness1: Int = 3

    // Rotated rectangles for detected objects
    private val crosshair2f = MatOfPoint2f()
    private val crosshairRotRects = ArrayList<RotatedRect>()

    // Image with rotated rectangles drawn
    private val crosshairImageRotRects = Mat()

    /**
     * Processes a frame from the camera and returns a modified frame.
     *
     * @param input The input frame from the camera.
     * @return The processed frame with detected objects highlighted.
     */
    override fun processFrame(input: Mat): Mat {
        // Threshold the input image to isolate objects within the defined color range
        Core.inRange(input, lowerRGBA, upperRGBA, rgbaBinaryMat)

        // Find contours of the detected objects
        contours.clear()
        hierarchy.release()
        Imgproc.findContours(
            rgbaBinaryMat,
            contours,
            hierarchy,
            Imgproc.RETR_EXTERNAL,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        // Copy the input image to the crosshair image
        input.copyTo(crosshairImage)

        // Calculate the center point of the image for the crosshair
        val crosshairPoint = Point(
            (input.cols().toDouble()) / 2, (input.rows()
                .toDouble()) / 2
        )
        val scaleFactor = (input.rows() + input.cols()) / 2 // Scale factor for crosshair size

        // Adjust crosshair size based on the scale factor
        val adjustedCrosshairSize = (crosshairSize * scaleFactor) / 100

        // Draw the crosshair on the image
        Imgproc.line(
            crosshairImage,
            Point(crosshairPoint.x - adjustedCrosshairSize, crosshairPoint.y),
            Point(crosshairPoint.x + adjustedCrosshairSize, crosshairPoint.y),
            lineColor,
            lineThickness
        )
        Imgproc.line(
            crosshairImage,
            Point(crosshairPoint.x, crosshairPoint.y - adjustedCrosshairSize),
            Point(crosshairPoint.x, crosshairPoint.y + adjustedCrosshairSize),
            lineColor,
            lineThickness
        )

        // Find contours that contain the crosshair point
        crosshair.clear()
        for (contour in contours) {
            val boundingRect = Imgproc.boundingRect(contour)
            if (boundingRect.contains(crosshairPoint)) {
                crosshair.add(contour)
            }
        }

        // Calculate and store rotated rectangles for the detected objects
        crosshairRotRects.clear()
        for (points in crosshair) {
            crosshair2f.release()
            points.convertTo(crosshair2f, CvType.CV_32F)
            crosshairRotRects.add(Imgproc.minAreaRect(crosshair2f))
        }

        // Copy the crosshair image to the rotated rectangles image
        crosshairImage.copyTo(crosshairImageRotRects)

        // Draw rotated rectangles on the image
        for (rect in crosshairRotRects) {
            val rectPoints = arrayOfNulls<Point>(4)
            rect.points(rectPoints)
            val matOfPoint = MatOfPoint(*rectPoints)

            Imgproc.polylines(
                crosshairImageRotRects,
                listOf(matOfPoint),
                true,
                lineColor1,
                lineThickness1
            )
        }

        // Return the processed image with rotated rectangles
        return crosshairImageRotRects
    }
}