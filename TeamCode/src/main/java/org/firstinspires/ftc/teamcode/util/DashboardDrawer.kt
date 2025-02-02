package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.MathFunctions
import com.pedropathing.pathgen.Path
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.pathgen.Vector
import com.pedropathing.util.DashboardPoseTracker

class DashboardDrawer {

    val ROBOT_RADIUS: Double = 9.0
    var packet: TelemetryPacket? = null

    fun Drawing() {
    }

    fun drawDebug(follower: Follower) {
        if (follower.currentPath != null) {
            drawPath(follower.currentPath, "#3F51B5")
            val closestPoint = follower.getPointFromPath(follower.currentPath.closestPointTValue)
            drawRobot(
                Pose(
                    closestPoint.x,
                    closestPoint.y,
                    follower.currentPath.getHeadingGoal(follower.currentPath.closestPointTValue)
                ), "#3F51B5"
            )
        }

        drawPoseHistory(follower.dashboardPoseTracker, "#4CAF50")
        drawRobot(follower.pose, "#4CAF50")
        sendPacket()
    }

    fun drawRobot(pose: Pose, color: String?) {
        if (packet == null) {
            packet = TelemetryPacket()
        }

        packet!!.fieldOverlay().setStroke(color)
        drawRobotOnCanvas(packet!!.fieldOverlay(), pose.copy())
    }

    fun drawPath(path: Path, color: String?) {
        if (packet == null) {
            packet = TelemetryPacket()
        }

        packet!!.fieldOverlay().setStroke(color)
        drawPath(packet!!.fieldOverlay(), path.dashboardDrawingPoints)
    }

    fun drawPath(pathChain: PathChain, color: String?) {
        for (i in 0 until pathChain.size()) {
            drawPath(pathChain.getPath(i), color)
        }
    }

    fun drawPoseHistory(poseTracker: DashboardPoseTracker, color: String?) {
        if (packet == null) {
            packet = TelemetryPacket()
        }

        packet!!.fieldOverlay().setStroke(color)
        packet!!.fieldOverlay()
            .strokePolyline(poseTracker.xPositionsArray, poseTracker.yPositionsArray)
    }

    fun sendPacket(): Boolean {
        if (packet != null) {
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
            packet = null
            return true
        } else {
            return false
        }
    }

    fun drawRobotOnCanvas(c: Canvas, t: Point) {
        c.setStrokeWidth(1)
        c.strokeCircle(t.x-144/2, t.y-144/2, 9.0)
        val halfv = Vector(4.5, t.theta)
        val p1 = MathFunctions.addVectors(halfv, Vector(t.r, t.theta))
        val p2 = MathFunctions.addVectors(p1, halfv)
        c.strokeLine(p1.xComponent, p1.yComponent, p2.xComponent, p2.yComponent)
    }

    fun drawRobotOnCanvas(c: Canvas, t: Pose) {
        c.strokeCircle(t.x, t.y, 9.0)
        val v = t.headingVector
        v.magnitude = v.magnitude * 9.0
        val x1 = t.x + v.xComponent / 2.0
        val y1 = t.y + v.yComponent / 2.0
        val x2 = t.x + v.xComponent
        val y2 = t.y + v.yComponent
        c.strokeLine(x1, y1, x2, y2)
    }

    fun drawPath(c: Canvas, points: Array<DoubleArray?>) {
        c.strokePolyline(points[0], points[1])
    }
}