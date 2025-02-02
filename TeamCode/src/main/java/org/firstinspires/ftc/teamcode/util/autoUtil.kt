package org.firstinspires.ftc.teamcode.util

import com.pedropathing.pathgen.Point
import dev.frozenmilk.mercurial.commands.Command
import kotlin.math.pow
import kotlin.math.sqrt
abstract class marker(val commands: Command){}

data class preDistenceMarker(val command: Command, val distance: Double):marker(command) {}
data class postDistenceMarker(val command: Command, val distance: Double):marker(command) {}
data class postTimeMarker(val command: Command, val time: Double):marker(command){}

fun getDistance(start: Point, end: Point): Double {
    return (sqrt((end.x - start.x).pow(2.0) + (end.y - start.y).pow(2.0)))
}