package org.firstinspires.ftc.teamcode.util

import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

class SquIDController(var p: Double = 0.0) {
    var i: Double = 0.0
    var d: Double = 0.0
    fun setPID(p: Double) {
        this.p = p
    }


    fun calculate(target: Double, current: Double): Double {
        return sqrt(abs((target - current) * p)) * sign(target - current)
    }
}