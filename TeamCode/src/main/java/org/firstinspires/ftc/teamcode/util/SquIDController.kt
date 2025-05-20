package org.firstinspires.ftc.teamcode.util

import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

class SquIDController(var p: Double = 0.0) {
    var i: Double = 0.0
    var d: Double = 0.0
    /**
     * Updates the proportional gain used by the controller.
     *
     * @param p The new proportional gain value.
     */
    fun setPID(p: Double) {
        this.p = p
    }


    /**
     * Computes a signed control output based on the square root of the proportional error between the target and current values.
     *
     * The output is calculated as the square root of the absolute value of the error (target minus current) multiplied by the proportional gain, with the sign of the original error preserved.
     *
     * @param target The desired setpoint value.
     * @param current The current measured value.
     * @return The signed control effort for the given error.
     */
    fun calculate(target: Double, current: Double): Double {
        return sqrt(abs((target - current) * p)) * sign(target - current)
    }
}