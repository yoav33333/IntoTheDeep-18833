package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.util.OpModeLazyCell

class SuperMotor(name: String) {
    private val motor= CachingDcMotorEx(HardwareDevice(name, DcMotorEx::class.java).get())

    /**
     * Returns the underlying CachingDcMotorEx instance wrapped by this SuperMotor.
     *
     * @return The wrapped CachingDcMotorEx motor.
     */
    fun get(): CachingDcMotorEx {
        return motor
    }

    /**
     * Sets the zero power behavior of the motor and returns this instance for chaining.
     *
     * @param zeroPowerBehavior The desired zero power behavior to apply to the motor.
     * @return This SuperMotor instance for method chaining.
     */
    fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior):SuperMotor {
        motor.zeroPowerBehavior = zeroPowerBehavior
        return this
    }
    /**
     * Sets the direction of the motor and returns this instance for chaining.
     *
     * @param direction The desired direction for the motor.
     * @return This SuperMotor instance for method chaining.
     */
    fun setDirection(direction: DcMotorSimple.Direction):SuperMotor {
        motor.direction = direction
        return this
    }
    /**
     * Sets the run mode of the motor and returns this instance for chaining.
     *
     * @param mode The desired run mode for the motor.
     * @return This SuperMotor instance for method chaining.
     */
    fun setMode(mode: DcMotor.RunMode):SuperMotor {
        motor.mode = mode
        return this
    }
}