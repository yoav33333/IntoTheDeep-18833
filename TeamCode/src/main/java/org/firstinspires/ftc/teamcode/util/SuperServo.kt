package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx

class SuperServo(name: String) {
    private val servo= HardwareDevice(name, ServoImplEx::class.java).get()

    /**
     * Returns the underlying ServoImplEx hardware device.
     *
     * @return The ServoImplEx instance managed by this SuperServo.
     */
    fun get(): ServoImplEx {
        return servo
    }

    /**
     * Sets the direction of the servo and returns this instance for chaining.
     *
     * @param direction The desired direction for the servo.
     * @return This SuperServo instance.
     */
    fun setDirection(direction: Servo.Direction):SuperServo {
        servo.direction = direction
        return this
    }


}