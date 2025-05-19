package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx

class SuperServo(name: String) {
    private val servo= HardwareDevice(name, ServoImplEx::class.java).get()

    fun get(): ServoImplEx {
        return servo
    }

    fun setDirection(direction: Servo.Direction):SuperServo {
        servo.direction = direction
        return this
    }


}