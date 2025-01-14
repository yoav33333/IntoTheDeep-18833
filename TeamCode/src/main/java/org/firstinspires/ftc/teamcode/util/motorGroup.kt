package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx

class motorGroup(val leadingMotor: CachingDcMotorEx, vararg motors: CachingDcMotorEx) {
    val motors = motors
    fun setPower(power: Double) {
        leadingMotor.power = power
        motors.forEach { it.power = power }
    }

    fun getPower(): Double {
        return leadingMotor.power
    }

    fun getPos(): Int {
        return leadingMotor.currentPosition
    }

    fun setRunMode(mode: DcMotor.RunMode) {
        leadingMotor.mode = mode
        motors.forEach { it.mode = mode }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: ZeroPowerBehavior) {
        leadingMotor.zeroPowerBehavior = zeroPowerBehavior
        motors.forEach { it.zeroPowerBehavior = zeroPowerBehavior }
    }

}