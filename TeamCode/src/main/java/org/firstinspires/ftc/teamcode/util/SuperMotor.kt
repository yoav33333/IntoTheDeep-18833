package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.util.OpModeLazyCell

class SuperMotor(name: String) {
    private val motor= CachingDcMotorEx(HardwareDevice(name, DcMotorEx::class.java).get())

    fun get(): CachingDcMotorEx {
        return motor
    }

    //builder
    fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior):SuperMotor {
        motor.zeroPowerBehavior = zeroPowerBehavior
        return this
    }
    fun setDirection(direction: DcMotorSimple.Direction):SuperMotor {
        motor.direction = direction
        return this
    }
    fun setMode(mode: DcMotor.RunMode):SuperMotor {
        motor.mode = mode
        return this
    }
}