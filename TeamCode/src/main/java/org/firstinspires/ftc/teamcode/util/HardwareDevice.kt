package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.util.OpModeLazyCell

class HardwareDevice<T>(name: String, val type: Class<T>){
    private val device =
        FeatureRegistrar.activeOpMode.hardwareMap.get(type, name)

    fun get(): T {
        return device
    }
}