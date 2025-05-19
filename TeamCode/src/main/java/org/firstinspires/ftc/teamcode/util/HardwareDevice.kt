package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.util.OpModeLazyCell

class HardwareDevice<T>(name: String, val type: Class<T>){
    private val device: T by OpModeLazyCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(type, name)
    }

    /**
     * Returns the hardware device instance associated with this wrapper.
     *
     * The device is retrieved from the active OpMode's hardware map on first access and cached for future calls.
     *
     * @return The hardware device of type T.
     */
    fun get(): T {
        return device
    }
}