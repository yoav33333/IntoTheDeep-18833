package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import dev.frozenmilk.dairy.core.util.OpModeLazyCell

class RobotHardware private constructor(hardwareMap: HardwareMap){
    private lateinit var INSTANCE: RobotHardware

    fun Init(hardwareMap: HardwareMap): RobotHardware{
        INSTANCE = RobotHardware(hardwareMap)
        return INSTANCE
    }
    fun getInstance(): RobotHardware{
        return INSTANCE
    }

    val leftFront: CachingDcMotorEx by OpModeLazyCell {
        CachingDcMotorEx(hardwareMap.get(
            DcMotorEx::class.java, "lf"
        ))
    }
    val leftBack: CachingDcMotorEx by OpModeLazyCell {
        CachingDcMotorEx(hardwareMap.get(
            DcMotorEx::class.java, "lb"
        ))
    }

    val rightBack: CachingDcMotorEx by OpModeLazyCell {
        val m = hardwareMap.get(DcMotorEx::class.java, "rb")
        m.direction = DcMotorSimple.Direction.REVERSE
        CachingDcMotorEx(m)
    }
    val rightFront: CachingDcMotorEx by OpModeLazyCell {
        val m = hardwareMap.get(DcMotorEx::class.java, "rf")
        m.direction = DcMotorSimple.Direction.REVERSE
        CachingDcMotorEx(m)
    }

}