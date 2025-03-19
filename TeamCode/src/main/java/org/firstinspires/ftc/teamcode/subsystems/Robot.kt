package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.ServoImplEx

@Config
object Robot {
    @JvmField
    val slides = true
    @JvmField
    val claw = true
    @JvmField
    val armClaw = true
    @JvmField
    val extendo = true
    @JvmField
    val deposit = true
    @JvmField
    val drive = true
    fun setPose(servo: ServoImplEx, pose: Double, run: Boolean) {
        if (run){
            servo.position = pose
        }
        else{
            servo.close()
        }
    }
}