package org.firstinspires.ftc.teamcode.subsystems.depositClaw

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawVariables.minDistance
import org.firstinspires.ftc.teamcode.util.HardwareDevice
import java.lang.annotation.Inherited

object DepositClawHardware: SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val depositClaw by OpModeLazyCell{HardwareDevice("small claw", Servo::class.java).get()}
    val colorSensor by OpModeLazyCell{HardwareDevice("dsc", RevColorSensorV3::class.java).get()}

    fun setDepositClawPosition(position: Double){
        depositClaw.position = position
    }
    fun getDepositClawPosition() = depositClaw.position

    fun getDistance(distanceUnit: DistanceUnit = DistanceUnit.MM)
        = colorSensor.getDistance(distanceUnit)

    fun isInRange() = getDistance()<minDistance

}