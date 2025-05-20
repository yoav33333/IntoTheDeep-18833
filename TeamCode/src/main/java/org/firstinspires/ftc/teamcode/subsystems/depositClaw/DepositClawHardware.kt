package org.firstinspires.ftc.teamcode.subsystems.depositClaw

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
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

    val depositClaw = HardwareDevice("small claw", Servo::class.java).get()
    val colorSensor = HardwareDevice("dsc", RevColorSensorV3::class.java).get()

    /**
     * Sets the position of the deposit claw servo.
     *
     * @param position The desired servo position, typically between 0.0 (minimum) and 1.0 (maximum).
     */
    fun setDepositClawPosition(position: Double){
        depositClaw.position = position
    }
    /**
 * Returns the current position of the deposit claw servo.
 *
 * @return The servo position as a value between 0.0 and 1.0.
 */
fun getDepositClawPosition() = depositClaw.position

    /**
         * Returns the distance measured by the color sensor in the specified unit.
         *
         * @param distanceUnit The unit in which to return the distance. Defaults to millimeters.
         * @return The measured distance from the color sensor in the specified unit.
         */
        fun getDistance(distanceUnit: DistanceUnit = DistanceUnit.MM)
        = colorSensor.getDistance(distanceUnit)

    /**
 * Determines whether the detected object is within the minimum distance threshold.
 *
 * @return `true` if the measured distance is less than the predefined minimum distance; otherwise, `false`.
 */
fun isInRange() = getDistance()<minDistance

}