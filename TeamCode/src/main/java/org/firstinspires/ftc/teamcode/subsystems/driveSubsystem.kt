//package org.firstinspires.ftc.teamcode.subsystems
//
//import android.R
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
//import com.qualcomm.robotcore.hardware.DcMotor
//import com.qualcomm.robotcore.hardware.DcMotorEx
//import com.qualcomm.robotcore.hardware.DcMotorSimple
//import com.qualcomm.robotcore.hardware.IMU
//import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
//import dev.frozenmilk.dairy.core.FeatureRegistrar
//import dev.frozenmilk.dairy.core.dependency.Dependency
//import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
//import dev.frozenmilk.dairy.core.util.OpModeLazyCell
//import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier
//import dev.frozenmilk.dairy.core.wrapper.Wrapper
//import dev.frozenmilk.mercurial.Mercurial
//import dev.frozenmilk.mercurial.commands.Lambda
//import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
//import dev.frozenmilk.mercurial.subsystems.Subsystem
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
//import java.lang.annotation.Inherited
//import kotlin.math.abs
//import kotlin.math.cos
//import kotlin.math.max
//import kotlin.math.pow
//import kotlin.math.sin
//
//
//object driveSubsystem: Subsystem{
//    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
//            SingleAnnotation(Mercurial.Attach::class.java)
//    @Target(AnnotationTarget.CLASS)
//    @Retention(AnnotationRetention.RUNTIME)
//    @MustBeDocumented
//    @Inherited
//    annotation class Attach
//
////    var x = 0.0
////    var y = 0.0
////    var rotate = 0.0
////    var speed = 1.0
//
//
//    val leftFront: CachingDcMotorEx by OpModeLazyCell {
//        val m = CachingDcMotorEx(FeatureRegistrar.activeOpMode.hardwareMap.get(
//            DcMotorEx::class.java, "dfl"
//        ))
//        m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        m
//    }
//    val leftBack: CachingDcMotorEx by OpModeLazyCell {
//        val m = CachingDcMotorEx(FeatureRegistrar.activeOpMode.hardwareMap.get(
//            DcMotorEx::class.java, "drl"
//        ))
//        m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        m
//    }
//
//    val rightBack: CachingDcMotorEx by OpModeLazyCell {
//        val m = FeatureRegistrar.activeOpMode.hardwareMap.get(DcMotorEx::class.java, "drr")
//        m.direction = DcMotorSimple.Direction.REVERSE
//        m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        CachingDcMotorEx(m)
//    }
//    val rightFront: CachingDcMotorEx by OpModeLazyCell {
//        val m = FeatureRegistrar.activeOpMode.hardwareMap.get(DcMotorEx::class.java, "dfr")
//        m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        m.direction = DcMotorSimple.Direction.REVERSE
//        CachingDcMotorEx(m)
//    }
//    val imu: IMU by OpModeLazyCell{
//        val i = FeatureRegistrar.activeOpMode.hardwareMap.get(IMU::class.java, "imu")
//        i.initialize(IMU.Parameters(RevHubOrientationOnRobot(
//            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
//            RevHubOrientationOnRobot.UsbFacingDirection.UP
//        )))
//        i
//    }
//
//
//    fun robotOrientedDrive(x: Double, y: Double, rotate: Double, speed: Double){
//
//        leftFront.setPowerRaw((y + x + rotate)*speed)
//        leftBack.setPowerRaw((y - x + rotate)*speed)
//        rightFront.setPowerRaw((- y + x + rotate)*speed)
//        rightBack.setPowerRaw((- y - x + rotate)*speed)
//    }
//
//    val forward = EnhancedDoubleSupplier{0.0}
//    val strafe = EnhancedDoubleSupplier{0.0}
//    val rotate = EnhancedDoubleSupplier{0.0}
//    val speed = EnhancedDoubleSupplier{1.0}
////    fun newRobotOrientedDrive(x: EnhancedDoubleSupplier, y: EnhancedDoubleSupplier, rotate: EnhancedDoubleSupplier, speed: EnhancedDoubleSupplier){
////        followerSubsystem.teleopDrive(x,y,rotate,speed).schedule()
////    }
//
//
//
//
//    fun fieldOrientedDrive(x: Double, y: Double, rotate: Double){
//        x.pow(2)
//        y.pow(2)
//        rotate.pow(2)
//        val botHeading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
//
//
//        var rotX = R.attr.x * cos(-botHeading) - R.attr.y * sin(-botHeading)
//        val rotY = R.attr.x * sin(-botHeading) + R.attr.y * cos(-botHeading)
//
//        rotX *= 1.1
//
//
//        val denominator =
//            max(abs(rotY) + abs(rotX) + abs(rotate), 1.0)
//        leftFront.setPowerRaw((rotY + rotX + rotate) / denominator)
//        leftBack.setPowerRaw((rotY - rotX + rotate) / denominator)
//        rightFront.setPowerRaw((rotY - rotX - rotate) / denominator)
//        rightBack.setPowerRaw((rotY + rotX - rotate) / denominator)
//    }
//    val fastGear = Lambda("gearF")
//        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .setInit{
//            speed.state = 1.0
//        }
//    val midFastGear = Lambda("gearFM")
//        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .setInit{
//            speed.state = 0.75
//        }
//    val midSlowGear = Lambda("gearSM")
//        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .setInit{
//            speed.state = 0.5
//        }
//    val slowGear = Lambda("gearS")
//        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .setInit{
//            speed.state = 0.25
//        }
//
////    override fun preUserStartHook(opMode: Wrapper) {
////        newRobotOrientedDrive(strafe, forward, rotate, speed)
////    }
////    override fun postUserLoopHook(opMode: Wrapper) {
////        strafe.state = Mercurial.gamepad1.leftStickX.state
////        forward.state = Mercurial.gamepad1.leftStickY.state
////        rotate.state = Mercurial.gamepad1.rightStickX.state
////        strafe.state+=Mercurial.gamepad1.rightTrigger.state-Mercurial.gamepad1.leftTrigger.state
////        rotate.state+=if(Mercurial.gamepad1.rightBumper.state)1 else 0 - if(Mercurial.gamepad1.leftBumper.state)1 else 0
////
//////        robotOrientedDrive(strafe.state, forward.state, rotate.state, speed.state)
////    }
//}