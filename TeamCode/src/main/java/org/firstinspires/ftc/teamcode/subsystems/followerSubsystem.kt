//package org.firstinspires.ftc.teamcode.subsystems
//
//import android.R
//import com.pedropathing.follower.Follower
//import com.pedropathing.pathgen.PathChain
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
//import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
//import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier
//import dev.frozenmilk.dairy.core.wrapper.Wrapper
//import dev.frozenmilk.mercurial.Mercurial
//import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
//import dev.frozenmilk.mercurial.commands.Lambda
//import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
//import dev.frozenmilk.mercurial.subsystems.Subsystem
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants
//
//import java.lang.annotation.Inherited
//import java.util.function.DoubleSupplier
//import kotlin.math.abs
//import kotlin.math.cos
//import kotlin.math.max
//import kotlin.math.pow
//import kotlin.math.sin
//
//
//object followerSubsystem: SDKSubsystem(){
//    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
//            SingleAnnotation(Mercurial.Attach::class.java)
//    @Target(AnnotationTarget.CLASS)
//    @Retention(AnnotationRetention.RUNTIME)
//    @MustBeDocumented
//    @Inherited
//    annotation class Attach
//
//    val follower = Follower(hardwareMap, LConstants.class,FConstants.class)
//
//    fun followPath(path: PathChain) = Lambda("follow-path")
//        .setInit {
//            follower.followPath(path)
//        }
//        .setExecute {
//            follower.update()
//
//        }
//        .setFinish {
//            !follower.isBusy
//        }
//
//    fun teleopDrive(x: EnhancedDoubleSupplier, y: EnhancedDoubleSupplier, rotate: EnhancedDoubleSupplier, speed: EnhancedDoubleSupplier) = Lambda("teleop-drive")
//        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .setInit{
//            follower.startTeleopDrive()
//        }
//        .setExecute{
//            follower.setTeleOpMovementVectors(-y.state/speed.state, -x.state/speed.state, -rotate.state/speed.state)
//            follower.update()
//        }
//        .setFinish{false}
//}