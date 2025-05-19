package org.firstinspires.ftc.teamcode.commands.util

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Command
import java.util.function.BooleanSupplier

/**
 * a command that waits until the supplier returns true
 */
class WaitUntil(private val supplier: BooleanSupplier) : Command {
    /**
     * Performs no initialization when the command starts.
     */
    override fun initialise() {
    }

    /**
     * No operation performed during command execution.
     */
    override fun execute() {
    }

    /**
     * Called when the command ends or is interrupted. No cleanup is performed.
     *
     * @param interrupted True if the command was interrupted before completion.
     */
    override fun end(interrupted: Boolean) {
    }

    /**
     * Returns true when the supplied condition is met, indicating the command should finish.
     *
     * @return True if the condition provided by the BooleanSupplier is satisfied.
     */
    override fun finished(): Boolean {
        return supplier.asBoolean
    }

    override val requirements: Set<Any> = emptySet()
    override val runStates: Set<Wrapper.OpModeState> = setOf(Wrapper.OpModeState.INIT, Wrapper.OpModeState.ACTIVE)
    /**
 * Returns the string representation of the command.
 *
 * @return The string "WaitUntil".
 */
override fun toString() = "WaitUntil"
}
