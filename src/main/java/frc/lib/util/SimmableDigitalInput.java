package frc.lib.util;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;

public class SimmableDigitalInput extends DigitalInput {
    private Optional<BooleanSupplier> simSupplier = Optional.empty();
    private boolean isSimulation = Robot.isSimulation();

    /**
     * Create an instance of a Digital Input class. Creates a digital input given a channel.
     * @param channel the DIO channel for the digital input 0-9 are on-board, 10-25 are on the MXP
     */
    public SimmableDigitalInput(int channel) {
        super(channel);
    }

    /**
     * Create an instance of a Digital Input class. Creates a digital input given a channel but includes a BooleanSupplier for simulation.
     * @param channel the DIO channel for the digital input 0-9 are on-board, 10-25 are on the MXP
     * @param simSupplier what to return if in simulation. this will override the value set on Glass
     */
    public SimmableDigitalInput(int channel, BooleanSupplier simSupplier) {
        this(channel);
        if (isSimulation) this.simSupplier = Optional.of(simSupplier);
    }

    @Override
    public boolean get() {
        if (!isSimulation)
            return super.get();

        if (simSupplier.isPresent())
            return simSupplier.get().getAsBoolean();
        else {
            System.out.println("SimmableDigitalInput: No BooleanSupplier given for simulation, falling back to value found on Glass.");
            return super.get();
        }
    }

    public void setSimSupplier(Optional<BooleanSupplier> simSupplier) {
        this.simSupplier = simSupplier;
    }

    public Optional<BooleanSupplier> getSimSupplier() {
        return simSupplier;
    }
}
