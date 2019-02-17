package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;

public class CargoRollerIn extends Command {

    public CargoRollerIn() {

        super();
    }

    @Override
    protected void execute() {
        
        OI.cargo.intake();
    }

    @Override
    protected boolean isFinished() {

        return false;
    }
}