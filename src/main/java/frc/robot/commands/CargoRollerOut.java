package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;

public class CargoRollerOut extends Command {

    public CargoRollerOut() {

        super();
    }

    @Override
    protected void execute() {

        OI.cargo.deposit();
    }

    @Override
    protected boolean isFinished() {

        return false;
    }
}