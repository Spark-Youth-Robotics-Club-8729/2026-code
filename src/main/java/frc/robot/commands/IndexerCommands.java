package main.java.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.Constants;
public class IndexerCommands {
    public static Command feed(IndexerSubsystem indexer){
        return indexer.run(()->indexer.rotate()).withName("indexerfeed");
    }

    public static Command unfeed(IndexerSubsystem indexer){
        return indexer.run(()->indexer.backwards()).withName("indexerreverse");
    }

    public static Command stop(IndexerSubsystem indexer){
        return indexer.runOnce(()->indexer.stoprotate()).withName("Indexerstop");
    }

}
