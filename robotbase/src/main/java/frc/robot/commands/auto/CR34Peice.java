package frc.robot.commands.auto;

public class CR34Peice extends AutoBase {

  public CR34Peice() {
    // do NOT start with this one. The other auto is the one to tune first.
    super("Cage Right 3 | 2 P", "CR3ToBranchF");
    makeAutoFromSegments(
      "branchFToRedS",
      "RedSToBranchE",
      "branchEToRedS",
      "RedSToBranchD"/*,
      "branchDToRedS",
       "RedSToBranchC",
       "branchCToRedS"*/
    );
  }
}
