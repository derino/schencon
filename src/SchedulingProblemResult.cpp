#include "SchedulingProblemResult.h"

SchedulingProblemResult::SchedulingProblemResult(string _name) : name(_name)
{
  
}

SchedulingProblemResult::~SchedulingProblemResult()
{
}

void SchedulingProblemResult::print(ostream& out)
{
  out << ">>>> RESULTS <<<<" << endl;
  out << "Problem: " << name << endl;

  if(isFeasibleASAP)
    out << "Min. peak point by ASAP: (" << minPeakASAP << ", " << minCostASAP << ")" << endl;
  else
    out << "Infeasible solution by ASAP" << endl;

  if(isFeasibleILP) 
    {
      out << "Min. Peak Pareto by ILP: (" << minPeakOfPeakParetoILP << ", " << minCostOfPeakParetoILP << ")" << endl;
    }
  else
    out << "Infeasible solution by ILP" << endl;

  if(isFeasibleLee)
    out << "Min. peak point by Lee: (" << minPeakLee << ", " << minCostLee << ")" << endl;
  else
    out << "Infeasible solution by Lee" << endl;

  if(isFeasiblePACMFG_wLee_Pmax)
    out << "Solution by PACMFG with Pmax = Lee: (" << minPeakPACMFG_wLee_Pmax << ", " << minCostPACMFG_wLee_Pmax << ")" << endl;
  else
    out << "Infeasible solution by PACMFG with Pmax = Lee" << endl;

  if(isFeasiblePACMGG_wLee_Pmax)
    out << "Solution by PACMGG with Pmax = Lee: (" << minPeakPACMGG_wLee_Pmax << ", " << minCostPACMGG_wLee_Pmax << ")" << endl;
  else
    out << "Infeasible solution by PACMGG with Pmax = Lee" << endl;

  if(isFeasiblePACMGF_wLee_Pmax)
    out << "Solution by PACMGF with Pmax = Lee: (" << minPeakPACMGF_wLee_Pmax << ", " << minCostPACMGF_wLee_Pmax << ")" << endl;
  else
    out << "Infeasible solution by PACMGF with Pmax = Lee" << endl;
  

  // not needed
  // if(isFeasiblePACMFG_wILP_Pmax)
  //   out << "Solution by PACMFG with Pmax = ILP: (" << minPeakPACMFG_wILP_Pmax << ", " << minCostPACMFG_wILP_Pmax << ")" << endl;
  // else
  //   out << "Infeasible solution by PACMFG with Pmax = ILP" << endl;



  // Cost min
  if(isFeasibleILP_costmin) 
    {
      out << "Min. Cost Pareto by ILP: (" << minPeakOfCostParetoILP << ", " << minCostOfCostParetoILP << ")" << endl;
    }
  else
    out << "Infeasible solution by ILP" << endl;

  if(isFeasiblePACMFG_costmin)
    out << "Solution by PACMFG: (" << minPeakPACMFG_costmin << ", " << minCostPACMFG_costmin << ")" << endl;
  else
    out << "Infeasible solution by PACMFG" << endl;

  if(isFeasiblePACMGG_costmin)
    out << "Solution by PACMGG: (" << minPeakPACMGG_costmin << ", " << minCostPACMGG_costmin << ")" << endl;
  else
    out << "Infeasible solution by PACMGG" << endl;

  if(isFeasiblePACMGF_costmin)
    out << "Solution by PACMGF: (" << minPeakPACMGF_costmin << ", " << minCostPACMGF_costmin << ")" << endl;
  else
    out << "Infeasible solution by PACMGF" << endl;


}

std::ostream& operator<<(std::ostream& out, SchedulingProblemResult& pr)
{
  pr.print(out);
  return out;
}
