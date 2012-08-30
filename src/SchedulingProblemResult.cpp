#include "SchedulingProblemResult.h"

SchedulingProblemResult::SchedulingProblemResult(SchedulingProblem* _sp) : sp(_sp)
{
  isFeasibleASAP = false;
  isFeasibleILP = false;
  isFeasibleLee = false;
  isFeasiblePACMFG_wLee_Pmax = false;
  isFeasiblePACMGG_wLee_Pmax = false;
  isFeasiblePACMGF_wLee_Pmax = false;
  // not used at the moment
  isFeasiblePACMFG_wILP_Pmax = false;

  // Cost min   
  isFeasibleILP_costmin = false;
  isFeasiblePACMFG_costmin = false;
  isFeasiblePACMGG_costmin = false;
  isFeasiblePACMGF_costmin = false;
}

SchedulingProblemResult::~SchedulingProblemResult()
{
}

void SchedulingProblemResult::print(ostream& out)
{
  out << ">>>> RESULTS <<<<" << endl;
  out << "Problem: " << sp->name() << endl;

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

  if(isFeasibleILP_wLee_Pmax)
    out << "Solution by ILP with Pmax = Lee: (" << minPeakILP_wLee_Pmax << ", " << minCostILP_wLee_Pmax << ")" << endl;
  else
    out << "Infeasible solution by ILP with Pmax = Lee" << endl;

  if(isFeasiblePACMFG_wLee_Pmax)
    out << "Solution by PACMFG with Pmax = Lee: (" << minPeakPACMFG_wLee_Pmax << ", " << minCostPACMFG_wLee_Pmax << ")" << endl;
  else
    out << "Infeasible solution by PACMFG with Pmax = Lee" << endl;

  if(isFeasiblePACMGG_wLee_Pmax)
    out << "Solution by PACMGG with Pmax = Lee: (" << minPeakPACMGG_wLee_Pmax << ", " << minCostPACMGG_wLee_Pmax << ")" << endl;
  else
    out << "Infeasible solution by PACMGG with Pmax = Lee" << endl;

#ifdef PACMGF
  if(isFeasiblePACMGF_wLee_Pmax)
    out << "Solution by PACMGF with Pmax = Lee: (" << minPeakPACMGF_wLee_Pmax << ", " << minCostPACMGF_wLee_Pmax << ")" << endl;
  else
    out << "Infeasible solution by PACMGF with Pmax = Lee" << endl;
#endif

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

#ifdef PACMGF
  if(isFeasiblePACMGF_costmin)
    out << "Solution by PACMGF: (" << minPeakPACMGF_costmin << ", " << minCostPACMGF_costmin << ")" << endl;
  else
    out << "Infeasible solution by PACMGF" << endl;
#endif

}

// prints tab-spaced results in a single line to be processed in oocalc later on.
// -1 for infeasible
// PACM: peak-aware cost minimization problem
// CM: cost minimization problem
// Format: #tasks \t Schedule length \t Ratio of jobs with low PAR \t Ratio of preemptable jobs \t ASAP.peak \t ASAP.cost \t PACM.ILP.peak \t PACM.ILP.cost \t PACM.Lee.peak \t PACM.Lee.cost \t PACM.ILP@Lee.peak \t PACM.ILP@Lee.cost \t PACM.FG.peak \t PACM.FG.cost \t PACM.GG.peak \t PACM.GG.cost \t PACM.GF.peak \t PACM.GF.cost \t CM.ILP.peak \t CM.ILP.cost \t CM.FG.peak \t CM.FG.cost \t CM.GG.peak \t CM.GG.cost \t CM.GF.peak \t CM.GF.cost
std::ostream& operator<<(std::ostream& out, SchedulingProblemResult& pr)
{
  //  pr.print(out);
  out << pr.sp->N() << "\t" << pr.sp->L() << "\t" << pr.sp->LPAR() << "\t" << pr.sp->PrR() << "\t";

  if(pr.isFeasibleASAP)
    out << pr.minPeakASAP << "\t" << pr.minCostASAP << "\t";
  else
    out << -1 << "\t" << -1 << "\t";

  if(pr.isFeasibleILP) 
    {
      out << pr.minPeakOfPeakParetoILP << "\t" << pr.minCostOfPeakParetoILP << "\t";
    }
  else
    out << -1 << "\t" << -1 << "\t";

  if(pr.isFeasibleLee)
    out << pr.minPeakLee << "\t" << pr.minCostLee << "\t";
  else
    out << -1 << "\t" << -1 << "\t";

  if(pr.isFeasibleILP_wLee_Pmax)
    out << pr.minPeakILP_wLee_Pmax << "\t" << pr.minCostILP_wLee_Pmax << "\t";
  else
    out << -1 << "\t" << -1 << "\t";

  if(pr.isFeasiblePACMFG_wLee_Pmax)
    out << pr.minPeakPACMFG_wLee_Pmax << "\t" << pr.minCostPACMFG_wLee_Pmax << "\t";
  else
    out << -1 << "\t" << -1 << "\t";

  if(pr.isFeasiblePACMGG_wLee_Pmax)
    out << pr.minPeakPACMGG_wLee_Pmax << "\t" << pr.minCostPACMGG_wLee_Pmax << "\t";
  else
    out << -1 << "\t" << -1 << "\t";

  if(pr.isFeasiblePACMGF_wLee_Pmax)
    out << pr.minPeakPACMGF_wLee_Pmax << "\t" << pr.minCostPACMGF_wLee_Pmax << "\t";
  else
    out << -1 << "\t" << -1 << "\t";
  

  // not needed
  // if(isFeasiblePACMFG_wILP_Pmax)
  //   out << "Solution by PACMFG with Pmax = ILP: (" << minPeakPACMFG_wILP_Pmax << ", " << minCostPACMFG_wILP_Pmax << ")" << endl;
  // else
  //   out << "Infeasible solution by PACMFG with Pmax = ILP" << endl;



  // Cost min
  if(pr.isFeasibleILP_costmin) 
    {
      out << pr.minPeakOfCostParetoILP << "\t" << pr.minCostOfCostParetoILP << "\t";
    }
  else
    out << -1 << "\t" << -1 << "\t";

  if(pr.isFeasiblePACMFG_costmin)
    out << pr.minPeakPACMFG_costmin << "\t" << pr.minCostPACMFG_costmin << "\t";
  else
    out << -1 << "\t" << -1 << "\t";

  if(pr.isFeasiblePACMGG_costmin)
    out << pr.minPeakPACMGG_costmin << "\t" << pr.minCostPACMGG_costmin << "\t";
  else
    out << -1 << "\t" << -1 << "\t";

  if(pr.isFeasiblePACMGF_costmin)
    out << pr.minPeakPACMGF_costmin << "\t" << pr.minCostPACMGF_costmin << "\t";
  else
    out << -1 << "\t" << -1 << "\t";

  return out;
}
