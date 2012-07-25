#ifndef SCHEDULINGMOMHSOLUTION_H
#define SCHEDULINGMOMHSOLUTION_H

//#include <iostream>
//#include <iomanip>

#include <momh/momhconstrainedsolution.h>
#include "Signal.h"
#include <vector>

using namespace std;

//template <typename T >
//class Signal;

class SchedulingMOMHSolution : public TDPAMConstrainedSolution/*TMOMHSolution*/
{
 public:
	SchedulingMOMHSolution(double peak, double cost);
	SchedulingMOMHSolution(SchedulingMOMHSolution&);
	~SchedulingMOMHSolution();

//	SchedulingMOMHSolution(SchedulingingMOMHSolution& schedulingMOMHSolution);

  /** Constructs new solution by recombination of Parent1 and Parent 2 */
//  MappingMOMHSolution(MappingMOMHSolution& parent1, MappingMOMHSolution& parent2);

  /** Mutates the solution */
//  virtual void Mutate();

  virtual TCompare Compare(TSolution & Solution) { return TDPAMConstrainedSolution::Compare(Solution); }

  /** Find next local move to be performed in local search type
   *    algorithms */
  //virtual void FindLocalMove();
  
  /**    Rejects the local move found by FindLocalMove () method */
  //virtual void RejectLocalMove();

  // int** getXnt();
  // int** getYpe();
  // /*double* getTn();*/
  // double getCommCost();
  // //void setCommCost(double);
  // double getCompCost();

  double getCost();
  double getPeak();
  void setTab(vector<Signal<int>*>* tab);
  vector<Signal<int>*>* getTab();


  // /*IloNum*/double getSolutionTime();
  // void setSolutionTime(double/*IloNum*/);
  // MappingSolutionStatus getStatus();
  // //  IloAlgorithm::Status getStatus();
  // void setStatus(MappingSolutionStatus);
  // //  void setStatus(IloAlgorithm::Status);
  
  // bool dominatesAbsolute(MappingSolution*);
  // bool dominatesEqual(MappingSolution*);
  void print(ostream&);


 private:
  double _peak;
  double _cost;
  vector<Signal<int>*>* _tab;
  //  void initializeYpe();
//  void calculateConstraintValues();
//
//  void Mutate_UnitePeers();
//  void Mutate_BalanceLoad();
//  void Mutate_MoveTaskToNeighborNode();
//  void migrate(int t, int newNode);
//  void fixRedundantsMapping(RAMappingProblem* rmp);

  //  MappingProblem* mp;
  //int** Xnt;
  //int** Ype;
  ////double* Tn;
  ////double commCost;
  //MappingSolutionStatus status;
  ////  IloAlgorithm::Status status;
  ///*IloNum*/double solutionTime;
};

std::ostream& operator<<(std::ostream&, SchedulingMOMHSolution&);

//std::ostream& operator<<(std::ostream&, MappingSolutionStatus& msStatus);

#endif
