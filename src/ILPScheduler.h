/** 
  * Copyright (c) 2012, Universita' della Svizzera Italiana,
  * All rights reserved.
  * 
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  * 
  * 1. Redistributions of source code must retain the above copyright notice, this list 
  * of conditions and the following disclaimer.
  * 
  * 2. Redistributions in binary form must reproduce the above copyright notice, this 
  * list of conditions and the following disclaimer in the documentation and/or other 
  * materials provided with the distribution.
  * 
  * 3. Neither the name of Universita' della Svizzera Italiana nor the names of its 
  * contributors may be used to endorse or promote products derived from this software 
  * without specific prior written permission.
  * 
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
  * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
  * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
  * SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
  * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * 
  * The licenses for the third party libraries used in this software can be found in 
  * the folders of the third party libraries.
  *
  * Author(s): Onur Derin <oderin@users.sourceforge.net>
  * 
  */
#ifndef ILPSCHEDULER_H
#define ILPSCHEDULER_H

#include <ilcplex/ilocplex.h>
#include <sstream>
#include <assert.h>

#include "SchedulingProblem.h"
#include "SchedulingSolution.h"

ILOSTLBEGIN

enum ObjectiveType {PEAK, COST};
enum GoalType {MINIMIZE, MAXIMIZE};
enum RelationType {LEQ, EQ, GEQ, NONE}; // NONE is used for single objective problem

class ILPScheduler
{
 public:
  ILPScheduler(SchedulingProblem* _sp);
  ILPScheduler(double timeLimit, double gapLimit, SchedulingProblem* _sp);
  ~ILPScheduler();
//  LPMapper(RemappingProblem* _rmp, /*MappingProblemType _mpt,*/ vector<int>* _vFaultyNodes);
//  LPMapper(double timeLimit, double gapLimit, RemappingProblem* _rmp, vector<int>* _vFaultyNodes);

  // single objective
//  MappingSolution* map(GoalType gType, ObjectiveType oType);
  SchedulingSolution* schedule(GoalType gType, ObjectiveType oType);

  // multi objective
//  MappingSolution* map(GoalType gType, ObjectiveType oType, RelationType rType, double thresholdConstraint);
  // solves the multi-objective problem
//  MappingSolutionSet* epsilonConstraintMethod(double delta);

 private:
  double timeLimit; // in seconds
  double gapLimit; // in percentages
  SchedulingProblem* sp;

/**
// timeLimit = -1: cplex.solve() runs until the optimal solution is found. 
// timeLimit = 2 sec: cplex.solve() runs until a specified time (2 sec) is reached. 
// timeLimit = 2 sec and gapLimit = 10 (%): cplex.solve() runs at least until a specified time (2 sec), 
// if the relative gap to the best solution is greater than the given acceptable gap value (10%), 
// it continues until the gap is smaller than the acceptable value (10%).
*/


 protected:
//  void routingConstraints(MappingProblem* p, IloModel model, IloBoolVarArray x, IloBoolVarArray y, IloRangeArray con);

//  void taskMappingConstraints(MappingProblem* mp, IloModel model, IloBoolVarArray x, IloRangeArray c);

//  void faultyCoreConstraints (MappingProblem* mp, IloModel model, IloBoolVarArray x, IloRangeArray c);

//  void taskMovingConstraints(MappingProblem* mp, IloModel model, IloBoolVarArray x, IloRangeArray c);

//  void communicationMappingConstraints(MappingProblem* mp, IloModel model, IloBoolVarArray y, IloRangeArray c);

//  void capacityConstraints(MappingProblem* mp, IloModel model, IloBoolVarArray y, IloRangeArray c);

//  void compObjective (GoalType gType, MappingProblem* mp, IloModel model, IloBoolVarArray x, IloNumVarArray Tn);

//  void compObjectiveAsConstraint (/*GoalType gType,*/ RelationType rType, double compConstraint, MappingProblem* mp, IloModel model, IloBoolVarArray x, IloNumVarArray Tn, IloRangeArray c);

//  void commObjective (GoalType gType, MappingProblem* mp, IloModel model, IloBoolVarArray y);

//  void commObjectiveAsConstraint (RelationType rType, double commConstraint, MappingProblem* mp, IloModel model, IloBoolVarArray y, IloRangeArray c);

//  MappingSolution* optimize(MappingProblem* mp, IloModel model, IloBoolVarArray x, IloBoolVarArray y, IloNumVarArray Tn, IloRangeArray routingCons, IloRangeArray taskMappingCons, IloRangeArray taskMovingCons, IloRangeArray communicationMappingCons, IloRangeArray capacityCons, IloRangeArray objectiveCons);
};

#endif
