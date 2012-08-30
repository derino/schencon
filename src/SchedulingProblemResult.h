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
#ifndef SCHEDULINGPROBLEMRESULT_H
#define SCHEDULINGPROBLEMRESULT_H

#include <iostream>
using namespace std;

#include "SchedulingProblem.h"

class SchedulingProblemResult
{
 public:
  SchedulingProblemResult(SchedulingProblem*);
  ~SchedulingProblemResult();
  void print(ostream&);

  SchedulingProblem* sp;

  bool isFeasibleASAP;
  double minPeakASAP;
  double minCostASAP;

  bool isFeasibleILP;
  double minPeakOfPeakParetoILP;
  double minCostOfPeakParetoILP;

  bool isFeasibleLee;
  double minPeakLee;
  double minCostLee;

  bool isFeasibleILP_wLee_Pmax;
  double minPeakILP_wLee_Pmax;
  double minCostILP_wLee_Pmax;

  bool isFeasiblePACMFG_wLee_Pmax;
  double minPeakPACMFG_wLee_Pmax;
  double minCostPACMFG_wLee_Pmax;

  bool isFeasiblePACMGG_wLee_Pmax;
  double minPeakPACMGG_wLee_Pmax;
  double minCostPACMGG_wLee_Pmax;

  bool isFeasiblePACMGF_wLee_Pmax;
  double minPeakPACMGF_wLee_Pmax;
  double minCostPACMGF_wLee_Pmax;

  // not used at the moment
  bool isFeasiblePACMFG_wILP_Pmax;
  double minPeakPACMFG_wILP_Pmax;
  double minCostPACMFG_wILP_Pmax;


  // Cost min   
  bool isFeasibleILP_costmin;
  double minCostOfCostParetoILP;
  double minPeakOfCostParetoILP;

  bool isFeasiblePACMFG_costmin;
  double minPeakPACMFG_costmin;
  double minCostPACMFG_costmin;

  bool isFeasiblePACMGG_costmin;
  double minPeakPACMGG_costmin;
  double minCostPACMGG_costmin;

  bool isFeasiblePACMGF_costmin;
  double minPeakPACMGF_costmin;
  double minCostPACMGF_costmin;
};

std::ostream& operator<<(std::ostream&, SchedulingProblemResult&);

#endif
