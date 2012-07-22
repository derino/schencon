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
#include "ILPScheduler.h"

ILPScheduler::ILPScheduler(SchedulingProblem* _sp) : timeLimit(-1), gapLimit(100), sp(_sp)
{
  // timeLimit = -1; => this is important. It means limiting is not active.
  // gapLimit = 100; => doesn't mean anything as long as limiting is not active.
}

ILPScheduler::ILPScheduler(double _timeLimit, double _gapLimit, SchedulingProblem* _sp) : timeLimit(_timeLimit), gapLimit(_gapLimit), sp(_sp)
{
}

ILPScheduler::~ILPScheduler()
{
  
}


/** J.-F. Berube et al. / European Journal of Operational Research 194 (2009) 39–50 
    An exact epsilon-constraint method for bi-objective combinatorial optimization problems:
    Application to the Traveling Salesman Problem with Profits
*/

// MappingSolutionSet* ILPScheduler::epsilonConstraintMethod(double delta)
// {
//   MappingSolutionSet* mss = new MappingSolutionSet();
//   MappingSolution* ms;
  
//   cout << "running map(MINIMIZE, COMMUNICATION)" << endl;
//   ms = map(MINIMIZE, COMMUNICATION);
//   double eps = ms->getCompCost();
//   cout << "Solution found: " << *ms << endl;
//   //ms->print(cout);
//   mss->add(ms);
  
//   ms = map(MINIMIZE, COMPUTATION);
//   double compIdeal = ms->getCompCost();
//   delete ms;
    
//   do
//     {
//       cout << "running map(MINIMIZE, COMMUNICATION, LEQ, " << eps << ")" << endl;
//       ms = map(MINIMIZE, COMMUNICATION, LEQ, eps);
//       //a= ms->getCommCost();
//       cout << "Solution found: " << *ms << endl;
//       mss->add(ms);
      
//       if(ms->getStatus() == OPTIMAL_SOLUTION/*IloAlgorithm::Optimal*/ || ms->getStatus() == FEASIBLE_SOLUTION)
// 	{
// 	  //ms = map(MINIMIZE, COMPUTATION, EQ, a);
// 	  //mss->add(ms);
// 	  //ms->print(cout);
// 	  eps= ms->getCompCost();      
// 	  eps=eps-delta; // delta = 1 when we have exe. times as integers. not realistic. // mpeg2 has delta=0.01;
// 	}
//     }
//   while (eps >= compIdeal); //(ms->getStatus() == OPTIMAL_SOLUTION/*IloAlgorithm::Optimal*/ || ms->getStatus() == FEASIBLE_SOLUTION /*feasible is the type of the time-limited solutions*/); 
//   // ideally we should create a grid and solve for a desired number of times instead of 0.01 steps.
  

//   return mss;
  
//   /*
//   // 1. example usage of map(): minimize communication such that comp. cost <= 30.0
//   ms = map(MINIMIZE, COMPUTATION_AS_CONSTRAINT, LEQ, 30.0);
//   mss->add(ms);
//   return mss;
//   */
//   /*
//   // 2. example usage of map(): minimize computation such that comm. cost <= 215.4
//   ms = map(MINIMIZE, COMMUNICATION_AS_CONSTRAINT, LEQ, 215.4);
//   mss->add(ms);
//   return mss;
//   */
//   /*
//   // 3. example usage of map(): minimize computation such that comm. cost = 0.0
//   ms = map(MINIMIZE, COMMUNICATION_AS_CONSTRAINT, EQ, 0.0);
//   mss->add(ms);
//   return mss;
//   */
//   /*
//     // MAXIMIZE doesn't work! Use only MINIMIZE!
//   ms = map(MAXIMIZE, COMPUTATION_AS_CONSTRAINT, GEQ, 20.0);
//   mss->add(ms);
//   return mss;
    
//   ms = map(MAXIMIZE, COMMUNICATION_AS_CONSTRAINT, LEQ, 105.0);
//   mss->add(ms);
//   return mss;
//   */
// }

SchedulingSolution*
ILPScheduler::schedule(GoalType gType, ObjectiveType oType)
{
  return schedule(gType, oType, NONE, 0.0);
}

SchedulingSolution*
ILPScheduler::schedule(GoalType gType, ObjectiveType oType, RelationType rType, double thresholdConstraint)
{
   SchedulingSolution* ss = NULL;

    IloEnv env;
    try {
      // the ILP model
      IloModel model(env);
     
      // define scheduling decision variables
      IloBoolVarArray x(env);
      //      int z = 0;
      for(int n=0; n<sp->N(); n++)
	{
	  int M = sp->J()->at(n)->getL()->size();
	  //	  cout << ">>>>>> M:" << M << endl;
	  for(int m=0; m<M; m++)
	    {
	      for(int l=0; l<sp->L(); l++)
		{
		  x.add(IloBoolVar(env));
		  std::stringstream n_str, m_str, l_str;
		  n_str << n+1;
		  m_str << m+1;
		  l_str << l+1;
		  //		  cout << z << ": " << sumM(n)*sp->L()+m*sp->L()+l << endl;
		  //		  z++;
		  x[sumM(n)*sp->L()+m*sp->L()+l].setName( ("x_nml("+ n_str.str() + "," + m_str.str() + "," + l_str.str() + ")").c_str() );
		} // end l
	    } // end m
	} // end n

      // define t decision variables
      IloNumVarArray t(env);
      for(int n=0; n<sp->N(); n++)
	{
	  int M = sp->J()->at(n)->getL()->size();
	  for(int m=0; m<M; m++)
	    {
	      t.add(IloNumVar(env));
	      std::stringstream n_str;
	      std::stringstream m_str;
	      n_str << n+1;
	      m_str << m+1;
	      t[sumM(n)+m].setName( ("t("+ n_str.str() + "," + m_str.str() + ")").c_str() );
	    }
        }
     
      // define constraints
      // 1- respect Pmax
      IloRangeArray PMaxCons(env);
      PMaxConstraints(sp, model, x, PMaxCons);

      // 2- t variables
      IloRangeArray tCons(env);
      tConstraints(sp, model, x, t, tCons);

      // 3- respect arrivals
      IloRangeArray arrivalCons(env);
      arrivalConstraints(sp, model, x, t, arrivalCons);

      // 4- respect deadlines
      IloRangeArray deadlineCons(env);
      deadlineConstraints(sp, model, x, t, deadlineCons);

      // 5- schedule each job piece once
      IloRangeArray scheduledOnceCons(env);
      scheduledOnceConstraints(sp, model, x, scheduledOnceCons);

      // 6- job piece dependencies
      IloRangeArray precedenceCons(env);
      precedenceConstraints(sp, model, t, precedenceCons);

      // 7- nonpreamptable job
      IloRangeArray nonpreamptableCons(env);
      nonpreamptableConstraints(sp, model, t, nonpreamptableCons);

     // 8- oType as objective, other as constraint. 
     //    if rType is NONE, then it is a single objective problem.
     IloRangeArray objectiveCons(env);
     if (rType == NONE && oType == COST) // single cost objective
       // define cost as the objective
       costObjective(gType, sp, model, x);
     else if (rType == NONE && oType == PEAK) // single peak objective
       // define peak as the objective
       peakObjective(gType, sp, model, x);
//      else if (oType == COMMUNICATION) // multi obj.
//        {
// 	 // define computation cost as a constraint
// 	 compObjectiveAsConstraint (/*gType,*/ rType, thresholdConstraint, mp, model, x, Tn, objectiveCons);
// 	 // define communication cost as the objective
// 	 commObjective(gType, mp, model, y);
//        }
//      else if (oType == COMPUTATION) // multi obj.
//        {
// 	 // define communication cost as a constraint
// 	 commObjectiveAsConstraint (rType, thresholdConstraint, mp, model, y, objectiveCons);
// 	 // define computation cost as the objective
// 	 compObjective(gType, mp, model, x, Tn);
//        }
     else
       {
	 cerr << "Invalid use of the schedule() method!" << endl;
	 throw(-1);
       }
      // May CPLEX solve the problem!
     ss = optimize(sp, model, x, t, PMaxCons, tCons, arrivalCons, deadlineCons, scheduledOnceCons, precedenceCons, nonpreamptableCons, objectiveCons);
    }
    catch (IloException& e) {
      cerr << "Concert exception caught: " << e << endl;
//      ms = new MappingSolution(mp);
//      ms->setStatus(ERROR_SOLUTION/*IloAlgorithm::Error*/);
    }
    catch (InfeasibleSolutionException& ise) {
	cerr << "TODO" << endl;
//        ms = ise.getMappingSolution();
      }
    catch (...) {
      cerr << "Unknown exception caught" << endl;
//      ms = new MappingSolution(mp);
//      ms->setStatus(ERROR_SOLUTION/*IloAlgorithm::Error*/);
    }
    
    env.end();
   
    return ss;
}


// Spend at least timeLimit sec. on optimization, but once
// this limit is reached, quit as soon as the solution is acceptable

ILOMIPINFOCALLBACK5(timeLimitCallback,
                    IloCplex, cplex,
                    IloBool,  aborted,
                    IloNum,   timeStart,
                    IloNum,   timeLimit,
                    IloNum,   acceptableGap)
{
   if ( !aborted  &&  hasIncumbent() ) {
      IloNum gap = 100.0 * getMIPRelativeGap();
      IloNum timeUsed = cplex.getCplexTime() - timeStart;
//      if ( timeUsed > 1 )  printf ("time used = %g\n", timeUsed);
      if ( timeUsed > timeLimit && gap < acceptableGap ) {
         getEnv().out() << endl
                        << "Good enough solution at "
                        << timeUsed << " sec., gap = "
                        << gap << "%, aborting." << endl;
         aborted = IloTrue;
         abort();
      }
   }
}

SchedulingSolution*
ILPScheduler::optimize(SchedulingProblem* sp, IloModel model, IloBoolVarArray x, IloNumVarArray t, IloRangeArray PMaxCons, IloRangeArray tCons, IloRangeArray arrivalCons, IloRangeArray deadlineCons, IloRangeArray scheduledOnceCons, IloRangeArray precedenceCons, IloRangeArray nonpreamptableCons, IloRangeArray objectiveCons)
{
  SchedulingSolution* ss = NULL; //new MappingSolution(mp);
  IloEnv env = model.getEnv();

  // Optimize the problem and obtain solution.
  IloCplex cplex(model);
  
  cplex.exportModel("problem.lp");
  
  // Turn off CPLEX logging
  cplex.setParam(IloCplex::MIPDisplay, 0);

  // if timeLimit is set then register the time limit callback
  if ( timeLimit != -1 ) {
    cplex.use(timeLimitCallback(env, cplex, IloFalse, cplex.getCplexTime(), timeLimit/*1.0*/, gapLimit/*10.0*/));
  }

  //cplex.resetTime();
   if ( !cplex.solve() ) 
     {
       env.error() << "Failed to optimize LP" << endl;
       //       ms->setSolutionTime( cplex.getTime() );
       //       ms->setStatus(mappingSolutionStatusAdapter(IloAlgorithm::Infeasible)/*IloAlgorithm::Infeasible*/);
       //       throw InfeasibleSolutionException(ms);
     }
//   ms->setSolutionTime( cplex.getTime() );
   cout << "Solution time = " << cplex.getTime() << endl;  

//   ms->setGap(cplex.getMIPRelativeGap()*100);

//   //cout /*env.out()*/ << "Solution status = " << cplex.getStatus() << endl;
//   ms->setStatus( mappingSolutionStatusAdapter(cplex.getStatus())/*cplex.getStatus()*/ );
//   //env.out() << "Solution value  = " << cplex.getObjValue() << endl;

//   /* Not needed anymore. Cost values are calculated by using the X and Y solution values inside MappingSolution class.
//   if(oType == COMPUTATION_AS_CONSTRAINT)
//     ms->setCommCost( cplex.getObjValue() );
//   else // oType == COMMUNICATION_AS_CONSTRAINT
//     ms->setCompCost( cplex.getObjValue() );
//   */
   cout << "Obj. value: " << cplex.getObjValue() << endl;

//   IloNumArray vals(env);
//   cplex.getValues(x, vals);
//   //IloIntArray valsInt(env);// = vals.toIntArray();

//   //  env.out() << "Xnt        = " << endl;
//   for(int i = 0; i < mp->N(); i++)
//     {
//       //  env.out() << "| ";
//       for(int j = 0; j < mp->T(); j++)
// 	{
// 	  //  env.out() << vals[i*mp->T() + j] << "'";
// 	  ms->getXnt()[i][j] = (bool)vals[i*mp->T() + j]; // casting to bool is VERY IMPORTANT!!!
// 	  //env.out() << "'" << ms->getXnt()[i][j] << " ";
// 	}
//       //env.out() << "| " << endl;
//     }

//   IloNumArray vals2(env);
//   cplex.getValues(vals2, y);
//   //env.out() << "Ype        = " << endl;
//   for(int i = 0; i < mp->P(); i++)
//     {
//       //env.out() << "| ";
//       for(int j = 0; j < mp->E(); j++)
// 	{
// 	  //env.out() << vals[i*mp->E() + j];
// 	  ms->getYpe()[i][j] = (bool) vals2[i*mp->E() + j]; // casting to bool is VERY IMPORTANT!!!
// 	}
//       //env.out() << "| " << endl;
//     }

//   /* Not needed anymore. values are calculated inside MappingSolution class.
//   if (!noTn)
//     {
//       cplex.getValues(vals, Tn);
//       //env.out() << "Tn        = " << endl;
//       for(int i = 0; i < mp->N(); i++)
// 	{
// 	  //env.out() << vals[i*mp->E() + j];
// 	  ms->getTn()[i] = vals[i];
// 	}
//     }
//   */

//   // cplex.getSlacks(vals, routingCons);
//   // env.out() << "Slacks        = " << vals << endl;
//   // cplex.getDuals(vals, routingCons);
//   // env.out() << "Duals         = " << vals << endl;
//   // cplex.getSlacks(vals, taskMappingCons);
//   // env.out() << "Slacks        = " << vals << endl;
//   // cplex.getDuals(vals, taskMappingCons);
//   // env.out() << "Duals         = " << vals << endl;
//   // cplex.getSlacks(vals, communicationMappingCons);
//   // env.out() << "Slacks        = " << vals << endl;
//   // cplex.getDuals(vals, communicationMappingCons);
//   // env.out() << "Duals         = " << vals << endl;
//   // cplex.getReducedCosts(vals, x);
//   // env.out() << "Reduced Costs = " << vals << endl;
//   // cplex.getReducedCosts(vals, y);
//   // env.out() << "Reduced Costs = " << vals << endl;

  return ss;
}

// returns the total number of job pieces of jobs until n-th job,
// i.e., excluding n
int ILPScheduler::sumM(int n)
{
  int sum = 0;
  for(int i=0; i<n; i++)
    sum += sp->J()->at(i)->getL()->size();
  return sum;
}

// // To populate by nonzero, we first create the rows, then create the
// // columns, and then change the nonzeros of the matrix 1 at a time.
void
ILPScheduler::PMaxConstraints (SchedulingProblem* sp, IloModel model, IloBoolVarArray x, IloRangeArray c)
{
  IloEnv env = model.getEnv();

  for(int l=0; l < sp->L(); l++)
    {
      c.add( IloRange(env, -IloInfinity, sp->PMax()) );
      std::stringstream l_str;
      l_str << l+1;
      c[l].setName( ("PMax_c(" + l_str.str() + ")").c_str() );

      for(int n=0; n < sp->N(); n++)
	{
	  int M = sp->J()->at(n)->getL()->size();
	  for(int m = 0; m < M; m++)
	    {
	      int nml = sumM(n)*sp->L() + m*sp->L() + l;
	      double l_nm = sp->J()->at(n)->getL()->at(m);
	      c[l].setLinearCoef(x[nml], l_nm);
	    } // end m
	} // end n
    } // end l
    
  model.add(c);

}  // END PMaxConstraints

void
ILPScheduler::tConstraints (SchedulingProblem* sp, IloModel model, IloBoolVarArray x, IloNumVarArray t, IloRangeArray c)
{
  IloEnv env = model.getEnv();

  for(int n=0; n < sp->N(); n++)
    {
      int M = sp->J()->at(n)->getL()->size();
      for (int m = 0; m < M; m++)
	{
	  c.add( IloRange(env, 0.0, 0.0) );
	  //std::stringstream nm_str;
	  //nm_str << sumM(n)+m+1;
	  std::stringstream n_str;
	  n_str << n+1;
	  std::stringstream m_str;
	  m_str << m+1;
	  c[sumM(n)+m].setName( ("t_c(" + n_str.str() + "," + m_str.str() + ")").c_str() );
	  c[sumM(n)+m].setLinearCoef(t[sumM(n)+m], -1.0);

	  for (int l = 0; l < sp->L(); l++)
	    {
	      int nml = sumM(n)*sp->L() + m*sp->L() + l;
	      c[sumM(n)+m].setLinearCoef(x[nml], l+1);
	    } // end l
	} // end m
    } // end n

  model.add(c);
} // END tConstraints

void
ILPScheduler::arrivalConstraints (SchedulingProblem* sp, IloModel model, IloBoolVarArray x, IloNumVarArray t, IloRangeArray c)
{
  IloEnv env = model.getEnv();

  for(int n=0; n < sp->N(); n++)
    {
      c.add( IloRange(env, sp->J()->at(n)->getA(), IloInfinity) );
      std::stringstream n_str;
      n_str << n+1;
      c[n].setName( ("arr_c(" + n_str.str() + ")").c_str() );

      c[n].setLinearCoef(t[sumM(n)], 1.0);
    }

  model.add(c);
} // END arrivalConstraints

void
ILPScheduler::deadlineConstraints (SchedulingProblem* sp, IloModel model, IloBoolVarArray x, IloNumVarArray t, IloRangeArray c)
{
  IloEnv env = model.getEnv();

  for(int n=0; n < sp->N(); n++)
    {
      c.add( IloRange(env, -IloInfinity, sp->J()->at(n)->getD()-1 ) ); // -1 is needed for saying "less than" and NOT "less than or equal"
      std::stringstream n_str;
      n_str << n+1;
      c[n].setName( ("dline_c(" + n_str.str() + ")").c_str() );
      int M = sp->J()->at(n)->getL()->size();
      c[n].setLinearCoef(t[sumM(n)+M-1], 1.0);
    }

  model.add(c);
} // END deadlineConstraints


void 
ILPScheduler::scheduledOnceConstraints (SchedulingProblem* sp, IloModel model, IloBoolVarArray x, IloRangeArray c)
{
  IloEnv env = model.getEnv();

  for(int n=0; n < sp->N(); n++)
    {
      int M = sp->J()->at(n)->getL()->size();
      for(int m = 0; m < M; m++)
	{
	  c.add( IloRange(env, 1.0, 1.0) );
	  std::stringstream n_str;
	  n_str << n+1;
	  std::stringstream m_str;
	  m_str << m+1;
	  c[sumM(n)+m].setName( ("once_c(" + n_str.str() + "," + m_str.str() + ")").c_str() );

	  for(int l=0; l < sp->L(); l++)
	    {
	      int nml = sumM(n)*sp->L() + m*sp->L() + l;
	      c[sumM(n)+m].setLinearCoef(x[nml], 1.0);
	    } // end l
	} // end m
    } // end n
    
  model.add(c);

}  // END scheduledOnceConstraints

void
ILPScheduler::precedenceConstraints (SchedulingProblem* sp, IloModel model, IloNumVarArray t, IloRangeArray c)
{
  IloEnv env = model.getEnv();

  for(int n=0; n < sp->N(); n++)
    {
      int M = sp->J()->at(n)->getL()->size();
      for (int m = 0; m < M-1; m++)
	{
	  c.add( IloRange(env, 1.0, IloInfinity) );
	  std::stringstream n_str;
	  n_str << n+1;
	  std::stringstream m_str;
	  m_str << m+1;
	  std::stringstream mpp_str;
	  mpp_str << m+2;
	  c[sumM(n)-n+m].setName( ("prec_c(" + n_str.str() + "," + m_str.str() + "_" + mpp_str.str() + ")").c_str() );
	  c[sumM(n)-n+m].setLinearCoef(t[sumM(n)+m], -1.0);
	  c[sumM(n)-n+m].setLinearCoef(t[sumM(n)+m+1], 1.0);
	} // end m
    } // end n

  model.add(c);
} // END precedenceConstraints

// returns the total number of job pieces of non-preamptable jobs until n-th job,
// i.e., excluding n
int ILPScheduler::sumMofNonPr(int n)
{
  int sum = 0;
  for(int i=0; i<n; i++)
    {
      if( !( sp->J()->at(i)->getPr() ) )
	sum += sp->J()->at(i)->getL()->size();
    }
  return sum;
}

// returns the number of non-preamptable jobs until n-th job,
// i.e., excluding n
int ILPScheduler::sumOfNonPr(int n)
{
  int count = 0;
  for(int i=0; i<n; i++)
    {
      if( !( sp->J()->at(i)->getPr() ) )
	count += 1;
    }
  return count;
}

void
ILPScheduler::nonpreamptableConstraints (SchedulingProblem* sp, IloModel model, IloNumVarArray t, IloRangeArray c)
{
  IloEnv env = model.getEnv();

  for(int n=0; n < sp->N(); n++)
    {
      if( sp->J()->at(n)->getPr() )
	continue;
      int M = sp->J()->at(n)->getL()->size();
      for (int m = 0; m < M-1; m++)
	{
	  c.add( IloRange(env, 1.0, 1.0) );
	  std::stringstream n_str;
	  n_str << n+1;
	  std::stringstream m_str;
	  m_str << m+1;
	  std::stringstream mpp_str;
	  mpp_str << m+2;
	  c[sumMofNonPr(n)-sumOfNonPr(n)+m].setName( ("pr_c(" + n_str.str() + "," + m_str.str() + "_" + mpp_str.str() + ")").c_str() );
	  c[sumMofNonPr(n)-sumOfNonPr(n)+m].setLinearCoef(t[sumM(n)+m], -1.0);
	  c[sumMofNonPr(n)-sumOfNonPr(n)+m].setLinearCoef(t[sumM(n)+m+1], 1.0);
	} // end m
    } // end n

  model.add(c);
} // END nonpreamptableConstraints



// void 
// ILPScheduler::compObjectiveAsConstraint (/*GoalType gType,*/ RelationType rType, double compConstraint, MappingProblem* mp, IloModel model, IloBoolVarArray x, IloNumVarArray Tn, IloRangeArray c)
// {
//   IloEnv env = model.getEnv();

//   //fprintf(file,"\n\\Obj 2\n\n");

  

//   // Tnt = Mnc Tct
//   double** Tnt = new double*[mp->N()];
//   for(int i=0; i<mp->N(); i++)
//     Tnt[i] = new double[mp->T()];

//   for(int i=0; i<mp->N(); i++){
//     for(int j=0; j<mp->T(); j++){
//       Tnt[i][j] = 0.0;
//     }
//   }

//   for(int i=0; i<mp->N(); i++){
//     for(int j=0; j<mp->T(); j++){
//       for(int k=0; k<mp->CT(); k++){
// 	Tnt[i][j] = Tnt[i][j] + mp->Ttc()[j][k] * mp->Mnc()[i][k] ;
//       }
//     }
//   }

//   // define constraints, first Tn = Ttn . Xtn (elementwise and summed)
//   for(int i=0; i < mp->N(); i++)
//     {
//       c.add(IloRange(env, 0.0, 0.0));
//       std::stringstream i_str;
//       i_str << i+1;
//       c[i].setName( ("computation_Tn_c" + i_str.str()).c_str() );

//       c[i].setLinearCoef( Tn[i], -1.0 );
//       //fprintf(file,"c%d: -B%d +",N*E+E+T+T+i+1,i+1);
//       for(int j=0; j<mp->T(); j++)
// 	{
// 	  c[i].setLinearCoef( x[j + i*mp->T()], Tnt[i][j] );
// 	  //if(j==mp->T()-1)
// 	  //  fprintf(file,"%lf x%d =0\n",Ttn[i][j],j+1+i*T);
// 	  //else
// 	  //  fprintf(file,"%lf x%d +",Ttn[i][j],j+1+i*T);
// 	}
//     }
//   //fprintf(file,"\n");

//   // define T = max (Tn) as a decision variable
//   IloNumVar T(env);
//   if(rType == LEQ)
//     T.setBounds(-IloInfinity, compConstraint);
//   else if(rType == EQ)
//     T.setBounds(compConstraint, compConstraint);
//   else // rType == GEQ
//     T.setBounds(compConstraint, IloInfinity);
//   T.setName("T");

//   // define constraints - second T - Tn >= 0
//   for(int i=0; i < mp->N(); i++)
//     {
//       //      if ( gType == MINIMIZE)
// 	c.add(IloRange(env, 0.0, IloInfinity));
// 	//else // gType == MAXIMIZE //TODO: this part doesn't work
// 	//c.add(IloRange(env, -IloInfinity, 0.0));
	
//       std::stringstream i_str;
//       i_str << i+mp->N()+1;
//       c[i+mp->N()].setName( ("computation_maxTn_c" + i_str.str()).c_str() );
//       c[i+mp->N()].setLinearCoef(T, 1.0);
//       c[i+mp->N()].setLinearCoef(Tn[i], -1.0);

//       //fprintf(file,"c%d: T - B%d => 0\n",N*E+E+T+T+N+i+1,i+1);
//     }
//   //  fprintf(file,"\n");
//   //fprintf(file,"Bounds\n\n");
//   //fprintf(file,"T  <= 8.51\n\n");

//   model.add(c);

//   // free mem
//   for(int i=0; i<mp->N(); i++)
//     delete[] Tnt[i];
//   delete[] Tnt;
// } // END computationObjectiveAsConstraint

void 
ILPScheduler::costObjective (GoalType gType, SchedulingProblem* sp, IloModel model, IloBoolVarArray x)
{
  IloEnv env = model.getEnv();
  IloRangeArray c(env);

  IloObjective obj;
  if(gType == MINIMIZE)
    obj = IloMinimize(env);
  else // gType == MAXIMIZE
    obj = IloMaximize(env);

  // // define cost as a decision variable
  // IloNumVar cost(env);
  // cost.setName("cost");

  // c.add(IloRange(env, 0.0, 0.0));
  // c[0].setName("cost_c");
  // c[0].setLinearCoef(cost, -1.0);

  for(int n=0; n < sp->N(); n++)
    {
      int M = sp->J()->at(n)->getL()->size();
      for(int m = 0; m < M; m++)
	{
	  for(int l=0; l < sp->L(); l++)
	    {
	      int nml = sumM(n)*sp->L() + m*sp->L() + l;
	      double p_l = sp->pMin()->at(l);
	      double l_nm = sp->J()->at(n)->getL()->at(m);
	      obj.setLinearCoef(x[nml], p_l*l_nm );
	    } // end l
	} // end m
    } // end n
  //  model.add(c);

  // minimize cost
  //obj.setLinearCoef(cost, 1.0);
  model.add(obj);

} // END costObjective

void 
ILPScheduler::peakObjective (GoalType gType, SchedulingProblem* sp, IloModel model, IloBoolVarArray x)
{
  IloEnv env = model.getEnv();
  IloRangeArray c(env);

  IloObjective obj;
  if(gType == MINIMIZE)
    obj = IloMinimize(env);
  else // gType == MAXIMIZE
    obj = IloMaximize(env);

  // // define cost as a decision variable
  // IloNumVar cost(env);
  // cost.setName("cost");

  // c.add(IloRange(env, 0.0, 0.0));
  // c[0].setName("cost_c");
  // c[0].setLinearCoef(cost, -1.0);

  for(int n=0; n < sp->N(); n++)
    {
      int M = sp->J()->at(n)->getL()->size();
      for(int m = 0; m < M; m++)
	{
	  for(int l=0; l < sp->L(); l++)
	    {
	      int nml = sumM(n)*sp->L() + m*sp->L() + l;
	      double p_l = sp->pMin()->at(l);
	      double l_nm = sp->J()->at(n)->getL()->at(m);
	      obj.setLinearCoef(x[nml], p_l*l_nm );
	    } // end l
	} // end m
    } // end n
  //  model.add(c);

  // minimize cost
  //obj.setLinearCoef(cost, 1.0);
  model.add(obj);

} // END peakObjective


// void ILPScheduler::commObjective (GoalType gType, MappingProblem* mp, IloModel model, IloBoolVarArray y)
// {
//    IloEnv env = model.getEnv();

//    IloObjective obj;
//    if(gType == MINIMIZE)
//      obj = IloMinimize(env);
//    else // gType == MAXIMIZE
//      obj = IloMaximize(env);

//    for(int i=0; i<mp->P(); i++)
//      {
//        int numLinksInPath = 0; // sum of 1 row in Mpl
//        for(int k=0; k<mp->L(); k++)
// 	   numLinksInPath = numLinksInPath + mp->Mpl()[i][k];
       
//        for(int j=0;j<mp->E();j++)
// 	 {
// 	   double dOnPath = (double)numLinksInPath * mp->d()[j];
// 	   obj.setLinearCoef(y[j+i*mp->E()], dOnPath);
// 	 }
//      }
   
//    model.add(obj);
// } // END commObjective


// void ILPScheduler::commObjectiveAsConstraint (RelationType rType, double commConstraint, MappingProblem* mp, IloModel model, IloBoolVarArray y, IloRangeArray c)
// {
//    IloEnv env = model.getEnv();

//    // define constraint
//    if (rType == LEQ)
//      c.add(IloRange(env, -IloInfinity, commConstraint));
//    else if (rType == EQ)
//      c.add(IloRange(env, commConstraint, commConstraint));
//    else // rType == GEQ
//      c.add(IloRange(env, commConstraint, IloInfinity));
   
//    c[0].setName("communication_c1");

//    for(int i=0; i<mp->P(); i++)
//      {
//        int numLinksInPath = 0; // sum of 1 row in Mpl
//        for(int k=0; k<mp->L(); k++)
// 	   numLinksInPath = numLinksInPath + mp->Mpl()[i][k];
       
//        for(int j=0;j<mp->E();j++)
// 	 {
// 	   double dOnPath = (double)numLinksInPath * mp->d()[j];
// 	   c[0].setLinearCoef(y[j+i*mp->E()], dOnPath);
// 	 }
//      }
   
//    model.add(c);
// } // END commObjectiveAsConstraint
