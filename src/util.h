#ifndef UTIL_H
#define UTIL_H

#include "Signal.h"

//Verilen schedule a gore bir array yaratip sonrasinda da sinyal formatina cevirerek bu signali geri gonderiyor.
//ex: tab:[0, 1, 1, 0] => res[j]=[0, 1.1, 2.2, 0]
Signal<double>* P_i(Signal<double>* L_i, Signal<int>* s_i)
{
  double* res = new double[s_i->size()];

  for(int j = 0; j < s_i->size(); j++)
    {
      if( s_i->at(j) == 1 )
      {
    	  int sum = 0;
    	  for(int k=0; k<j; k++)
    		  sum += s_i->at(k);
	  //cout << "sum:" << sum << endl;
    	  res[j] = L_i->at(sum);
      }
      else
    	  res[j] = 0;
    }

  Signal<double>* resSignal = new Signal<double>( "P_i for " + L_i->getName(), res, s_i->size() );
  return resSignal;
}

Signal<double>* zeroNegatives(Signal<double>& P)
{
  double* res = new double[P.size()];

  for(int j = 0; j < P.size(); j++)
    {
      if( P[j] < 0 )
	  res[j] = 0;
      else
	res[j] = P[j];
    }

  Signal<double>* resSignal = new Signal<double>( "zeroNegatives(" + P.getName() + ")", res, P.size() );
  return resSignal;
}

Signal<double>* P__tot( vector<Signal<int>*>& S, vector<Task*>& J )
{
  Signal<double>* P_0 = new Signal<double>("all 0s", S[0]->size(), 0); // initialized to 0.
  Signal<double>* P_tot = P_0;
  for( unsigned int i = 0; i < J.size(); i++ )
    {
      Signal<double>* P__i = P_i( J[i]->getL(), S[i]);
      //ONDEBUG( P__i->printIntoMatFile() );
      //ONDEBUG( P__i->print() );
      P_0 = P_tot;
      P_tot = *P_tot + *P__i;
      delete P_0;
      delete P__i;
    }
  return P_tot;
}

double cost( vector<Signal<int>*>& S, vector<Task*>& J, Signal<double>& p_min, Signal<double>& P_H )
{
  Signal<double>* P_tot = P__tot(S, J);
  //ONDEBUG( P_tot->printIntoMatFile() );
  //ONDEBUG( P_tot->print() );

  Signal<double>* P_billed_with_neg = *P_tot - P_H;
  Signal<double>* P_billed = zeroNegatives(*P_billed_with_neg);
  //ONDEBUG( P_billed->printIntoMatFile() );
  //ONDEBUG( P_H.print() );
  //ONDEBUG( p_min.print() );
  //ONDEBUG( P_billed->print() );
  delete P_tot;
  delete P_billed_with_neg;

  double C = *P_billed * p_min;
  delete P_billed;
  return C;
}

// tasks should be such that 
// 1- duration(PP_i) <= d_i - a_i (checked in the Task constructor)

int checkTasksValidity(vector<Task*>& J, double P_max)
{
  // !start
  // 2- P_max x ( max(d_i) - min(a_i) ) > sum( E_i )     total energy consumed working at P_max should be enough to run all tasks.
  double sum_E_i = 0;
  int max_d_i = 0;
  int min_a_i = 0; // ASSUMPTION
  for( vector<Task*>::iterator it = J.begin(); it != J.end(); ++it )
    {
      // create .mat
      //ONDEBUG( (*it)->getL()->printIntoMatFile() );
      //ONDEBUG( (*it)->print() );

      sum_E_i += (*it)->getL()->getEnergy();
      if( (*it)->getD() > max_d_i )
	max_d_i = (*it)->getD();
    }
  //TRACE("max_d_i: " << max_d_i);
  //TRACE("sum_E_i: " << sum_E_i);
  assert( P_max * (max_d_i - min_a_i) >= sum_E_i );
  // !end

  // !start
  // 3- at any instant, the power demand of any single task shouldn't be more than P_max
  for( vector<Task*>::iterator it = J.begin(); it != J.end(); ++it )
    {
      for(int i=0; i< (*it)->getL()->size(); i++)
	assert( (*it)->getL()->at(i) < P_max );
    }
  // !end

  return max_d_i;
}

// number of 1 values in the signal
template <typename T>
int numOfOnes(Signal<T>& s)
{
  int count = 0;
  for(int i=0; i<s.size(); i++)
    if(s[i]==1)
      count++;
  return count;
}

// index of the first 1 in the signal
template <typename T>
int start(Signal<T>& s)
{
  for(int i=0; i<s.size(); i++)
    {
      if(s[i]==1)
	return i;
    }
  return -1;      
}

// index of the last 1 in the signal
template <typename T>
int end(Signal<T>& s)
{
  int end = 0;
  for(int i=0; i<s.size(); i++)
    if(s[i]==1)
      end = i;
  return end;
}

// max value in the signal
template <typename T>
T max(Signal<T>& s)
{
  T max = 0;
  for(int i=0; i<s.size(); i++)
    if(s[i] > max)
      max = s[i];
  return max;
}

enum ScheduleValidity {VALID=0, EARLY_START, LATE_FINISH, NOT_PREEMPTABLE, LESS_OR_MORE_SCHEDULED, POWER_OVERUSE};

// s_i is an array of 0s and 1s that denote whether the task is running or not.
// schedule is constrained by 
// - preemptability of tasks. 
//   Once a non preemptable task is started, it should run until its end.
// - P_max: At any interval, total power used by scheduled tasks should not exceed P_max.
ScheduleValidity isValidSchedule(vector<Signal<int>*>& S, vector<Task*>& J, double P_max)
{
  ScheduleValidity sv = VALID;
  for(unsigned int i=0; i<J.size(); i++)
    {
      // tasks can't be started before their arrival times
      if( start(*S[i]) < J[i]->getA() )
	{
	  sv = EARLY_START;
	  return sv;
	}
      
      // tasks can't be finished after their deadlines
      if( end(*S[i]) > J[i]->getD()-1 )
	{
	  sv = LATE_FINISH;
	  return sv;
	}

      // Task Ji is scheduled as many times as the length of its load power profile.
      if( numOfOnes(*S[i]) != J[i]->getL()->size() )
	{
	  sv = LESS_OR_MORE_SCHEDULED;
	  return sv;
	}

      // If task Ji is not preemptable, it should be scheduled all at once.
      if( !J[i]->getPr() )
	{
	  for (int j=start(*S[i]); j<=end(*S[i]); j++)
	    if(S[i]->at(j) != 1)
	      {
		sv = NOT_PREEMPTABLE;
		return sv;
	      }
	}
    }

  // At no time, the total power withdrawn by all tasks exceeds the allowed maximum, P_max
  Signal<double>* P_tot = P__tot(S, J);
  if ( max(*P_tot) > P_max )
    sv = POWER_OVERUSE;
  delete P_tot;

  return sv;
}

void printSchedule( vector<Signal<int>*>& S)
{
  for( vector<Signal<int>*>::iterator it = S.begin(); it != S.end(); it++ )
    {
      (*it)->print();
    }
}

void printTasks( vector<Task*>& J)
{
  for( vector<Task*>::iterator it = J.begin(); it != J.end(); it++ )
    {
      (*it)->print();
    }
}

vector< Signal<int>* >* recfun(int tlen, int slen)
{
  vector< Signal<int>* >* allS = new vector<Signal<int>*>();  
  
  if(tlen == 0)
    {
      Signal<int>* s_3 = new Signal<int>("s_0", slen, 0);
      allS->push_back(s_3);
      return allS;
    }
  else if(tlen == slen)
    {
      //      int* p_s_3 = new int[tlen];
      //for(int i=0; i<tlen; i++) 
      //p_s_3[i] = 1;
      Signal<int>* s_3 = new Signal<int>("s_1", tlen, 1);
      allS->push_back(s_3);
      return allS;
    }
  else
    {
      // generate S1 assuming first index is 1
      vector< Signal<int>* >* S1 = NULL;
      S1 = recfun(tlen-1, slen-1);
      // put 1 for all in S1
      Signal<int>* s_1 = new Signal<int>("s_1", 1, 1);
      for ( vector< Signal<int>* >::iterator it = S1->begin(); it != S1->end(); ++it )
	{
	  Signal<int>* con = s_1->concat( *(*it) );
	  allS->push_back(con);
	  delete *it;
	}
      delete s_1;
      delete S1;
      // generate S0 assuming first index is 0
      vector< Signal<int>* >* S0 = NULL;
      S0 = recfun(tlen, slen-1);
      // put 0 for all in S0
      Signal<int>* s_0 = new Signal<int>("s_1", 1, 0);
      for ( vector< Signal<int>* >::iterator it = S0->begin(); it != S0->end(); ++it )
	{
	  Signal<int>* con = s_0->concat( *(*it) );
	  allS->push_back(con);
	  delete *it;
	}
      delete s_0;
      delete S0;
      // return S1 union S0
      //S1->insert(S1->end(), S0->begin(), S0->end());
      return allS;
    }
}

vector< Signal<int>* >* generateAllSchedules( Task* t, int size)
{
  if (t->getPr() == 0)
  {
	  cout << "non-preemptable task" << endl;
      vector< Signal<int>* >* allS = new vector<Signal<int>*>();
//      for(int i = t->getA(); i < t->getD() - t->getL()->size(); i++)
      for(int i = t->getA(); i < size - t->getL()->size(); i++)
	{
	  // 0s until i, then getL()->size() many 1s, then again 0s until end
	  Signal<int>* s_0 = new Signal<int>("s_0", i, 0);
	  //s_0->print();
	  Signal<int>* s_1 = new Signal<int>("s_1", t->getL()->size(), 1);
	  //s_1->print();
	  Signal<int>* con = s_0->concat( *s_1 );
	  //con->print();
	  Signal<int>* s_2 = new Signal<int>("s_0", size - con->size(), 0);
	  //s_2->print();
	  Signal<int>* con2 = con->concat( *s_2 );
	  //con2->print();
	  delete s_0;
	  delete s_1;
	  delete s_2;
	  delete con;
	  allS->push_back(con2);
	}
      return allS;
  }
  else
    {
      cout << "recfun(" << t->getL()->size() << ", " << size << ")" << endl;
      return recfun(t->getL()->size(), size);
    }
}

struct CostSchedule
{
  double C;
  vector<Signal<int>*> S;
};

CostSchedule evaluateSchedule(vector< vector<Signal<int>*>* >& ST, int* digit, vector<Task*>& J, Signal<double>& p_min, Signal<double>& P_H, double P_max)
{
  vector<Signal<int>*> S;
  for(unsigned int i=0; i<ST.size(); i++)
      S.push_back( (*ST[i])[digit[i]] );

  ScheduleValidity sv = isValidSchedule(S, J, P_max);
  if ( sv == VALID )
    {
      double C = cost(S, J, p_min, P_H);
      
      //ONDEBUG( printSchedule(S) ); //for_each (S.begin(), S.end(), cout << _1 << endl);
      //TRACE("cost:" << C);
      //cout << "cost:" << C << endl;
      
      CostSchedule res;
      res.C = C;
      res.S = S;
      return res;
    }
  else
    {
      //TRACE ("Following schedule is not valid (code: " << sv << "):");
      //ONDEBUG( printSchedule(S) );
      //TRACE ("with inputs: P_max=" << P_max << " and " << endl);
      //ONDEBUG( printTasks(J) );
      CostSchedule res;
      res.C = -1;
      return res;
    }
}

void fullSearch( vector<Task*>& J, double P_max, Signal<double>& p_min, Signal<double>& P_H ) 
{
  int size = p_min.size(); // size: length of schedule
  vector< vector<Signal<int>*>* > ST;
  for( vector<Task*>::iterator it = J.begin(); it != J.end(); ++it)
    {
      cout << "generating schedule" << endl;
      vector< Signal<int>* >* allS = generateAllSchedules( (*it), size );
      ST.push_back(allS);
    }

  cout << "generated all schedules" << endl;

  // this is a trick to do indefinite number of nested loop (using mixed radix number system)
  int* steps = new int[ST.size()];
  for( unsigned int m = 0; m < ST.size(); m++ )
    {
      steps[m] = ST[m]->size();
      cout << "steps[m]" << steps[m] << endl;
    }

  unsigned long long limit = 1;
  for (unsigned int i = 0; i < ST.size(); ++i)
    limit *= steps[i];
  cout << "limit: " << limit << endl;

  // min C and S
  CostSchedule csmin;
  csmin.C = numeric_limits<double>::max();

  int* digit = new int[ ST.size() ];
  for (unsigned long long index = 0; index < limit; ++index) 
    {
      unsigned long long temp = index;
      for (unsigned int i = 0; i < ST.size(); ++i) 
	{
	  digit[i] = temp % steps[i];
	  temp /= steps[i];
	}
      //cout << " dd" << endl;
      CostSchedule cs = evaluateSchedule(ST, digit, J, p_min, P_H, P_max); // digit holds indices of the allS vector for each task
      if(cs.C != -1 && cs.C < csmin.C)
	{
	  csmin = cs;
	}
    }
  delete steps;
  delete digit;

  // TODO: check if csmin == max_double, then it means there was no valid schedule.
  cout << "min cost: " << csmin.C << endl;
  cout << "with schedule: " << endl;
  printSchedule(csmin.S);

  // print 'P_i's for gnuplot to see it nicely
    for( unsigned int i = 0; i < J.size(); i++ )
    {
      Signal<double>* P__i = P_i( J[i]->getL(), csmin.S[i]);
      P__i->printIntoFile();
      delete P__i;
      //ONDEBUG( P__i->print() );
    }

    for(int i=0; i<ST.size(); i++)
      delete ST[i];
}

//================================================================================================
// Extended functions for our new heuristic
typedef struct _Pair
{
  int index;
  double value;
} Pair;

bool compare(Pair p1, Pair p2)
{
  return (p1.value < p2.value);
}

typedef struct _Range
{
  int left;
  int right;
} Range;

Range intersect(Range r1, Range r2)
{
  Range r_int;
  r_int.left = max(r1.left, r2.left);
  r_int.right = min(r1.right, r2.right);
  return r_int;
}

int getScheduleIndexOf(int taskPartIndex, vector<int>& s)
{
	for(int i = 0; i < s.size(); i++)
	{
		if(s[i] == taskPartIndex)
			return i;
	}
	assert(false);
	return -1;
}

/*bool isScheduled(int index, vector<int>& s)
{
	if(s[index] == -1)
		return false;
	else
		return true;
}*/

bool isScheduled(int index, vector<int>& s)
{
	for(int i=0; i<s.size(); i++)
		if(s[i] == index)
			return true;

	return false;
}

int getLeftScheduled(int taskPart, vector<int>& s)
{
	for(int j = 1; taskPart-j >= 0; j++)
	{
		if(isScheduled(taskPart-j, s))
			return taskPart-j;
	}
	return numeric_limits<int>::min();
}

int getRightScheduled(int taskPart, vector<int>& s, Task* task)
{
	for(int j = 1; taskPart+j < task->getL()->size(); j++)
	{
		if(isScheduled(taskPart+j, s))
			return taskPart+j;
	}
	return numeric_limits<int>::max();
}

bool isInInterval(int index, Range r)
{
	if(index >= r.left && index <= r.right)
		return true;
	else
		return false;
}

// change the values -1 to 0 and the rest to 1.. -> -1, -1, 0, 1, -1 -> 0, 0, 1, 1, 0
int* scheduleNorm(vector<int>& s)
{
	int* s_values = new int[s.size()];


	for(int i = 0; i < s.size(); i++)
	{
		if(s[i] == -1)
			s_values[i] = 0;
		else
			s_values[i] = 1;
	}
	return s_values;
}
//========================================================================================
// AllocTab functions moved to here to be shared with LeesPart

Signal<double>* P__tot2( vector<Signal<int>*>& S, vector<Task*>& J)
{
  //S[0]->print();
  Signal<double>* P_0 = new Signal<double>("all 0s", S[0]->size(), 0); // initialized to 0.
  Signal<double>* P_tot = P_0;
  for( unsigned int i = 0; i < S.size(); i++ )
    {
      Signal<double>* P__i = P_i( J[i]->getL(), S[i]);
      //ONDEBUG( P__i->printIntoMatFile() );
      //ONDEBUG( P__i->print() );
      P_0 = P_tot;
      P_tot = *P_tot + *P__i;
      delete P_0;
      delete P__i;
    }
  return P_tot;
}

Signal<int>* getNonPreemptiveSchedule(Task& task, int startTime, int schedulesLength) {
	  // 0s until i, then getL()->size() many 1s, then again 0s until end
	  Signal<int>* s_0 = new Signal<int>("s_0", startTime, 0);
	  //s_0->print();
/*	  cout<< "simdi s_0->print calisiyor"<<endl;
	  s_0->print();
	  cout<< "size of s_0: " << s_0->size() << endl;*/
	  Signal<int>* s_1 = new Signal<int>("s_1", task.getL()->size(), 1);
	  //s_1->print();
	  Signal<int>* con = s_0->concat( *s_1 );
	  //con->print();
	  Signal<int>* s_2 = new Signal<int>("s_0", schedulesLength - con->size(), 0);
	  //s_2->print();
	  Signal<int>* con2 = con->concat( *s_2 );
	  //con2->print();
	  delete s_0;
	  delete s_1;
	  delete s_2;
	  delete con;

	  //alttaki 2 satir anlama amaciyla yazildi
//	  cout<< "simdi con2->print calisiyor"<<endl;
	  con2->print();

	  return con2;
}

void printTab( vector<Signal<int>*>& S)
{
  for( vector<Signal<int>*>::iterator it = S.begin(); it != S.end(); it++ )
    {
      (*it)->print();
    }
}

bool isConstraintSatisfied(vector<Signal<int>*>& S, vector<Task*>& J, double P_max) {
	//We apply CheckConstraint here to speed up the search procedure with pruning the unnecessary
	//search tree expansion. (i.e. a task with a deadline already violated. There is no need to continue scheduling)

	//return !(startTime + task.getL()->size() > task.getD());
	  // At no time, the total power withdrawn by all tasks exceeds the allowed maximum, P_max
	  Signal<double>* P_tot = P__tot2(S, J);
	  double pmax = max(*P_tot);
	  delete P_tot;
	  return ( pmax <= P_max );
	    //sv = POWER_OVERUSE;
	}

double evaluateCost(vector<Signal<int>*>& inputTab, vector<Task*>& J, double P_max, Signal<double>& p_min, Signal<double>& P_H) {

//	printTab(inputTab);
	ScheduleValidity sv = isValidSchedule(inputTab, J, P_max);

	if ( sv == VALID )
	  {
	    //		ONDEBUG( printSchedule(inputTab) ); //for_each (S.begin(), S.end(), cout << _1 << endl);
		return cost(inputTab, J, p_min, P_H);
	  }
	else
	  {
	    //	TRACE ("Following schedule is not valid (code: " << sv << "):");
	    //	ONDEBUG( printSchedule(inputTab) );
	    //	TRACE ("with inputs: P_max=" << P_max << " and " << endl);
	    //	ONDEBUG( printTasks(J) );
		return numeric_limits<double>::max();
	  }
}

double evaluateCostForLees(vector<Signal<int>*>& S, vector<Task*>& J)
{
	Signal<double>* P_tot = P__tot2(S, J);
	double pmax = max(*P_tot);
	delete P_tot;
	return pmax;
}
#endif
//===================================================================================================
// Sorting functions for task sets!

bool sortTasksViaEnergy(Task* J1, Task* J2)
{
	return (J1->getL()->getEnergy() > J2->getL()->getEnergy());
}

bool sortTasksViaPeakValue(Task* J1, Task* J2)
{
	return (max(*(J1->getL())) > max(*(J2->getL())));
}

bool sortTasksViaPreemption(Task* J1, Task* J2)
{
	if(J1->getPr() == true && J2->getPr() == false)
		return false;
	else if(J1->getPr() == false && J2->getPr() == true)
		return true;
//	else if(J1->getPr() == false && J2->getPr() == false)
//		return true;
//	else if(J1->getPr() == true && J2->getPr() == true)
//		return true;
}
//==================================================================================================

