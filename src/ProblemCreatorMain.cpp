#include <stdio.h>
#include <stdlib.h>
#include <fstream>  
#include <vector>
#include <algorithm>
#include <limits>
#include <sstream>
#include <math.h>
#include <sys/stat.h>
//#include <boost/lambda/lambda.hpp>

#include "Signal.h"
#include "Task.h"
#include "util.h"
#include "SchedulingProblem.h"
using namespace std;
//using namespace boost::lambda;



// Global Variables

int L = 96;


float rand01()
{
  return ((float) rand()) / RAND_MAX;
}

Signal<double>* createPMin()
{
  // configuration for p_min duration = schedule lengt, # of period repeats = 1
  int tp = L;
  int R = 1;
  double  aS = 0.17;
  double  bS = 0.02;  
  Signal<double>* pMin = new Signal<double>("p_min", tp*R, 0);
  
  for( int n=0; n<tp*R; n++)
  {
    // calculate value at n
    double val = aS + bS * sin ((2 * M_PI * n / tp)+M_PI);
    pMin->setValueAt(n, val);
  }

  return pMin;
}

Task* createTask(int i, bool preemptable, float par)
{
  Task* j = new Task();

  // - set task's name
  stringstream jNameStr;
  jNameStr << "J_" << i;
  j->setName( jNameStr.str() );

  // - set preemptability
  j->setPr(preemptable);

  // - set task's arrival time
  j->setA(0);

  // - set task's deadline
  // Task period
  int tp = 2 + rand() % 22;

  //  Max # of repeating periods = maxR
  int maxR = L/tp; // floor of the division. not necessary due to dividing two ints.

  //  # of repeating periods = R
  int R = 1 + rand() % maxR;          //gives a number in [1,maxR]

  // Randomly deadline assigning
  j->setD( ((L - tp*R)*rand01()) + tp*R );

  // - set task's load profile
  //  average power of sinosoid: aS
  float aS = 0.5 + rand01();
  float bS = (par-1)*aS;

  stringstream lNameStr;
  lNameStr << "L_" << i;
  Signal<double>* l = new Signal<double>(lNameStr.str(), tp*R, 0);

  for( int n=0; n<tp*R; n++)
    {
      // calculate value at n
      double val = aS + bS * sin (2 * M_PI * n / tp);
      l->setValueAt(n, val);
    }

  j->setL(l);

  return j;
}

void calculatePMaxRange(vector<Task*>* J, double& Pmax_low, double& Pmax_med, double& Pmax_high)
{
  double* peaks = new double[J->size()];

  Pmax_high = 0;
  for(int i=0; i<J->size(); i++)
    {
      peaks[i] = J->at(i)->getL()->peak();
      Pmax_high += peaks[i];
    }
  Pmax_low = max(peaks, J->size());
  Pmax_med = (Pmax_high + Pmax_low)/2;
}


int main()
{

  //>>>>>>>> Synthetic Task Set Creator <<<<<<<<<

  /* initialize random seed: */
  srand ( time(NULL) );


  // Preemptive Ratio
  float prr_arr[6] = {0, 0.2, 0.4, 0.6, 0.8, 1};

  // Task Set Size
  int tss = 10;

  // Peak to Average Ratio
  float lpar_arr[3] = { 0, 0.5, 1};

  //task features
  bool preemptive;
  float par;

  for (int a = 0; a < 6; a++)
    {
      cout << endl << "prr_arr[" << a << "] = " << prr_arr[a] << endl;

      for (int b = 0; b < 3; b++)
	{
	  cout << endl << "\t lpar_arr[" << b << "] = " << lpar_arr[b] << endl;

	  // create problem
	  SchedulingProblem* sp = new SchedulingProblem();
	  sp->setPrR( prr_arr[a] );
	  sp->setLPAR( lpar_arr[b] );
	  sp->setN(tss);
	  sp->setL(L);
	  sp->setPH();

	   // creating p_min signal
	  Signal<double>* pMin = createPMin();
  	  sp->setPMin(pMin);
	  
	  for (int i = 1; i <= tss; i++)
	    {
	      float r = rand01();
	      if(r < prr_arr[a])
		preemptive = true;
	      else
		preemptive = false;

	      float r2 = rand01();
	      if(r2 < lpar_arr[b])
		par = 1 + 0.2*rand01();
	      else
		par = 1.8 + 0.2*rand01();

	      Task* j_i = createTask(i,preemptive, par);
	      sp->J()->push_back(j_i);

	    }
	  
	  // calculate p_max range
	  // calculate Pmax_low, Pmax_med, Pmax_high
	  double Pmax_low, Pmax_med, Pmax_high;
	  calculatePMaxRange(sp->J(), Pmax_low, Pmax_med, Pmax_high);
	  sp->setPMaxRange(Pmax_low, Pmax_med, Pmax_high);
	  	  
	  // create problem folder
	  mkdir(sp->name().c_str(), S_IRWXU | S_IRWXG);
	  // change curr. dir. to the problem folder
	  chdir(sp->name().c_str());
	  // write problem files	  
	  sp->write();
	  // change curr. dir. back to top
	  chdir("..");
	}
    }
  
  return 0;
  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Syntetic Task Set Creater<<<<<<<<<<<<<<<<<<<<<<<<<<<<

}
