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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <string.h>

#include <fstream>

#include "ILPScheduler.h"
#include "SchedulingProblem.h"
#include "SchedulingProblemReader.h"
//#include "SchedulingSolutionSet.h"
ILOSTLBEGIN


void usage( void )
{
  puts( "Usage for ilpscheduler: ilpscheduler [--time-limit time_limit [--gap-limit gap_limit]]" );
  /* ... */
  exit( EXIT_FAILURE );
}



/*
 * The optString global tells getopt_long() which options we
 * support, and which options have arguments.
 * 
 * The longOpts global tells getopt_long() which long options
 * we support, and which long options have arguments.
 */

struct globalArgs_t {
  double timeLimit;				// -t option
  double gapLimit;                              // -g option
} globalArgs;

static const char *optString = "t:g:h?";

static const struct option longOpts[] = {
  { "time-limit", required_argument, NULL, 't' },
  { "gap-limit", required_argument, NULL, 'g'},
  { "help", no_argument, NULL, 'h' },
  { NULL, no_argument, NULL, 0 }
};

void print_args( void )
{
  printf("ILPScheduler is executed with the following parameters:\n");
  printf( "timeLimit: %f\n", globalArgs.timeLimit );
  printf( "gapLimit: %f\n", globalArgs.gapLimit );
}

bool verify_args()
{
  bool proper = true;
  if (globalArgs.gapLimit != 100 && globalArgs.timeLimit == -1)
    {
      cout << "Error: Specify either only time limit or both time limit and gap limit. e.g., -t 1 OR -t 1 -g 10" << endl;
      return false;
    }

  return proper;

}


// void ilp_scheduler(double timeLimit, double gapLimit, MappingProblem* mp)
// {
//   ILPScheduler ilps(timeLimit, gapLimit, mp);
//   SchedulingSolutionSet* sss = ilps.epsilonConstraintMethod();
      
//   if (sss != NULL)
//     {
//       cout << "All calculated solutions: " << endl;
//       sss->printObjValues(cout, *(sss->getSolutionSet()) );
      
//       cout << "Pareto solutions: " << endl;
//       vector<SchedulingSolution*>* pSet = sss->getParetoSet();
//       sss->printObjValues(cout, *pSet );
//       delete pSet;

// /*      // write details in the log file.
//       ofstream fout("pareto_set.log");
//       sss->printAllValues(fout, *(sss->getParetoSet()) );
//       fout.close();
// */
//     }

//   delete sss;
//   return;
// }



void execute () // old main
{
  // Read the problem
  SchedulingProblemReader* spr = new SchedulingProblemReader();
  SchedulingProblem* sp = NULL;
  sp = spr->read();
  sp->print();

  // Solve the problem
  //  ilp_scheduler(globalArgs.timeLimit, globalArgs.gapLimit, sp);
  ILPScheduler ilps(globalArgs.timeLimit, globalArgs.gapLimit, sp);  
  SchedulingSolution* minCostSol = ilps.schedule(MINIMIZE, COST);
  SchedulingSolution* minPeakSol = ilps.schedule(MINIMIZE, PEAK);

  // Print solutions
  cout << "Solutions:" << endl;
  minCostSol->print(cout);
  minPeakSol->print(cout);

  // Relevant results of the ILP solution
  // minCostSol->getCost();
  // minPeakSol->getPeak();

  // cout << *minCostSol << endl;
  // cout << *minPeakSol << endl;

  delete minCostSol;
  delete minPeakSol;
  delete spr;
  return;
}  // END execute


int main( int argc, char *argv[] )
{
  int opt = 0;
  int longIndex = 0;
	
  /* Initialize globalArgs before we get to work. */
  globalArgs.timeLimit = -1;
  globalArgs.gapLimit = 100;
	
  /* Process the arguments with getopt_long(), then 
   * populate globalArgs. 
   */
  opt = getopt_long( argc, argv, optString, longOpts, &longIndex );
  while( opt != -1 ) {
    switch( opt ) {
    case 't':
      globalArgs.timeLimit = atof(optarg);
      break;
    case 'g':
      globalArgs.gapLimit = atof(optarg);
      break;
    case 'h':	/* fall-through is intentional */
    case '?':
      usage();
      break;
	    	    
    default:	    /* You won't actually get here. */
      break;
    }
	  
    opt = getopt_long( argc, argv, optString, longOpts, &longIndex );
  }

  if( !verify_args() )
    {
      usage();
      return EXIT_FAILURE;
    }

  print_args();
	
  execute();

  return EXIT_SUCCESS;
}

