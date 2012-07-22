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
#include "SchedulingSolution.h"

int SchedulingSolution::uniqueID = 0;

SchedulingSolution::SchedulingSolution()
{
  
}

SchedulingSolution::~SchedulingSolution()
{
  // TODO
  // if (Xnml != NULL)
  //   {
  //     for(int i=0; i<mp->N(); i++)
  // 	delete[] Xnt[i];
  //     delete[] Xnt;
  //     Xnt = NULL;
  //   }
  // else
  //   cout << "Warning: Double delete of Xnt avoided!" << endl;
 
}

SchedulingSolution::SchedulingSolution(SchedulingSolution& ss)
{
  
}

SchedulingSolution::SchedulingSolution(SchedulingProblem* _sp) : sp(_sp), status(UNKNOWN_SOLUTION/*IloAlgorithm::Unknown*/), solutionTime(0)
{
  // TODO
  // Xnt = new int*[mp->N()];
  // for (int i=0; i< mp->N(); i++)
  //   Xnt[i] = new int[mp->T()];

  // for(int i=0; i<mp->N(); i++)
  //   for(int j=0; j<mp->T(); j++)
  //     Xnt[i][j] = 5;
}

int*** SchedulingSolution::getXnml()
{
  return Xnml;
}

void SchedulingSolution::setXnml(int*** xnml)
{
  Xnml = xnml;
}

double SchedulingSolution::getCost()
{
  double cost = 0.0;

  for(int n=0; n < sp->N(); n++)
    {
      int M = sp->J()->at(n)->getL()->size();
      for(int m = 0; m < M; m++)
	{
	  for(int l=0; l < sp->L(); l++)
	    {
	      if( Xnml[n][m][l] == 1)
		{
		  double p_l = sp->pMin()->at(l);
		  double l_nm = sp->J()->at(n)->getL()->at(m);
		  cost += p_l*l_nm;
		}
	    } // end l
	} // end m
    } // end n

  return cost;
}

/*void SchedulingSolution::setCost(double c)
{
  cost = c;
}*/

double SchedulingSolution::getPeak()
{
  double* Ptot = new double[sp->L()];

  for(int l=0; l < sp->L(); l++)
    {
      Ptot[l] = 0.0;
      for(int n=0; n < sp->N(); n++)
	{
	  int M = sp->J()->at(n)->getL()->size();
	  for(int m = 0; m < M; m++)
	    {
	      if( Xnml[n][m][l] == 1)
		{
		  double l_nm = sp->J()->at(n)->getL()->at(m);
		  Ptot[l] += l_nm;
		}
	    }
	}
    }

  // could have used max from util.h but there were problems with compilation
  double max = 0.0;
  for(int l=0; l<sp->L(); l++)
    if(Ptot[l] > max)
      max = Ptot[l];
  return max;
}

/*void SchedulingSolution::setPeak(double p)
{
  peak = p;
  }*/

/*IloAlgorithm::Status*/SchedulingSolutionStatus SchedulingSolution::getStatus()
{
  return status;
}

void SchedulingSolution::setStatus(SchedulingSolutionStatus s)
{
  status = s;
}

/*IloNum*/double SchedulingSolution::getSolutionTime()
{
  return solutionTime;
}

void SchedulingSolution::setSolutionTime(/*IloNum*/double t)
{
  solutionTime = t;
}

double SchedulingSolution::getGap()
{
  return gap;
}

void SchedulingSolution::setGap(double g)
{
  gap = g;
}

SchedulingProblem* SchedulingSolution::getSP()
{
  return sp;
}

bool SchedulingSolution::dominatesEqual(SchedulingSolution* ss)
{
  return ( getCost() <= ss->getCost() 
	   && getPeak() <= ss->getPeak() );
}

bool SchedulingSolution::dominatesAbsolute(SchedulingSolution* ss)
{
  return ( (getCost() <= ss->getCost() 
	    && getPeak() < ss->getPeak())
	   ||
	   (getCost() < ss->getCost() 
	    && getPeak() <= ss->getPeak())
	   );
}

std::ostream& operator<<(std::ostream& os, SchedulingSolution& ss)
{
  return os << ss.getCost() << "\t" << ss.getPeak() << "\t@" << ss.getSolutionTime() << "\tgap:" << ss.getGap();
}

void SchedulingSolution::print(ostream& out)
{
  out << "Solution status = " << status << endl;
  if (status == OPTIMAL_SOLUTION/*IloAlgorithm::Optimal*/ || status == FEASIBLE_SOLUTION)
    {
      out << "cost  = " << getCost() << endl;
      out << "peak  = " << getPeak() << endl;

      out << "Xnml as schedules: " << endl;

      for(int n=0; n < sp->N(); n++)
	{
	  cout << "schedule for job " << n+1 << ": {";

	  int M = sp->J()->at(n)->getL()->size();
	  cout << "(" << M << ") ";
	  for(int m = 0; m < M; m++)
	    {
	      for(int l=0; l < sp->L(); l++)
		{
		  if( Xnml[n][m][l] == 1)
		    {
		      cout /*<< "(" << n << "," << m << "," << l << ")"*/ << l << " ";
		    }
		} // end l
	    } // end m
	  cout << "}" << endl;
	} // end n


      // printing Xnt in the format of a *.map file. It can be copy-pasted as a map file for further use (input to remapping or visualization)
      // out << endl;
      // out << mp->N() << endl;
      // out << mp->T() << endl;
      // out << endl;
      // for(int i = 0; i < mp->N(); i++)
      // 	{
      // 	  //out << "| ";
      // 	  for(int j = 0; j < mp->T(); j++)
      // 	    {
      // 	      out << Xnt[i][j] << " ";
      // 	    }
      // 	  out << endl; //out << "| " << endl;
      // 	}
      // out << endl;

    } // end if optimal

  if (status == OPTIMAL_SOLUTION/*IloAlgorithm::Optimal*/ || status == INFEASIBLE_SOLUTION/*IloAlgorithm::Infeasible*/ || status == FEASIBLE_SOLUTION)
    out << "Solution time = " << solutionTime << endl;
    out << "Gap = " << this->getGap() << endl;
}

std::ostream& operator<<(std::ostream& os, SchedulingSolutionStatus& ssStatus)
{
    switch(ssStatus)
    {
    case FEASIBLE_SOLUTION:
      return os << "FEASIBLE";
      break;
    case OPTIMAL_SOLUTION:
      return os << "OPTIMAL";
      break;
    case INFEASIBLE_SOLUTION:
      return os << "INFEASIBLE";
      break;
    case UNBOUNDED_SOLUTION:
      return os << "UNBOUNDED";
      break;
    case INFEASIBLE_OR_UNBOUNDED_SOLUTION:
      return os << "INFEASIBLE_OR_UNBOUNDED";
      break;
    case ERROR_SOLUTION:
      return os << "ERROR";
      break;
    default:
      return os << "UNKNOWN";
      break;
    }
}
