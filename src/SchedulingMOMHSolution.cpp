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
#include "SchedulingMOMHSolution.hpp"

//extern MappingProblem* theMP;
//extern ReliabilityCalculator* theRC;

SchedulingMOMHSolution::SchedulingMOMHSolution(double peak, double cost) : TDPAMConstrainedSolution()/*TMOMHSolution()*/, _peak(peak), _cost(cost)
{// DONE: create a random mapping solution using a global mapping problem variablergamon

  //cout << "const." << endl;

	ObjectiveValues.resize(2);
	ObjectiveValues[0] = getPeak();
	ObjectiveValues[1] = getCost();

}

SchedulingMOMHSolution::~SchedulingMOMHSolution()
{
  for(vector<Signal<int>*>::iterator it = _tab->begin(); it != _tab->end(); it++)
    {
      delete (*it);
    }
  delete _tab;
}

double SchedulingMOMHSolution::getPeak()
{
	return _peak;
}

double SchedulingMOMHSolution::getCost()
{
	return _cost;
}

void SchedulingMOMHSolution::setTab(vector<Signal<int>*>* tab)
{
        _tab = tab;
}

vector<Signal<int>*>* SchedulingMOMHSolution::getTab()
{
  return _tab;
}

std::ostream& operator<<(std::ostream& os, SchedulingMOMHSolution& ms)
{
  os << ms.getPeak() << "\t" << ms.getCost();
  return os;
}

void SchedulingMOMHSolution::print(ostream& out)
{
      out << "Peak  = " << getPeak() << endl;
      out << "Cost  = " << getCost() << endl;
}
