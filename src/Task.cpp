#include "Task.h"

Task::Task()
{
}

Task::Task(string name, int a, int d, bool pr, string lFileName): _name(name), _a(a), _d(d), _pr(pr)
{
  _L = new Signal<double>("L_" + name, lFileName);
  assert( d-a >= _L->size() ); // 
}

// Below constructor assumes the presence of a task file with name, 
// for ex,  J_1.txt for i=1
// with format:
// a
// d
// pr.
// also the L_1.txt should be present for the load profile.
Task::Task(int fileID)
{ 
  std::stringstream taskID_str;
  taskID_str << "J_" << fileID;
  _name = taskID_str.str();

  taskID_str << ".txt";
  ifstream fin( taskID_str.str().c_str() );
  if ( !fin.is_open())
    {
      cout << "Unable to open file " << taskID_str.str(); 
      exit(EXIT_FAILURE);
    }

  fin >> _a;
  fin >> _d;
  fin >> _pr;
  fin.close();

  std::stringstream loadID_str;
  loadID_str << "L_" << fileID;
  string signalName( loadID_str.str() );

  loadID_str << ".txt";
  _L = new Signal<double>(signalName, loadID_str.str());

  assert( _d-_a >= _L->size() );    
}

Task::~Task()
{
  delete _L;
}

string Task::getName()
{
  return _name;
}

void Task::setName(string name)
{
  _name = name;
}

int Task::getA()
{
  return _a;
}

void Task::setA(int a)
{
  _a = a;
}

int Task::getD()
{
  return _d;
}

void Task::setD(int d)
{
  _d = d;
}

bool Task::getPr()
{
  return _pr;
}

void Task::setPr(bool pr)
{
  _pr = pr;
}

Signal<double>* Task::getL()
{
  return _L;
}

void Task::setL(Signal<double>* L)
{
  _L = L;
}

void Task::print()
{
  cout << "Name: " << _name << endl;
  cout << "a: " << _a << endl;
  cout << "d: " << _d << endl;
  cout << "pr: " << _pr << endl;
  _L->print();
}
