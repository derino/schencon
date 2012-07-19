#ifndef SIGNAL_H
#define SIGNAL_H

#include <iostream>
#include <fstream>  
#include <sstream>  
#include <assert.h>
#include <cstdlib>

using namespace std;

template <typename T >
class Signal
{
public:
  Signal(string, T*, int);
  Signal(string, string);
  Signal(string, int, int);
  virtual ~Signal();

  //  friend ostream& operator<< <>(ostream&, const Signal<T>&);
  Signal<T>* operator+(Signal<T>&);
  Signal<T>* operator-(Signal<T>&);
  Signal<T>* concat(Signal<T>&);
  double operator*(Signal<T>&);
  T operator[](int);
  T at(int);
  string getName();
  int size();
  double getEnergy();
  void setValueAt(int, T);


  void readFromFile(string);
  void print();
  //ostream& print(ostream&);
  void printIntoMatFile();
  void printIntoFile(); // for gnuplot

private:
  string _name;
  T* _value;
  int _size;
};

template <typename T>
Signal<T>::Signal(string name, T* v, int s): _name(name), _value(v), _size(s)
{

}

template <typename T>
Signal<T>::Signal(string name, string fileName):_name(name)
{
  readFromFile(fileName);
}

template <typename T>
Signal<T>::Signal(string name, int s, int init): _name(name), _size(s)
{
  _value = new T[_size];
  for(int i=0; i < _size; i++)
    _value[i] = init;
}

template <typename T>
Signal<T>::~Signal()
{
  delete _value;
}

template <typename T>
T Signal<T>::at(int i)
{
  assert(i<_size);
  return _value[i];
}

template <typename T>
T Signal<T>::operator[](int i)
{
  return at(i);
}

template <typename T>
Signal<T>* Signal<T>::operator+(Signal<T>& rhs)
{
  assert( _size == rhs.size() );

  T* res = new T[_size];
  for(int i=0; i<_size; i++)
    res[i] = _value[i] + rhs[i];

  Signal<T>* resSignal = new Signal<T>(_name + " + " + rhs.getName(), res, _size);
  return resSignal;
}

template <typename T>
Signal<T>* Signal<T>::operator-(Signal<T>& rhs)
{
  assert( _size == rhs.size() );

  T* res = new T[_size];
  for(int i=0; i<_size; i++)
    res[i] = _value[i] - rhs[i];

  Signal<T>* resSignal = new Signal<T>(_name + " - " + rhs.getName(), res, _size);
  return resSignal;
}

template <typename T>
double Signal<T>::operator*(Signal<T>& rhs)
{
  assert( _size == rhs.size() );

  double res = 0;
  for(int i=0; i<_size; i++)
    res += _value[i] * rhs[i];

  return res;
}

template <typename T>
Signal<T>* Signal<T>::concat(Signal<T>& rhs)
{
  T* res = new T[_size + rhs.size()];
  
  for(int i=0; i<_size; i++)
    res[i] = _value[i];
  for(int i=0; i<rhs.size(); i++)
    res[_size+i] = rhs[i];

  Signal<T>* resSignal = new Signal<T>(_name + " & " + rhs.getName(), res, _size+rhs.size() );
  return resSignal;
}


template <typename T>
string Signal<T>::getName()
{
  return _name;
}

template <typename T>
int Signal<T>::size()
{
  return _size;
}

template <typename T>
double Signal<T>::getEnergy()
{
  double E = 0;
  for(int i=0; i<_size; i++ )
    E += _value[i];
  return E;
}

template <typename T>
void Signal<T>::setValueAt(int i, T val)
{
  _value[i] = val;
}

// reads a file with time-value pairs into an array
// e.g., 
// 0 4.1 <-- always starts with time 0.
// 2 5.2
// 4 0 <-- always finishes with 0 value.
// is read into an array {4.1, 4.1, 5.2, 5.2}
template <typename T>
void Signal<T>::readFromFile(string fileName)
{
  int endTime = 0;
  int time = 0;
  T value;
  T* res = NULL;

  ifstream fin(fileName.c_str());
  if ( !fin.is_open())
  {
    cout << "Unable to open file"; 
    exit(EXIT_FAILURE);
  }

  string tmp = "";
  string tmp2 = "";
  while( !fin.eof() )
    {
      tmp2 = tmp;
      getline(fin, tmp);
    }
  stringstream ss(stringstream::in | stringstream::out);
  ss << tmp2;

  ss >> endTime;
  //  TRACE("endTime " << endTime);
  res = new T[endTime];

  //  fin.seekg(0, ios::beg); // doesn't work.
  fin.close();
  fstream fin2(fileName.c_str());
  if ( !fin2.is_open())
  {
    cout << "Unable to open file"; 
    exit(EXIT_FAILURE);
  }

  int i = 0;
  //while ( !fin2.eof() )
  //  {
      while(i<endTime)
	{
	  fin2 >> time;
	  //	  TRACE("time " << time);
	  fin2 >> value;
	  //	  TRACE("value " << value);
	  //	  TRACE("i " << i);
	  
	  //if(i<time)
	  //  {
	      while(i<time)
		{
		  res[i] = res[i-1]; //value;
		  //		  TRACE("res[" << i << "] = " << res[i]);
		  i++;
		}
	      //res[i] = value;
	      //cout << "if res[" << i << "] = " << res[i] << endl;
	      //i++;
	      //}
	      //else
	      //{
	      if(i<endTime){
	      res[i] = value;
	      //	      TRACE("else res[" << i << "] = " << res[i]);
	      i++;}
	      //}
	}
      //}
  fin2.close();
  //return new Signal<T>(res, endTime);
  _value = res;
  _size = endTime;
}

template <typename T>
void Signal<T>::print()
{
  cout << _name << ": " << endl << "[";
  for(int i=0; i<_size-1; i++ )
    cout << _value[i] << ", ";
  cout << _value[_size-1] << "]" << endl;
}

/*
template <typename T>
void Signal<T>::print()
{
  print(cout);
}

template <typename T>
ostream& Signal<T>::print(ostream& out)
{
  out << _name << ": " << endl << "[";
  for(int i=0; i<_size-1; i++ )
    out << _value[i] << ", ";
  out << _value[_size-1] << "]" << endl;
  return out;
}


template <typename T>
ostream& operator<< <>(ostream& out, const Signal<T>& sig)
{
  return sig.print(out);
}
*/

// for easy plotting with "./plot.octave fname.mat"
template <typename T>
void Signal<T>::printIntoMatFile()
{
  string fname = _name + ".mat";
  ofstream fout(fname.c_str());
  assert(fout.is_open());
  fout << "# name: v" << endl
       << "# type: matrix" << endl
       << "# rows: 1" << endl
       << "# columns: " << _size+1 << endl;
  for(int i=0; i<_size; i++ )
    fout << _value[i] << " ";
  fout << 0 << endl;
  /*for(int i=0; i<_size-1; i++ )
    fout << _value[i] << " ";
  fout << _value[_size-1] << endl;*/
  fout.close();
}


template <typename T>
void Signal<T>::printIntoFile()
{
  string fname = _name + ".gnuplot.data";
  ofstream fout(fname.c_str());
  assert(fout.is_open());
  //  fout << "# name: v" << endl
  //     << "# type: matrix" << endl
  //     << "# rows: 1" << endl
  //     << "# columns: " << _size+1 << endl;
  int i;
  for(i=0; i<_size; i++ )
    fout << i << " " <<  _value[i] << endl;
  fout << i << " " << 0 << endl;
  /*for(int i=0; i<_size-1; i++ )
    fout << _value[i] << " ";
  fout << _value[_size-1] << endl;*/
  fout.close();
}


/*
// reads a file with time-value pairs into an array
// e.g., 
// 4     <-- end time (also the size of the array)
// 0 4.1 <-- always starts with time 0.
// 2 5.2
// is read into an array {4.1, 4.1, 5.2, 5.2}
template <typename T>
void Signal<T>::readFromFile(string fileName)
{
  int endTime = 0;
  int time = 0;
  T value;
  T* res = NULL;

  ifstream fin(fileName.c_str());
  if ( !fin.is_open())
  {
    cout << "Unable to open file"; 
    exit(EXIT_FAILURE);
  }
  
  fin >> endTime;
  cout << "endTime " << endTime << endl;
  res = new T[endTime];
  
  int i = 0;
  while ( !fin.eof() )
    {
      while(i<endTime)
	{
	  fin >> time;
	  cout << "time " << time << endl;
	  fin >> value;
	  cout << "value " << value << endl;
	  cout << "i " << i << endl;
	  
	  //if(i<time)
	    while(i<time)
	      {
		res[i] = res[i-1]; //value;
		i++;
	      }
	  //else
	  //{
	      res[i] = value;
	      i++;
	      //}
	}
    }
  fin.close();
  //return new Signal<T>(res, endTime);
  _value = res;
  _size = endTime;
}
*/

#endif
