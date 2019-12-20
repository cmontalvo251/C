//This basic.c file is an attempt at creating a sudo MATLAB feel to ease the transition from
//MATLAB to C and C++. Hopefully these function will assist in the very trivial matters such as using
//the length() command in MATLAB or the linspace() command.

//Function List (all commands are void unless stated otherwise)
// PAUSE() - pauses the function. asks for an input to restart
// vecdisp(double vec[],int len,char name[]) - displays contents of the vector
// ivecdisp(int vec[],int len,char name[]) - same as disp only with integers
// strdisp(char name[],char value[]) - displays name = value
// matrixdisp(double **mat,int row,int col,char name[]) - same as disp only with a matrix
// eye(double **mat,int row,int col) - creates an identity matrix
// double** diag(double vec[],int len) - just like diag in matlab. it will return a double star pointer
// double** matrixallocatedbl(int row,int col) - allocated enough memory for a matrix and returns the pointer
// disp(char name[]) - displays a string name
// scalardisp(double var,char name[]) - same as disp except it is a scalar
// iscalardisp(int var,char name[]); - same as disp only for integers
// CreateVec(double vec[],int len,int start,int inc) - exactly like the linspace command in MATLAB
// iCreateVec(int vec[],int len,int start,int inc) - same as createvec only with integers
// int CalcLength(int start,int end,int inc) - calculates how long a vector will be
// double sum(double vec[],int len) - just like sum command in MATLAB
// long long int isum(int vec[],int len) - integer sum
// double squaresum(double vec[],int len) - calculates the square of every element and then sums them up
// int numprime(int number) - calculates whether a number is a prime number or not
// ClearHome() - just like clear in MATLAB
// int2str(char str[],int val) - converts an integer to a string
// int str2int(char str[],int pointer) - converts a string that is a number from 0-9 into an integer
// int GetLength(char str[]) - calculates the length of a string of numbers
// int ispalindrome(int number) - calculates whether or not a number is a palindrome
// SetStr(char outvec[],char invec[],int start,int end,int inc) - sets outvec equal to the invec only it starts
     //copying at the start variable and increments by inc.
// setvalue(int vec[],int start,int inc,int len,double val) - this function sets the value of a vector equal to
    //val by starting at start and incrementing by inc
// isetvalue(int vec[],int start,int inc,int len,int val) - same as setvalue only with integers
// primes(int vec[],int len) - if you give this function a vector starting from 0 to X it will set all of the non prime numbers to zero

//HEADERS

#include <cmath> //only for g++
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream> //only for g++
#include <string> //only for g++
#include <string.h>

//GLOBALS
char zero = '0';
char one = '1';
char two = '2';
char three = '3';
char four = '4';
char five = '5';
char six = '6';
char seven = '7';
char eight = '8';
char nine = '9';
int DLMLENGTH = 0;

//MACROS
#define SQUARE(x) (((x)*(x)))
#define CUBE(x) (((x)*(x)*(x)))
#define PI 3.1415926535897932384626433832795

//using namespace std;

void ClearHome()
{
  system("CLS");
}

void PAUSE()
{
  printf("%s","Function Paused\n");
  printf("Type in any number(1-inf) and press enter to continue\n");
  int  a;
  scanf("%i",&a);
}

void inv2(double **invA,double **A){
  //!Compute 2x2 inverse
  double detA;
  detA = A[0][0]*A[1][1] - A[1][0]*A[0][1];
  if (detA != 0){
    invA[0][0] = A[1][1]/detA;
    invA[1][1] = A[0][0]/detA;
    invA[0][1] = -A[0][1]/detA;
    invA[1][0] = -A[1][0]/detA;
  }
  else
    {
      printf("Matrix is singular \ n");
    }
}

double** matrixallocatedbl(int row,int col)
{
  double **matout;
  matout = (double**)malloc(row*sizeof(double*));
  int i;
  for (i = 0;i<row;i++)
    {
      matout[i] = (double*)malloc(col*sizeof(double));
    }
  return matout;
}

double** diag(double vec[],int len) // double** A = diag({1,2},2)

{
  double **mat;
  mat = matrixallocatedbl(len,len);
  int ihat,jhat;
  for (ihat = 0;ihat <len;ihat++)
    {
      for(jhat = 0;jhat<len;jhat++)
	{
	  mat[ihat][jhat] = 0;
	  if (ihat == jhat)
	    {
	      mat[ihat][jhat] = vec[ihat];
	    }
	}
    }
  return mat;
}

double** eye(int row,int col)
{
  double **mat;
  mat = matrixallocatedbl(row,col);
  int ihat,jhat;
  for (ihat = 0;ihat <col;ihat++)
    {
      for(jhat = 0;jhat<row;jhat++)
	{
	  mat[ihat][jhat] = 0;
	  if (ihat == jhat)
	    {
	      mat[ihat][jhat] = 1;
	    }
	}
    }
  return mat;
}

void matrixdisp(double **mat,int row,int col,char name[])
{
  printf("%s%s%s",name," = ","\n");
  int i;
  int j;
  for(i=0;i<row;i++)
    {
      for(j=0;j<col;j++)
	{
	  printf("%lf%s",mat[i][j]," ");
	}
      printf("%s","\n");
    }
  printf("%s","\n");
}

void copy(double outvec[],double invec[],int len)
{
  int idx;
  for(idx=0;idx<len;idx++)
    {
      outvec[idx] = invec[idx];
    }

}

void vecdisp(double vec[],int len,char name[])
{
  printf("%s%s",name," = ");
  int i;
  for(i=0;i<len;i++)
    {
      printf("%lf%s",vec[i]," ");
    }
  printf("%s","\n");
}

void ivecdisp(int vec[],int len,char name[])
{
  printf("%s%s",name," = ");
  int i;
  for(i=0;i<len;i++)
    {
      printf("%i%s",vec[i]," ");
    }
  printf("%s","\n");
}

void disp(char name[])
{
  printf("%s%s",name,"\n");
}

void icopyvec(int original[],int copy[],int len)

{
  int counter;
  for (counter = 0;counter < len;counter++)
    {
      copy[counter] = original[counter];
    }
}

void strdisp(char name[],char value[])
{
  printf("%s%s%s",name," = ",value);
}
void scalardisp(double var,char name[])
{
  printf("%s%s%f%s",name," = ",var,"\n");
}

void iscalardisp(long long int var,char name[])
{
  printf("%s%s%lld%s",name," = ",var,"\n");
}

int CalcLength(int start,int end,int inc)
{
  return(((double)end-(double)start+1)/(inc));
}

int* ivecallocate(int length)
{
  int* vecout;
  vecout = (int*)malloc(length*sizeof(int));
  return vecout;
}

int* izeros(int length)
{
  int *vec;
  vec = ivecallocate(length);
  int i;
  for(i = 0;i < length;i++)
    {
      vec[i] = 0;
    }
  return vec;
}

double* vecallocate(int length)
{
  double* vecout;
  vecout = (double*)malloc(length*sizeof(double));
  return vecout;
}

void CreateVec(double vec[],int len,int start,int inc)
{
  double val = start-inc;
  int i;
  for(i = 0;i < len;i++)
    {
      val = val + inc;
      vec[i] = val;
    }
}

int* iCreateVec(int start,int end,int inc)
{
  int length = CalcLength(start,end,inc);
  int *vec;
  vec = ivecallocate(length);
  int val = start-inc;
  int i;
  for(i = 0;i < length;i++)
    {
      val = val + inc;
      vec[i] = val;
    }
  return vec;
}
double** matzeros(int row,int col)
{
  double **mat;
  mat = matrixallocatedbl(row,col);
  int i,j;
  for(i = 0;i < row;i++)
    {
      for(j=0;j<col;j++)
	{
	  mat[i][j] = 0;
	}
    }
  return mat;
}

double** transpose(double **inmat,int row,int col)
{
  double **outmat;
  outmat = matrixallocatedbl(col,row);
  int i,j;
  for(i = 0;i < row;i++)
    {
      for(j=0;j<col;j++)
	{
	  outmat[j][i] = inmat[i][j];
	}
    }
  return outmat;

}

double* zeros(int length)
{
  double *vec;
  vec = vecallocate(length);
  int i;
  for(i = 0;i < length;i++)
    {
      vec[i] = 0;
    }
  return vec;
}

double sum(double vec[],int len)
{
  double ans = 0;
  int i;
  for(i = 0;i<len;i++)
    {
      ans = ans + vec[i];
    }

  return ans;

}

long long int isum(int vec[],int len)
{
  long long int ans = 0;
  long int ansbefore = 0;
  int i;
  for(i = 0;i<len;i++)
    {
      if(vec[i] != 0)
	{
	  ansbefore = ans;
	  ans = ans + vec[i];
	  //Check Sum
	  //if ((ans - vec[i]) != ansbefore)
	  //{
	  //  iscalardisp(ansbefore,"ansbefore");
	  //  iscalardisp(vec[i],"vec[i]");
	  //  iscalardisp(ans,"ansnow");
	  //  PAUSE();
	  //}
	}
    }

  return ans;

}

double squaresum(double vec[],int len)
{
  double ans = 0;
  int i;
  for(i = 0;i<len;i++)
    {
      ans = ans + SQUARE(vec[i]);
    }

  return ans;

}

int numprime(int number)
{
  //Is this number a prime number
  //ans = isprime(number);
  int counter = 2;
  int go = 1;
  while((counter < (number/2)) && go)
  {
    if(number % counter != 0)
      {
	counter = counter + 1;
      }
    else
      {
	go = 0;
	return 0;
      }
  }
  if(counter >= (number/2))
    {
      return 1;
    }
}

void int2str(char str[],int val)
{
  sprintf(str,"%d",val);
}

int str2int(char str[],int pointer)
{
  int val;
  //strdisp(str);
  if(str[pointer] == zero)
    {
      val = 0;
    }
  if(str[pointer] == one)
    {
      val = 1;
    }
  if(str[pointer] == two)
    {
      val = 2;
    }
  if(str[pointer] == three)
    {
      val = 3;
    }
  if(str[pointer] == four)
    {
      val = 4;
    }
  if(str[pointer] == five)
    {
      val = 5;
    }
  if(str[pointer] == six)
    {
      val = 6;
    }
  if(str[pointer] == seven)
    {
      val = 7;
    }
  if(str[pointer] == eight)
    {
      val = 8;
    }
  if(str[pointer] == nine)
    {
      val = 9;
    }
  return val;
}


int GetLength(char str[])
//Gets length of a string of numbers
{
  int i = 0;
  while(1)
    {
      if((str[i] == zero) || (str[i] == one) || (str[i] == two) || (str[i] == three) || (str[i] == four) || (str[i] == five) || (str[i] == six) || (str[i] == seven) || (str[i] == eight) || (str[i] == nine))
	{
	  i = i + 1;
	}
      else
	{
	  return i;
	}
    }
}

int ispalindrome(int number)
{
  int out = 1;
  char str[30];
  char cur;
  int2str(str,number);
  int len = GetLength(str);
  int mark;
  if((len % 2) > 0)
    {
      //odd number
      mark = (len-1)/2;
    }
  else
    {
      //even number
      mark = (len/2);
    }
  int i;
  for(i=0;i<mark;i++)
    {
      if(str[i] != str[len-i-1])
	{
	  out = 0;
	}
    }
  return out;
}

void SetStr(char outvec[],char invec[],int start,int end,int inc)
{
  int pointer = 0;
  int i = start;
  while(i<=end)
    {
      outvec[pointer] = invec[i];
      pointer++;
      i += inc;
    }
}

void setvalue(int vec[],int start,int inc,int len,double val)
{
  int i = start;
  while(i<=len)
    {
      vec[i] = val;
      i += inc;
    }
}

void isetvalue(int vec[],int start,int inc,int len,int val)
{
  int i = start;
  while(i<=len)
    {
      vec[i] = val;
      i += inc;
    }
}

void primes(int vec[],int len)
{

  vec[0] = 0;
  double end = (double)vec[len-1];
  //first get rid of all multiples of 2
  isetvalue(vec,3,2,len,0);
  //idisp(vec,len,"primenumbers w/o 2's");
  int i=2;

  while(i<=len-1)
    {
      //      if(vec[i] <= sqrt(end)) //only for g++
      if(vec[i] <= end)
	{
	  if(vec[i] != 0)
	    {
	      isetvalue(vec,SQUARE(vec[i])-1,vec[i],len,0);
	      //idisp(vec,len,"primenumbers w/o #'s");
	      //PAUSE();
	    }
	}
      i += 2;
    }
}

double randnum(int start,int end)
{
  srand(time(NULL)+rand());
  //rand() returns a random integer from 0 to RAND_MAX
  //So first we need to scale rand by the start and end values
  double scalefactor = RAND_MAX/(double)(end-start);
  double num = rand()/scalefactor;
  return (num + start);
}

int* dlmread(char name[])

{
  FILE* file;
  file = fopen(name,"r");

  if (!file)
    {
      disp("file not found");
      return 0;
    }
  int ii;
  int counter = 0;
  int length = 1;
  int* coord = izeros(1);
  int* copy = izeros(1);

  while (!feof(file))
    {
      //Save Time File
      fscanf(file,"%i",&coord[counter]);
      //Copy tcoord into tcopy
      icopyvec(coord,copy,length);
      //Make tcoord longer
      coord = izeros(length+1);
      //Copy tcopy back into tcoord
      icopyvec(copy,coord,length);
      //Make tcopy longer
      copy = izeros(length+1);
      length++;
      counter++;
    }
  length-=2;
  ivecdisp(coord,length,"vec");
  DLMLENGTH = length;
  return coord;

}

void importfile(char* filename)
{
  //successive calls of strtok(NULL,"=")) will pull in data
  //Import file
  FILE*file;
  file = fopen(filename,"r");

  if(!file)
    {
      disp(filename);
      disp("not found");
    }

  //Obtain File Size
  
  fseek(file,0,SEEK_END); //this places the cursor at the end of the file
  int filesize = ftell(file); //this returns the current position of the cursor
  rewind(file); //this returns the cursor at the begginning of the file

  //Allocate memory for entire file
  char * buffer; 
  buffer = (char*) malloc(sizeof(char)*filesize);

  //Now Copy entire File into buffer
  fread(buffer,1,filesize,file);

  //Now We can Read in all of the initialization variables
  char*CurrentChar = NULL;
  CurrentChar = strtok(buffer,"="); //breaks buffer into tokens
  //  return CurrentChar;
}

void vecplusminus(double c[],double a[],double b[],double factor,int len)
{
  int i;
  for(i=0;i<len;i++)
    {
      c[i] = a[i] + factor*b[i];
    }
}

double** matmult(double** a,int rowa,int cola,double** b,int rowb,int colb)
{
  int mm,nn,ll;
  double** mat = matzeros(rowa,colb);
  for(mm = 0;mm<rowa;mm++)
    {
      for(nn = 0;nn<colb;nn++)
	{
	  for(ll = 0;ll<cola;ll++)
	    {
	      mat[mm][nn] = mat[mm][nn] + a[mm][ll]*b[ll][nn];
	    }
	}
    }
  return mat;
      
}

double norm(double vec[],int len)
{
  double ans;
  int idx;
  for(idx=0;idx<len;idx++)
    {
      ans = ans + SQUARE(vec[idx]);
    }
  
  return(sqrt(ans));

}

double ** cross3(double** a,double **b)
{
  //out = a x b
  double a1 = a[0][0];
  double a2 = a[1][0];
  double a3 = a[2][0];
  double b1 = b[0][0];
  double b2 = b[1][0];
  double b3 = b[2][0];
  double** out = matzeros(3,1);
  out[0][0] = -a3*b2 + a2*b3;
  out[1][0] = a3*b1 - a1*b3;
  out[2][0] = -a2*b1 + a1*b2;

  return(out);

}

double** matplusminus(double** a,double** b,double factor,int row,int col)
{
  int i,j;
  double**outmat = matzeros(row,col);
  for(i=0;i<row;i++)
    {
      for(j=0;j<col;j++)
	{
	  outmat[i][j] = a[i][j] + factor*b[i][j];
	}
    }
  return(outmat);

}
/* void odeRK4(char*filename,double tinitial,double tfinal,double xinitial[],int numstates,double timestep,int extra,int next) */
/* { */
/*   //Initialize Variables */
/*   double rkalfa[4] = {1,2,2,1},rkbeta[4] = {0,0.5,0.5,1}; */
/*   double T = tinitial; */
/*   //Initialize State Vector */
/*   double xstate[numstates],xnext[numstates],xdot[numstates]; */
/*   double xdotRK4[numstates],xdottemp[numstates]; */
/*   copy(xstate,xinitial,numstates); */
/*   copy(xnext,xstate,numstates); */
/*   vecdisp(xstate,numstates,"Initial Conditions"); */
/*   //Setup File for writing */
/*   FILE*outfile; */
/*   outfile = fopen(filename,"w"); */
/*   int threshold = 0,idx; */
/*   int percent; */
/*   //Main Integration */
/*   while (T <= tfinal+1e-8) */
/*     { */
/*       //clear variables */
/*       for(idx=0;idx<numstates;idx++) */
/* 	{ */
/* 	  xdotRK4[idx] = 0; */
/* 	  xdot[idx] = 0; */
/* 	} */

/*       //Output State to File */
/*       fprintf(outfile,"%lf%s",T," "); */
/*       for(idx=0;idx<numstates;idx++) */
/* 	{ */
/* 	  fprintf(outfile,"%lf%s",xstate[idx]," "); */
/* 	} */
/*       fprintf(outfile,"%s","\n"); */
/*       //Notify User of Progress */
/*       percent = (100*T/tfinal); */
/*       if (percent >= threshold) */
/* 	{ */
/* 	  printf("%s%i%s%s","Simulation ",percent," Complete","\n"); */
/* 	  threshold = threshold + next; */
/* 	} */
      
/*       //Call Derivative Function 4 times */
/*       for(idx=0;idx<4;idx++) */
/* 	{ */
/* 	  //!xnext = xstate + xdot*rkbeta(idx)*timestep */
/* 	  vecplusminus(xnext,xstate,xdot,rkbeta[idx]*timestep,numstates); */
/* 	  //DerivativeFunction(xdot,t,xnext,extra); */
/* 	  //!xdottemp = xdotRK4 + xdot*rkalfa(idx)/6 */
/* 	  vecplusminus(xdottemp,xdotRK4,xdot,rkalfa[idx]/6,numstates); */
/* 	  //!xdotRK4 = xdottemp */
/* 	  copy(xdotRK4,xdottemp,numstates); */
/* 	} */
      
/*       //Step states and Time */
/*       T = T + timestep; */
/*       vecplusminus(xnext,xstate,xdotRK4,timestep,numstates); */
/*       copy(xstate,xnext,numstates); //xstate = xnext */

/*     } */
/* } */

// Copyright - Carlos Montalvo 2015
// You may freely distribute this file but please keep my name in here
// as the original owner
