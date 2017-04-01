#include <iostream>
using namespace std;

void f(long long i)
{
    long long f0 = 0,f1 = 1;
    if(i == 0)
    {
        cout<< f0 << endl;
    }
    else if( i == 1)
        cout<< f1 <<endl;
    else
    {
        long long fn = 0 ;
        long long fnMinusOne = 1;
        long long fnMinusTwo = 0;
        for(long long j = 2 ;j< i+1 ;j++)
        {
            fn = fnMinusOne +fnMinusTwo;
            fnMinusTwo = fnMinusOne;
            fnMinusOne = fn;

        }

        cout<<fn<<endl;
    }
}

int main()
{
    for(long long  i = 0; i<80;i++)
    {
        f(i);
    }
    return 0;
}
