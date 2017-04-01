/*Plus one until ....*/

#include <iostream>
#include <vector>
using namespace std;

class PrintNumber
{

public:
    int a;
    PrintNumber* ptr;
public:
    PrintNumber(int i = 0,PrintNumber* p = 0):a(i),ptr(p)
    {}
    void plusOne()
    {
        if(a == 9 && ptr == 0)
        {
            this->a = 0;
            this->ptr = new PrintNumber(1,0);
        }
        else if(a == 9 && ptr != 0)
        {
            a = 0;
            ptr->plusOne();
        }
        else
            a++;
    }
    void print()
    {

        if(ptr != 0)
        {
            ptr->print();
            cout<<a;
        }
        if(ptr == 0)
        {
            cout<<a;
        }
    }
};


int main()
{
    PrintNumber* start = new PrintNumber(1);
    int i =120;
    while(i>0)
    {
        i--;
        start->print();
        cout<<endl;
        start->plusOne();

    }

    return 0;
}
