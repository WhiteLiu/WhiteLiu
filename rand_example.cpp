#include "common.h"

int main()
{

    int a = std::rand();
    int i = 0;
    while(true)
    {
        srand(++i);
        cout<<a<<endl;
    }
}
