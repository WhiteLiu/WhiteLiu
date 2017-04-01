/*
 * sort by value*/


#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

typedef pair<char,int> Pair;
ostream& operator <<(ostream& out,const Pair& a)
{
    return out<<a.first<<" "<<a.second<<endl;
}

bool sort_by_value(const Pair& a,const Pair& b )
{
    return a.second>b.second;
}

struct sortPair
{
    bool operator()(const Pair& a,const Pair& b)
    {
        return a.second>b.second;
    }
};

int main()
{

    vector<Pair> vec;
    int ii = 0;
    for(char i = 'A';i != 'z';i++,ii++)
    {
        vec.push_back(Pair(i,ii%9));
    }

    sort(vec.begin(),vec.end(),sort_by_value);
    for(size_t i = 0;i!=vec.size();i++)
    {
        cout<<vec[i]<<endl;

    }

    return 0;
}
