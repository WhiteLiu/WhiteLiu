/*
 * sort by value*/


#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

class ListNode
{
public:
    ListNode(int v = 0):value(v),next(NULL){}
    int value;
    ListNode* next;
};

ListNode* add(ListNode* list,int value = 0)
{
    if(list == NULL)
    {
        list = new ListNode(value);
        return list;

    }
    else if(list->next == NULL)
    {
        //cout<<value<<endl;
        list->next = new ListNode(value);

    }
    else add(list->next,value);
    return list;
}

ListNode* listNodeReverse(ListNode* list)
{
    if(list == NULL)
    {
        return list;
    }
    else if(list->next == NULL)
    {
        return list;
    }
    else if(list->next->next == NULL)
    {
        ListNode* prev = list->next;
        ListNode* list_reverse = prev;
        prev->next = list;
        list->next = NULL;
        return list_reverse;
    }
    else
    {
        ListNode* list_reverse = listNodeReverse(list->next);
        ListNode* next = list_reverse;
        while (next->next != NULL)
        {
            next = next->next;
        }
        next->next = list;
        list->next = NULL;
        return list_reverse;
    }
}

void ListNodePrint(ListNode* list)
{
    if(list != NULL)
    {
        cout<<list->value<<endl;
        ListNodePrint(list->next);
    }
}

void ListNodePrintReverse(ListNode* list)
{
    if(list != NULL)
    {
        ListNodePrintReverse(list->next);
        cout<<list->value<<endl;
    }
}


int main()
{

    ListNode* p = new ListNode;
    for(int i = 1;i<10;i++)
    {
        p = add(p,i);
    }

    ListNodePrint(p);
    p = listNodeReverse(p);
    //ListNodePrint(p);
    ListNodePrintReverse(p);
    return 0;
}
