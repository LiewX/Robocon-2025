#include <iostream>
#include <string>

void testfunc(double num1, double num2,double* ans);

int main()
{
    double num1=9.2;
    double num2=1.21;
    double ans;
    testfunc(num1,num2,&ans);
    std::cout<<ans;
    return 0;
}

void testfunc(double num1, double num2, double* ans)
{
    double number;
    *ans=num1*num2;
}