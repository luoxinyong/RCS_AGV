#include <iostream>
#include <time.h>

using namespace std;


constexpr double square(double x)
{
    return x * x;
}

int main()
{
    double a = 4;
    double y = square(a);
    cout << "square(" << a << ") = " << y << endl;
    printf("square(%f) = %f\n", a, y);
    cin>>a;
    y = square(a);
    printf("square(%f) = %f\n", a, y);


}