#include <iostream>
#include <base.h>

int main(){
    my::Vector3 v = my::Vector3(1,2,3);
    my::Vector3 u = my::Vector3(1,3,2);
    v*=2;
    std::cout<<v*u<<std::endl;
    return 0;
}