#include "structre/particle.h"
#include <iostream>
#include <my.h>

int main(){
    my::Particle part;
    if (part.hasFiniteMass()){
        std::cout<<"true"<<part.getInverseMass()<<std::endl;
    }else{
        std::cout<<"false"<<std::endl;
    }
    return 0;
}