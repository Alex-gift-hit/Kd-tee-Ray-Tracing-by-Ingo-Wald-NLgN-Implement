#include<iostream>
#include<vector>
#include"Vector.hpp"

int main(){
    Vector3f a{1,2,3};
    std::cout<<a<<std::endl;
    a[0]=2;
    std::cout<<a<<std::endl;
    std::cout<<a[0]<<" "<<a[1]<<" "<<a[2]<<std::endl;
    std::vector<int> c;
    std::cout<<c.size()<<std::endl;

    float e = 3.3;
    int f = 5;
    std::cout<<e*f<<std::endl;

    std::vector<int *> cc;
    int * i1;
    i1 = &f;
    cc.push_back(i1);
    std::cout<<i1<<std::endl;
    for (auto i: cc){
        std::cout<<i<<std::endl;
    }

    std::vector<int> bb {1,5,4,6,3,2};
    std::sort(bb.begin(),bb.end(),[](int i1, int i2){
        return i1<i2;
    });
    for(auto i : bb){
        std::cout<<i<<std::endl;
    }
    float cd=1.0, ef=2.0,temp;
    for(int i=0; i<99;i++){
        temp = (cd + ef)/2;
        ef = temp;
        std::cout<< cd << " "<< ef << std::endl;
    }

    c.push_back(f);
    c.push_back(f);
    std::cout<<c.size()<<std::endl;
    return 0;
}