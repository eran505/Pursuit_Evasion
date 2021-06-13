//
// Created by eranhe on 6/6/21.
//

//
// Created by ise on 18.11.2019.
//

#include "States/State.hpp"


template<typename S>
std::array<int,14> State<S>::to_mini_string() const
{
    std::array<int,14> arr{};
    int ctr=0;
    for(int j =0;j<this->budgets.size();++j){
        auto position = this->dataPoint[j*2];
        auto speed = this->dataPoint[j*2+1];
        for(int i=0;i<position.capacity;++i) arr[ctr+i]= position.array[i];
        ctr+=position.capacity;
        for(int i=0;i<speed.capacity;++i) arr[ctr+i]=speed.array[i];
        ctr+=speed.capacity;
    }
    arr[12]=this->budgets[0];
    arr[13]=this->budgets[1];
    return arr;
}
template<typename S>
std::vector<int> State<S>::to_mini_vector() const
{
    std::vector<int> arr(13);
    int ctr=0;
    for(int j =0;j<this->budgets.size();++j){
        auto position = this->dataPoint[j*2];
        auto speed = this->dataPoint[j*2+1];
        for(int i=0;i<position.capacity;++i) arr[ctr+i]= position.array[i];
        ctr+=position.capacity;
        for(int i=0;i<speed.capacity;++i) arr[ctr+i]=speed.array[i];
        ctr+=speed.capacity;
    }
    arr[12]=this->budgets[0];
    //arr[13]=this->budgets[1];
    return arr;
}


//:dataPoint(other.dataPoint),budgets(other.budgets),g_grid(other.g_grid),takeOff(other.takeOff) { }






template<typename S>
bool State<S>::isEndState(agentEnum idStr) const{
    const auto& pos = dataPoint[idStr*2];
    return this->g_grid->is_at_goal(pos);
}





template<typename S>
vector<float> State<S>::state_to_features() const
{
    vector<float> arr(12);
    return arr;
}








template<typename S>
void State<S>::getAllPos(vector<Point> &vec)const
{        for (short x = A; x != LAST; ++x) vec.push_back(this->dataPoint[x]); }

template<typename S>
State<S> State<S>::make_state_from_array(std::array<int, 14> a) {
    State s;
    int acc=0;
    for(int k=0;k<3;k++)
        s.dataPoint[0].array[k]=a[acc+k];
    acc+=3;
    for(int k=0;k<3;k++)
        s.dataPoint[1].array[k]=a[acc+k];
    acc+=3;
    for(int k=0;k<3;k++)
        s.dataPoint[2].array[k]=a[acc+k];
    acc+=3;
    for(int k=0;k<3;k++)
        s.dataPoint[3].array[k]=a[acc+k];
    s.budgets[0]=a[12];
    s.budgets[1]=a[13];
    return s;

}










