//
// Created by eranhe on 6/6/21.
//

#include "Search/Astar.hpp"

#include <algorithm>

using namespace std::placeholders;

bool AStar::StatePoint::operator == (const StatePoint& coordinates_) const
{
    for (int i = 0; i < this->speed.capacity; ++i)
        if (coordinates_.pos.operator[](i) != this->pos.operator[](i) or
            coordinates_.speed.operator[](i) != this->speed.operator[](i))
            return false;
    return true;

}



AStar::Node::Node(StatePoint *coordinates_, Node *parent_)
{
    coordinates = coordinates_;
    this->parent.push_back(parent_);
    G = H = 0;
}

AStar::Node::Node(AStar::StatePoint *coord_) {
    coordinates = coord_;
    G = H = 0;
}



AStar::uint AStar::Node::getScore() const
{
    return G + H;
}

AStar::Node::Node(const AStar::Node &other):G(other.G),H(other.H) {
    //cout<<"cpy Node\n";
    this->coordinates=new StatePoint(*other.coordinates);
    for (auto i : other.parent) {
        this->parent.push_back(i);
    }
}


AStar::Generator::Generator(uint absMaxSpeed,Point& girdSize)
{
    auto size_vector=pow(int(Point::D_point ::actionMax),int(Point::D_point::D));
    this->absMaxSpeed=absMaxSpeed;
    this->gridSize=girdSize;
    operatorAction.reserve(size_vector);
    setHeuristic(&Heuristic::manhattan2);
    //TODO: change it to rec function
    Point::getAllAction(operatorAction);
//    this->dictPoly = new policyDict();
//    hashDictStates = new unordered_map<u_int64_t ,std::pair<short,StatePoint>>();
}



void AStar::Generator::setHeuristic(const HeuristicFunction& heuristic_)
{
    heuristic = [heuristic_](auto && PH1, auto && PH2, auto && PH3) { return heuristic_(PH1, PH2, PH3); };
}
int  AStar::Generator::count_pathz(vector<Node*> *l ){
    int res=0;
    if (l->empty())
        return 1;
    for (auto & i : *l) {
        auto *list_i = &i->parent;
        res+=count_pathz(list_i);
    }
    return res;
}


AStar::listNode AStar::Generator::findComplexPath(AStar::StatePoint &source_, Point &mid, const AStar::StatePoint &target_) {
    // set the area that the agent must go in
    const int interval = 0;
    unsigned long oldMAXpath=this->maxPath;
    bool isFound=false;
    Point area;
    for (int i = 0; i < Point::D; ++i) {
        int min_val=mid[i]-interval;
        int max_val = mid[i]+interval;
        if (min_val<0)
            min_val = 0;
        if (max_val >= this->gridSize[i])
            max_val=this->gridSize[i]-1;

        area.array[i]=range_random(min_val,max_val);
    }
    //Point::getAllAction2(operatorAction,0);
    StatePoint midArea(area,Point());
    this->findPath(source_,midArea, false);
    //Point::getAllAction(operatorAction);
    int ctr_path=0;
    for (auto &pathI:this->deepListNode)
    {
        int sizeVec = pathI.size();
        if (ctr_path>=oldMAXpath) break;
        //this->maxPath=1;
        auto res = findPath(pathI.operator[](0),target_);
        cout<<"=="<<endl;
        if (res>0){//add the pathI to dictPolicy
            isFound= true;
            this->allPath.clear();
            vector<StatePoint*> l;
            for (auto &item: pathI) l.push_back(&item);
            this->allPath.push_back(l);
            pathsToDict();
            ctr_path++;
        }

    }

    if (!isFound)
        findComplexPath(source_,mid,target_);
    this->maxPath=oldMAXpath;
    return AStar::listNode();
}



int AStar::Generator::findPath( StatePoint& source_,const StatePoint& target_,bool toDict,bool not_exactly) {

    double epsilon = 0.000; // e>0 eliminate unnecessary movement in z-axis
    int k = 0; // finding sp+k  TODO: fix it missing paths
    Node *current = nullptr;

    //this->operatorAction=Point::getAllAction2(operatorAction,2); // TODO: del it the attacker cant take off

    u_int64_t optCost = this->gridSize.multi();
    CoordinateList path;
    multimap<double, Node *> openSetQ;
    unordered_map<u_int64_t , Node *> openSetID;
    unordered_map<u_int64_t , Node *> closedSet;
    auto *tmp = new StatePoint(source_);
    auto *first = new Node(tmp);
    first->G = 0;
    first->H = 0;
    vector<Node *> res;
    u_int MAX_PATH=1;
    openSetID.insert({first->getHashNode(), first});
    openSetQ.insert({first->getScore(), first});
    int ctr_my=0;
    while (!openSetQ.empty()) {

        //expand node
        current = openSetQ.begin()->second;

        if(res.size()>MAX_PATH)
            break;
        //debug
//        cout<<"expand:\t"<<current->toStr()<<endl;
        if (current->coordinates->pos.operator==(target_.pos)) {
            if(not_exactly or current->coordinates->speed.operator==(target_.speed))
            {
                if (current->G - k <= optCost) {
                    optCost = int(current->G);
                    res.push_back(current);
                }
            }
        }
        if (current->G - k > optCost)
            break;

        //inset to close list
        closedSet.insert({current->getHashNode(), current});

        // del elem from openList

        auto pos = openSetID.find(current->getHashNode());
        if (pos == openSetID.end())
            throw;
        openSetID.erase(pos);
        openSetQ.erase(openSetQ.begin());

        double totalCost = current->G + 1;

        for (const auto &itemI : this->operatorAction) {
            StatePoint *newCoordinates = this->applyActionState(*current->coordinates, itemI);

            // check bound
            if (newCoordinates->pos.out_of_bound(this->gridSize)) {
                delete (newCoordinates);
                continue;
            }
            // check if the new state diff from the cur
            if (*newCoordinates == *current->coordinates) {
                delete (newCoordinates);
                continue;
            }
            // check if in close list
            if (findNodeOnList(closedSet, *newCoordinates)) {
                delete (newCoordinates);
                continue;
            }

            // if change in z axis add epsilon cost to G
//            if (itemI.array[itemI.capacity - 1] != 0)
//                totalCost += epsilon;
            Node *successor = findNodeOnList(openSetID, *newCoordinates);
            //debug
            //cout<<"action:\t"<<itemI.to_str()<<"\t\t";

            if (successor == nullptr) {

                //debug
                //cout<<"new node:\t"<<newCoordinates->toStr()<<endl;

                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(*successor->coordinates, target_, this->absMaxSpeed);
                // insert to open list

                openSetID.insert({successor->getHashNode(), successor});
                openSetQ.insert({successor->getScore(), successor});
            } else {

                delete (newCoordinates);
                if (totalCost < successor->G) { //totalCost < successor->G
                    //debug
                    //cout<<"del open:\t"<<successor->toStr()<<endl;

                    auto ret = openSetQ.equal_range(successor->getScore());
                    for (auto it = ret.first; it != ret.second; ++it) {
                        if (it->second->coordinates == successor->coordinates) {
                            openSetQ.erase(it);
                            break;
                        }
                    }

                    successor->parent.clear();
                    successor->parent.push_back(current);
                    successor->G = totalCost;

                    openSetQ.insert({successor->getScore(), successor});
                } else if (successor->G == totalCost) {
                    //continue;
                    bool appendTo = true;
                    //debug
                    //cout<<"append:\t"<<successor->toStr()<<endl;
                    for (auto &index : successor->parent)
                        if (current->toStr() == index->toStr())
                            appendTo = false;
                    if (appendTo)
                        successor->parent.push_back(current);
                }
            }
        }
    }
    this->allPath.clear();
    //cout<<"A star find #"<<res.size()<<" paths"<<endl;
    printMee(res);
    //consistentZFilter();
    //shuffle path
#ifndef DEBUG
    //std::shuffle(allPath.begin(), allPath.end(),   std::default_random_engine(rand()));
#endif
    int size_paths = allPath.size();
    if (!toDict)
        deepCopyPaths();
//    auto ctr_path = this->count_pathz(&res);
    //this->getDictPolicy(res);
    // remove path if need
    //cout<<"allPath:\t"<<this->allPath.size()<<endl;

    filterPaths();
    this->pathsToDict();
    //TODO: clean up memo
    releaseMAP(openSetID);
    releaseMAP(closedSet);
    return size_paths;
}


[[maybe_unused]] void AStar::Generator::consistentZFilter(){
    if (!this->consistentZ)
        return;
    //cout<<"Before Size: "<<allPath.size()<<endl;

    auto newList = vector<vector<StatePoint*>>();

    // copy only positive numbers:
    std::copy_if (allPath.begin(), allPath.end(), std::back_inserter(newList), [](vector<StatePoint*> &vec) {
                      int idPath;

                      auto indexI = int(Point::D) - 1;
                      unsigned int sizeVec = vec.size();
                      bool isConsistent = true;
                      bool isFound = false;
                      for (size_t i = sizeVec-1; i != 0 ; --i) {
                          idPath = vec[i]->speed.array[indexI];
//            auto action = vec[i-1]->speed-vec[i]->speed;
//            auto actionZ = action.array[indexI];
//            cout<<action.to_str()<<endl;
                          if (!isFound) {
                              if (idPath < 0)
                                  isFound = true;
                          } else {
                              if (idPath > 0) {
                                  isConsistent = false;
                                  break;
                              }
                          }
                      }
//        if (!isConsistent)
//            //cout<<"skip"<<endl;
                      return isConsistent;
                  }
    );
    this->allPath.clear();
    this->allPath=move(newList);

    //cout<<"After Size: "<<allPath.size()<<endl;
}

void AStar::Generator::filterPaths() {
    unordered_map<string,vector<int>> dict;
    for (size_t k = 0 ; k<this->allPath.size();++k)
    {
        string idPath;
        for (size_t i = 0; i <allPath[k].size()-1; ++i) {
            idPath+=allPath[k][i]->pos.to_str();
        }
        auto pos = dict.find(idPath);
        if (pos == dict.end())
        {
            dict.insert({idPath,vector<int>()});
        }
        else
        {cout<<"keyIn"<<endl;}
        dict[idPath].push_back(k);
    }
    //cout<<"size:\t"<<dict.size()<<endl;
    //cout<<"end"<<endl;
}


AStar::Node* AStar::Generator::findNodeOnList(const unordered_map<u_int64_t ,Node*>& nodes_, StatePoint &coordinates_)
{
    auto pos = nodes_.find(coordinates_.getHashStateAttacker());
    if (pos==nodes_.end())
        return nullptr;
    return pos->second;
}

[[maybe_unused]] void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

void AStar::Generator::pathsToDict() {
    //auto dictTMP = new policyDict();
    uint ctr=0;

    for (auto &itemF : this->allPath) {
        if (maxPath<=ctr)
            break;
        ctr++;

        for (unsigned long i = 0; i < itemF.size()-1; ++i) {
            Point difAction = itemF[i]->speed.operator-(itemF[i + 1]->speed);

            u_int64_t key = Point::hashNnN(itemF[i+1]->pos.hashConst(),
                                           itemF[i+1]->speed.hashConst(Point::maxSpeed));
            if (key<u_int64_t(0))
                cout<<endl;
            if (i==0)
                cout<<itemF[i]->pos.to_str()<<endl;
            cout<<itemF[i+1]->pos.to_str()<<endl;
            addToStateDict(key,itemF[i+1]);
            auto ation_h = difAction.hashMeAction(Point::D_point::actionMax);
            auto pos = dictPoly->find(key);
            if (pos == dictPoly->end()) {
                dictPoly->insert({key, new map<int, int>()});
            }
            pos = dictPoly->find(key);
            auto posSec = pos->second->find(ation_h);
            if (posSec == pos->second->end()) {
                pos->second->insert({ation_h, 1});
            } else {
                posSec->second++;
            }
        }

    }
}

void AStar::Generator::addToStateDict(u_int64_t key,AStar::StatePoint *stateS) const
{
    auto pos = hashDictStates->find(key);
    if (pos==hashDictStates->end())
        hashDictStates->insert({key,{1,*stateS}});
    else
        pos->second.first++;
}

void AStar::Generator::pathsToDict_rec(Node &item) {
    if(!item.parent.empty())
    {
        for (unsigned long i = 0; i <item.parent.size(); ++i) {
            Point difAction = item.coordinates->speed.operator-(item.parent[i]->coordinates->speed);
            int key = Point::hashNnN(item.parent[i]->coordinates->pos.hashConst(),
                                     item.parent[i]->coordinates->speed.hashConst(Point::maxSpeed));


            auto ation_h=difAction.hashMeAction(Point::D_point::actionMax);
            auto pos = dictPoly->find(key);
            if (pos==dictPoly->end()) {
                dictPoly->insert({key, new map<int,int>()});
            }
            pos = dictPoly->find(key);
            auto posSec = pos->second->find(ation_h);
            if (posSec==pos->second->end())
            {
                pos->second->insert({ation_h,1});
            }
            else{
                posSec->second++;
            }
            pathsToDict_rec(*item.parent[i]);
        }
    }
}

void AStar::Generator::getDictPolicy(const AStar::listNode &l) {

    for(auto &item : l)
    {
        pathsToDict_rec(*item);
    }

}

void AStar::Generator::releaseMAP(const unordered_map<u_int64_t , Node *>& map_) {

    for (auto &item : map_) {
        //cout<<"del:\t"<<item.second->toStr()<<endl;
        delete(item.second);
    }
}

void AStar::Generator::getDict(unordered_map<u_int64_t, vector<double>*>* mapStateAction,const double weight) const {

for(const auto &item: *this->dictPoly)
{
int sumAll = accumulate( item.second->begin(), item.second->end(), 0,
                         []( int acc, std::pair<int, int> p ) { return ( acc + p.second ); } );

auto pos_tmp = mapStateAction->find(item.first);
if (pos_tmp==mapStateAction->end())
{
auto *vec = new vector<double>();
for (auto mapItem: *item.second) {
int tmp = mapItem.first;
int tmp2 = mapItem.second;
vec->push_back(tmp);
vec->push_back(double(tmp2)/double(sumAll)*weight);
}
mapStateAction->insert({item.first,vec});
}
else
{
for (auto mapItem: *item.second) {
int tmp = mapItem.first;
int tmp2 = mapItem.second;
pos_tmp->second->push_back(tmp);
pos_tmp->second->push_back(double(tmp2)/double(sumAll)*weight);
}
}
}
}

void AStar::Generator::print_pathz(Node *l) {
    listPrint.push_back(l);
    if (l->parent.size()==0)
    {
        vector<StatePoint*> x;
        for (auto &item:listPrint){
            //cout<<item->toStr()<<" <- ";
            x.push_back(item->coordinates);
        }
        allPath.push_back(x);
        listPrint.remove(l);
        //cout<<endl;
        return;
    }
    for (auto & i : l->parent) {
        print_pathz(i);
    }
    listPrint.remove(l);

}



AStar::uint AStar::Heuristic::manhattan(const StatePoint &source_, const StatePoint &target_,int maxSpeed)
{
    vector<int> l(source_.pos.capacity);

    for (int i = 0; i < source_.pos.capacity; ++i) {
        l[i]=abs((source_.pos.array[i]-target_.pos.array[i])/maxSpeed);
    }
    auto it = *max_element(std::begin(l), std::end(l));
    return it;
}

AStar::uint AStar::Heuristic::zero(const StatePoint &source_, const StatePoint &target_,int maxSpeed)
{
    return uint(0);
}

AStar::uint AStar::Heuristic::manhattan2(const StatePoint &source_, const StatePoint &target_,int maxSpeed)
{
    vector<int> l(source_.pos.capacity);
    int sum=0;
    for (int i = 0; i < source_.pos.capacity; ++i) {
        l[i]=abs((source_.pos.array[i]-target_.pos.array[i])/maxSpeed);
        sum+=l[i];
    }
    auto max_ele = *max_element(std::begin(l), std::end(l));
    auto min_ele = *min_element(std::begin(l), std::end(l));
    //return max_ele-1;
    return (1*max_ele+(1-sqrt(2))*min_ele);
    //return 1*(sum)+(1)*min_ele;
    //return it;
}
