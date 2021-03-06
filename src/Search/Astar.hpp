//
// Created by eranhe on 6/6/21.
//

#ifndef PE_ASTAR_HPP
#define PE_ASTAR_HPP

#include "utils/game_util.hpp"
#include <vector>
#include <deque>
#include <numeric>
#include <functional>
#include <set>
#include <cmath>
namespace AStar
{
//    enum D{d=2};
    struct StatePoint
    {
        Point pos;
        Point speed;
        bool operator == (const StatePoint& coordinates_) const;
        inline Point & get_speed(){return speed;}
        inline Point & get_position_ref(){return pos;}
        //~StatePoint(){delete (pos);delete (speed);}
        [[nodiscard]] string toStr() const { return pos.to_hash_str()+speed.to_hash_str();}
        StatePoint(const Point& p , const Point& s){
            this->pos=p;
            this->speed=s;
        }
        StatePoint(const StatePoint &other){
            //cout<<"copy StatePoint"<<endl;
            pos=Point(other.pos);
            speed=Point(other.speed);
        }
        friend ostream& operator<<(ostream& os, const StatePoint& dt)
        {
            os<<"{"<<dt.pos.to_str()<<","<<dt.speed.to_str()<<"} ";
            return os;
        }
        [[nodiscard]] uint64_t getHashStateAttacker() const
        {
            return Point::hashNnN(pos.hashConst(),
                                  speed.hashConst(Point::maxSpeed));
        }
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(const StatePoint &a, const StatePoint &b,int maxSpeed)>;
    using CoordinateList = std::vector<StatePoint>;

    struct Node
    {
        double G, H;
        StatePoint *coordinates;
        vector<Node*> parent;

        explicit Node(StatePoint *coord_, Node *parent_);
        explicit Node(StatePoint *coord_);
        ~Node(){//cout<<"~ node"<<endl;
            delete coordinates;}
        Node(const Node &other);
        uint getScore() const;
        string toStr(){ return coordinates->toStr();}
        [[nodiscard]] u_int64_t getHashNode()const{return coordinates->getHashStateAttacker();}
        //unsigned int hash(int maxSize){ return coordinates->hash()}
    };



    using NodeSet = std::set<Node*>;
    typedef vector<uint> unitVector;
    typedef std::vector<Node*> listNode ;
    typedef unordered_map<u_int64_t, map<int,int>*>  policyDict;
    class Generator
    {

        static Node* findNodeOnList(const unordered_map<u_int64_t ,Node*>& nodes_, StatePoint &coordinates_);

        [[maybe_unused]] static void releaseNodes(NodeSet& nodes_);
        static void releaseMAP(const unordered_map <u_int64_t ,Node*>& map_);

    public:
        const vector<vector<StatePoint>>& get_deep_list_nodes_ref_const(){return this->deepListNode;}
        auto get_deep_list_nodes(){return this->deepListNode;}
        unordered_map<u_int64_t ,std::pair<short,StatePoint>> * hashDictStates;
        unordered_map<u_int64_t, map<int,int>*> *dictPoly;
        void print_pathz(Node *l);
        void getDict(unordered_map<u_int64_t,vector<double>*>* dict,double weight=1.0) const;
        void pathsToDict();
        void pathsToDict_rec(Node &item);
        void getDictPolicy(const listNode &l);
        void setConsistentZ(bool bol){this->consistentZ=bol;}
        Generator(uint absMaxSpeed, Point& girdSize);
        ~Generator()=default;


        void filterPaths();

        [[maybe_unused]] void consistentZFilter();
        void setHeuristic(const HeuristicFunction& heuristic_);
        int findPath( StatePoint& source_, const StatePoint& target,bool toDict = true,bool not_exactly=true);
        listNode findComplexPath(StatePoint& source_,Point& mid, const StatePoint& target_);
        int count_pathz(vector<Node*> *l );
        void changeMaxSpeed(uint speedMaxNew){this->absMaxSpeed=speedMaxNew;}
        void setMaxPATH(unsigned long numberMax){maxPath=numberMax;}
        void dictPolyClean() const{
            for (auto &itemI:*this->dictPoly)
                delete(itemI.second);
            this->dictPoly->clear();
        }

    private:
        vector<vector<StatePoint*>> allPath;
        unsigned long maxPath;
        uint absMaxSpeed;
        Point gridSize;
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        bool consistentZ = true;
        vector<Point> operatorAction;
        list<Node*> listPrint;
        vector<vector<StatePoint>> deepListNode;

        void copyAgentPaths();
        StatePoint* applyActionState(const StatePoint &cur,const Point &action) const{
            auto speed_copy = Point(cur.speed);
            speed_copy+=action;
            speed_copy.change_speed_max(absMaxSpeed);
            auto pos_copy = Point(cur.pos);
            pos_copy+=speed_copy;
            return new StatePoint{pos_copy,speed_copy};
        }
        void printMee(listNode nz){
            for(Node* item: nz){
                //cout<<"------\t------\t------\t------"<<endl;
                print_pathz(item);
            }
        }
        void deepCopyPaths()
        {
            this->deepListNode.clear();
            for (auto &item : allPath){
                vector<StatePoint> listI;
                listI.reserve(item.size());
                for(auto &x : item)
                {
                    listI.push_back(*x);
                }
                this->deepListNode.push_back(listI);
            }
            this->allPath.clear();
        }


        void addToStateDict(u_int64_t key, StatePoint *stateS) const;

        void helper(bool toDict);
    };

    class Heuristic
    {

    public:
        static uint manhattan(const StatePoint &source_, const StatePoint &target_, int maxSpeed);
        static uint zero(const StatePoint &source_, const StatePoint &target_, int maxSpeed);

        static uint manhattan2(const StatePoint &source_, const StatePoint &target_, int maxSpeed);
    };

    struct StateSearch{
        Point pos;
        Point speed;

        template<typename P, //Template type checking
                typename = typename std::enable_if<std::is_constructible<Point, P>::value>
        >
        StateSearch(P &&pos_arg,P &&speed_arg)
                :pos(std::forward<P>(pos_arg)),speed(std::forward<P>(speed_arg)){}

    };
}



#endif //PE_ASTAR_HPP
