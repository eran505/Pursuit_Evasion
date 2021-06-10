//
// Created by eranhe on 6/6/21.
//

#ifndef PE_STATE_HPP
#define PE_STATE_HPP


#include "utils/game_util.hpp"
#include "Game/Grid.hpp"
#include "States/Data/StateData.hpp"

enum agentEnum :short{A=0,D=1,LAST=2};


template<typename S = Single>
class State{

public:

    std::array<Point,4> dataPoint; // [A_POS,A_SPEED,D_POS,D_SPEED]
    //std::array<int,2> budgets{};
    u_int32_t time_t=0;
    S budgets;
    Grid *g_grid = nullptr;
    bool takeOff = false;



    [[nodiscard]] const Point& get_speed_ref(agentEnum agent_id)const{return dataPoint[agent_id*2+1];}
    [[nodiscard]] Point get_speed(agentEnum agent_id)const{return dataPoint[agent_id*2+1];}
    //void set_speed(agentEnum agent_id,const Point& p){dataPoint[agent_id*2+1]=p;}

    [[nodiscard]] const Point& get_position_ref(agentEnum agent_id)const{return dataPoint[agent_id*2];}
    //void set_position(agentEnum agent_id,const Point& p){dataPoint[agent_id*2]=p;}
    [[nodiscard]] Point get_position(agentEnum agent_id)const{return dataPoint[agent_id*2];}

    template<typename P>
    void set_speed(agentEnum agent_id,P&& p){dataPoint[agent_id*2+1]=std::forward<P>(p);}

    template<typename P>
    void set_position(agentEnum agent_id,P&& p){dataPoint[agent_id*2]=std::forward<P>(p);}

    [[nodiscard]] S get_budget()const{return budgets;}

    [[maybe_unused]] [[nodiscard]] S& get_budget_ref()const{return budgets;}

    void set_budget(S b){budgets=b;}





    State():g_grid(nullptr),takeOff(false){};

    State(const State &other) = default;


    [[nodiscard]] vector<float> state_to_features()const;

    void assignment( State &other);

    void assignment( const State *other,agentEnum idname);

    [[nodiscard]] double isGoal(agentEnum idStr)const;

    [[nodiscard]] bool isEndState(agentEnum idStr)const;


    void getAllPos(vector<Point> &vec)const;

    bool applyAction(agentEnum id, const Point &action, int max_speed);

    [[nodiscard]] bool is_collusion(agentEnum id_player,agentEnum op_player)const;
    //void getAllPosOpponent(vector<Point> &results,char team);
    std::ostream& operator<<(std::ostream &strm) const {
        return strm <<this->to_string_state();
    }


    [[nodiscard]] u_int64_t getHashValue()const {

        //u_int64_t  seed = this->budgets[0];//+1000*this->budgets[0];
        u_int64_t  seed = 0 ;
        size_t i=0;

        while(true)
        {
            seed ^= this->dataPoint[i].array[0] + 0x9e3779b9 + (seed << 7u) + (seed >> 2u);
            seed ^= this->dataPoint[i].array[1] + 0x9e3779b9 + (seed << 7u) + (seed >> 2u);
            seed ^= this->dataPoint[i].array[2] + 0x9e3779b9 + (seed << 7u) + (seed >> 2u);
            if (i%2==0)
                seed ^= this->budgets[i].hash_it() + 0x9e3779b9 + (seed << 7u) + (seed >> 2u);
            if(i++==3) break;
        }
        seed ^=  this->dataPoint[2].accMulti(1) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }



    [[nodiscard]] string to_string_state() const {
        string sep="_";
        string str;
        str.append(this->time_t+"_");
        for(int j =0;j<this->budgets.size();++j){
            string id_name = j==0?"A":"D";
            auto my_pos = this->dataPoint[j*2];
            auto my_speed =  this->dataPoint[j*2+1];

            str.append(id_name);
            str.append(sep);
            str.append(my_pos.to_str());
            str.append(sep);
            str.append(my_speed.to_str());
            str+="|";
        }
        str.append(sep);
        str.append(this->budgets.to_string());
        return str;

    }

    [[nodiscard]] std::array<int,14> to_mini_string() const;


    [[nodiscard]] static const Point& getValue(const map<string const,Point>& map , const string &str_name)
    {
        if(auto pos = map.find(str_name); pos==map.end())
        {
            throw std::invalid_argument( "received missing value" );
        }else{ return pos->second;}

    }
    [[nodiscard]] std::vector<int> to_mini_vector() const;


    bool is_collusion_radius(agentEnum id_player, agentEnum op_player, const Point &window);


    void add_player_state(agentEnum name_id, const Point &m_pos, const Point *m_speed, S budget_b) {
        this->dataPoint[name_id*2]=m_pos;
        this->dataPoint[name_id*2+1]=*m_speed;
        this->budgets[name_id]=budget_b;
    }

    void add_player_state(agentEnum name_id, const Point& m_pos, const Point& m_speed, S budget_b) {
        this->set_budget(name_id,budget_b);
        this->set_speed(name_id,m_speed);
        this->set_position(name_id,m_pos);
    }

    bool applyAction(agentEnum id,  Point &action, int max_speed, int jumps);
    static State make_state_from_array(std::array<int,14> a);


};




#endif //PE_STATE_HPP
