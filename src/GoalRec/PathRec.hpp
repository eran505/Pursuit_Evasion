//
// Created by ERANHER on 9.6.2021.
//

#ifndef PE_PATHREC_HPP
#define PE_PATHREC_HPP

//
// Created by eranhe on 4/19/21.
//


#include "utils/game_util.hpp"
//#define MAX_SPEED_E 2
//#define MAX_SPEED_P 1
#include "NodeGR.hpp"
//#define VERBOSE


class GoalRecognition{
    std::vector<std::vector<Point>> all_pathz;
    //std::unique_ptr<std::unordered_map<u_int64_t,NodeG*>> node_dict;
    std::unique_ptr<NodeG> root;
    NodeG* curr_ptr;
    std::vector<u_int32_t> min_step_path;
    Point my_loction;
    std::vector<double> probabilities;
    std::default_random_engine rng;
    bool start_move=false;
    int found_path=-1;
    int MAX_SPEED_E;
    int MAX_SPEED_P;
    vector<NodeG*> l_cur;
public:



    explicit GoalRecognition(int _seed,int max_speedA,int max_speedD):root(std::make_unique<NodeG>()),my_loction(-1),rng(_seed)
    {
        cout<<root->pos.to_str()<<endl;
        curr_ptr=root.get();
        MAX_SPEED_E=max_speedA;
        MAX_SPEED_P = max_speedD;

    }
    NodeG* get_root(){return this->root.get(); ;}
    void set_my_location(const Point& p);

    void load_agent_paths(const std::vector<std::vector<Point>> &pathz,vector<double> &&path_probabilties,const vector<u_int16_t>& paths_name);

    void add_path(const std::vector<Point> &l_path,u_int16_t  id_path,NodeG* cur_root,u_int16_t name);

    static NodeG* search_node(const vector<std::unique_ptr<NodeG>> &continer,const Point& p);

    std::vector<NodeG*> get_list_cur(){return l_cur;};

    void printTree();

    [[nodiscard]] NodeG* get_curr_ptr()const{return curr_ptr;};

    void reset_ptr()
    {

        curr_ptr= root.get();
        start_move=false;
        this->found_path=-1;
        set_my_location(my_loction);
        l_cur.push_back(curr_ptr);
    }
    Point get_my_loc(){return this->my_loction;}
    void search_tree(const Point &p, NodeG* ptr,int d);
    bool is_stay_inplace(const Point &evader);
    size_t make_decsion(const Point &evader);
    template< typename C>
    u_int32_t get_the_most_likely_path(std::vector<C> idx_list);
    void populate_distances();

    template< typename V>
    static size_t get_max_index_random(std::vector<V> list_vec,std::default_random_engine &rng);

    Point do_action(const Point &evader,const Point &my_speed );

    void advance_curr_ptr(const Point &p,int d);
    static std::vector<NodeG*> get_all_successors(NodeG* ptr,u_int32_t steps);


    void search_tree_with_jumps(const Point &p, int d, const vector<NodeG *> &list_nodes);

    static vector<NodeG *> get_all_successors(const vector<NodeG *> &ptr, u_int32_t steps);

    std::vector<int> get_split_vector();
};


void GoalRecognition::load_agent_paths(const std::vector<std::vector<Point>> &pathz,vector<double> &&path_probabilties,const std::vector<u_int16_t>& paths_name) {
    probabilities = std::move(path_probabilties);
    all_pathz = pathz;
    for (size_t i = 0; i < pathz.size(); ++i)
        GoalRecognition::add_path(pathz[i], u_int16_t(i), root.get(),paths_name[i]);

    cout<<"init"<<endl;
}


void GoalRecognition::add_path(const std::vector<Point> &l_path,const u_int16_t id_path,NodeG* cur_root,u_int16_t name) {

    NodeG* node = cur_root;
    for (size_t i=0;i<l_path.size();++i)
    {

        if (auto res = search_node(node->child,l_path[i]); res == nullptr){
            auto& bPtr = node->child.emplace_back(std::make_unique<NodeG>(l_path[i],l_path.back(),name,probabilities[id_path]));
            node = bPtr.get();
        }
        else{
            res->add_goal(l_path.back(),name,probabilities[id_path]); // add goal if need
            node = res;

        }
        if (node->min_step>l_path.size()-i)
            node->min_step=int(l_path.size()-i);

    }

}

NodeG *GoalRecognition::search_node(const vector<std::unique_ptr<NodeG>> &continer, const Point &p) {
    for (const auto &item : continer){
        if(item->pos==p)
            return item.get();
    }
    return nullptr;
}

void GoalRecognition::printTree()
{
    cout<<"----Tree---"<<endl;
    auto q = std::queue<NodeG*>();
    auto lv_q = std::queue<int>();
    auto map_d = std::unordered_map<u_int64_t,short>();
    int _level;
    q.push(this->root.get());
    lv_q.push(0);
    while(!q.empty())
    {

        _level = lv_q.front();
        lv_q.pop();
        NodeG* cur_node = q.front();
        q.pop();
        cout<<"lv:"<<_level<<"\t";
        for (auto& item :cur_node->child) {
            cout<<"[";

            std::for_each(item->goal_list.begin(),item->goal_list.end(),[](const pair<Point,std::vector<u_int16_t>>& g){
                std::string s;
                if (g.second.empty())
                    s="";
                else {
                    s = std::accumulate(g.second.begin() + 1, g.second.end(), std::to_string(g.second[0]),
                                        [](const std::string &a, int b) {
                                            return a + ',' + std::to_string(b);
                                        });
                }
                cout<<g.first[1]<<" {"<<s<<"} :";
            });

            q.push(item.get());
            cout<<"]";
            lv_q.push(++_level);
        }
        if (cur_node->child.empty())
            cout<<"[/]";


        cout<<endl;
    }
    cout<<_level<<endl;
}

void GoalRecognition::search_tree(const Point &p, NodeG* ptr,int d) {
    //cout<<"d:"<<d<<"\t"<<ptr->pos.to_str()<<endl;
    if(d==0 or ptr->child.empty()) {

        if(ptr->pos==p){
            curr_ptr = ptr;
            //cout<<" [y] ";
        }
        return;
    }
    for (const auto& ele : ptr->child)
    {
        search_tree(p,ele.get(),d-1);
    }

}

void GoalRecognition::advance_curr_ptr(const Point &p,int d)
{
    //this->search_tree(p,curr_ptr,d);
    this->search_tree_with_jumps(p,d,l_cur);
}


void GoalRecognition::search_tree_with_jumps(const Point &p,int d,const std::vector<NodeG*> &list_nodes)
{
    std::vector<NodeG*> l;
    for (auto& item : list_nodes) {
        auto res = GoalRecognition::get_all_successors(item, d);
        std::copy_if(res.begin(),res.end(),std::back_inserter(l),[&p](const NodeG* n){
            if (n->pos==p) return true;
            return false;
        });
    }
    this->l_cur=l;
    cout<<"";

}



size_t GoalRecognition::make_decsion(const Point &evader ) {
    this->search_tree(evader,curr_ptr,1);
    //assert(evader==curr_ptr->pos);

    vector<u_int32_t > most_likely_paths;
    double max_prob=0;
    size_t ctr=0;
    size_t max_indx=0;
    size_t likely_path=0;

    if (curr_ptr->goal_list.size()>1) {
        max_indx = get_max_index_random(curr_ptr->goal_likelihood, rng);
    }
    if (curr_ptr->goal_list[max_indx].second.size()>1) {
        std::vector<u_int16_t> list_of_paths = curr_ptr->goal_list[max_indx].second;
        likely_path = get_the_most_likely_path(list_of_paths);
    }
    else  likely_path=curr_ptr->goal_list[max_indx].second.front();

#ifdef VERBOSE
    cout << "goal_likly:" << curr_ptr->goal_list[max_indx].first.to_str() << endl;
    cout<<"likely_path:"<<likely_path<<endl;


    for(auto& [g,list_idx]: curr_ptr->goal_list)
    {
        cout<<"Goal:\t"<<g.to_str()<<"\n{";
        cout<<"P(Goal):\t"<<curr_ptr->goal_likelihood[ctr]<<"\n{";
        for (auto idx_i : list_idx)
        {
            cout<<idx_i<<",";
        }
        cout<<"}\n";
        ctr++;
    }
#endif
    return likely_path;
}

void GoalRecognition::populate_distances() {
    //cout<<my_loction.to_str()<<endl;
    assert(my_loction.sum()>0);
    for(const auto& path_i : this->all_pathz)
        this->min_step_path.push_back(getMaxDistance(path_i[path_i.size()-2], this->my_loction));
}
void GoalRecognition::set_my_location(const Point& p)
{
    if(this->my_loction == p)
        return;
    my_loction=p;
    this->populate_distances();
}

template<typename V>
size_t GoalRecognition::get_max_index_random(vector<V> list_vec, std::default_random_engine &rng) {
    std::vector<u_int32_t> l;
    auto it = std::max_element(list_vec.begin(),list_vec.end());
    size_t index_it = 0;
    std::for_each(list_vec.begin(),list_vec.end(),[&](V &val){
        if (val==*it)
            l.push_back(index_it);
        index_it++;
    });
    if (l.size()==1)
        return l.front();
    std::shuffle(std::begin(l), std::end(l), rng);
    return l.front();
}

template< typename C>
u_int32_t GoalRecognition::get_the_most_likely_path(std::vector<C> idx_list) {
    std::vector<double> l;
    std::transform(idx_list.begin(), idx_list.end(), std::back_inserter(l),
                   [this](u_int32_t c) -> double { return probabilities[c]; });
    return idx_list[get_max_index_random(l,this->rng)];
}

bool GoalRecognition::is_stay_inplace(const Point &evader) {

    this->search_tree(evader,curr_ptr,1);
    //cout<<"E:"<<evader.to_str()<<"\tP:"<<curr_ptr->pos.to_str()<<endl;
    size_t min_step = curr_ptr->min_step;
    assert(evader==curr_ptr->pos);
    std::vector<size_t> relevent_pathz;
    if (curr_ptr->goal_list.size()==1)
        if (curr_ptr->goal_list.front().second.size()==1)
        {
            this->found_path=curr_ptr->goal_list.front().second.front();
            return false;
        }

    for (const auto& [g,list_idx]:curr_ptr->goal_list){
        for (const auto id_pathz : list_idx ) {
            if (this->min_step_path[id_pathz] >= (curr_ptr->min_step - MAX_SPEED_E-1))
                return false;
        }
    }
    return true;

}

Point GoalRecognition::do_action(const Point &evader,const Point &my_speed) {
    if (!start_move){
        if (is_stay_inplace(evader))
            return Point(0);
        else
            start_move=true;
    }
    auto path = this->make_decsion(evader);
    if (found_path!=-1)
        path=found_path;
    auto goal_loc = this->all_pathz[path][this->all_pathz[path].size()-2];
    Point dif = goal_loc-my_loction;
    dif.change_speed_max(MAX_SPEED_P);
    return dif-my_speed;
}

//void GoalRecognition::build_fast_node_dict() {
//    auto q = std::queue<NodeG*>();
//    q.push(this->root.get());
//    while (!q.empty())
//    {
//        NodeG* cur = q.front();
//        q.pop();
//        node_dict->try_emplace(cur->hash_it(),cur);
//        for (auto & i : cur->child) {
//            q.push(i.get());
//        }
//    }
//
//
//}
//
//NodeG *GoalRecognition::fast_search(u_int64_t id_hash) const {
//    return this->node_dict->at(id_hash);
//}

std::vector<NodeG *> GoalRecognition::get_all_successors(const std::vector<NodeG*> &ptr, u_int32_t steps) {
    auto results = std::vector<NodeG *>();
    for(auto i : ptr) {
        auto l = get_all_successors(i, steps);
        if(results.empty()) results=l;
        else{
            for(auto& item :l)
                if(auto pos = std::find_if(results.begin(),results.end(),[&item](const NodeG* rc)->bool{
                        return rc->hash_it()==item->hash_it();}); pos==results.end())
                    results.push_back(item);
        }
    }
    return results;
}

std::vector<NodeG *> GoalRecognition::get_all_successors(NodeG *ptr, u_int32_t steps) {
    auto results = std::vector<NodeG *>();
    auto q = std::queue<NodeG*>();
    q.push(ptr);
    auto q_depth = std::queue<int>();
    q_depth.push(steps);
    while (!q.empty())
    {
        NodeG* cur = q.front();
        q.pop();
        int depth = q_depth.front();
        q_depth.pop();
        if (depth==0 or cur->child.empty()){
            results.push_back(cur);
            continue;
        }
        --depth;
        for (auto & i : cur->child) {
            q.push(i.get());
            q_depth.push(depth);
        }
    }
    return results;
}


std::vector<int> GoalRecognition::get_split_vector()
{
    std::vector<int> d;
    auto size_max_path = std::max_element(std::begin(all_pathz),std::end(all_pathz),[](const vector<Point> &A, const vector<Point> &B){return A.size()<B.size();})->size();
    int t=0;
    while (t<size_max_path)
    {
        std::vector<Point> l;
        for (auto & path_num : all_pathz)
        {
            if (t>path_num.size())
                continue;
            if (std::find(l.begin(),l.end(),path_num[t])==l.end())
                l.push_back(path_num[t]);

        }
        d.push_back(l.size());
        t++;
    }
    cout<<d<<endl;
    cout<<endl;
    return d;

};




#endif //PE_PATHREC_HPP
