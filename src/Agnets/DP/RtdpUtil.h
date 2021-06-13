//
// Created by eranhe on 6/12/21.
//

#ifndef PE_RTDPUTIL_H
#define PE_RTDPUTIL_H


#include "States/State.hpp"

namespace RtdpUtils {

    struct StackItemRtdp {
        State<> state;
        Point action;
        u_int64_t entryID;

        StackItemRtdp(State<> &&state_val,Point &&action_val,u_int64_t entryID_val)
                :state(state_val),action(action_val),entryID(entryID_val){}

        [[nodiscard]] string to_string() const {
            return "{"+state.to_string_state() + ", " + action.to_str()+ "}";
        }


    };

    template<typename S>
    class StackActionzer {

        std::vector<S> stack;
        u_int32_t ctr_stack=0;
    public:
        void inset_to_stack(S &&item) {
            stack.emplace_back(item);
            ctr_stack++;
        }
        bool is_empty()
        {
            return ctr_stack == 0;
        }
        S& pop() {
            return stack[--ctr_stack];
        }
        State<> get_last_state(){
            assert(!stack.empty());
            return (stack.front()).state;
        }
        Point get_last_action(){
            assert(!stack.empty());
            return (stack.front()).action;
        }
        void clear()
        {
            stack.clear();
            ctr_stack=0;
        }
        void print_stak()
        {
            if(is_empty())
            {
                cout<<"{}"<<endl;
                return;
            }
            for(int i=stack.size()-1;i>=0;--i)
            {
                cout<<" [stack] ["<<i<<"] "<<stack[i].to_string()<<"\n";
            }
            cout<<endl;
        }


    };
}


#endif //PE_RTDPUTIL_H
