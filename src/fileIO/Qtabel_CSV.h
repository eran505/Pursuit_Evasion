//
// Created by eranhe on 7/18/21.
//

#ifndef PE_QTABEL_CSV_H
#define PE_QTABEL_CSV_H

#include "Agnets/DP/MemoryRTDP.hpp"
#include "utils/game_util.hpp"
#include "fileIO/saver.hpp"
#include "fileIO/processerCSV.hpp"

namespace QTabel_CSV{

    void Q_to_csv(Table *Q,const string& home,string&& prefix)
    {

        auto d = Point::getVectorActionUniqie();
        Saver<string> file_manger(home+LOG_DIR+prefix+"Q.csv",10);
//        file_manger.inset_one_item("ID");
//        for (int i = 0; i < 27; ++i) file_manger.inset_one_item(std::to_string(d[i][0])+"|"+std::to_string(d[i][1])+"|"+std::to_string(d[i][2]));
//        file_manger.inset_endLine();
        file_manger.inset_one_item("ID");
        for (int i = 0; i < 27; ++i) file_manger.inset_one_item(std::to_string(i));
        file_manger.inset_endLine();
        for(auto &item:*Q)
        {
            file_manger.inset_one_item(std::to_string(item.first));
            for (int i = 0; i < 27; ++i) {
                file_manger.inset_one_item(std::to_string(item.second.operator[](i)));
            }
            file_manger.inset_endLine();
        }
    }

    void state_map_to_csv(const std::unordered_map<u_int64_t,State<>> &map,const string& home,string &&prefix)
    {

        auto d = Point::getVectorActionUniqie();
        Saver<string> file_manger(home+LOG_DIR+prefix+"MAP.csv",10);
        file_manger.inset_one_item("ID");
        file_manger.inset_one_item("State");
        file_manger.inset_endLine();
        for(auto &item:map)
        {
            file_manger.inset_one_item(std::to_string(item.first));
            file_manger.inset_one_item(item.second.to_string_state());
            file_manger.inset_endLine();
        }
    }


}


#endif //PE_QTABEL_CSV_H
