//
//from os_util import walk_rec
//import pandas as pd
//import matplotlib.pyplot as plt
//        import numpy as np
//        import os
//# "episodes";"Collision";"Wall";"Goal"
//def namer(mode,option,h):
//name=""
//if mode == 0:
//name+="RTDP(position,"
//elif mode ==1:
//name+="RTDP(plan,"
//elif mode ==2:
//name+="RTDP(posT,"
//
//if option==0:
//name+=" atomic,"
//elif option==1:
//name += " option,"
//
//if h == 0:
//name += " zero"
//elif h == 1:
//name += " all_path_next_step"
//elif h == 2:
//name += " rel_path_next_step"
//elif h == 3:
//name += "all_paths_end"
//elif h == 4:
//name += "all_paths_end_fast"
//
//
//name+=")"
//return name
//
//        def con_to_dico(path_to_con="/home/eranhe/car_model/debug/con16.csv"):
//x=5
//father = os.path.dirname(path_to_con)
//d_data,max_ep = plot_all_csvs()
//df = pd.read_csv(path_to_con)
//df['MAX_Collision']=df.apply(Max_coll,data=d_data,axis=1)
//df['AVG_Collision_{}'.format(x)]=df.apply(avg_coll,data=d_data,tail=x,axis=1)
//df['episodes']=df.apply(get_T,data=d_data,axis=1)
//df['MAX_episodes']=max_ep
//for item_ky in d_data.keys():
//        df_data = d_data[item_ky]['df']
//if d_data[item_ky]['mode']==-1:
//val =  d_data[item_ky]['df']['Collision'][-x:].mean()
//arr = np.full(max_ep, val)
//plt.plot(arr, label="Plan_Rec",color='k')
//else:
//name = namer(d_data[item_ky]['mode'],d_data[item_ky]['option'],d_data[item_ky]['h'])
//plt.plot(df_data['Collision'].values , label= name )
//
//print(df[["ID","X","Y","Z","h","o","m","MAX_Collision",'AVG_Collision_{}'.format(x),"episodes","MAX_episodes"]])
//df.to_csv("{}/res.csv".format(father))
//plt.legend()
//plt.savefig("{}/plot.png".format(father))
//plt.show()
//
//def Max_coll(row,data):
//id_ = str(row['ID'])
//data = data[id_]
//return data['df']['Collision'].max()
//
//def avg_coll(row,data,tail):
//id_ = str(row['ID'])
//data = data[id_]
//res = data['df']['Collision'][-tail:].mean()
//return res
//
//        def get_T(row,data):
//id_ = str(row['ID'])
//data = data[id_]
//res = len(data['df']['Collision'].values)
//return res
//
//
//        def path_to_config(name):
//arr_data = str(name).split("_")
//d={}
//for item in arr_data:
//if item[0]=="S":
//d["seed"]=item[1:]
//elif item[0]=="I":
//d['id_exp']=item[1:]
//elif item[0]=="M":
//d['mode']=int(item[1:])
//elif item[0]=="O":
//d['option']=int(item[1:])
//elif item[0]=="H":
//d['h']=int(item[1:])
//d['name']='_'.join(str(name).split('_')[:-1])
//return d
//
//        def plot_all_csvs(dir_path="/home/eranhe/car_model/debug"):
//res = walk_rec(dir_path,[],"Eval.csv")
//d_l={}
//max_ep=0
//for item in res:
//name = str(item).split('/')[-1].split('.')[0]
//d = path_to_config(name)
//df=pd.read_csv(item,sep=';')
//df.dropna(axis=0,inplace=True)
//d['df'] = df
//if max_ep<len(df):
//max_ep=len(df)
//d_l[d["id_exp"]]=d
//return d_l,max_ep
//
//
//        def just_plot(dir_path="/home/eranhe/car_model/debug"):
//res = walk_rec(dir_path,[],"Eval.csv")
//d_l={}
//for item in res:
//name = str(item).split('/')[-1].split('.')[0]
//d = path_to_config(name)
//df=pd.read_csv(item,sep=';')
//df.dropna(axis=0,inplace=True)
//d['df'] = df
//        plt.plot(d['df']["Collision"].values,label=name)
//plt.legend()
//plt.show()
//exit(0)
//if __name__ == '__main__':
//# just_plot()
//con_to_dico()