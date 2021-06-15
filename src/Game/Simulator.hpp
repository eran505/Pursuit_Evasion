//from os_util import walk_rec
//import pandas as pd
//import matplotlib.pyplot as plt
//
//
//# "episodes";"Collision";"Wall";"Goal"
//
//def plot_all_csvs(dir_path="/home/eranhe/car_model/debug"):
//res = walk_rec(dir_path,[],"Eval.csv")
//for item in res:
//name = str(item).split('/')[-1].split('.')[0]
//mode = int(str(name).split("_")[2][-1])
//
//df_i = pd.read_csv(item,sep=';')
//plt.plot(df_i['Collision'].values,label=name)
//plt.legend()
//plt.show()
//
//if __name__ == '__main__':
//plot_all_csvs()