import numpy as np
from scipy.optimize import linear_sum_assignment
from collections import OrderedDict

#  use the algorithm from https://pdfs.semanticscholar.org/7826/ab4b502957df07e0fc35586bf2ffe0a35c87.pdf

def hungarian(cost_matrix):
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    return row_ind,col_ind



def labp_solve(cost_matrix):
    #step1  define the c_star
    len_r=len(cost_matrix)
    len_c=len(cost_matrix[0])
    c_star=max([min([row[j] for row in cost_matrix]) for j in range(0,len_c)]+[min(cost_matrix[i][:]) for i in range(0,len_r)])
    infeasible=True
    cost_list=[j  for sub in cost_matrix for j in sub if j>c_star]
    #cost_list=[x for x in cost_list if x>c_star]
    cost_list.sort()
    cost_list=list(OrderedDict.fromkeys(cost_list))
    while infeasible:
        try:
            cost=[[cost_matrix[i][j] if cost_matrix[i][j]<=c_star else np.inf for j in range(len_c)] for i in range(len_r)]
            row_ind,col_ind=hungarian(cost)
            return row_ind,col_ind,c_star
        except ValueError:
            if len(cost_list)>0:
                c_star=cost_list.pop(0)
            else:
                print("infeasible!")
                raise ValueError
            

def test_lbap():
    cost_matrix=[[8,2,3,3],[2,7,5,8],[0,9,8,4],[2,5,6,3]]
    row_ind,col_ind,c_star=labp_solve(cost_matrix)
    print(row_ind,col_ind,c_star)
        
if __name__=="__main__":
    test_lbap()
