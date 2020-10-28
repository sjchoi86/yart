import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
from scipy.spatial import distance
from tensorflow.keras.datasets import mnist


class NormalizerClass(object):
    def __init__(self,raw_data,eps=1e-8):
        self.raw_data = raw_data
        self.eps      = eps
        self.mu       = np.mean(self.raw_data,axis=0)
        self.std      = np.std(self.raw_data,axis=0)
        
        self.std[np.where(self.std < 0.1)] = 1.0 # small variance exception handling
        
        self.nzd_data = self.get_nzdval(self.raw_data)
        self.org_data = self.get_orgval(self.nzd_data)
        self.max_err  = np.max(self.raw_data-self.org_data)
    def get_nzdval(self,data):
        n = data.shape[0]
        nzd_data = (data - np.tile(self.mu,(n,1))) / np.tile(self.std+self.eps,(n,1))
        return nzd_data
    def get_orgval(self,data):
        n = data.shape[0]
        org_data = data*np.tile(self.std+self.eps,(n,1))+np.tile(self.mu,(n,1))
        return org_data
    
def get_color_2d(x):
    """
        Get color with 
    """
    c = np.concatenate((x[:,0:1],x[:,1:2]),axis=1)
    c = (c-np.min(x,axis=0))/(np.max(x,axis=0)-np.min(x,axis=0))
    r, g, b = 1.0-c[:,1:2], c[:,0:1], 0.5-np.zeros_like(c[:,0:1])
    c = np.concatenate((r,g,b),axis=1)
    return c

def gpu_sess(): 
    config = tf.ConfigProto(); 
    config.gpu_options.allow_growth=True
    sess = tf.Session(config=config)
    return sess    

def print_tf_tensor(sess,tf_tensor):
    tf_tensor_val = sess.run(tf_tensor)
    print ("name:[%s] shape:%s"%(tf_tensor.name,tf_tensor_val.shape))
    print (tf_tensor_val)
    
def grid_2d(x0min,x0max,x1min,x1max,nx0,nx1):
    x0s,x1s = np.meshgrid(np.arange(x0min,x0max,(x0max-x0min)/nx0),
                          np.arange(x1min,x1max,(x1max-x1min)/nx1))
    x = np.dstack([x0s,x1s]).reshape(-1,2) 
    return x

def get_2d_synthetic_data_for_swae(
    xmin=0.0,xmax=1.5,ymin=0.0,ymax=1.5,MAKE_IMBALANCE=True):
    # Relaxed X (with imbalance)
    x0res,x1res = 200,200
    # xmin,xmax,ymin,ymax = 0,1.5,0,1.5
    if MAKE_IMBALANCE:
        x0res_imb,x1res_imb = 200,200 # 30% of data are concentrated in imb region 
        x_imb = grid_2d(0,1/4,0,1/4,x0res_imb,x1res_imb)
        x_relaxed = np.concatenate((grid_2d(xmin,xmax,ymin,ymax,x0res,x1res),
                                    x_imb),axis=0)
    else:
        x_relaxed = grid_2d(xmin,xmax,ymin,ymax,x0res,x1res)
    def proj_x(x):
        x_proj = np.copy(x)
        x_proj[np.where(x[:,0] > 1)[0],0] = 1.0
        x_proj[np.where(x[:,1] > 1)[0],1] = 1.0
        return x_proj
    x_feasible = proj_x(x_relaxed)
    c = get_color_2d(x_feasible) # compute the color *after* the projection
    def map_x_to_y(x):
        x0_org = np.copy(x[:,0])
        x1_org = np.copy(x[:,1])
        y = np.copy(x)
        
        y[:,0] = 0.25 + 1.5*(1-x0_org) + 0.0*x1_org # 1.5*(1-x0_org) + 0.5*x1_org
        y[:,1] = 0.3*np.sin(2*np.pi*(1-x0_org)) + 0.75*x1_org + 0.5 
        
        # y[:,0] = 1.5*(1-x0_org) + 0.5*x1_org
        # y[:,1] = 0.3*np.sin(2*np.pi*(1-x0_org)) + 0.5*x1_org + 0.75
        
        return y
    y_feasible = map_x_to_y(x_feasible) # mapping with the projected x
    return x_relaxed,x_feasible,y_feasible,c

def plot_2d_synthetic_data_for_swae(
    x_relaxed,x_feasible,y_feasible,c,MAKE_IMBALANCE=True,
    x_train_rel=None,x_train_feas=None,y_train_feas=None):
    # Plot
    ms,ma,am,tfs = 15,0.4,0.1,20
    plt.figure(figsize=(15,4.5))
    plt.subplot(131)
    plt.scatter(
        x_relaxed[:,0],x_relaxed[:,1],
        c=c,edgecolors=c,s=ms,marker='o',alpha=ma)
    if x_train_rel is not None:
        plt.scatter(
            x_train_rel[:,0],x_train_rel[:,1],
            c='w',edgecolors='k',s=40,marker='o',alpha=1.0,label='Training Data')
    rect = plt.Rectangle(
        xy=(0,0),width=1,height=1,fill=False,edgecolor='k',linewidth=2.5,
        label='Feasible Region')
    plt.gca().add_patch(rect)
    if MAKE_IMBALANCE:
        rect = plt.Rectangle(
            xy=(0,0),width=1/4,height=1/4,fill=False,edgecolor='b',linewidth=2.5,
            label='Imbalance Region')
        plt.gca().add_patch(rect)
    plt.title("Relaxed X",fontsize=tfs)
    plt.axis('square')
    plt.axis([0-am,2+am,0-am,2+am])
    plt.legend(loc='upper right',fontsize=15)

    plt.subplot(132)
    plt.scatter(
        x_feasible[:,0],x_feasible[:,1],
        c=c,edgecolors=c,s=ms,marker='o',alpha=ma)
    if x_train_feas is not None:
        plt.scatter(
            x_train_feas[:,0],x_train_feas[:,1],
            c='w',edgecolors='k',s=40,marker='o',alpha=1.0)
    plt.title("Feasible X",fontsize=tfs)
    plt.axis('square')
    plt.axis([0-am,2+am,0-am,2+am])

    plt.subplot(133)
    plt.scatter(
        y_feasible[:,0],y_feasible[:,1],
        c=c,edgecolors=c,s=ms,marker='o',alpha=ma)
    if y_train_feas is not None:
        plt.scatter(
            y_train_feas[:,0],y_train_feas[:,1],
            c='w',edgecolors='k',s=40,marker='o',alpha=1.0)
    plt.title("Feasible Y",fontsize=tfs)
    plt.axis('square')
    plt.axis([0-am,2+am,0-am,2+am])
    plt.show()
    
def get_nce_loss(hidden1,hidden2):
    # L2 normalize
    hidden1 = tf.math.l2_normalize(hidden1,-1)
    hidden2 = tf.math.l2_normalize(hidden2,-1)
    # Label and mask
    hidden1_large = hidden1 
    hidden2_large = hidden2 
    # [G_aa]_{i,j} = cosdist(xa_i,xa_j) - LARGE_NUM*delta(xa_i,xa_j)
    LARGE_NUM = 1e9
    temperature = 1.0
    n_batch = tf.shape(hidden1)[0]
    masks = tf.one_hot(tf.range(n_batch),n_batch,name='masks')
    logits_aa = tf.matmul(hidden1, hidden1_large, transpose_b=True) / temperature
    logits_aa = tf.subtract(logits_aa,LARGE_NUM*masks,name='logits_aa')
    # [G_bb]_{i,j} = cosdist(xb_i,xb_j) - LARGE_NUM*delta(xb_i,xb_j)
    logits_bb = tf.matmul(hidden2, hidden2_large, transpose_b=True) / temperature
    logits_bb = tf.subtract(logits_bb,LARGE_NUM*masks,name='logits_bb')
    # [G_ab]_{i,j} = cosdist(xa_i,xb_j)
    logits_ab = tf.matmul(hidden1, hidden2_large, transpose_b=True) / temperature
    logits_ba = tf.matmul(hidden2, hidden1_large, transpose_b=True) / temperature
    # Define the nce loss function
    labels = tf.one_hot(tf.range(n_batch),n_batch*2,name='labels')
    weights = 1.0
    loss_a = tf.losses.softmax_cross_entropy(
      labels, tf.concat([logits_ab, logits_aa], 1), weights=weights)
    loss_b = tf.losses.softmax_cross_entropy(
      labels, tf.concat([logits_ba, logits_bb], 1), weights=weights)
    nce_loss = loss_a + loss_b
    return nce_loss

def get_dpp_subset_from_K(K,n_sel):
    n_total = K.shape[0]
    remain_idxs = np.arange(n_total)
    sub_idx = np.zeros((n_sel))
    sum_K_vec = np.zeros(n_total)
    for i_idx in range(n_sel):
        if i_idx == 0:
            sel_idx = np.random.randint(n_total)
        else:
            curr_K_vec = K[(int)(sub_idx[i_idx-1]),:] 
            sum_K_vec = sum_K_vec + curr_K_vec
            k_vals = sum_K_vec[remain_idxs]
            min_idx = np.argmin(k_vals)
            sel_idx = remain_idxs[min_idx] 
        sub_idx[i_idx] = (int)(sel_idx)
        remain_idxs = np.delete(remain_idxs,np.argwhere(remain_idxs==sel_idx))
    sub_idx = sub_idx.astype(np.int) # make it int
    return sub_idx

def get_dpp_subset(x,n_sel,alpha=1000):
    from scipy.spatial import distance
    D = distance.cdist(x,x,'sqeuclidean') 
    K = np.exp(-alpha*D)
    sub_idx = get_dpp_subset_from_K(K,n_sel)
    x_sel = x[sub_idx,:] 
    return x_sel,sub_idx

def get_dpp_subset_trim(x,n_sel,alpha=1000):
    n_total = x.shape[0]
    n_th = 5000 # we assume that (n_th > n_sel)
    if n_total > n_th:
        x_temp,sub_idx_temp = get_rand_subset(x,n_sel=n_th)
        x_sel,sub_idx_local = get_dpp_subset(x_temp,n_sel=n_sel,alpha=alpha)
        sub_idx = sub_idx_temp[sub_idx_local]
    else:
        x_sel,sub_idx = get_dpp_subset(x,n_sel=n_sel,alpha=alpha)
    return x_sel,sub_idx

def get_rand_subset(x,n_sel):
    n_total = x.shape[0]
    sub_idx = np.random.permutation(n_total)[:n_sel]
    x_sel = x[sub_idx,:] 
    return x_sel,sub_idx

def dpp_mean(x,n_sel=100,alpha=1000):
    x_subset,_ = get_dpp_subset_trim(x,n_sel=n_sel,alpha=alpha)
    x_mean = np.mean(x_subset,axis=0)
    return x_mean

def dpp_partition(x,n_dpp=4000,seed=None):
    n = x.shape[0]
    if seed is not None:
        np.random.seed(seed=seed)
        idx_total = np.random.permutation(n)
    else:
        idx_total = np.random.permutation(n)
    x_dpps,K_dpps = [],[]
    for i_idx in range(n//n_dpp):
        idx_dpp = idx_total[(i_idx)*n_dpp:(i_idx+1)*n_dpp]
        x_dpp = x[idx_dpp,:] 
        x_dpps.append(x_dpp)
        K_dpp = np.exp(-1000*distance.cdist(x_dpp,x_dpp,'sqeuclidean'))
        K_dpps.append(K_dpp)
    return x_dpps,K_dpps

def dpp_sample(x_dpps,K_dpps,batch_size):
    idx = np.random.permutation(len(K_dpps))[0]
    x_dpp,K_dpp = x_dpps[idx],K_dpps[idx]
    return x_dpp[get_dpp_subset_from_K(K_dpp,batch_size),:]

def dpp_sample_pair(x1_dpps,K1_dpps,x2_dpps,K2_dpps,batch_size):
    idx = np.random.permutation(len(K1_dpps))[0] # select partition
    x1_dpp,K1_dpp = x1_dpps[idx],K1_dpps[idx]
    x2_dpp,K2_dpp = x2_dpps[idx],K2_dpps[idx]
    idx_subset = get_dpp_subset_from_K(K1_dpp,batch_size)
    x1_batch = x1_dpp[idx_subset,:]
    x2_batch = x2_dpp[idx_subset,:]
    return x1_batch,x2_batch

def get_nzr_stats(x_raw):
    x_min,x_max,x_mean = np.min(x_raw,axis=0),np.max(x_raw,axis=0),np.mean(x_raw,axis=0)
    x_range = x_max-x_min
    return x_mean,x_range

def nzd(x_raw,x_mean,x_range):
    """
    <Usage>
    dim = 2
    x_raw = np.random.randn(1000,dim) + 5*np.random.rand(1,dim)
    x_mean,x_range = get_nzr_stats(x_raw) # init normalizer
    x_nzd = nzd(x_raw,x_mean,x_range) # normalize
    x_org = unnzd(x_nzd,x_mean,x_range) # unnormalize
    plt.plot(x_raw[:,0],x_raw[:,1],'ro',label='Original')
    plt.plot(x_nzd[:,0],x_nzd[:,1],'bx',label='Normalized')
    plt.plot(x_org[:,0],x_org[:,1],'kx',label='Unnormalized')
    plt.axis('equal'); plt.grid()
    plt.legend(fontsize=15)
    plt.show()
    """
    x_nzd = (x_raw - x_mean) / x_range * 2
    return x_nzd

def unnzd(x_nzd,x_mean,x_range):
    x_org = x_nzd*x_range/2 + x_mean
    return x_org













