import os
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
from wae import WassersteinAutoEncoderClass_v2
from util import get_2d_synthetic_data_for_swae,plot_2d_synthetic_data_for_swae,\
    gpu_sess,get_dpp_subset_from_K,get_dpp_subset_trim,get_nce_loss,\
    dpp_mean,dpp_partition,dpp_sample,dpp_sample_pair
        
        
        
class SharedWassersteinAutoEncoderClass(object):
    def __init__(self,name='swae',xname='wae_x',yname='wae_y',xdim=2,ydim=2,zdim=2,
                 hdims_Q=[256]*2,hdims_P=[256]*2,hdims_D=[256]*2,
                 actv_Q=tf.nn.relu,actv_P=tf.nn.relu,actv_D=tf.nn.relu,
                 actv_latent=None,actv_out=None,
                 adam_beta1=0.5,adam_beta2=0.9,adam_epsilon=1e-1,
                 ki=tf.contrib.layers.variance_scaling_initializer() 
                 ):
        self.name = name
        self.xdim = xdim
        self.ydim = ydim
        self.zdim = zdim
        self.adam_beta1 = adam_beta1
        self.adam_beta2 = adam_beta2
        self.adam_epsilon = adam_epsilon
        self.ki = ki
        self.W_x = WassersteinAutoEncoderClass_v2(
            name=xname,xdim=xdim,zdim=zdim,
            hdims_Q=hdims_Q,hdims_P=hdims_P,hdims_D=hdims_D,
            actv_Q=tf.nn.relu,actv_P=tf.nn.relu,actv_D=tf.nn.relu,
            actv_latent=None,actv_out=None,ki=self.ki
            )
        self.W_y = WassersteinAutoEncoderClass_v2(
            name=yname,xdim=ydim,zdim=zdim,
            hdims_Q=hdims_Q,hdims_P=hdims_P,hdims_D=hdims_D,
            actv_Q=tf.nn.relu,actv_P=tf.nn.relu,actv_D=tf.nn.relu,
            actv_latent=None,actv_out=None,ki=self.ki
            )
        self.build_graph()
        
    def build_graph(self):
        """
            Computational graph
        """
        # 1. Latent concensus loss
        STOP_GRAD = False
        if STOP_GRAD:
            self.l1_lc_losses = tf.reduce_sum(tf.abs(tf.stop_gradient(self.W_x.z_real)-self.W_y.z_real),axis=1) # [N]
            self.l2_lc_losses = tf.reduce_sum((tf.stop_gradient(self.W_x.z_real)-self.W_y.z_real)**2,axis=1) # [N]
        else:
            self.l1_lc_losses = tf.reduce_sum(tf.abs(self.W_x.z_real-self.W_y.z_real),axis=1) # [N]
            self.l2_lc_losses = tf.reduce_sum((self.W_x.z_real-self.W_y.z_real)**2,axis=1) # [N]
            
        self.l1_lc_coef = tf.placeholder(shape=[],dtype=tf.float32,name='l1_lc_coef') # [1]
        self.l2_lc_coef = tf.placeholder(shape=[],dtype=tf.float32,name='l2_lc_coef') # [1]
        self.lc_losses = self.l1_lc_coef*self.l1_lc_losses + self.l2_lc_coef*self.l2_lc_losses # [N]
        
        self.lc_norms = tf.reduce_sum((self.W_x.z_real)**2,axis=1) + tf.reduce_sum((self.W_y.z_real)**2,axis=1) # [N]
        self.lc_losses = self.lc_losses / (self.lc_norms + 0.1) # [N]
        
        self.lc_loss = tf.reduce_mean(self.lc_losses) # [1] 
        self.lr_lc = tf.placeholder(shape=[],dtype=tf.float32,name='lr_lc') # [1]
        with tf.variable_scope('%s/optm_lc'%(self.name),reuse=False):
            self.optm_lc = tf.train.AdamOptimizer(
                self.lr_lc,
                beta1=self.adam_beta1,beta2=self.adam_beta2,epsilon=self.adam_epsilon).minimize(
                self.lc_loss,
                var_list=self.W_x.q_vars+self.W_y.q_vars,name='optm_lc') # X&Y encoder only
        
        # 2. Constrastive loss
        self.nce_coef = tf.placeholder(shape=[],dtype=tf.float32,name='nce_coef') # [1]
        self.nce_loss = self.nce_coef*get_nce_loss(self.W_x.z_real,self.W_x.z_real_r)
        self.lr_nce = tf.placeholder(shape=[],dtype=tf.float32,name='lr_nce') # [1]
        with tf.variable_scope('%s/optm_nce'%(self.name),reuse=False):
            self.optm_nce = tf.train.AdamOptimizer(
                self.lr_nce,
                beta1=self.adam_beta1,beta2=self.adam_beta2,epsilon=self.adam_epsilon).minimize(
                self.nce_loss,
                var_list=self.W_x.q_vars,name='optm_nce') # X encoder only
        
        #  3. X to Y loss
        self.l1_x2y_coef = tf.placeholder(shape=[],dtype=tf.float32,name='l1_x2y_coef') # [1]
        self.l2_x2y_coef = tf.placeholder(shape=[],dtype=tf.float32,name='l2_x2y_coef') # [1]
        self.ph_x_x2y = tf.placeholder(shape=[None,self.xdim],dtype=tf.float32,name='ph_x_x2y') # [n x ydim]
        self.ph_y_x2y = tf.placeholder(shape=[None,self.ydim],dtype=tf.float32,name='ph_y_x2y') # [n x ydim]
        ki = tf.contrib.layers.variance_scaling_initializer() # tf.contrib.layers.xavier_initializer()
        bi = tf.constant_initializer(value=0)
        with tf.variable_scope(self.W_x.name,reuse=True):
            with tf.variable_scope('Q',reuse=True):
                net = self.ph_x_x2y
                for h_idx,hdim in enumerate(self.W_x.hdims_Q):
                    net = tf.layers.dense(net,hdim,activation=self.W_x.actv_Q,
                                          kernel_initializer=ki,bias_initializer=bi,
                                          name='hid_Q_lin_%d'%(h_idx))
                self.z_real_x2y = tf.layers.dense(net,self.W_x.zdim,activation=self.W_x.actv_latent,
                                                  kernel_initializer=ki,bias_initializer=bi,
                                                  name='z_real') # [n x z_dim]
        with tf.variable_scope(self.W_y.name,reuse=True):
            with tf.variable_scope('P',reuse=True):
                net = self.z_real_x2y
                for h_idx,hdim in enumerate(self.W_y.hdims_P):
                    net = tf.layers.dense(net,hdim,activation=self.W_y.actv_P,
                                          kernel_initializer=ki,bias_initializer=bi,
                                          name='hid_P_lin_%d'%(h_idx))
                self.x_recon_x2y = tf.layers.dense(net,self.W_y.xdim,activation=self.W_y.actv_out,
                                                   kernel_initializer=ki,bias_initializer=bi,
                                                   name='x_recon') # [n x x_dim]
                
        self.l1_x2y_losses = tf.reduce_sum(tf.abs(self.ph_y_x2y-self.x_recon_x2y),axis=1) # [N]
        self.l2_x2y_losses = tf.reduce_sum((self.ph_y_x2y-self.x_recon_x2y)**2,axis=1) # [N]
        self.x2y_losses = self.l1_x2y_coef*self.l1_x2y_losses+self.l2_x2y_coef*self.l2_x2y_losses # [N]
        self.x2y_loss = tf.reduce_mean(self.x2y_losses) # [1]
        self.lr_x2y = tf.placeholder(shape=[],dtype=tf.float32,name='lr_x2y') # [1]
        with tf.variable_scope('optm_x2y',reuse=False):
            self.optm_x2y = tf.train.AdamOptimizer(
                self.lr_x2y,
                beta1=self.adam_beta1,beta2=self.adam_beta2,epsilon=self.adam_epsilon).minimize(
                self.x2y_loss,
                var_list=self.W_x.q_vars+self.W_y.p_vars,name='optm_x2y') # X encoder + Y decoder
        
        
    def update(self,sess,
               x_recon_in=None,x_recon_out=None,x_latent=None,
               y_recon_in=None,y_recon_out=None,y_latent=None,
               x_glue=None,y_glue=None,
               x_nce_anc=None,x_nce_pos=None,
               x_x2y=None,y_x2y=None,
               latent_beta=1.0,l1_recon_coef=1.0,l2_recon_coef=1.0,wd_coef=1e-6,
               lr_recon_x=1e-3,lr_d_x=1e-3,lr_g_x=1e-3,
               lr_recon_y=1e-3,lr_d_y=1e-3,lr_g_y=1e-3,
               lr_lc=1e-3,l1_lc_coef=0.1,l2_lc_coef=0.1,
               lr_nce=1e-3,nce_coef=1e-3,
               lr_x2y=1e-3,l1_x2y_coef=1.0,l2_x2y_coef=1.0,
               lr_rate=1.0
              ):
        """
            Update
        """
        
        # 1. Projection-invariant mapping
        if (x_nce_anc is not None) and (nce_coef > 0):
            feeds = {self.W_x.ph_x_real:x_nce_anc,self.W_x.ph_x_real_r:x_nce_pos,
                     self.lr_nce:lr_nce*lr_rate,self.nce_coef:nce_coef
                    }
            nce_loss,_ = sess.run([self.nce_loss,self.optm_nce],feed_dict=feeds)
        else:
            nce_loss = 0.0
        
        # 2. Domain-specific X
        if (x_recon_in is not None) and ((l1_recon_coef > 0) or (l2_recon_coef > 0)):
            recon_loss_x,wd_loss_x,d_loss_x,g_loss_x = self.W_x.update(
                sess,x_recon_in,x_recon_out,x_latent,
                beta=latent_beta,l1_coef=l1_recon_coef,l2_coef=l2_recon_coef,wd_coef=wd_coef,
                lr_recon=lr_recon_x*lr_rate,lr_d=lr_d_x*lr_rate,lr_g=lr_g_x*lr_rate)
        else:
            recon_loss_x,wd_loss_x,d_loss_x,g_loss_x = 0.0,0.0,0.0,0.0
        
        # 3. Domain-specific Y
        if (y_recon_in is not None) and ((l1_recon_coef > 0) or (l2_recon_coef > 0)):
            recon_loss_y,wd_loss_y,d_loss_y,g_loss_y = self.W_y.update(
                sess,y_recon_in,y_recon_out,y_latent,
                beta=latent_beta,l1_coef=l1_recon_coef,l2_coef=l2_recon_coef,wd_coef=wd_coef,
                lr_recon=lr_recon_y*lr_rate,lr_d=lr_d_y*lr_rate,lr_g=lr_g_y*lr_rate)
        else:
            recon_loss_y,wd_loss_y,d_loss_y,g_loss_y = 0.0,0.0,0.0,0.0
        
        # 4. Glue X and Y (latent concensus)
        if (x_glue is not None) and (l2_lc_coef > 0):
            feeds = {self.W_x.ph_x_real:x_glue,self.W_y.ph_x_real:y_glue,
                     self.lr_lc:lr_lc*lr_rate,self.l1_lc_coef:l1_lc_coef,self.l2_lc_coef:l2_lc_coef
                    }
            lc_loss,_ = sess.run([self.lc_loss,self.optm_lc],feed_dict=feeds)
        else:
            lc_loss = 0.0
        
        # 5. X to Y mapping
        if (x_x2y is not None) and (l2_x2y_coef > 0):
            feeds = {self.ph_x_x2y:x_x2y,self.ph_y_x2y:y_x2y,
                     self.lr_x2y:lr_x2y*lr_rate,self.l1_x2y_coef:l1_x2y_coef,self.l2_x2y_coef:l2_x2y_coef
                    }
            x2y_loss,_ = sess.run([self.x2y_loss,self.optm_x2y],feed_dict=feeds)
        else:
            x2y_loss = 0.0
        
        return recon_loss_x,wd_loss_x,d_loss_x,g_loss_x,\
            recon_loss_y,wd_loss_y,d_loss_y,g_loss_y,\
            lc_loss,nce_loss,x2y_loss

            

def run_swae_synthetic_experiment(DATA_IMBALANCE=True,USE_DPP=True,USE_SSL=True):
    
    
    # Hyper parameters
    zdim = 2
    n_glue,n_ssl,n_dpp = 100,10000,5000
    seed = 2
    max_iter,batch_size,print_every,plot_every = 100000,8,1000,5000
    batch_size4latent = 256
    adam_beta1,adam_beta2,adam_epsilon = 0.9,0.9,1e-0 # 0.5,0.9,1e-0
    actv = tf.nn.relu # tf.nn.relu / tf.nn.softplus
    hdims,actv_Q,actv_P,actv_D = [128]*3,actv,actv,tf.nn.tanh
    latent_beta,wd_coef = 0.1,1e-8
    l1_recon_coef,l2_recon_coef = 0.0,5.0
    lr_rate_fr,lr_rate_to = 1.0,0.1 # global learning rate 
    lr_recon,lr_d,lr_g = 1e-3,2e-4,5e-4 # 1e-3,2e-4,5e-4
    lr_lc,l1_lc_coef,l2_lc_coef = 1e-3,0.0,10.0 # latent concensus
    if USE_SSL:
        lr_nce,nce_coef = 1e-3,0.01 # noise contrastive estimator
    else:
        lr_nce,nce_coef = 1e-3,0.0 
    lr_x2y,l1_x2y_coef,l2_x2y_coef = 1e-3,0.0,10.0 # x2y direct mapping 
    # ki = tf.contrib.layers.variance_scaling_initializer() 
    # ki = tf.truncated_normal_initializer(stddev=0.1) 
    ki = tf.contrib.layers.xavier_initializer()
    
    
    # Things to change
    max_iter = 40000
    lr_rate_fr,lr_rate_to = 1.0,0.1
    n_glue = 20
    batch_size = 16
    batch_size4latent = 16
    l2_x2y_coef = 50
    
    
    # Data
    # x_dom_rel / x_dom_feas / y_dom_feas: x from imbalanced, y from balanced 
    # x_glue_rel / y_glue_feas: from imbalanced 
    # x_ssl_rel / x_ssl_feas: from imbalanced

    xmin=0.0
    xmax=2.0
    ymin=0.0
    ymax=2.0
    # x_dom_rel / x_dom_feas / y_dom_feas: x from imbalanced, y from balanced 
    x_dom_rel,x_dom_feas,_,c_x_dom = get_2d_synthetic_data_for_swae(
        xmin=xmin,xmax=xmax,ymin=ymin,ymax=ymax,MAKE_IMBALANCE=DATA_IMBALANCE)
    x_dom_rel_test,_,y_dom_feas,c_y_dom = get_2d_synthetic_data_for_swae(
        xmin=xmin,xmax=xmax,ymin=ymin,ymax=ymax,MAKE_IMBALANCE=False)
    n_x_dom,n_y_dom = x_dom_rel.shape[0],y_dom_feas.shape[0]
    # x_glue_rel / y_glue_feas: from imbalanced 
    x_rel4glue,x_feas4glue,y_feas4glue,c = get_2d_synthetic_data_for_swae(
        xmin=xmin,xmax=xmax,ymin=ymin,ymax=ymax,MAKE_IMBALANCE=DATA_IMBALANCE)
    np.random.seed(seed=seed)
    idx_glue = np.random.permutation(x_rel4glue.shape[0])[:n_glue] # select
    x_glue_rel,x_glue_feas,y_glue_feas = x_rel4glue[idx_glue,:],x_feas4glue[idx_glue,:],y_feas4glue[idx_glue,:]
    # x_ssl_rel / x_ssl_rel: from imbalanced 
    x_rel4ssl,x_feas4ssl,_,_ = get_2d_synthetic_data_for_swae(
        xmin=xmin,xmax=xmax,ymin=ymin,ymax=ymax,MAKE_IMBALANCE=False)
    idx_ssl = np.random.permutation(x_rel4ssl.shape[0])[:n_ssl] # select
    x_ssl_rel,x_ssl_feas = x_rel4ssl[idx_ssl,:],x_feas4ssl[idx_ssl,:]
    print ("n_x_dom:[%d], n_y_dom:[%d], n_glue:[%d], n_ssl:[%d]"%(n_x_dom,n_y_dom,n_glue,n_ssl))

    # Plot data
    ms,ma,am,tfs,lfs = 15,0.4,0.1,13,11
    plt.figure(figsize=(3.5*3,3))
    # Relaxed X and glue data
    plt.subplot(131)
    plt.scatter(x_dom_rel[:,0],x_dom_rel[:,1],c=c_x_dom,edgecolors=c_x_dom,
                s=ms,marker='o',alpha=ma)
    if x_glue_rel is not None:
        plt.scatter(
            x_glue_rel[:,0],x_glue_rel[:,1],
            c='w',edgecolors='k',s=20,marker='o',alpha=1.0,label='Glue Data')
    rect = plt.Rectangle(
        xy=(0,0),width=1,height=1,fill=False,edgecolor='k',linewidth=2.5,
        label='Feasible Region')
    plt.gca().add_patch(rect)
    MAKE_IMBALANCE = True
    if MAKE_IMBALANCE:
        rect = plt.Rectangle(
            xy=(0,0),width=1/4,height=1/4,fill=False,edgecolor='b',linewidth=1.5,
            label='Imbalance Region')
        plt.gca().add_patch(rect)
    plt.title("Relaxed X",fontsize=tfs)
    plt.grid();plt.gca().set_aspect('equal', adjustable='box')
    plt.axis([0-am,2+am,0-am,2+am])
    plt.legend(loc='upper right',fontsize=lfs)    

    # Feasible X and glue data
    plt.subplot(132)
    plt.scatter(x_dom_feas[:,0],x_dom_feas[:,1],
                c=c_x_dom,edgecolors=c_x_dom,s=ms,marker='o',alpha=ma)
    if x_glue_feas is not None:
        plt.scatter(x_glue_feas[:,0],x_glue_feas[:,1],
                    c='w',edgecolors='k',s=20,marker='o',alpha=1.0)
    plt.title("Feasible X",fontsize=tfs)
    plt.grid();plt.gca().set_aspect('equal', adjustable='box')
    plt.axis([0-am,2+am,0-am,2+am])

    # Feasible Y
    plt.subplot(133)
    plt.scatter(y_dom_feas[:,0],y_dom_feas[:,1],
                c=c_y_dom,edgecolors=c,s=ms,marker='o',alpha=ma)
    if y_glue_feas is not None:
        plt.scatter(y_glue_feas[:,0],y_glue_feas[:,1],
                    c='w',edgecolors='k',s=20,marker='o',alpha=1.0)
    plt.title("Feasible Y",fontsize=tfs)
    plt.grid();plt.gca().set_aspect('equal', adjustable='box')
    plt.axis([0-am,2+am,0-am,2+am])
    plt.show()

    # Make all mean-zero
    xm,ym = dpp_mean(x_dom_rel),dpp_mean(y_dom_feas)
    x_dom_rel_mz,x_dom_feas_mz,x_glue_rel_mz,x_glue_feas_mz,x_ssl_rel_mz,x_ssl_feas_mz = \
        x_dom_rel-xm,x_dom_feas-xm,x_glue_rel-xm,x_glue_feas-xm,x_ssl_rel-xm,x_ssl_feas-xm
    x_dom_rel_mz_test = x_dom_rel_test - xm
    y_dom_feas_mz,y_glue_feas_mz = y_dom_feas-ym,y_glue_feas-ym

    # DPP sets
    x_dom_rel_mz_dpps,K_x_dom_rel_mz_dpps = dpp_partition(x_dom_rel_mz,n_dpp=n_dpp,seed=seed)
    x_dom_feas_mz_dpps,K_x_dom_feas_mz_dpps = dpp_partition(x_dom_feas_mz,n_dpp=n_dpp,seed=seed)
    y_dom_mz_dpps,K_y_dom_mz_dpps = dpp_partition(y_dom_feas_mz,n_dpp=n_dpp,seed=seed)

    # Instantiate SWAE
    tf.reset_default_graph()
    tf.set_random_seed(seed=seed)
    np.random.seed(seed=seed)
    S = SharedWassersteinAutoEncoderClass(
        xname='wae_x',yname='wae_y',xdim=2,zdim=zdim,
        hdims_Q=hdims,hdims_P=hdims,hdims_D=hdims,
        actv_Q=tf.nn.relu,actv_P=tf.nn.relu,actv_D=tf.nn.relu,
        actv_latent=None,actv_out=None,ki=ki,
        adam_beta1=adam_beta1,adam_beta2=adam_beta2,adam_epsilon=adam_epsilon,
    )
    sess = gpu_sess() # open session, old legacy of TF 
    tf.set_random_seed(seed=seed)
    np.random.seed(seed=seed)
    sess.run(tf.global_variables_initializer())

    # Loop
    lr_rate = 1.0
    lr_d_weight_x,lr_d_weight_y = 1.0,1.0 # adaptive learning rate 
    for it in range(max_iter): 
        zero_to_one = it/max_iter
        lr_rate = lr_rate_fr - (lr_rate_fr-lr_rate_to)*zero_to_one
        
        # if zero_to_one > 0.5:  latent_beta = 0.05*zero_to_one

        r_idx = np.random.permutation(n_x_dom)[:batch_size]
        x_dom_rel_mz_rand,x_dom_feas_mz_rand = x_dom_rel_mz[r_idx,:],x_dom_feas_mz[r_idx,:]
        if USE_DPP:
            x_dom_rel_mz_dpp,x_dom_feas_mz_dpp = dpp_sample_pair(
                x_dom_rel_mz_dpps,K_x_dom_rel_mz_dpps,x_dom_feas_mz_dpps,K_x_dom_feas_mz_dpps,batch_size)
        r_idx = np.random.permutation(n_y_dom)[:batch_size]
        y_dom_feas_mz_rand = y_dom_feas_mz[r_idx,:]
        if USE_DPP:
            y_dom_feas_mz_dpp = dpp_sample(y_dom_mz_dpps,K_y_dom_mz_dpps,batch_size)

        # Recon & latent fitting
        if USE_DPP:
            if USE_SSL:
                if np.random.rand() < 0.3:
                    x_recon_in,x_recon_out = x_dom_rel_mz_dpp,x_dom_feas_mz_dpp
                    y_recon_in,y_recon_out = y_dom_feas_mz_dpp,y_dom_feas_mz_dpp
                else:
                    x_recon_in,x_recon_out = x_dom_rel_mz_rand,x_dom_feas_mz_rand
                    y_recon_in,y_recon_out = y_dom_feas_mz_rand,y_dom_feas_mz_rand
            else:
                if np.random.rand() < 0.3:
                    x_recon_in,x_recon_out = x_dom_rel_mz_dpp,x_dom_rel_mz_dpp
                    y_recon_in,y_recon_out = y_dom_feas_mz_dpp,y_dom_feas_mz_dpp
                else:
                    x_recon_in,x_recon_out = x_dom_rel_mz_rand,x_dom_rel_mz_rand
                    y_recon_in,y_recon_out = y_dom_feas_mz_rand,y_dom_feas_mz_rand
            if np.random.rand() < 0.5:
                x_latent = x_dom_rel_mz_dpp
            else:
                x_latent = x_dom_rel_mz_rand
                
            y_latent = y_dom_feas_mz_dpp
        else:
            if USE_SSL:
                x_recon_in,x_recon_out,x_latent = x_dom_rel_mz_rand,x_dom_feas_mz_rand,x_dom_rel_mz_rand
                y_recon_in,y_recon_out,y_latent = y_dom_feas_mz_rand,y_dom_feas_mz_rand,y_dom_feas_mz_rand
            else:
                x_recon_in,x_recon_out,x_latent = x_dom_rel_mz_rand,x_dom_rel_mz_rand,x_dom_rel_mz_rand
                y_recon_in,y_recon_out,y_latent = y_dom_feas_mz_rand,y_dom_feas_mz_rand,y_dom_feas_mz_rand
                
        # Override latent learning batch
        if USE_DPP:
            x_latent = dpp_sample(x_dom_rel_mz_dpps,K_x_dom_rel_mz_dpps,batch_size4latent)
            y_latent = dpp_sample(y_dom_mz_dpps,K_y_dom_mz_dpps,batch_size4latent)

        # Latent glue
        glue_idx = np.random.permutation(n_glue)[:batch_size]
        x_glue = x_glue_rel_mz[glue_idx,:]
        y_glue = y_glue_feas_mz[glue_idx,:]

        # SSL
        ssl_idx = np.random.permutation(n_ssl)[:batch_size]
        x_nce_anc = x_ssl_rel_mz[ssl_idx,:]
        x_nce_pos = x_ssl_feas_mz[ssl_idx,:]

        # Update
        recon_loss_x,wd_loss_x,d_loss_x,g_loss_x,\
            recon_loss_y,wd_loss_y,d_loss_y,g_loss_y,\
            lc_loss,nce_loss,x2y_loss = S.update(
                sess,x_recon_in,x_recon_out,x_latent,y_recon_in,y_recon_out,y_latent,
                x_glue,y_glue,x_nce_anc,x_nce_pos,x_glue,y_glue,
                latent_beta=latent_beta,l1_recon_coef=l1_recon_coef,l2_recon_coef=l2_recon_coef,wd_coef=wd_coef,
                lr_recon_x=lr_recon,lr_d_x=lr_d*lr_d_weight_x,lr_g_x=lr_g,
                lr_recon_y=lr_recon,lr_d_y=lr_d*lr_d_weight_y,lr_g_y=lr_g,
                lr_lc=lr_lc,l1_lc_coef=l1_lc_coef,l2_lc_coef=l2_lc_coef,lr_nce=lr_nce,nce_coef=nce_coef,
                lr_x2y=lr_x2y,l1_x2y_coef=l1_x2y_coef,l2_x2y_coef=l2_x2y_coef,lr_rate=lr_rate
                )
        lr_d_weight_x,lr_d_weight_y = min(1.0,d_loss_x/(0.1+g_loss_x)),min(1.0,d_loss_y/(0.1+g_loss_y))
        total_loss = recon_loss_x + wd_loss_x + d_loss_x + g_loss_x + recon_loss_y + wd_loss_y + d_loss_y + g_loss_y + \
            lc_loss + nce_loss + x2y_loss

        # Print results every some iterations 
        if ((it % print_every) == 0) or ((it+1) == max_iter): 
            print (("[%d][%.1f%%] X R:[%.3f] D:[%.3f] G:[%.3f] WD:[%.3f] / Y R:[%.3f] D:[%.3f] G:[%.3f] WD:[%.3f]"
                   " / LC:[%.3f] / NCE:[%.3f] / X2Y:[%.3f]")%
                   (it,zero_to_one*100,recon_loss_x,d_loss_x,g_loss_x,wd_loss_x,
                    recon_loss_y,d_loss_y,g_loss_y,wd_loss_y,
                    lc_loss,nce_loss,x2y_loss))

            # Compute error 
            z_from_x_relaxed = sess.run(S.W_x.z_real,feed_dict={S.W_x.ph_x_real:x_dom_rel_mz_test})
            y_recon_from_x_relaxed = sess.run(
                S.W_y.x_sample,feed_dict={S.W_y.ph_z_sample:z_from_x_relaxed})
            abs_err = 10*np.mean(np.abs(y_recon_from_x_relaxed - y_dom_feas_mz))
            print ("Total loss is [%.3f]. Avergae Absolute Error is [%.3f]."%(total_loss, abs_err))

        # Plot results every some iterations 
        if ((it % plot_every) == 0) or ((it+1) == max_iter): 
            print ("[%d] Plot "%(it))
            tfs,am = 12,0.1

            # 1. Plot the latent space 
            plt.figure(figsize=(3.5*3,3))
            plt.subplot(1,3,1)
            z_prior = S.W_x.S.sampler(1000)
            plt.scatter(z_prior[:,0],z_prior[:,1],c='k',cmap='jet')
            plt.title('Latent Prior',fontsize=tfs);plt.grid()
            plt.gca().set_aspect('equal', adjustable='box')
            plt.xlim([-1.5,1.5]); plt.ylim([-1.5,1.5])
            plt.subplot(1,3,2)
            z_real = sess.run(S.W_x.z_real,feed_dict={S.W_x.ph_x_real:x_dom_rel_mz_test})
            plt.scatter(z_real[:,0],z_real[:,1],c=c_y_dom,cmap='jet')
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title('Latent of X',fontsize=tfs);plt.grid()
            plt.xlim([-1.5,1.5]); plt.ylim([-1.5,1.5])
            plt.subplot(1,3,3)
            z_real = sess.run(S.W_y.z_real,feed_dict={S.W_y.ph_x_real:y_dom_feas_mz})
            plt.scatter(z_real[:,0],z_real[:,1],c=c_y_dom,cmap='jet')
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title('Latent of Y',fontsize=tfs);plt.grid()
            plt.xlim([-1.5,1.5]); plt.ylim([-1.5,1.5])
            plt.show()

            # 2. Plot recontruction results
            xmin,xmax,ymin,ymax,am = -1,+1,-1,+1,0.1
            plt.figure(figsize=(3.5*5,3))
            plt.subplot(1,5,1)
            plt.scatter(x_dom_rel_mz_test[:,0],x_dom_rel_mz_test[:,1],c=c_y_dom,cmap='jet')
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title('Relaxed X',fontsize=tfs);plt.grid()
            plt.axis([xmin-am,xmax+am,ymin-am,ymax+am])
            plt.subplot(1,5,2)
            x_recon = sess.run(S.W_x.x_recon,feed_dict={S.W_x.ph_x_real:x_dom_rel_mz_test})
            plt.scatter(x_recon[:,0],x_recon[:,1],c=c_y_dom,cmap='jet')
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title('Reconstructed X',fontsize=tfs);plt.grid()
            plt.axis([xmin-am,xmax+am,ymin-am,ymax+am])
            plt.subplot(1,5,3)
            plt.scatter(y_dom_feas_mz[:,0],y_dom_feas_mz[:,1],c=c_y_dom,cmap='jet')
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title('Feasible Y',fontsize=tfs);plt.grid()
            plt.axis([xmin-am,xmax+am,ymin-am,ymax+am])
            plt.subplot(1,5,4)
            y_recon = sess.run(S.W_y.x_recon,feed_dict={S.W_y.ph_x_real:y_dom_feas_mz})
            plt.scatter(y_recon[:,0],y_recon[:,1],c=c_y_dom,cmap='jet')
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title('Reconstructed Y',fontsize=tfs);plt.grid()
            plt.axis([xmin-am,xmax+am,ymin-am,ymax+am])
            plt.subplot(1,5,5)
            z_from_x_relaxed = sess.run(S.W_x.z_real,feed_dict={S.W_x.ph_x_real:x_dom_rel_mz_test})
            y_recon_from_x_relaxed = sess.run(
                S.W_y.x_sample,feed_dict={S.W_y.ph_z_sample:z_from_x_relaxed})
            plt.scatter(y_recon_from_x_relaxed[:,0],y_recon_from_x_relaxed[:,1],c=c_y_dom,cmap='jet')
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title('Mapped Y from Relaxed X',fontsize=tfs);plt.grid()
            plt.axis([xmin-am,xmax+am,ymin-am,ymax+am])
            plt.show()

            # Compute error 
            abs_err = 10*np.mean(np.abs(y_recon_from_x_relaxed - y_dom_feas_mz))
            print ("Avergae Absolute Error is [%.3f]."%(abs_err))
    


















































        